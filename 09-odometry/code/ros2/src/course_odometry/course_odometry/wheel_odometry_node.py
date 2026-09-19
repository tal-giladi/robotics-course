"""wheel_odometry — YOUR odometry as a ROS 2 node (lesson 09.07).

Subscribes
  joint_states   sensor_msgs/JointState   wheel joint positions (rad), from karmel_base's base_node
                                          or ros2_control's joint_state_broadcaster
Publishes
  odom_mine      nav_msgs/Odometry        pose in odom_frame, twist in base_frame, growing covariance
  /tf            odom_frame -> base_frame only if publish_tf:=true (default false: on karmel
                 base_node or diff_drive_controller already publishes it, and a frame may have
                 only one parent publisher)

    ros2 run course_odometry wheel_odometry
    ros2 run course_odometry wheel_odometry --ros-args -p wheel_separation:=0.2078 \
        -p left_wheel_radius:=0.04471 -p right_wheel_radius:=0.04528
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

from course_odometry.odometry_core import DiffDriveOdometry, planar_to_ros_covariance, yaw_to_quaternion


class WheelOdometryNode(Node):

    def __init__(self) -> None:
        super().__init__('wheel_odometry')
        radius = self.declare_parameter('wheel_radius', 0.045).value
        self.core = DiffDriveOdometry(
            wheel_radius_left=self.declare_parameter('left_wheel_radius', radius).value,
            wheel_radius_right=self.declare_parameter('right_wheel_radius', radius).value,
            wheel_separation=self.declare_parameter('wheel_separation', 0.200).value,
            k_left=self.declare_parameter('wheel_noise_k', 1e-5).value,
            k_right=self.get_parameter('wheel_noise_k').value,
        )
        self.left_joint = self.declare_parameter('left_joint', 'left_wheel_joint').value
        self.right_joint = self.declare_parameter('right_joint', 'right_wheel_joint').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.twist_variance = self.declare_parameter('twist_variance', [1e-3, 5e-3]).value  # [vx, vyaw]
        publish_tf = self.declare_parameter('publish_tf', False).value

        self.pub = self.create_publisher(Odometry, 'odom_mine', 10)
        self.tf = TransformBroadcaster(self) if publish_tf else None
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 20)
        self.get_logger().info(
            f'wheel_odometry: r=({self.core.wheel_radius_left}, {self.core.wheel_radius_right}) '
            f'b={self.core.wheel_separation} {self.odom_frame}->{self.base_frame} publish_tf={publish_tf}')

    def on_joint_states(self, msg: JointState) -> None:
        try:
            left = msg.position[msg.name.index(self.left_joint)]
            right = msg.position[msg.name.index(self.right_joint)]
        except (ValueError, IndexError):
            return  # a JointState from another publisher (an arm, a gripper): not ours
        stamp = Time.from_msg(msg.header.stamp)
        # Use the MEASUREMENT time, not now(): the wheels were at these angles at header.stamp.
        if not self.core.update(left, right, stamp.nanoseconds * 1e-9):
            return
        self.publish(msg.header.stamp)

    def publish(self, stamp) -> None:
        c = self.core
        qx, qy, qz, qw = yaw_to_quaternion(c.yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = c.x
        odom.pose.pose.position.y = c.y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y = qx, qy
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = qz, qw
        odom.pose.covariance = planar_to_ros_covariance(c.covariance)
        odom.twist.twist.linear.x = c.v  # in child_frame_id (base frame): forward speed
        odom.twist.twist.angular.z = c.omega
        twist_cov = [0.0] * 36
        for i, var in enumerate([self.twist_variance[0], 1e-6, 1e-6, 1e-6, 1e-6, self.twist_variance[1]]):
            twist_cov[i * 7] = var
        odom.twist.covariance = twist_cov
        self.pub.publish(odom)

        if self.tf is not None:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = c.x
            t.transform.translation.y = c.y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
