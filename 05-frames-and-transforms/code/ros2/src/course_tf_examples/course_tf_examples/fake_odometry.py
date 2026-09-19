"""Dynamic broadcaster: odom -> base_footprint for a robot driving a circle (no hardware).

    ros2 run course_tf_examples fake_odometry
    ros2 run course_tf_examples fake_odometry --ros-args -p linear_speed:=0.0 -p angular_speed:=0.0

On the real robot this transform comes from wheel odometry (karmel_base, lesson 09.07).
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from course_tf_examples.tf_math import circle_pose, quaternion_from_euler


class FakeOdometry(Node):
    def __init__(self) -> None:
        super().__init__('fake_odometry')
        self.v = self.declare_parameter('linear_speed', 0.1).value       # m/s
        self.w = self.declare_parameter('angular_speed', 0.2).value      # rad/s -> 0.5 m radius
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        rate = self.declare_parameter('rate_hz', 30.0).value
        self.broadcaster = TransformBroadcaster(self)
        self.t0 = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self) -> None:
        now = self.get_clock().now()
        x, y, yaw = circle_pose((now - self.t0).nanoseconds * 1e-9, self.v, self.w)
        t = TransformStamped()
        t.header.stamp = now.to_msg()     # the time at which this pose was TRUE — never skip it
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeOdometry()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
