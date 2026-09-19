"""Dynamic broadcaster: map -> odom, the way a localizer (AMCL, slam_toolbox) publishes it.

    ros2 run course_tf_examples fake_localization
    ros2 run course_tf_examples fake_localization --ros-args -p jump_every_s:=0.0   # never jumps

It starts at (2.0, 1.0, 90 deg): the robot's odometry started at that spot of the map.
Every ``jump_every_s`` seconds it applies a small correction, the way a real localizer does when
a laser scan shows that odometry drifted. map -> odom jumps; odom -> base_footprint does not.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from course_tf_examples.tf_math import quaternion_from_euler


class FakeLocalization(Node):
    def __init__(self) -> None:
        super().__init__('fake_localization')
        self.x = self.declare_parameter('x', 2.0).value
        self.y = self.declare_parameter('y', 1.0).value
        self.yaw = math.radians(self.declare_parameter('yaw_deg', 90.0).value)
        jump_every = self.declare_parameter('jump_every_s', 5.0).value
        self.broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.publish)                       # 20 Hz
        if jump_every > 0.0:
            self.create_timer(jump_every, self.correct)

    def correct(self) -> None:
        self.x += 0.03
        self.y -= 0.02
        self.yaw += math.radians(1.0)
        self.get_logger().info(
            f'correction: map->odom is now ({self.x:.2f}, {self.y:.2f}, '
            f'{math.degrees(self.yaw):.1f} deg)')

    def publish(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.yaw)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
