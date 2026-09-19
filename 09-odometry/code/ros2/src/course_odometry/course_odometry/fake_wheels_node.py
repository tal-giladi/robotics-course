"""fake_wheels — publish sensor_msgs/JointState for two wheels turning at set speeds (no robot needed).

    ros2 run course_odometry fake_wheels                                    # left 4, right 8 rad/s: 0.3 m circle
    ros2 run course_odometry fake_wheels --ros-args -p left:=-3.0 -p right:=3.0 -p noise:=0.02

Lets you test wheel_odometry, odom_compare, RViz and TF on a laptop. ``noise`` adds a random walk
to each wheel angle (rad per sqrt(s)), a crude stand-in for slip.
"""
from __future__ import annotations

import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


class FakeWheelsNode(Node):

    def __init__(self) -> None:
        super().__init__('fake_wheels')
        self.speeds = [self.declare_parameter('left', 4.0).value, self.declare_parameter('right', 8.0).value]
        self.noise = self.declare_parameter('noise', 0.0).value
        rate = self.declare_parameter('rate', 50.0).value
        self.names = [self.declare_parameter('left_joint', 'left_wheel_joint').value,
                      self.declare_parameter('right_joint', 'right_wheel_joint').value]
        self.dt = 1.0 / rate
        self.angles = [0.0, 0.0]
        self.pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(self.dt, self.tick)

    def tick(self) -> None:
        for i in (0, 1):
            self.angles[i] += self.speeds[i] * self.dt + random.gauss(0.0, self.noise * self.dt ** 0.5)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.names)
        msg.position = list(self.angles)
        msg.velocity = [float(s) for s in self.speeds]
        self.pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeWheelsNode()
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
