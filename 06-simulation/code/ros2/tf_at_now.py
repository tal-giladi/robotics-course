#!/usr/bin/env python3
"""06.04 — Reproduce (and fix) the classic "extrapolation" error of simulation.

    python3 tf_at_now.py --ros-args -p use_sim_time:=false            # wall clock vs sim stamps
    python3 tf_at_now.py --ros-args -p use_sim_time:=true             # right clock, still "now"
    python3 tf_at_now.py --ros-args -p use_sim_time:=true -p at:=latest   # right clock, latest data

Twice a second, looks up target <- source (default first_world <- drop_box, bridged from Gazebo's
PosePublisher) and prints the result or the exception. at:=now asks for the node's current time;
at:=latest asks for the newest transform in the buffer (rclpy Time() = time zero).
"""
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TfAtNow(Node):
    def __init__(self) -> None:
        super().__init__('tf_at_now')
        self.target = self.declare_parameter('target', 'first_world').value
        self.source = self.declare_parameter('source', 'drop_box').value
        self.at = self.declare_parameter('at', 'now').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_timer(0.5, self.tick)

    def tick(self) -> None:
        when = self.get_clock().now() if self.at == 'now' else Time()
        try:
            tf = self.buffer.lookup_transform(self.target, self.source, when, timeout=Duration(seconds=0.1))
            p = tf.transform.translation
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            self.get_logger().info(f'{self.target} <- {self.source}: z = {p.z:.3f} m (stamp {stamp:.3f})')
        except TransformException as exc:
            self.get_logger().warn(f'{type(exc).__name__}: {exc}')


def main() -> None:
    rclpy.init()
    node = TfAtNow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
