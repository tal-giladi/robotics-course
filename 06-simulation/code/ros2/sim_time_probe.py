#!/usr/bin/env python3
"""06.04 — Which clock does this node see?

    python3 sim_time_probe.py                                   # wall clock
    python3 sim_time_probe.py --ros-args -p use_sim_time:=true  # /clock from Gazebo

Prints, once per *node-clock* second, the node's now(), the wall clock, and how much wall time
passed since the previous tick. With use_sim_time:=true and the simulation running at a real-time
factor of 0.5, a 1 s timer fires every 2 wall seconds; with the simulation paused it never fires.
"""
import time

import rclpy
from rclpy.node import Node


class SimTimeProbe(Node):
    def __init__(self) -> None:
        super().__init__('sim_time_probe')
        self.last_wall = time.monotonic()
        self.create_timer(1.0, self.tick)  # 1 s on the NODE's clock (sim or wall)
        sim = self.get_parameter('use_sim_time').value
        self.get_logger().info(f'use_sim_time={sim}')

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        wall = time.time()
        gap = time.monotonic() - self.last_wall
        self.last_wall = time.monotonic()
        self.get_logger().info(f'node now = {now:14.3f} s   wall = {wall:14.3f} s   '
                               f'wall seconds since last tick = {gap:5.2f}')


def main() -> None:
    rclpy.init()
    node = SimTimeProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
