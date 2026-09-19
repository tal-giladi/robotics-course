"""timer_starvation — a slow callback starving a 20 Hz control loop (lesson 04.12).

    ros2 run course_executor_examples timer_starvation --ros-args -p executor:=single
    ros2 run course_executor_examples timer_starvation --ros-args -p executor:=multi

A 20 Hz "control" timer measures the gap between its own ticks. A 1 Hz "planner" timer
simulates 0.3 s of blocking work (time.sleep releases the GIL, like numpy/OpenCV/IO do).
Every 5 s the node prints the control loop's real rate and its worst gap.

executor:=single  both timers share one thread          -> control loop stalls
executor:=multi   MultiThreadedExecutor + one MutuallyExclusiveCallbackGroup per timer
"""
import time

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node


class TimerStarvation(Node):

    def __init__(self) -> None:
        super().__init__('timer_starvation')
        self.executor_kind = self.declare_parameter('executor', 'single').value
        self.control_period_s = 0.05
        work_s = self.declare_parameter('planner_work_s', 0.3).value
        self.work_s = work_s

        if self.executor_kind == 'multi':
            control_group, planner_group = MutuallyExclusiveCallbackGroup(), MutuallyExclusiveCallbackGroup()
        else:
            control_group = planner_group = None  # default group: one lane for everything

        self.create_timer(self.control_period_s, self.on_control, callback_group=control_group)
        self.create_timer(1.0, self.on_planner, callback_group=planner_group)
        self.create_timer(5.0, self.on_report, callback_group=control_group)
        self.last = time.monotonic()
        self.window_start = self.last
        self.ticks = 0
        self.max_gap = 0.0

    def on_control(self) -> None:
        now = time.monotonic()
        self.max_gap = max(self.max_gap, now - self.last)
        self.last = now
        self.ticks += 1

    def on_planner(self) -> None:
        time.sleep(self.work_s)  # stands in for a slow path planner or a blocking read

    def on_report(self) -> None:
        now = time.monotonic()
        rate = self.ticks / (now - self.window_start)
        self.get_logger().info(
            f'executor={self.executor_kind}: control loop {rate:5.1f} Hz (target 20.0), '
            f'worst gap {self.max_gap * 1000:4.0f} ms (target 50)')
        self.window_start, self.ticks, self.max_gap = now, 0, 0.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TimerStarvation()
    executor = MultiThreadedExecutor(num_threads=3) if node.executor_kind == 'multi' else SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
