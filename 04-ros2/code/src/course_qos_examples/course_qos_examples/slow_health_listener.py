"""slow_health_listener — what history depth does to a slow subscriber (lesson 04.11).

    ros2 run karmel_tutorial battery_sim --ros-args -p rate_hz:=50.0
    ros2 run karmel_tutorial battery_health          # or course_params_examples battery_health_params
    ros2 run course_qos_examples slow_health_listener --ros-args -p depth:=10
    ros2 run course_qos_examples slow_health_listener --ros-args -p depth:=1

Subscribes  battery/health  karmel_tutorial_interfaces/BatteryHealth  (RELIABLE / KEEP_LAST(depth))

The callback takes processing_s (default 0.1 s). Every 5 s the node logs how many messages it
processed and how OLD they were when processing started (now - header.stamp).
"""
import time

from karmel_tutorial_interfaces.msg import BatteryHealth
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time


class SlowHealthListener(Node):

    def __init__(self) -> None:
        super().__init__('slow_health_listener')
        depth = int(self.declare_parameter('depth', 10).value)
        self._work_s = float(self.declare_parameter('processing_s', 0.1).value)
        self._ages: list[float] = []
        self.create_subscription(BatteryHealth, 'battery/health', self._on_msg, QoSProfile(depth=depth))
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'depth={depth}, processing_s={self._work_s}')

    def _on_msg(self, msg: BatteryHealth) -> None:
        age = (self.get_clock().now() - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        self._ages.append(age)
        time.sleep(self._work_s)  # stands in for real work

    def _report(self) -> None:
        if self._ages:
            mean_ms = 1000.0 * sum(self._ages) / len(self._ages)
            self.get_logger().info(
                f'processed {len(self._ages)} msgs, mean age {mean_ms:.0f} ms, '
                f'max age {1000.0 * max(self._ages):.0f} ms')
        self._ages.clear()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlowHealthListener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
