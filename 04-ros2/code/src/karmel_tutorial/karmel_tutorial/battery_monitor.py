"""
battery_monitor: warn when the pack voltage is low or stops arriving (lesson 04.04).

Subscribes
  battery/voltage   std_msgs/Float32

Deliberately self-contained (no imports from this package) so it runs as a plain script
in lesson 04.04, before you learn packaging in 04.05.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32

LOW_WARNING_V = 10.5   # labs/config/karmel.yaml battery.low_warning_v
HYSTERESIS_V = 0.2     # must rise above 10.7 V before the warning clears
STALE_AFTER_S = 3.0    # no message for this long = publisher gone or network broken


class BatteryMonitor(Node):
    """Log a warning on low voltage (with hysteresis) and on silence."""

    def __init__(self) -> None:
        super().__init__('battery_monitor')
        self._low = False
        self._last_rx = None
        self._stale_reported = False
        self._subscription = self.create_subscription(
            Float32, 'battery/voltage', self._on_voltage, 10)
        # Pub/sub has no connection to break: silence is the only symptom of a dead
        # publisher, so we check for it ourselves.
        self._watchdog = self.create_timer(1.0, self._check_stale)
        self.get_logger().info(f'Watching battery/voltage, low below {LOW_WARNING_V:.2f} V')

    def _on_voltage(self, msg: Float32) -> None:
        self._last_rx = self.get_clock().now()
        self._stale_reported = False
        voltage = msg.data
        if not self._low and voltage < LOW_WARNING_V:
            self._low = True
            self.get_logger().warn(
                f'Battery LOW: {voltage:.2f} V (threshold {LOW_WARNING_V:.2f} V)')
        elif self._low and voltage > LOW_WARNING_V + HYSTERESIS_V:
            self._low = False
            self.get_logger().info(f'Battery recovered: {voltage:.2f} V')
        else:
            self.get_logger().debug(f'{voltage:.2f} V')

    def _check_stale(self) -> None:
        if self._last_rx is None or self._stale_reported:
            return
        silent_s = (self.get_clock().now() - self._last_rx).nanoseconds * 1e-9
        if silent_s > STALE_AFTER_S:
            self._stale_reported = True
            self.get_logger().error(f'No battery data for {silent_s:.1f} s')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
