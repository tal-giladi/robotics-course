"""
battery_health: turn raw voltage into a BatteryHealth message (04.06) + a service (04.07).

Subscribes
  battery/voltage              std_msgs/Float32
Publishes
  battery/health               karmel_tutorial_interfaces/BatteryHealth
Serves (lesson 04.07)
  battery/set_low_threshold    karmel_tutorial_interfaces/SetLowThreshold
"""
from karmel_tutorial.battery_logic import (BatteryHealthTracker, estimate_percent,
                                           HealthState, LowPassFilter)
from karmel_tutorial_interfaces.msg import BatteryHealth
from karmel_tutorial_interfaces.srv import SetLowThreshold
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class BatteryHealthNode(Node):
    """Filter the voltage, classify it, publish the result, accept threshold changes."""

    def __init__(self) -> None:
        super().__init__('battery_health')
        self._filter = LowPassFilter(alpha=0.3)
        self._tracker = BatteryHealthTracker()
        self._publisher = self.create_publisher(BatteryHealth, 'battery/health', 10)
        self._subscription = self.create_subscription(
            Float32, 'battery/voltage', self._on_voltage, 10)
        # --- lesson 04.07: the service -------------------------------------------------
        self._service = self.create_service(
            SetLowThreshold, 'battery/set_low_threshold', self._on_set_low_threshold)
        self.get_logger().info('battery_health ready')

    def _on_voltage(self, msg: Float32) -> None:
        filtered = self._filter.update(msg.data)
        previous = self._tracker.state
        state = self._tracker.update(filtered)

        out = BatteryHealth()
        out.header.stamp = self.get_clock().now().to_msg()
        out.voltage_v = filtered
        out.raw_voltage_v = msg.data
        out.percent = estimate_percent(filtered)
        out.state = int(state)
        out.low_threshold_v = self._tracker.low_threshold_v
        self._publisher.publish(out)

        if state != previous:
            text = f'{previous.name} -> {state.name} at {filtered:.2f} V ({out.percent:.0f} %)'
            if state == HealthState.CRITICAL:
                self.get_logger().error(text)
            elif state == HealthState.LOW:
                self.get_logger().warn(text)
            else:
                self.get_logger().info(text)

    # --- lesson 04.07 --------------------------------------------------------------------
    def _on_set_low_threshold(self, request: SetLowThreshold.Request,
                              response: SetLowThreshold.Response) -> SetLowThreshold.Response:
        # Service callbacks must be fast: validate, update in-memory state, return.
        response.previous_threshold_v = self._tracker.low_threshold_v
        problem = self._tracker.check_threshold(request.low_threshold_v)
        if problem is not None:
            response.accepted = False
            response.message = problem
            self.get_logger().warn(f'Rejected threshold change: {problem}')
            return response
        self._tracker.low_threshold_v = request.low_threshold_v
        response.accepted = True
        response.message = (f'low threshold {response.previous_threshold_v:.2f} V -> '
                            f'{request.low_threshold_v:.2f} V')
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryHealthNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
