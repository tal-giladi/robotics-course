"""lifecycle_battery_health — battery_health as a lifecycle (managed) node (lesson 04.15).

    ros2 run course_lifecycle_examples lifecycle_battery_health
    ros2 lifecycle get /battery_health                 # unconfigured [1]
    ros2 lifecycle set /battery_health configure       # -> inactive: params read, publisher created
    ros2 lifecycle set /battery_health activate        # -> active:   subscribes, publishes battery/health
    ros2 lifecycle set /battery_health deactivate      # -> inactive: stops, keeps its configuration
    ros2 lifecycle set /battery_health cleanup         # -> unconfigured: releases everything
    ros2 lifecycle set /battery_health shutdown        # -> finalized

Same topics and parameters as course_params_examples/battery_health_params:
  Subscribes  battery/voltage   std_msgs/Float32                          (only while ACTIVE)
  Publishes   battery/health    karmel_tutorial_interfaces/BatteryHealth  (lifecycle publisher)

Parameters are declared in __init__ (so tools and YAML can set them at any time) but READ in
on_configure: change them while inactive, then cleanup + configure to pick them up.
An invalid configuration makes on_configure return FAILURE and the node stays unconfigured.
"""
from __future__ import annotations

from karmel_tutorial.battery_logic import BatteryHealthTracker, estimate_percent, LowPassFilter
from karmel_tutorial_interfaces.msg import BatteryHealth
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, LifecyclePublisher, LifecycleState, TransitionCallbackReturn
from rclpy.subscription import Subscription
from std_msgs.msg import Float32

from course_params_examples.health_config import HealthConfig


class LifecycleBatteryHealth(LifecycleNode):

    def __init__(self) -> None:
        super().__init__('battery_health')
        for name, default in HealthConfig().__dict__.items():
            self.declare_parameter(name, default)
        self._pub: LifecyclePublisher | None = None
        self._sub: Subscription | None = None
        self._config: HealthConfig | None = None
        self._filter: LowPassFilter | None = None
        self._tracker: BatteryHealthTracker | None = None
        self.get_logger().info('created (unconfigured): nothing allocated, nothing subscribed')

    # unconfigured -> inactive
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        config = HealthConfig(**{n: float(self.get_parameter(n).value) for n in HealthConfig.names()})
        problems = config.problems()
        if problems:
            self.get_logger().error('configure FAILED: ' + '; '.join(problems))
            return TransitionCallbackReturn.FAILURE
        self._config = config
        self._filter = LowPassFilter(alpha=config.filter_alpha)
        self._tracker = BatteryHealthTracker(low_threshold_v=config.low_threshold_v, critical_v=config.critical_v,
                                             hysteresis_v=config.hysteresis_v)
        self._pub = self.create_lifecycle_publisher(BatteryHealth, 'battery/health', 10)
        self.get_logger().info(f'on_configure: {config}')
        return TransitionCallbackReturn.SUCCESS

    # inactive -> active
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._sub = self.create_subscription(Float32, 'battery/voltage', self._on_voltage, 10)
        self.get_logger().info('on_activate: subscribed to battery/voltage, publishing battery/health')
        return super().on_activate(state)  # activates the lifecycle publisher(s)

    # active -> inactive
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.destroy_subscription(self._sub)
        self._sub = None
        self.get_logger().info('on_deactivate: unsubscribed; configuration kept')
        return super().on_deactivate(state)  # deactivates the lifecycle publisher(s)

    # inactive -> unconfigured
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._release()
        self.get_logger().info('on_cleanup: released publisher and state')
        return TransitionCallbackReturn.SUCCESS

    # unconfigured / inactive / active -> finalized
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._release()
        self.get_logger().info(f'on_shutdown (from {state.label})')
        return TransitionCallbackReturn.SUCCESS

    def _release(self) -> None:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if self._pub is not None:
            self.destroy_lifecycle_publisher(self._pub)
            self._pub = None
        self._config = self._filter = self._tracker = None

    def _on_voltage(self, msg: Float32) -> None:
        filtered = self._filter.update(msg.data)
        previous = self._tracker.state
        state = self._tracker.update(filtered)
        out = BatteryHealth()
        out.header.stamp = self.get_clock().now().to_msg()
        out.voltage_v = filtered
        out.raw_voltage_v = msg.data
        out.percent = estimate_percent(filtered, empty_v=self._config.critical_v, full_v=self._config.full_v)
        out.state = int(state)
        out.low_threshold_v = self._tracker.low_threshold_v
        self._pub.publish(out)
        if state != previous:
            self.get_logger().warn(f'{previous.name} -> {state.name} at {filtered:.2f} V')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LifecycleBatteryHealth()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
