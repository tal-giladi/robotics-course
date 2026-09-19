r"""battery_health_params — battery_health from 04.06/04.07, configured by parameters (lesson 04.09).

A drop-in replacement for karmel_tutorial's battery_health node (same node name, same topics).
Instead of hard-coded constants and a custom SetLowThreshold service, every setting is a
declared, described, range-checked parameter that can come from YAML and change at runtime.

    ros2 run karmel_tutorial battery_sim
    ros2 run course_params_examples battery_health_params \
        --ros-args --params-file $(ros2 pkg prefix --share course_params_examples)/config/battery_health.yaml
    ros2 param set /battery_health low_threshold_v 10.8     # accepted
    ros2 param set /battery_health low_threshold_v 10.0     # rejected by the validation rule (< 9.9 + 0.2)
    ros2 param set /battery_health low_threshold_v 13.0     # rejected by the range (3S: 9.0 .. 12.75 V)

Subscribes  battery/voltage   std_msgs/Float32
Publishes   battery/health    karmel_tutorial_interfaces/BatteryHealth
Parameters  cells_series (int, read-only), full_v, low_threshold_v, critical_v, hysteresis_v,
            filter_alpha (double)
"""
from __future__ import annotations

from karmel_tutorial.battery_logic import BatteryHealthTracker, estimate_percent, LowPassFilter
from karmel_tutorial_interfaces.msg import BatteryHealth
from rcl_interfaces.msg import FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32

from course_params_examples.health_config import HealthConfig, voltage_range

DESCRIPTIONS = {
    'full_v': 'Pack voltage at 100 % charge [V]',
    'low_threshold_v': 'Below this filtered voltage the state becomes LOW [V]',
    'critical_v': 'Below this filtered voltage the state becomes CRITICAL [V]',
    'hysteresis_v': 'Margin the voltage must recover by before the state improves [V]',
    'filter_alpha': 'Low-pass weight of the newest sample, (0, 1]',
}


class BatteryHealthParams(Node):

    def __init__(self) -> None:
        # enable_logger_service: lets `ros2 service call .../set_logger_levels` change log levels (04.13)
        super().__init__('battery_health', enable_logger_service=True)

        # 1. A read-only parameter: fixed by the hardware, may come from YAML, never changes at runtime.
        cells = self.declare_parameter(
            'cells_series', 3,
            ParameterDescriptor(description='Li-ion cells in series (fixed by the hardware)', read_only=True,
                                integer_range=[IntegerRange(from_value=1, to_value=6, step=1)])).value

        # 2. Voltage parameters with a range derived from the pack. The range is checked by rclpy
        #    itself and is visible to tools: `ros2 param describe /battery_health low_threshold_v`.
        lo_v, hi_v = voltage_range(cells)
        defaults = HealthConfig()
        for name in HealthConfig.names():
            if name.endswith('_v') and name != 'hysteresis_v':
                rng = FloatingPointRange(from_value=lo_v, to_value=hi_v, step=0.0)
            elif name == 'hysteresis_v':
                rng = FloatingPointRange(from_value=0.0, to_value=1.0, step=0.0)
            else:
                rng = FloatingPointRange(from_value=0.01, to_value=1.0, step=0.0)
            self.declare_parameter(name, getattr(defaults, name),
                                   ParameterDescriptor(description=DESCRIPTIONS[name], floating_point_range=[rng]))

        # 3. Validate the start-up values (YAML overrides were applied during declare_parameter,
        #    BEFORE any on-set callback existed). Fail fast: a robot with a nonsense config should not start.
        self._config = HealthConfig(**{n: self.get_parameter(n).value for n in HealthConfig.names()})
        problems = self._config.problems()
        if problems:
            raise ValueError('invalid battery_health parameters: ' + '; '.join(problems))

        self._filter = LowPassFilter(alpha=self._config.filter_alpha)
        self._tracker = BatteryHealthTracker(low_threshold_v=self._config.low_threshold_v,
                                             critical_v=self._config.critical_v,
                                             hysteresis_v=self._config.hysteresis_v)

        # 4. Runtime changes: validate in the on-set callback, apply in the post-set callback.
        self.add_on_set_parameters_callback(self._validate)
        self.add_post_set_parameters_callback(self._apply)

        self._publisher = self.create_publisher(BatteryHealth, 'battery/health', 10)
        self.create_subscription(Float32, 'battery/voltage', self._on_voltage, 10)
        self.get_logger().info(f'{cells}S pack, config: {self._config}')

    # ---- parameter callbacks ----------------------------------------------------------------
    def _validate(self, params: list[Parameter]) -> SetParametersResult:
        """Accept or reject a batch of changes. Must NOT change node state (a later check may still reject)."""
        candidate = self._config.with_changes({p.name: p.value for p in params})
        problems = candidate.problems()
        if problems:
            return SetParametersResult(successful=False, reason='; '.join(problems))
        return SetParametersResult(successful=True)

    def _apply(self, params: list[Parameter]) -> None:
        """Apply a change; called only after it was accepted and stored."""
        self._config = self._config.with_changes({p.name: p.value for p in params})
        self._filter.alpha = self._config.filter_alpha
        self._tracker.low_threshold_v = self._config.low_threshold_v
        self._tracker.critical_v = self._config.critical_v
        self._tracker.hysteresis_v = self._config.hysteresis_v
        changed = ', '.join(f'{p.name}={p.value}' for p in params)
        self.get_logger().info(f'applied {changed}')

    # ---- data path (same as karmel_tutorial/battery_health) ----------------------------------
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
        self._publisher.publish(out)
        self.get_logger().debug(f'{msg.data:.2f} V raw, {filtered:.2f} V filtered, {state.name}')

        if state != previous:
            self.get_logger().warn(f'{previous.name} -> {state.name} at {filtered:.2f} V')


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = BatteryHealthParams()
    except ValueError as err:
        rclpy.logging.get_logger('battery_health').fatal(str(err))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
