"""
battery_sim: publish a simulated, slowly draining 3S pack voltage (lesson 04.04).

Publishes
  battery/voltage   std_msgs/Float32   pack voltage [V] at rate_hz

Run it (after `source /opt/ros/jazzy/setup.bash`):
  python3 battery_sim.py                      # lesson 04.04, before packaging
  ros2 run karmel_tutorial battery_sim        # lesson 04.05 onwards
  ros2 run karmel_tutorial battery_sim --ros-args -p drain_v_per_s:=0.2
"""
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


class BatterySim(Node):
    """Fake battery: linear discharge plus Gaussian measurement noise."""

    FLOOR_V = 9.0  # a real pack's BMS disconnects around here

    def __init__(self) -> None:
        super().__init__('battery_sim')
        # Parameters are covered properly in 04.09. Here they just make the demo tunable.
        self._start_v = float(self.declare_parameter('start_v', 12.6).value)
        self._drain_v_per_s = float(self.declare_parameter('drain_v_per_s', 0.05).value)
        self._noise_v = float(self.declare_parameter('noise_v', 0.03).value)
        rate_hz = float(self.declare_parameter('rate_hz', 2.0).value)

        self._rng = random.Random()
        self._t0 = self.get_clock().now()
        self._publisher = self.create_publisher(Float32, 'battery/voltage', 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)
        self.get_logger().info(
            f'Simulating a pack from {self._start_v:.2f} V, draining '
            f'{self._drain_v_per_s:.3f} V/s, publishing at {rate_hz:.1f} Hz')

    def _on_timer(self) -> None:
        elapsed_s = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
        true_v = max(self.FLOOR_V, self._start_v - self._drain_v_per_s * elapsed_s)
        msg = Float32()
        msg.data = true_v + self._rng.gauss(0.0, self._noise_v)
        self._publisher.publish(msg)
        self.get_logger().info(f'Publishing {msg.data:.2f} V')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatterySim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
