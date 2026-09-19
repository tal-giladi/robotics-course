r"""voltage_publisher_qos — battery_sim's voltage with the SENSOR DATA QoS profile (lesson 04.11).

    ros2 run course_qos_examples voltage_publisher_qos
    # QoS is overridable at start-up (read-only parameters created by QoSOverridingOptions):
    ros2 run course_qos_examples voltage_publisher_qos --ros-args \
        -p qos_overrides./battery/voltage.publisher.reliability:=reliable

Publishes  battery/voltage  std_msgs/Float32  (BEST_EFFORT / VOLATILE / KEEP_LAST(5) by default)

Start it next to karmel_tutorial's battery_monitor (RELIABLE subscriber) and the monitor
receives nothing: that is the QoS incompatibility this lesson teaches you to diagnose.
"""
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos_overriding_options import QoSOverridingOptions
from std_msgs.msg import Float32

from course_qos_examples.qos_util import describe


class VoltagePublisherQos(Node):

    def __init__(self) -> None:
        super().__init__('battery_sim')
        self._v = float(self.declare_parameter('start_v', 12.6).value)
        self._drain = float(self.declare_parameter('drain_v_per_s', 0.05).value)
        rate_hz = float(self.declare_parameter('rate_hz', 10.0).value)
        self._dt = 1.0 / rate_hz
        self._pub = self.create_publisher(
            Float32, 'battery/voltage', qos_profile_sensor_data,
            qos_overriding_options=QoSOverridingOptions.with_default_policies())
        self._count = 0
        self.create_timer(self._dt, self._on_timer)
        self.get_logger().info(
            f'publishing battery/voltage at {rate_hz:g} Hz with QoS {describe(self._pub.qos_profile)}')

    def _on_timer(self) -> None:
        self._v = max(9.0, self._v - self._drain * self._dt)
        self._pub.publish(Float32(data=self._v + random.gauss(0.0, 0.03)))
        self._count += 1
        if self._count % 50 == 0:
            self.get_logger().info(f'published {self._count} messages')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoltagePublisherQos()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
