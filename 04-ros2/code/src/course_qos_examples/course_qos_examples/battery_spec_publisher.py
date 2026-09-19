"""battery_spec_publisher — a "latched" topic with TRANSIENT_LOCAL durability (lesson 04.11).

    ros2 run course_qos_examples battery_spec_publisher
    # a subscriber that joins LATER still gets the message, if it asks for TRANSIENT_LOCAL too:
    ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /battery/spec

Publishes  battery/spec  sensor_msgs/BatteryState  ONCE at start-up, RELIABLE / TRANSIENT_LOCAL / KEEP_LAST(1)

The pack's static facts (chemistry, cells, capacity) never change while the node runs, so
publishing them periodically is waste and publishing them once with VOLATILE durability
loses them for every late subscriber. /robot_description and /map use the same pattern.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState

from course_qos_examples.qos_util import describe

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class BatterySpecPublisher(Node):

    def __init__(self) -> None:
        super().__init__('battery_spec_publisher')
        cells = int(self.declare_parameter('cells_series', 3).value)
        pub = self.create_publisher(BatteryState, 'battery/spec', LATCHED)

        spec = BatteryState()
        spec.header.stamp = self.get_clock().now().to_msg()
        spec.header.frame_id = 'base_link'
        spec.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        spec.design_capacity = float(self.declare_parameter('capacity_ah', 3.5).value)  # labs/config/karmel.yaml
        spec.voltage = float('nan')          # not a measurement: this message only carries the spec
        spec.cell_voltage = [float('nan')] * cells
        spec.location = 'karmel main pack'
        spec.present = True
        pub.publish(spec)
        self._pub = pub  # keep a reference: the publisher's history is what late joiners receive
        self.get_logger().info(
            f'published battery/spec once ({cells}S, {spec.design_capacity} Ah) with QoS {describe(LATCHED)}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatterySpecPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
