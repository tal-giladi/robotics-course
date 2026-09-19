r"""voltage_listener_qos — a subscriber that SAYS when QoS is incompatible (lesson 04.11).

    ros2 run course_qos_examples voltage_listener_qos      # RELIABLE: incompatible with sensor-data QoS
    ros2 run course_qos_examples voltage_listener_qos --ros-args \
        -p qos_overrides./battery/voltage.subscription.reliability:=best_effort   # compatible

Subscribes  battery/voltage  std_msgs/Float32   (RELIABLE / VOLATILE / KEEP_LAST(10) by default)

Every 5 s it logs how many messages arrived. On an incompatible publisher it logs an
ERROR naming the offending policy, instead of staying silent.
"""
import rclpy
from rclpy.event_handler import QoSRequestedIncompatibleQoSInfo, SubscriptionEventCallbacks
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSPolicyKind, QoSProfile
from rclpy.qos_overriding_options import QoSOverridingOptions
from std_msgs.msg import Float32

from course_qos_examples.qos_util import describe


class VoltageListenerQos(Node):

    def __init__(self) -> None:
        super().__init__('voltage_listener')
        self._received = 0
        events = SubscriptionEventCallbacks(incompatible_qos=self._on_incompatible)
        self._sub = self.create_subscription(
            Float32, 'battery/voltage', self._on_msg, QoSProfile(depth=10),
            event_callbacks=events,
            qos_overriding_options=QoSOverridingOptions.with_default_policies())
        self.create_timer(5.0, self._report)
        self.get_logger().info(f'subscribed to battery/voltage with QoS {describe(self._sub.qos_profile)}')

    def _on_msg(self, msg: Float32) -> None:
        self._received += 1

    def _on_incompatible(self, info: QoSRequestedIncompatibleQoSInfo) -> None:
        policy = QoSPolicyKind(info.last_policy_kind).name
        self.get_logger().error(
            f'incompatible publisher on battery/voltage: policy {policy} '
            f'(total incompatible offers so far: {info.total_count})')

    def _report(self) -> None:
        self.get_logger().info(f'received {self._received} messages in the last 5 s')
        self._received = 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoltageListenerQos()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
