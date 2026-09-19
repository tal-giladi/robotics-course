"""battery_status_server — a tiny std_srvs/Trigger service used by lesson 04.12.

    ros2 run course_executor_examples battery_status_server

Serves  battery/get_status  (std_srvs/srv/Trigger) and answers with a fake voltage.
It logs every request, so you can see that the server DID answer while a deadlocked
client never receives the response.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger


class BatteryStatusServer(Node):

    def __init__(self) -> None:
        super().__init__('battery_status_server')
        self.voltage_v = self.declare_parameter('voltage_v', 11.42).value
        self.create_service(Trigger, 'battery/get_status', self.on_request)

    def on_request(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        response.success = True
        response.message = f'{self.voltage_v:.2f} V'
        self.get_logger().info(f'answered get_status: {response.message}')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BatteryStatusServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
