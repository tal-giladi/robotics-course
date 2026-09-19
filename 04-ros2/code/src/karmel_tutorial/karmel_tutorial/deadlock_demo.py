"""
deadlock_demo: the classic rclpy service deadlock, and its fix (lesson 04.07).

A timer fires every 2 s and asks battery_health to set the threshold.

  ros2 run karmel_tutorial deadlock_demo sync    # blocking call() inside a callback: hangs
  ros2 run karmel_tutorial deadlock_demo async   # call_async() + done callback: works

Why sync hangs: rclpy.spin() runs every callback on ONE thread. call() blocks that thread
waiting for the response, but the response can only be delivered by... that same thread.
"""
import sys

from karmel_tutorial_interfaces.srv import SetLowThreshold
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from rclpy.utilities import remove_ros_args


class DeadlockDemo(Node):
    """Call a service from a timer callback, either the wrong way or the right way."""

    def __init__(self, mode: str) -> None:
        super().__init__('deadlock_demo')
        self._mode = mode
        self._count = 0
        self._client = self.create_client(SetLowThreshold, 'battery/set_low_threshold')
        self._timer = self.create_timer(2.0, self._on_timer)
        self.get_logger().info(f'mode={mode}')

    def _request(self) -> SetLowThreshold.Request:
        self._count += 1
        request = SetLowThreshold.Request()
        request.low_threshold_v = 10.5 + 0.1 * (self._count % 3)
        return request

    def _on_timer(self) -> None:
        if not self._client.service_is_ready():
            self.get_logger().info('waiting for battery/set_low_threshold ...')
            return
        if self._mode == 'sync':
            self.get_logger().info('calling synchronously ...')
            response = self._client.call(self._request())  # never returns
            self.get_logger().info(f'got: {response.message}')
        else:
            self.get_logger().info('calling asynchronously ...')
            future = self._client.call_async(self._request())
            future.add_done_callback(self._on_response)

    def _on_response(self, future: Future) -> None:
        self.get_logger().info(f'got: {future.result().message}')


def main(args=None) -> None:
    rclpy.init(args=args)
    argv = remove_ros_args(sys.argv)
    mode = argv[1] if len(argv) > 1 else 'sync'
    node = DeadlockDemo(mode)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
