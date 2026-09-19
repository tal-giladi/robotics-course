"""
set_threshold_client: call battery/set_low_threshold once and print the answer (04.07).

Usage:
  ros2 run karmel_tutorial set_threshold_client 11.0
"""
import sys

from karmel_tutorial_interfaces.srv import SetLowThreshold
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


def main(args=None) -> int:
    rclpy.init(args=args)
    argv = remove_ros_args(sys.argv)
    if len(argv) != 2:
        print('usage: ros2 run karmel_tutorial set_threshold_client <volts>')
        rclpy.try_shutdown()
        return 2

    node = Node('set_threshold_client')
    try:
        client = node.create_client(SetLowThreshold, 'battery/set_low_threshold')
        # Discovery is asynchronous: the server may exist but not be discovered yet.
        if not client.wait_for_service(timeout_sec=5.0):
            node.get_logger().error('battery/set_low_threshold not available after 5 s')
            return 1

        request = SetLowThreshold.Request()
        request.low_threshold_v = float(argv[1])
        future = client.call_async(request)
        # Spin this node until the response arrives (or 5 s pass). There is no
        # built-in deadline in ROS 2 services: the timeout is the client's job.
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if not future.done():
            client.remove_pending_request(future)
            node.get_logger().error('no response within 5 s')
            return 1

        response = future.result()
        status = 'accepted' if response.accepted else 'REJECTED'
        print(f'{status}: {response.message} '
              f'(previous {response.previous_threshold_v:.2f} V)')
        return 0 if response.accepted else 3
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
