"""supervisor — a timer that calls a service, and the four ways to wire it (lesson 04.12).

    ros2 run course_executor_examples battery_status_server          # terminal 1
    ros2 run course_executor_examples supervisor                     # terminal 2: DEADLOCKS

Parameters (all read once at start-up):
  call_style      sync | async       sync = client.call() inside the timer callback
  executor        single | multi     SingleThreadedExecutor or MultiThreadedExecutor
  separate_group  false | true       put the client in its own MutuallyExclusiveCallbackGroup
  timeout_s       0.0                0 = wait forever; >0 = client.call(timeout_sec=...)

Only three combinations work:
  call_style:=async                                        (any executor)
  call_style:=sync executor:=multi separate_group:=true
  (sync + timeout "works" in the sense that it no longer hangs forever — it fails every time.)
"""
from functools import partial

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger


class Supervisor(Node):

    def __init__(self) -> None:
        super().__init__('supervisor')
        self.call_style = self.declare_parameter('call_style', 'sync').value
        self.executor_kind = self.declare_parameter('executor', 'single').value
        separate_group = self.declare_parameter('separate_group', False).value
        timeout_s = self.declare_parameter('timeout_s', 0.0).value
        self.timeout = timeout_s if timeout_s > 0.0 else None

        # None = the node's default callback group, which is MutuallyExclusive and is also
        # the group the timer below lives in.
        client_group = MutuallyExclusiveCallbackGroup() if separate_group else None
        self.client = self.create_client(Trigger, 'battery/get_status', callback_group=client_group)
        self.tick = 0
        self.create_timer(1.0, self.on_timer)
        self.get_logger().info(
            f'call_style={self.call_style} executor={self.executor_kind} '
            f'separate_group={separate_group} timeout_s={timeout_s}')

    def on_timer(self) -> None:
        self.tick += 1
        n = self.tick
        if not self.client.service_is_ready():
            self.get_logger().warn(f'tick {n}: battery/get_status not available yet')
            return
        self.get_logger().info(f'tick {n}: calling battery/get_status')

        if self.call_style == 'async':
            future = self.client.call_async(Trigger.Request())
            future.add_done_callback(partial(self.on_future_done, n))
            return  # the callback returns at once; the executor stays free

        # Synchronous: this thread blocks until the response arrives. The response is
        # delivered BY THE EXECUTOR — which needs a free thread allowed to run the client.
        response = self.client.call(Trigger.Request(), timeout_sec=self.timeout)
        if response is None:
            self.get_logger().error(f'tick {n}: no response after {self.timeout} s')
        else:
            self.get_logger().info(f'tick {n}: got "{response.message}"')

    def on_future_done(self, n: int, future: Future) -> None:
        self.get_logger().info(f'tick {n}: got "{future.result().message}"')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Supervisor()
    executor = MultiThreadedExecutor(num_threads=4) if node.executor_kind == 'multi' else SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
