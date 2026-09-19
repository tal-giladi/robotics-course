"""
drive_distance_server: a simulated "drive N metres" action with feedback and cancel (04.08).

Serves
  drive_distance     karmel_tutorial_interfaces/action/DriveDistance
Subscribes
  battery/health     karmel_tutorial_interfaces/BatteryHealth   (abort when CRITICAL)

The motion is simulated: distance += speed * dt every 0.1 s. In lesson 04.16 the same
structure publishes cmd_vel to the real robot and measures distance from wheel odometry.
"""
import threading
import time

from karmel_tutorial_interfaces.action import DriveDistance
from karmel_tutorial_interfaces.msg import BatteryHealth
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

MAX_SPEED_M_S = 0.5     # labs/config/karmel.yaml drive.max_linear_speed_m_s
MAX_DISTANCE_M = 5.0    # refuse silly goals indoors
CONTROL_RATE_HZ = 10.0


class DriveDistanceServer(Node):
    """One goal at a time; reject bad goals up front; honor cancel; abort on dead battery."""

    def __init__(self) -> None:
        super().__init__('drive_distance_server')
        # A reentrant group + MultiThreadedExecutor lets cancel requests and battery
        # messages be processed WHILE a goal executes (details in lesson 04.12).
        group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False
        self._battery_state = None
        self._health_sub = self.create_subscription(
            BatteryHealth, 'battery/health', self._on_health, 10, callback_group=group)
        self._server = ActionServer(
            self, DriveDistance, 'drive_distance',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=group)
        self.get_logger().info('drive_distance action server ready')

    def _on_health(self, msg: BatteryHealth) -> None:
        self._battery_state = msg.state

    def _battery_critical(self) -> bool:
        return self._battery_state == BatteryHealth.STATE_CRITICAL

    def _on_goal(self, goal: DriveDistance.Goal) -> GoalResponse:
        """Decide quickly whether to accept. Do not start work here."""
        reason = None
        if abs(goal.distance_m) > MAX_DISTANCE_M:
            reason = f'|distance| {abs(goal.distance_m):.2f} m > {MAX_DISTANCE_M:.1f} m'
        elif not 0.0 < goal.speed_m_s <= MAX_SPEED_M_S:
            reason = f'speed {goal.speed_m_s:.2f} m/s not in (0, {MAX_SPEED_M_S}]'
        elif self._battery_critical():
            reason = 'battery CRITICAL'
        with self._lock:
            if reason is None and self._busy:
                reason = 'another goal is executing'
            if reason is None:
                self._busy = True
        if reason is not None:
            self.get_logger().warn(f'Rejecting goal: {reason}')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'Accepting goal: {goal.distance_m:+.2f} m at {goal.speed_m_s:.2f} m/s')
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle: ServerGoalHandle) -> CancelResponse:
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle: ServerGoalHandle) -> DriveDistance.Result:
        goal = goal_handle.request
        target = abs(goal.distance_m)
        dt = 1.0 / CONTROL_RATE_HZ
        travelled = 0.0
        start = time.monotonic()
        feedback = DriveDistance.Feedback()
        result = DriveDistance.Result()
        try:
            while travelled < target:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.message = f'canceled after {travelled:.2f} m'
                    self.get_logger().info(result.message)
                    break
                if self._battery_critical():
                    goal_handle.abort()
                    result.message = f'aborted after {travelled:.2f} m: battery CRITICAL'
                    self.get_logger().error(result.message)
                    break
                time.sleep(dt)  # stands in for "send cmd_vel, wait for the next odometry"
                travelled = min(target, travelled + goal.speed_m_s * dt)
                feedback.distance_travelled_m = travelled
                feedback.distance_remaining_m = target - travelled
                feedback.progress = travelled / target if target > 0.0 else 1.0
                goal_handle.publish_feedback(feedback)
            else:
                goal_handle.succeed()
                result.message = f'arrived: {travelled:.2f} m'
                self.get_logger().info(result.message)
        finally:
            with self._lock:
                self._busy = False
        result.distance_travelled_m = travelled
        result.elapsed_s = time.monotonic() - start
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriveDistanceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
