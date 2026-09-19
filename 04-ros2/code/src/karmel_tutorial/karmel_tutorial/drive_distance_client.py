"""
drive_distance_client: send a DriveDistance goal, print feedback, optionally cancel (04.08).

Usage:
  ros2 run karmel_tutorial drive_distance_client 1.0 0.25          # drive 1 m at 0.25 m/s
  ros2 run karmel_tutorial drive_distance_client 1.0 0.25 1.5      # cancel after 1.5 s
"""
import sys

from action_msgs.msg import GoalStatus
from karmel_tutorial_interfaces.action import DriveDistance
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


def main(args=None) -> int:
    rclpy.init(args=args)
    argv = remove_ros_args(sys.argv)
    if len(argv) not in (3, 4):
        print('usage: drive_distance_client <distance_m> <speed_m_s> [cancel_after_s]')
        rclpy.try_shutdown()
        return 2
    cancel_after_s = float(argv[3]) if len(argv) == 4 else None

    node = Node('drive_distance_client')
    try:
        client = ActionClient(node, DriveDistance, 'drive_distance')
        if not client.wait_for_server(timeout_sec=5.0):
            node.get_logger().error('drive_distance server not available after 5 s')
            return 1

        goal = DriveDistance.Goal()
        goal.distance_m = float(argv[1])
        goal.speed_m_s = float(argv[2])

        def on_feedback(msg) -> None:
            fb = msg.feedback
            print(f'feedback: {fb.distance_travelled_m:.2f} m travelled, '
                  f'{fb.distance_remaining_m:.2f} m remaining ({100 * fb.progress:.0f} %)')

        # Step 1: send the goal and wait for accept/reject.
        send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(node, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            print('goal REJECTED')
            return 3
        print('goal accepted')

        # Step 2: wait for the result, optionally cancelling part-way.
        result_future = goal_handle.get_result_async()
        if cancel_after_s is not None:
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=cancel_after_s)
            if not result_future.done():
                print(f'requesting cancel after {cancel_after_s:.1f} s')
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel_future)
        rclpy.spin_until_future_complete(node, result_future)

        wrapped = result_future.result()
        result = wrapped.result
        status = STATUS_NAMES.get(wrapped.status, str(wrapped.status))
        print(f'result: {status}, {result.distance_travelled_m:.2f} m in '
              f'{result.elapsed_s:.1f} s, "{result.message}"')
        return 0 if wrapped.status == GoalStatus.STATUS_SUCCEEDED else 4
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
