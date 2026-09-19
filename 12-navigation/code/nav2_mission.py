"""12.09 — The same mission, driven by Nav2 through nav2_simple_commander. Runs on the robot.

This file needs ROS 2 Jazzy and a running Nav2 (12.07 in simulation, 12.08 on the real robot).
It is NOT run by the course test suite on your laptop; ``mission.py`` is the testable half.

    # terminal 1 — simulation (or the real robot: robot.launch.py sim:=false)
    ros2 launch karmel_bringup robot.launch.py sim:=true headless:=true world:=apartment x:=-1.5 y:=-0.5
    # terminal 2
    ros2 launch karmel_bringup navigation.launch.py initial_pose:=true x:=-1.5 y:=-0.5
    # terminal 3
    python3 12-navigation/code/nav2_mission.py --places places/karmel_gazebo.yaml bedroom kitchen home

``Nav2Navigator`` below implements the same ``Navigator`` protocol as ``mission.py``'s
``SimNavigator``, so ``WaypointMission`` — the retry/skip/task logic you actually care about —
is byte-for-byte the code you tested offline.

> Safety: this drives the robot autonomously. Wheels on a stand for the first run, a reachable
> power switch, nobody in the test area (SAFETY.md).
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from mission import (  # noqa: E402  (the path insert must come first)
    Feedback,
    Place,
    TaskResult,
    load_places,
    mission_from_names,
    wait_at_waypoint,
)


class Nav2Navigator:
    """``nav2_simple_commander.BasicNavigator`` behind the course's ``Navigator`` protocol.

    Everything here is one-to-one with the Nav2 API:

    ======================  =========================================
    protocol                BasicNavigator
    ======================  =========================================
    ``go_to_pose``          ``goToPose(PoseStamped)``
    ``is_task_complete``    ``isTaskComplete()``   (100 ms timeout per call)
    ``get_feedback``        ``getFeedback()``      (NavigateToPose.Feedback)
    ``get_result``          ``getResult()``        (TaskResult enum)
    ``cancel_task``         ``cancelTask()``
    ======================  =========================================
    """

    def __init__(self, behavior_tree: str = "") -> None:
        from nav2_simple_commander.robot_navigator import BasicNavigator

        self.nav = BasicNavigator()
        self.behavior_tree = behavior_tree
        self.nav.waitUntilNav2Active()        # blocks until bt_navigator and amcl are active

    def _pose_stamped(self, place: Place):
        from geometry_msgs.msg import PoseStamped

        pose = PoseStamped()
        pose.header.frame_id = place.frame_id
        pose.header.stamp = self.nav.get_clock().now().to_msg()
        pose.pose.position.x = place.x
        pose.pose.position.y = place.y
        pose.pose.orientation.z = math.sin(place.yaw / 2.0)   # yaw -> quaternion about z
        pose.pose.orientation.w = math.cos(place.yaw / 2.0)
        return pose

    def go_to_pose(self, place: Place) -> bool:
        return bool(self.nav.goToPose(self._pose_stamped(place), behavior_tree=self.behavior_tree))

    def is_task_complete(self) -> bool:
        return bool(self.nav.isTaskComplete())

    def get_feedback(self) -> Feedback | None:
        feedback = self.nav.getFeedback()
        if feedback is None:
            return None
        return Feedback(
            distance_remaining=float(feedback.distance_remaining),
            navigation_time=feedback.navigation_time.sec + feedback.navigation_time.nanosec * 1e-9,
            number_of_recoveries=int(feedback.number_of_recoveries),
        )

    def get_result(self) -> TaskResult:
        from nav2_simple_commander.robot_navigator import TaskResult as Nav2TaskResult

        return {Nav2TaskResult.SUCCEEDED: TaskResult.SUCCEEDED,
                Nav2TaskResult.CANCELED: TaskResult.CANCELED,
                Nav2TaskResult.FAILED: TaskResult.FAILED}.get(self.nav.getResult(), TaskResult.UNKNOWN)

    def cancel_task(self) -> None:
        self.nav.cancelTask()

    def set_initial_pose(self, place: Place) -> None:
        """Only needed when navigation.launch.py was started without ``initial_pose:=true``."""
        self.nav.setInitialPose(self._pose_stamped(place))


def main() -> None:
    import rclpy

    parser = argparse.ArgumentParser(description="Drive a named-place mission with Nav2.")
    parser.add_argument("stops", nargs="+", help="place names, in order")
    parser.add_argument("--places", type=Path, default=None, help="places YAML (default: the sim apartment)")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument("--attempts", type=int, default=2, help="attempts per stop")
    parser.add_argument("--wait", type=float, default=0.0, help="seconds to pause at each stop")
    parser.add_argument("--set-initial-pose", metavar="PLACE", default=None)
    parser.add_argument("--behavior-tree", default="", help="path to a BT XML for these goals (12.06)")
    args = parser.parse_args()

    places = load_places(args.places) if args.places else load_places()
    mission = mission_from_names(args.stops, places, stop_on_failure=args.stop_on_failure)
    for stop in mission.stops:
        stop.max_attempts = args.attempts
        if args.wait > 0:
            stop.task = wait_at_waypoint(args.wait)

    rclpy.init()
    try:
        navigator = Nav2Navigator(behavior_tree=args.behavior_tree)
        if args.set_initial_pose:
            navigator.set_initial_pose(places[args.set_initial_pose])
        report = mission.run(navigator)
        print("\nreport")
        for line in report.lines():
            print(line)
        sys.exit(0 if report.complete else 1)
    except KeyboardInterrupt:
        print("interrupted: cancelling the current goal")
        navigator.cancel_task()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
