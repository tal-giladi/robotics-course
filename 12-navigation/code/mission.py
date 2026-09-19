"""12.09 — A mission layer: named places, a waypoint state machine, retries, and a sim run.

    python 12-navigation/code/mission.py                 # drive a 3-stop mission in the simulator
    python 12-navigation/code/mission.py --fail kitchen  # what a failing waypoint does
    python 12-navigation/code/mission.py --stop-on-failure

Nav2's ``nav2_simple_commander`` gives you one primitive: *send a goal, poll until it finishes,
read the result*. Everything above that — what "the kitchen" means, what to do when a goal fails,
what to do at each stop, how to cancel — is your code. This module is that layer, written against
a ``Navigator`` protocol so exactly the same ``WaypointMission`` runs against

* ``SimNavigator`` here (robotlab's simulator + the pure pursuit of 12.05), and
* ``nav2_simple_commander.robot_navigator.BasicNavigator`` on the robot, in ``nav2_mission.py``.

The protocol is deliberately BasicNavigator's: ``go_to_pose``, ``is_task_complete``,
``get_feedback``, ``get_result``, ``cancel_task``.
"""

from __future__ import annotations

import argparse
import math
from collections.abc import Callable, Iterable
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Protocol

import numpy as np
import yaml

import nav_common

PLACES_FILE = Path(__file__).resolve().parent / "places" / "karmel_apartment.yaml"


# --- named places -----------------------------------------------------------------------------
@dataclass(frozen=True)
class Place:
    """A goal pose with a name a human uses: "the kitchen", not "(5.0, 2.3, 0 rad)"."""

    name: str
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"

    @property
    def xy(self) -> tuple[float, float]:
        return (self.x, self.y)


def load_places(path: Path | str = PLACES_FILE) -> dict[str, Place]:
    """Load ``places: {name: {x, y, yaw}}``. One file, edited by a human, is the whole "semantic map"."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    frame = data.get("frame_id", "map")
    return {name: Place(name, float(p["x"]), float(p["y"]), float(p.get("yaw", 0.0)), frame)
            for name, p in data["places"].items()}


# --- the navigator contract -------------------------------------------------------------------
class TaskResult(Enum):
    """Mirrors ``nav2_simple_commander.robot_navigator.TaskResult``."""

    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


@dataclass
class Feedback:
    """The parts of ``NavigateToPose.Feedback`` a mission layer actually uses."""

    distance_remaining: float = 0.0
    navigation_time: float = 0.0
    number_of_recoveries: int = 0


class Navigator(Protocol):
    """What a mission needs from a navigation stack. BasicNavigator satisfies this."""

    def go_to_pose(self, place: Place) -> bool: ...
    def is_task_complete(self) -> bool: ...
    def get_feedback(self) -> Feedback | None: ...
    def get_result(self) -> TaskResult: ...
    def cancel_task(self) -> None: ...


# --- the mission state machine ------------------------------------------------------------------
class StopState(str, Enum):
    PENDING = "pending"
    ACTIVE = "active"
    SUCCEEDED = "succeeded"
    FAILED = "failed"
    SKIPPED = "skipped"
    CANCELED = "canceled"


@dataclass
class Stop:
    """One waypoint of a mission, plus what to do when the robot gets there."""

    place: Place
    task: Callable[[Place], None] | None = None     # "take a photo", "announce", "wait"
    max_attempts: int = 2
    state: StopState = StopState.PENDING
    attempts: int = 0
    nav_time_s: float = 0.0
    recoveries: int = 0

    @property
    def finished(self) -> bool:
        return self.state in (StopState.SUCCEEDED, StopState.FAILED,
                              StopState.SKIPPED, StopState.CANCELED)


@dataclass
class MissionReport:
    stops: list[Stop]
    canceled: bool = False

    @property
    def succeeded(self) -> int:
        return sum(s.state is StopState.SUCCEEDED for s in self.stops)

    @property
    def complete(self) -> bool:
        return all(s.state is StopState.SUCCEEDED for s in self.stops)

    @property
    def total_time_s(self) -> float:
        return sum(s.nav_time_s for s in self.stops)

    def lines(self) -> list[str]:
        out = [f"  {s.place.name:<14} {s.state.value:<10} attempts {s.attempts}  "
               f"{s.nav_time_s:5.1f} s  recoveries {s.recoveries}" for s in self.stops]
        out.append(f"  -> {self.succeeded}/{len(self.stops)} stops, {self.total_time_s:.1f} s"
                   + (", CANCELED" if self.canceled else ""))
        return out


@dataclass
class WaypointMission:
    """Visit ``stops`` in order. Retry a failed stop, then either skip it or abort the mission.

    This is the piece Nav2 does **not** give you. ``waypoint_follower`` has one policy
    (``stop_on_failure``) and no retries; ``goThroughPoses`` plans one path through everything and
    fails as a unit. Anything smarter is a state machine you own, and it belongs in your code
    because the right answer is domain-specific: a delivery robot skips a blocked room and reports
    it, a security patrol retries it, a vacuum gives up on the room and finishes the floor.
    """

    stops: list[Stop]
    stop_on_failure: bool = False
    log: Callable[[str], None] = print

    def run(self, navigator: Navigator, max_iterations: int = 100_000) -> MissionReport:
        report = MissionReport(self.stops)
        for stop in self.stops:
            if not self._run_stop(stop, navigator, max_iterations):
                for rest in self.stops:
                    if rest.state is StopState.PENDING:
                        rest.state = StopState.SKIPPED
                report.canceled = stop.state is StopState.CANCELED
                self.log(f"mission {'canceled' if report.canceled else 'aborted'} at {stop.place.name}")
                break
        return report

    def _run_stop(self, stop: Stop, navigator: Navigator, max_iterations: int) -> bool:
        """True if the mission may continue."""
        while stop.attempts < stop.max_attempts:
            stop.attempts += 1
            stop.state = StopState.ACTIVE
            self.log(f"-> {stop.place.name} ({stop.place.x:.2f}, {stop.place.y:.2f}) "
                     f"attempt {stop.attempts}/{stop.max_attempts}")
            if not navigator.go_to_pose(stop.place):
                self.log(f"   goal rejected for {stop.place.name}")
                continue
            polls = 0
            while not navigator.is_task_complete():          # exactly BasicNavigator's poll loop
                polls += 1
                if polls > max_iterations:
                    navigator.cancel_task()
                    break
            feedback = navigator.get_feedback()
            if feedback is not None:
                stop.nav_time_s += feedback.navigation_time
                stop.recoveries += feedback.number_of_recoveries
            result = navigator.get_result()
            if result is TaskResult.SUCCEEDED:
                stop.state = StopState.SUCCEEDED
                self.log(f"   reached {stop.place.name} in {stop.nav_time_s:.1f} s")
                if stop.task is not None:
                    stop.task(stop.place)
                return True
            if result is TaskResult.CANCELED:
                stop.state = StopState.CANCELED
                self.log(f"   {stop.place.name} canceled")
                return False
            self.log(f"   {stop.place.name} failed ({result.name})")
        stop.state = StopState.FAILED
        return not self.stop_on_failure


def mission_from_names(names: Iterable[str], places: dict[str, Place], **kwargs: Any) -> WaypointMission:
    """``mission_from_names(["kitchen", "bedroom"], load_places())``; unknown names fail loudly."""
    stops = []
    for name in names:
        if name not in places:
            raise KeyError(f"unknown place {name!r}; known: {sorted(places)}")
        stops.append(Stop(places[name]))
    return WaypointMission(stops, **kwargs)


# --- a navigator backed by the course simulator ---------------------------------------------------
@dataclass
class SimNavigator:
    """Plan with A* on the apartment map, follow with pure pursuit — the 12.02 + 12.05 stack.

    ``fail_at`` makes named goals fail on their **first** attempt (a person in the doorway who
    moves away); ``blocked`` makes them fail every time (a closed door). That is how the exercises
    drive the retry and skip logic without needing real obstacles.
    """

    realistic: bool = False
    seed: int = 0
    fail_at: tuple[str, ...] = ()
    blocked: tuple[str, ...] = ()
    speed_m_s: float = 0.25
    goal_tolerance_m: float = 0.15

    def __post_init__(self) -> None:
        import costmap as cm
        import grid_planning as gp
        from robotlab.config import load_config
        from robotlab.sim import World

        self._cm, self._gp = cm, gp
        self.cfg = load_config()
        self.world = World.apartment()
        self.grid = self.world.to_occupancy_grid(nav_common.GRID_RESOLUTION)
        self.costs = cm.inflate(cm.static_layer(self.grid), self.grid.resolution,
                                self.cfg.chassis.footprint_radius_m, 3.0, 0.8)
        self.pose: tuple[float, float, float] = (*nav_common.START_XY, 0.0)
        self.traces: list[np.ndarray] = []
        self._attempts: dict[str, int] = {}
        self._result = TaskResult.UNKNOWN
        self._feedback = Feedback()
        self._done = True

    # --- the Navigator protocol ------------------------------------------------------------------
    def go_to_pose(self, place: Place) -> bool:
        """Accept the goal and set the leg up. The driving happens in ``is_task_complete``."""
        import local_planning as lp
        from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase

        self._attempts[place.name] = self._attempts.get(place.name, 0) + 1
        self._place = place
        self._collided = False
        self._trace = [self.pose]
        self._done = False
        self._result = TaskResult.UNKNOWN

        if place.name in self.blocked or (place.name in self.fail_at and self._attempts[place.name] == 1):
            self._finish(TaskResult.FAILED, Feedback(0.0, 1.0, 2))     # recoveries ran, then abort
            return True
        path = self.plan(self.pose[:2], place.xy)
        if path is None:
            self._finish(TaskResult.FAILED, Feedback(0.0, 0.5, 0))     # NO_VALID_PATH
            return True

        self._controller = lp.PurePursuit(path=path, desired_linear_vel=self.speed_m_s,
                                          xy_goal_tolerance=self.goal_tolerance_m, regulated=True)
        params = DiffDriveParams.realistic(self.cfg) if self.realistic else DiffDriveParams.ideal(self.cfg)
        sensors = SensorParams.realistic(self.cfg) if self.realistic else SensorParams.ideal(self.cfg)
        self._base = SimBase(DiffDriveSim(self.world, params, sensors, pose=self.pose, seed=self.seed), dt=0.02)
        self._deadline_s = 20.0 + 8.0 * float(lp.path_distances(path)[-1])
        self._feedback = Feedback(distance_remaining=float(lp.path_distances(path)[-1]))
        return True

    def is_task_complete(self) -> bool:
        """Poll: advance the simulation by 0.1 s (five 50 Hz control cycles) and report."""
        import local_planning as lp

        if self._done:
            return True
        base, controller = self._base, self._controller
        for _ in range(5):
            v, w = controller.compute(tuple(base.true_pose))
            base.set_wheel_velocity(*lp.twist_to_wheels(v, w, self.cfg.drive.wheel_radius_m,
                                                        self.cfg.drive.wheel_separation_m))
            base.read()
            self._collided |= base.sim.collided
            self._trace.append(tuple(base.true_pose))
            self.pose = tuple(base.true_pose)
            if controller.done and max(abs(s) for s in base.sim.wheel_rad_s) < 0.05:
                self._finish(TaskResult.FAILED if self._collided else TaskResult.SUCCEEDED)
                return True
            if base.sim.t > self._deadline_s:
                self._finish(TaskResult.FAILED)                        # the progress checker's job
                return True
        self._feedback = Feedback(
            distance_remaining=float(np.linalg.norm(np.array(self.pose[:2]) - self._place.xy)),
            navigation_time=base.sim.t)
        return False

    def get_feedback(self) -> Feedback | None:
        return self._feedback

    def get_result(self) -> TaskResult:
        return self._result

    def cancel_task(self) -> None:
        self._finish(TaskResult.CANCELED)

    def _finish(self, result: TaskResult, feedback: Feedback | None = None) -> None:
        if len(self._trace) > 1:
            self.traces.append(np.array(self._trace)[:, :2])
        self._feedback = feedback if feedback is not None else Feedback(
            distance_remaining=float(np.linalg.norm(np.array(self.pose[:2]) - self._place.xy)),
            navigation_time=getattr(self, "_base", None).sim.t if hasattr(self, "_base") else 0.0)
        self._result = result
        self._done = True

    # --- the plan/drive machinery ------------------------------------------------------------------
    def plan(self, start_xy: tuple[float, float], goal_xy: tuple[float, float]) -> np.ndarray | None:
        """Cost-aware A* on the inflated static map, shortcut and densified to 5 cm."""
        blocked, extra = self._cm.cost_to_planner_penalty(self.costs)
        result = self._gp.astar(blocked, self.grid.world_to_cell(*start_xy),
                                self.grid.world_to_cell(*goal_xy), cell_cost=extra)
        if not result.path:
            return None
        smooth = self._gp.shortcut(self.costs >= 120, result.path)
        return self._gp.densify(self._gp.cells_to_world(self.grid, smooth), 0.05)


# --- demo -----------------------------------------------------------------------------------------
def wait_at_waypoint(seconds: float) -> Callable[[Place], None]:
    """Nav2's ``WaitAtWaypoint`` task executor, as a plain Python callable."""

    def task(place: Place) -> None:
        print(f"   waiting {seconds:.0f} s at {place.name}")

    return task


def plot(navigator: SimNavigator, report: MissionReport, name: str) -> Path:
    from robotlab.sim import viz

    fig, ax = viz.new_axes(navigator.world, title=f"12.09 mission: {' -> '.join(s.place.name for s in report.stops)}")
    viz.draw_world(ax, navigator.world, show_landmarks=False)
    for trace in navigator.traces:
        ax.plot(trace[:, 0], trace[:, 1], lw=1.6, color="tab:blue")
    for stop in report.stops:
        color = {"succeeded": "tab:green", "failed": "tab:red"}.get(stop.state.value, "tab:orange")
        ax.plot(*stop.place.xy, "*", ms=16, color=color)
        ax.annotate(stop.place.name, stop.place.xy, textcoords="offset points", xytext=(8, 6))
    ax.plot(*nav_common.START_XY, "o", ms=8, color="k")
    return nav_common.save(fig, name)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a waypoint mission in the course simulator.")
    parser.add_argument("--stops", nargs="*", default=["kitchen", "bedroom", "study"])
    parser.add_argument("--fail", nargs="*", default=[], help="place names whose first attempt fails")
    parser.add_argument("--block", nargs="*", default=[], help="place names that always fail (closed door)")
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument("--realistic", action="store_true", help="noisy motors and encoders")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    places = load_places()
    print("named places (" + str(PLACES_FILE.relative_to(nav_common.ROOT)) + ")")
    for place in places.values():
        print(f"  {place.name:<14} ({place.x:.2f}, {place.y:.2f}, {math.degrees(place.yaw):+.0f} deg)")

    mission = mission_from_names(args.stops, places, stop_on_failure=args.stop_on_failure)
    for stop in mission.stops:
        stop.task = wait_at_waypoint(2.0)
    navigator = SimNavigator(fail_at=tuple(args.fail), blocked=tuple(args.block),
                             realistic=args.realistic)

    print(f"\nmission: {' -> '.join(args.stops)}"
          f"{'  (stop_on_failure)' if args.stop_on_failure else '  (skip failed stops)'}")
    report = mission.run(navigator)
    print("\nreport")
    for line in report.lines():
        print(line)
    if not args.no_plot:
        plot(navigator, report, "12.09_mission")


if __name__ == "__main__":
    main()
