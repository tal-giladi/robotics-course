"""Tests for 12.09 — named places, the waypoint state machine, and the simulated navigator."""

from __future__ import annotations

import math

import mission as ms
import pytest


# --- a fake navigator, so the state machine is tested without any driving --------------------------
class FakeNavigator:
    """Returns scripted results, one per goal, and counts the calls."""

    def __init__(self, results: list[ms.TaskResult], accept: bool = True) -> None:
        self.results = list(results)
        self.accept = accept
        self.goals: list[str] = []
        self.polls = 0
        self.cancels = 0
        self._result = ms.TaskResult.UNKNOWN

    def go_to_pose(self, place: ms.Place) -> bool:
        self.goals.append(place.name)
        self._result = self.results.pop(0) if self.results else ms.TaskResult.SUCCEEDED
        return self.accept

    def is_task_complete(self) -> bool:
        self.polls += 1
        return True

    def get_feedback(self) -> ms.Feedback:
        return ms.Feedback(distance_remaining=0.0, navigation_time=4.0, number_of_recoveries=1)

    def get_result(self) -> ms.TaskResult:
        return self._result

    def cancel_task(self) -> None:
        self.cancels += 1
        self._result = ms.TaskResult.CANCELED


S, F, C = ms.TaskResult.SUCCEEDED, ms.TaskResult.FAILED, ms.TaskResult.CANCELED


def mission(names, **kwargs):
    places = ms.load_places()
    return ms.mission_from_names(names, places, log=lambda _: None, **kwargs)


# --- named places -----------------------------------------------------------------------------
def test_places_load_with_names_a_human_would_use():
    places = ms.load_places()
    assert {"home", "kitchen", "bedroom", "study"} <= set(places)
    assert places["kitchen"].xy == (5.0, 2.3)
    assert places["kitchen"].frame_id == "map"
    assert places["kitchen"].yaw == pytest.approx(math.pi / 2, abs=1e-4)


def test_every_named_place_is_inside_the_apartment_and_not_in_a_wall():
    from robotlab.config import load_config
    from robotlab.sim import World

    world, radius = World.apartment(), load_config().chassis.footprint_radius_m
    for place in ms.load_places().values():
        assert not world.collides(place.x, place.y, radius), f"{place.name} is inside something"


def test_an_unknown_place_fails_loudly():
    with pytest.raises(KeyError, match="pantry"):
        ms.mission_from_names(["pantry"], ms.load_places())


# --- the state machine ------------------------------------------------------------------------
def test_a_clean_mission_visits_every_stop_once():
    nav = FakeNavigator([S, S, S])
    report = mission(["kitchen", "bedroom", "study"]).run(nav)
    assert report.complete and report.succeeded == 3
    assert nav.goals == ["kitchen", "bedroom", "study"]
    assert [s.attempts for s in report.stops] == [1, 1, 1]
    assert report.total_time_s == pytest.approx(12.0)


def test_a_failed_stop_is_retried_then_skipped():
    nav = FakeNavigator([F, S, S])
    report = mission(["kitchen", "bedroom"]).run(nav)
    assert nav.goals == ["kitchen", "kitchen", "bedroom"]
    assert report.complete
    assert report.stops[0].attempts == 2
    assert report.stops[0].recoveries == 2, "feedback from both attempts is accumulated"


def test_a_stop_that_fails_every_attempt_is_marked_failed_and_the_mission_continues():
    nav = FakeNavigator([F, F, S])
    report = mission(["kitchen", "bedroom"]).run(nav)
    assert report.stops[0].state is ms.StopState.FAILED
    assert report.stops[1].state is ms.StopState.SUCCEEDED
    assert not report.complete and report.succeeded == 1


def test_stop_on_failure_aborts_and_skips_the_rest():
    nav = FakeNavigator([F, F, S])
    report = mission(["kitchen", "bedroom", "study"], stop_on_failure=True).run(nav)
    assert [s.state.value for s in report.stops] == ["failed", "skipped", "skipped"]
    assert nav.goals == ["kitchen", "kitchen"], "the later stops were never attempted"


def test_a_cancelled_goal_ends_the_mission_without_retrying():
    nav = FakeNavigator([C, S])
    report = mission(["kitchen", "bedroom"]).run(nav)
    assert report.stops[0].state is ms.StopState.CANCELED
    assert report.stops[1].state is ms.StopState.SKIPPED
    assert nav.goals == ["kitchen"]


def test_max_attempts_is_per_stop():
    nav = FakeNavigator([F, F, S, S])
    m = mission(["kitchen", "bedroom"])
    m.stops[0].max_attempts = 3
    report = m.run(nav)
    assert report.stops[0].attempts == 3 and report.complete


def test_the_arrival_task_runs_only_on_success():
    visited: list[str] = []
    m = mission(["kitchen", "bedroom"])
    for stop in m.stops:
        stop.task = lambda place: visited.append(place.name)
        stop.max_attempts = 1
    m.run(FakeNavigator([F, S]))
    assert visited == ["bedroom"]


def test_a_rejected_goal_costs_an_attempt_but_is_not_a_failure_report():
    nav = FakeNavigator([S, S], accept=False)
    report = mission(["kitchen"]).run(nav)
    assert report.stops[0].state is ms.StopState.FAILED
    assert nav.goals == ["kitchen", "kitchen"], "both attempts were spent on a rejected goal"


def test_the_poll_limit_cancels_a_task_that_never_finishes():
    class NeverFinishes(FakeNavigator):
        def is_task_complete(self) -> bool:
            self.polls += 1
            return self.cancels > 0

    nav = NeverFinishes([S])
    m = mission(["kitchen"])
    m.stops[0].max_attempts = 1
    report = m.run(nav, max_iterations=50)
    assert nav.cancels == 1 and nav.polls >= 50
    assert report.stops[0].state is ms.StopState.CANCELED


# --- the simulated navigator ---------------------------------------------------------------------
@pytest.fixture(scope="module")
def sim_navigator():
    return ms.SimNavigator()


def test_the_sim_navigator_plans_a_path_from_the_living_room_to_the_kitchen(sim_navigator):
    path = sim_navigator.plan((1.0, 1.3), (5.0, 2.3))
    assert path is not None and len(path) > 50
    assert path[-1] == pytest.approx((5.0, 2.3), abs=0.1)


def test_planning_into_a_wall_returns_no_path(sim_navigator):
    assert sim_navigator.plan((1.0, 1.3), (0.05, 0.05)) is None


def test_a_two_stop_mission_actually_drives_there():
    navigator = ms.SimNavigator()
    report = mission(["kitchen", "bedroom"]).run(navigator)
    assert report.complete
    assert navigator.pose[0] == pytest.approx(5.0, abs=0.2)
    assert navigator.pose[1] == pytest.approx(3.2, abs=0.2)
    assert len(navigator.traces) == 2
    assert 5.0 < report.total_time_s < 120.0


def test_a_blocked_place_fails_both_attempts_and_the_mission_skips_it():
    navigator = ms.SimNavigator(blocked=("bedroom",))
    report = mission(["kitchen", "bedroom", "study"]).run(navigator)
    assert [s.state.value for s in report.stops] == ["succeeded", "failed", "succeeded"]
    assert report.stops[1].attempts == 2


def test_a_place_that_fails_once_is_reached_on_the_retry():
    navigator = ms.SimNavigator(fail_at=("kitchen",))
    report = mission(["kitchen"]).run(navigator)
    assert report.complete and report.stops[0].attempts == 2


def test_cancelling_mid_leg_stops_the_robot_where_it_is():
    navigator = ms.SimNavigator()
    navigator.go_to_pose(ms.load_places()["kitchen"])
    for _ in range(20):                      # 2 s of simulated driving
        navigator.is_task_complete()
    part_way = navigator.pose
    navigator.cancel_task()
    assert navigator.get_result() is ms.TaskResult.CANCELED
    assert navigator.is_task_complete()
    assert part_way[0] > 1.0, "the robot did move before the cancel"
    assert part_way[0] < 5.0, "but nowhere near the kitchen yet"
