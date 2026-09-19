"""Checker for 12.09 — the mission layer above Nav2.

Run: ``python course.py check 12.09`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import pytest

PLACES_YAML = """
frame_id: map
places:
  home: {x: 1.0, y: 1.3}
  kitchen: {x: 5.0, y: 2.3, yaw: 1.5708}
  bedroom: {x: 5.0, y: 3.2, yaw: 0.0}
  study: {x: 0.8, y: 3.8, yaw: 1.5708}
"""


@pytest.fixture
def places(impl, tmp_path):
    path = tmp_path / "places.yaml"
    path.write_text(PLACES_YAML, encoding="utf-8")
    return impl.load_places(path)


class FakeNavigator:
    """Scripted results, one per accepted goal, plus call counters."""

    def __init__(self, impl, results, accept=True, polls_per_goal=1):
        self.impl = impl
        self.results = list(results)
        self.accept = accept
        self.polls_per_goal = polls_per_goal
        self.goals: list[str] = []
        self.polls = 0
        self.cancels = 0
        self._left = 0
        self._result = impl.TaskResult.UNKNOWN

    def go_to_pose(self, place):
        self.goals.append(place.name)
        self._result = self.results.pop(0) if self.results else self.impl.TaskResult.SUCCEEDED
        self._left = self.polls_per_goal
        return self.accept

    def is_task_complete(self):
        self.polls += 1
        self._left -= 1
        return self._left <= 0

    def get_feedback(self):
        return self.impl.Feedback(distance_remaining=0.0, navigation_time=4.0, number_of_recoveries=1)

    def get_result(self):
        return self._result

    def cancel_task(self):
        self.cancels += 1
        self._result = self.impl.TaskResult.CANCELED


def build(impl, names, **kwargs):
    """A mission over the test places, with logging silenced."""
    import pathlib
    import tempfile

    kwargs.setdefault("log", lambda _: None)
    with tempfile.TemporaryDirectory() as directory:
        path = pathlib.Path(directory) / "places.yaml"
        path.write_text(PLACES_YAML, encoding="utf-8")
        loaded = impl.load_places(path)
    return impl.mission_from_names(names, loaded, **kwargs)


# --- load_places ----------------------------------------------------------------------------------
def test_places_are_loaded_with_names_and_frame(places):
    assert set(places) == {"home", "kitchen", "bedroom", "study"}
    kitchen = places["kitchen"]
    assert (kitchen.name, kitchen.xy, kitchen.frame_id) == ("kitchen", (5.0, 2.3), "map")
    assert kitchen.yaw == pytest.approx(math.pi / 2, abs=1e-4)


def test_yaw_defaults_to_zero(places):
    assert places["home"].yaw == 0.0


def test_frame_id_defaults_to_map(impl, tmp_path):
    path = tmp_path / "p.yaml"
    path.write_text("places:\n  dock: {x: 0.0, y: 0.0}\n", encoding="utf-8")
    assert impl.load_places(path)["dock"].frame_id == "map"


def test_a_non_default_frame_is_honoured(impl, tmp_path):
    path = tmp_path / "p.yaml"
    path.write_text("frame_id: odom\nplaces:\n  dock: {x: 0.0, y: 0.0}\n", encoding="utf-8")
    assert impl.load_places(path)["dock"].frame_id == "odom"


# --- mission_from_names ---------------------------------------------------------------------------
def test_mission_from_names_keeps_the_order(impl, places):
    mission = impl.mission_from_names(["study", "kitchen"], places)
    assert [s.place.name for s in mission.stops] == ["study", "kitchen"]
    assert all(s.state is impl.StopState.PENDING for s in mission.stops)


def test_an_unknown_place_raises_with_a_useful_message(impl, places):
    with pytest.raises(KeyError) as excinfo:
        impl.mission_from_names(["pantry"], places)
    message = str(excinfo.value)
    assert "pantry" in message and "kitchen" in message


def test_kwargs_reach_the_mission(impl, places):
    assert impl.mission_from_names(["kitchen"], places, stop_on_failure=True).stop_on_failure


# --- the state machine ----------------------------------------------------------------------------
def test_a_clean_mission_visits_every_stop_once(impl):
    S = impl.TaskResult.SUCCEEDED
    nav = FakeNavigator(impl, [S, S, S])
    report = build(impl, ["kitchen", "bedroom", "study"]).run(nav)
    assert report.complete and report.succeeded == 3
    assert nav.goals == ["kitchen", "bedroom", "study"]
    assert [s.attempts for s in report.stops] == [1, 1, 1]
    assert report.total_time_s == pytest.approx(12.0)


def test_feedback_is_read_after_the_poll_loop_and_accumulated(impl):
    S, F = impl.TaskResult.SUCCEEDED, impl.TaskResult.FAILED
    nav = FakeNavigator(impl, [F, S], polls_per_goal=3)
    report = build(impl, ["kitchen"]).run(nav)
    assert report.complete
    assert report.stops[0].nav_time_s == pytest.approx(8.0), "4 s per attempt, both counted"
    assert report.stops[0].recoveries == 2
    assert nav.polls == 6, "three polls per goal, two goals"


def test_a_failed_stop_is_retried(impl):
    S, F = impl.TaskResult.SUCCEEDED, impl.TaskResult.FAILED
    nav = FakeNavigator(impl, [F, S, S])
    report = build(impl, ["kitchen", "bedroom"]).run(nav)
    assert nav.goals == ["kitchen", "kitchen", "bedroom"]
    assert report.complete and report.stops[0].attempts == 2


def test_a_stop_that_always_fails_is_skipped_and_the_mission_goes_on(impl):
    S, F = impl.TaskResult.SUCCEEDED, impl.TaskResult.FAILED
    nav = FakeNavigator(impl, [F, F, S])
    report = build(impl, ["kitchen", "bedroom"]).run(nav)
    assert [s.state.value for s in report.stops] == ["failed", "succeeded"]
    assert not report.complete and report.succeeded == 1


def test_stop_on_failure_aborts_and_marks_the_rest_skipped(impl):
    S, F = impl.TaskResult.SUCCEEDED, impl.TaskResult.FAILED
    nav = FakeNavigator(impl, [F, F, S])
    report = build(impl, ["kitchen", "bedroom", "study"], stop_on_failure=True).run(nav)
    assert [s.state.value for s in report.stops] == ["failed", "skipped", "skipped"]
    assert nav.goals == ["kitchen", "kitchen"]


def test_a_cancelled_goal_ends_the_mission_without_retrying(impl):
    C, S = impl.TaskResult.CANCELED, impl.TaskResult.SUCCEEDED
    nav = FakeNavigator(impl, [C, S])
    report = build(impl, ["kitchen", "bedroom"]).run(nav)
    assert [s.state.value for s in report.stops] == ["canceled", "skipped"]
    assert nav.goals == ["kitchen"]


def test_max_attempts_is_per_stop(impl):
    S, F = impl.TaskResult.SUCCEEDED, impl.TaskResult.FAILED
    nav = FakeNavigator(impl, [F, F, S, S])
    mission = build(impl, ["kitchen", "bedroom"])
    mission.stops[0].max_attempts = 3
    report = mission.run(nav)
    assert report.stops[0].attempts == 3 and report.complete


def test_zero_attempts_means_the_stop_fails_without_a_goal(impl):
    nav = FakeNavigator(impl, [impl.TaskResult.SUCCEEDED])
    mission = build(impl, ["kitchen"])
    mission.stops[0].max_attempts = 0
    report = mission.run(nav)
    assert nav.goals == [] and report.stops[0].state is impl.StopState.FAILED


def test_the_arrival_task_runs_only_on_success(impl):
    S, F = impl.TaskResult.SUCCEEDED, impl.TaskResult.FAILED
    visited: list[str] = []
    mission = build(impl, ["kitchen", "bedroom"])
    for stop in mission.stops:
        stop.task = lambda place: visited.append(place.name)
        stop.max_attempts = 1
    mission.run(FakeNavigator(impl, [F, S]))
    assert visited == ["bedroom"]


def test_a_rejected_goal_does_not_poll_and_still_costs_an_attempt(impl):
    nav = FakeNavigator(impl, [impl.TaskResult.SUCCEEDED] * 3, accept=False)
    report = build(impl, ["kitchen"]).run(nav)
    assert nav.goals == ["kitchen", "kitchen"] and nav.polls == 0
    assert report.stops[0].state is impl.StopState.FAILED


def test_the_poll_limit_cancels_a_task_that_never_finishes(impl):
    class NeverFinishes(FakeNavigator):
        def is_task_complete(self):
            self.polls += 1
            return self.cancels > 0

    nav = NeverFinishes(impl, [impl.TaskResult.SUCCEEDED])
    mission = build(impl, ["kitchen"])
    mission.stops[0].max_attempts = 1
    report = mission.run(nav, max_iterations=50)
    assert nav.cancels == 1 and nav.polls >= 50
    assert report.stops[0].state is impl.StopState.CANCELED


def test_the_report_totals_are_right_for_a_mixed_mission(impl):
    S, F = impl.TaskResult.SUCCEEDED, impl.TaskResult.FAILED
    nav = FakeNavigator(impl, [S, F, F, S])
    report = build(impl, ["kitchen", "bedroom", "study"]).run(nav)
    assert report.succeeded == 2 and not report.complete
    assert report.total_time_s == pytest.approx(16.0), "1 + 2 + 1 attempts x 4 s"
