"""Checker for 20.05 — scenario verdicts and the regression report.

Run: ``python course.py check 20.05`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import functools
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "20-final-robot" / "code",):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import scenario_suite as ss  # noqa: E402

REACH = {"outcome": "reach", "max_time_s": 60.0, "min_clearance_m": 0.04,
         "max_interventions": 4, "max_blind_distance_m": 0.05}
STOP = {**REACH, "outcome": "stop-safely", "max_time_s": 40.0}
GOOD = {"reached": True, "collided": False, "duration_s": 13.1, "min_clearance_m": 0.118,
        "interventions": 0, "blind_distance_m": 0.0}
STOPPED = {"reached": False, "collided": False, "duration_s": 10.7, "min_clearance_m": 0.302,
           "interventions": 1, "blind_distance_m": 0.0}


def as_run(metrics: ss.Metrics) -> dict:
    return {"reached": metrics.reached, "collided": metrics.collided,
            "duration_s": metrics.duration_s, "min_clearance_m": metrics.min_clearance_m,
            "interventions": metrics.interventions, "blind_distance_m": metrics.blind_distance_m}


def as_thresholds(scenario: ss.Scenario) -> dict:
    return {"outcome": scenario.outcome, "max_time_s": scenario.max_time_s,
            "min_clearance_m": scenario.min_clearance_m,
            "max_interventions": scenario.max_interventions,
            "max_blind_distance_m": scenario.max_blind_distance_m}


@functools.lru_cache(maxsize=None)
def suite_metrics(build: str) -> tuple[ss.Metrics, ...]:
    """One simulated suite run per build, cached: the simulator is cheap but not free."""
    return tuple(ss.run_suite(ss.builds()[build]).runs)


def suite_runs(build: str) -> dict[str, dict]:
    return {m.scenario: as_run(m) for m in suite_metrics(build)}


# --- verdict ----------------------------------------------------------------------------------
def test_a_clean_run_passes(impl):
    assert impl.verdict(GOOD, REACH) == []
    assert impl.verdict(STOPPED, STOP) == []


def test_each_threshold_on_its_own(impl):
    assert impl.verdict({**GOOD, "reached": False}, REACH) == ["not-reached"]
    assert impl.verdict({**GOOD, "collided": True}, REACH) == ["collided"]
    assert impl.verdict({**GOOD, "duration_s": 61.0}, REACH) == ["too-slow"]
    assert impl.verdict({**GOOD, "min_clearance_m": 0.039}, REACH) == ["too-close"]
    assert impl.verdict({**GOOD, "interventions": 5}, REACH) == ["too-many-stops"]
    assert impl.verdict({**GOOD, "blind_distance_m": 0.06}, REACH) == ["drove-blind"]


def test_thresholds_are_inclusive_at_the_limit(impl):
    assert impl.verdict({**GOOD, "duration_s": 60.0}, REACH) == []
    assert impl.verdict({**GOOD, "min_clearance_m": 0.04}, REACH) == []
    assert impl.verdict({**GOOD, "interventions": 4}, REACH) == []
    assert impl.verdict({**GOOD, "blind_distance_m": 0.05}, REACH) == []


def test_a_stop_safely_scenario_inverts_the_reach_rule(impl):
    assert impl.verdict({**STOPPED, "reached": True}, STOP) == ["should-stop"]
    assert impl.verdict({**GOOD, "reached": False, "interventions": 1}, REACH) == ["not-reached"]


def test_stopping_by_luck_is_a_failure(impl):
    assert impl.verdict({**STOPPED, "interventions": 0}, STOP) == ["lucky-stop"]


def test_should_stop_and_lucky_stop_are_exclusive(impl):
    result = impl.verdict({**STOPPED, "reached": True, "interventions": 0}, STOP)
    assert result == ["should-stop"], "it reached the goal; whether it also stopped is moot"


def test_every_reason_is_reported_together_and_sorted(impl):
    bad = {"reached": False, "collided": True, "duration_s": 90.0, "min_clearance_m": 0.0,
           "interventions": 99, "blind_distance_m": 2.0}
    assert impl.verdict(bad, REACH) == ["collided", "drove-blind", "not-reached", "too-close",
                                        "too-many-stops", "too-slow"]


def test_it_agrees_with_the_course_suite(impl):
    scenarios = {s.id: s for s in ss.suite()}
    for build in ("shipping", "no-monitor"):
        for metrics in suite_metrics(build):
            scenario = scenarios[metrics.scenario]
            mine = impl.verdict(as_run(metrics), as_thresholds(scenario))
            assert bool(mine) == bool(metrics.failures(scenario)), (scenario.id, build, mine)


# --- regressions ------------------------------------------------------------------------------
def test_identical_runs_report_nothing(impl):
    assert impl.regressions({"S1": GOOD}, {"S1": GOOD}, {"S1": REACH}) == []


def test_a_new_failure_is_a_regression_with_its_reasons(impl):
    after = {"S1": {**GOOD, "collided": True, "min_clearance_m": 0.0}}
    assert impl.regressions({"S1": GOOD}, after, {"S1": REACH}) == [
        "REGRESSION S1: collided, too-close", "WORSE S1: clearance"]


def test_a_repaired_scenario_is_reported_too(impl):
    assert impl.regressions({"S1": {**GOOD, "collided": True}}, {"S1": GOOD}, {"S1": REACH}) == [
        "FIXED S1"]


def test_a_shrinking_margin_is_reported_even_while_it_passes(impl):
    after = {"S1": {**GOOD, "min_clearance_m": 0.08}}
    assert impl.regressions({"S1": GOOD}, after, {"S1": REACH}) == ["WORSE S1: clearance"]
    small = {"S1": {**GOOD, "min_clearance_m": 0.10}}
    assert impl.regressions({"S1": GOOD}, small, {"S1": REACH}) == []


def test_a_slower_run_needs_both_a_relative_and_an_absolute_jump(impl):
    assert impl.regressions({"S1": GOOD}, {"S1": {**GOOD, "duration_s": 15.0}}, {"S1": REACH}) == []
    short = {"reached": True, "collided": False, "duration_s": 2.0, "min_clearance_m": 0.12,
             "interventions": 0, "blind_distance_m": 0.0}
    assert impl.regressions({"S1": short}, {"S1": {**short, "duration_s": 2.8}}, {"S1": REACH}) == []
    assert impl.regressions({"S1": GOOD}, {"S1": {**GOOD, "duration_s": 20.0}}, {"S1": REACH}) == [
        "WORSE S1: time"]


def test_scenarios_missing_from_either_run_are_skipped(impl):
    assert impl.regressions({"S1": GOOD}, {"S2": GOOD}, {"S1": REACH, "S2": REACH}) == []


def test_lines_are_grouped_by_scenario_in_id_order(impl):
    before = {"S2": GOOD, "S1": GOOD}
    after = {"S2": {**GOOD, "collided": True}, "S1": {**GOOD, "reached": False}}
    assert impl.regressions(before, after, {"S1": REACH, "S2": REACH}) == [
        "REGRESSION S1: not-reached", "REGRESSION S2: collided"]


def test_it_finds_the_regression_the_course_suite_finds(impl):
    limits = {s.id: as_thresholds(s) for s in ss.suite()}
    shipping, broken = suite_runs("shipping"), suite_runs("no-monitor")
    lines = impl.regressions(shipping, broken, limits)
    assert any(line.startswith("REGRESSION S6-blocked") and "collided" in line for line in lines)
    assert any(line.startswith("REGRESSION S7-blind") and "drove-blind" in line for line in lines)
    assert impl.regressions(shipping, shipping, limits) == []
