"""Checker for 20.03 — bringup waves, staleness and the robot state machine.

Run: ``python course.py check 20.03`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "20-final-robot" / "code",):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import bringup_health as bh  # noqa: E402

KARMEL = {u.name: list(u.requires) for u in bh.karmel_units()}
CRITICALITY = {u.name: u.criticality for u in bh.karmel_units()}


# --- start_waves ------------------------------------------------------------------------------
def test_a_diamond(impl):
    assert impl.start_waves({"a": [], "b": ["a"], "c": ["a"], "d": ["b", "c"]}) == [
        ["a"], ["b", "c"], ["d"]]


def test_independent_units_share_a_wave_and_are_sorted(impl):
    assert impl.start_waves({"z": [], "a": [], "m": []}) == [["a", "m", "z"]]


def test_a_chain_is_one_unit_per_wave(impl):
    assert impl.start_waves({"a": [], "b": ["a"], "c": ["b"]}) == [["a"], ["b"], ["c"]]


def test_a_cycle_is_an_error_not_a_hang(impl):
    with pytest.raises(ValueError):
        impl.start_waves({"a": ["b"], "b": ["a"]})


def test_a_dependency_that_does_not_exist_is_an_error(impl):
    with pytest.raises(ValueError):
        impl.start_waves({"nav2": ["amcl"]})


def test_karmels_waves(impl):
    waves = impl.start_waves(KARMEL)
    assert waves == bh.start_waves(bh.karmel_units())
    position = {name: i for i, wave in enumerate(waves) for name in wave}
    for unit, deps in KARMEL.items():
        for dep in deps:
            assert position[dep] < position[unit], f"{unit} starts before {dep}"
    assert "pico_link" in waves[0], "nothing can talk to the base before the serial link is up"


# --- component_level --------------------------------------------------------------------------
def test_a_fresh_report_is_taken_at_face_value(impl):
    assert impl.component_level("OK", 10.0, 10.5, period_s=1.0) == "OK"
    assert impl.component_level("WARN", 10.0, 12.9, period_s=1.0) == "WARN"


def test_silence_beats_the_last_good_news(impl):
    assert impl.component_level("OK", 10.0, 14.0, period_s=1.0) == "STALE"
    assert impl.component_level("ERROR", 10.0, 14.0, period_s=1.0) == "STALE"


def test_a_component_that_never_reported_is_stale(impl):
    assert impl.component_level(None, None, 5.0, period_s=1.0) == "STALE"
    assert impl.component_level(None, 1.0, 1.1, period_s=1.0) == "STALE"


def test_the_tolerance_scales_with_the_reporting_period(impl):
    # A 0.2 Hz component (period 5 s) may be quiet for 15 s; a 10 Hz one may not.
    assert impl.component_level("OK", 0.0, 14.0, period_s=5.0) == "OK"
    assert impl.component_level("OK", 0.0, 16.0, period_s=5.0) == "STALE"
    assert impl.component_level("OK", 0.0, 0.4, period_s=0.1) == "STALE"


def test_it_agrees_with_the_course_staleness_rule(impl):
    for now in (0.5, 2.9, 3.1, 10.0):
        mine = impl.component_level("OK", 0.0, now, period_s=1.0)
        theirs = bh.staleness_level(0.0, now, 1.0) or bh.Level.OK
        assert (mine == "STALE") == (theirs is bh.Level.STALE)


# --- robot_state ------------------------------------------------------------------------------
def all_ok() -> dict[str, str]:
    return {name: "OK" for name in CRITICALITY}


def test_everything_ok_is_ready(impl):
    assert impl.robot_state(all_ok(), CRITICALITY) == "READY"


def test_the_estop_wins_over_everything(impl):
    levels = {name: "STALE" for name in CRITICALITY}
    assert impl.robot_state(levels, CRITICALITY, estop=True) == "ESTOP"
    assert impl.robot_state(all_ok(), CRITICALITY, estop=True) == "ESTOP"


def test_an_incomplete_bringup_is_init_not_fault(impl):
    levels = {**all_ok(), "nav2": "STALE"}
    assert impl.robot_state(levels, CRITICALITY, bringup_complete=False) == "INIT"


def test_a_bringup_that_timed_out_stops_excusing_missing_units(impl):
    levels = {**all_ok(), "nav2": "STALE"}
    assert impl.robot_state(levels, CRITICALITY, bringup_complete=False,
                            bringup_timed_out=True) == "FAULT"


def test_a_required_component_in_error_is_a_fault(impl):
    for level in ("ERROR", "STALE"):
        assert impl.robot_state({**all_ok(), "lidar": level}, CRITICALITY) == "FAULT"


def test_a_degraded_component_degrades_but_does_not_fault(impl):
    assert impl.robot_state({**all_ok(), "camera": "ERROR"}, CRITICALITY) == "DEGRADED"
    assert impl.robot_state({**all_ok(), "arm": "STALE"}, CRITICALITY) == "DEGRADED"


def test_a_warning_anywhere_that_matters_degrades(impl):
    assert impl.robot_state({**all_ok(), "imu": "WARN"}, CRITICALITY) == "DEGRADED"
    assert impl.robot_state({**all_ok(), "ekf": "WARN"}, CRITICALITY) == "DEGRADED"


def test_an_optional_component_never_changes_the_state(impl):
    for level in ("WARN", "ERROR", "STALE"):
        assert impl.robot_state({**all_ok(), "agent_bridge": level}, CRITICALITY) == "READY"
        assert impl.robot_state({**all_ok(), "rosbag2": level}, CRITICALITY) == "READY"


def test_a_fault_is_not_hidden_behind_a_degraded(impl):
    levels = {**all_ok(), "camera": "ERROR", "lidar": "ERROR"}
    assert impl.robot_state(levels, CRITICALITY) == "FAULT"


def test_components_not_in_this_robot_are_ignored(impl):
    assert impl.robot_state({**all_ok(), "some_other_robot": "STALE"}, CRITICALITY) == "READY"


def test_it_agrees_with_the_course_state_machine(impl):
    units = bh.karmel_units()
    cases = [all_ok(), {**all_ok(), "camera": "ERROR"}, {**all_ok(), "lidar": "STALE"},
             {**all_ok(), "imu": "WARN"}, {**all_ok(), "agent_bridge": "ERROR"}]
    for levels in cases:
        mine = impl.robot_state(levels, CRITICALITY)
        theirs = bh.robot_state(units, {k: bh.Level[v] for k, v in levels.items()})
        assert mine == theirs.name, levels
