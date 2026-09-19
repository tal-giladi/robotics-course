"""Checker for 20.04 — stopping distance, safe speed and open hazards.

Run: ``python course.py check 20.04`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "20-final-robot" / "code",):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import safety_case as sc  # noqa: E402


def hazard(hid: str, severity: int, *controls: tuple[str, str]) -> dict:
    return {"id": hid, "severity": severity,
            "controls": [{"layer": layer, "tested_by": test} for layer, test in controls]}


# --- stopping_distance_m ----------------------------------------------------------------------
def test_the_two_terms(impl):
    assert impl.stopping_distance_m(0.3, 0.195, 1.5) == pytest.approx(0.3 * 0.195 + 0.09 / 3.0)
    assert impl.stopping_distance_m(0.3, 0.0, 1.5) == pytest.approx(0.03)     # braking only
    assert impl.stopping_distance_m(0.3, 1.0, 1e9) == pytest.approx(0.3, abs=1e-6)  # dead time only


def test_distance_grows_faster_than_speed(impl):
    single = impl.stopping_distance_m(0.3, 0.2, 1.5)
    double = impl.stopping_distance_m(0.6, 0.2, 1.5)
    assert double > 2 * single, "the braking term is quadratic: doubling the speed is worse than 2x"


def test_standing_still_and_no_braking(impl):
    assert impl.stopping_distance_m(0.0, 1.0, 1.5) == 0.0
    assert impl.stopping_distance_m(0.3, 0.2, 0.0) == float("inf")


def test_it_reproduces_the_course_stop_table(impl):
    for mechanism in sc.stop_mechanisms():
        assert impl.stopping_distance_m(0.3, mechanism.dead_time_s, mechanism.decel_m_s2) == (
            pytest.approx(mechanism.distance_m(0.3)))


def test_the_estop_stops_slower_than_the_collision_monitor_despite_acting_faster(impl):
    estop = next(m for m in sc.stop_mechanisms() if m.name.startswith("hardware e-stop"))
    monitor = next(m for m in sc.stop_mechanisms() if m.name.startswith("collision_monitor"))
    fast = impl.stopping_distance_m(1.0, estop.dead_time_s, estop.decel_m_s2)
    slow = impl.stopping_distance_m(1.0, monitor.dead_time_s, monitor.decel_m_s2)
    assert estop.dead_time_s < monitor.dead_time_s
    assert fast > slow, "removing power means coasting; at speed that is the longer stop"


# --- max_safe_speed_m_s -----------------------------------------------------------------------
def test_it_is_the_exact_inverse(impl):
    for dead, decel, margin in ((0.195, 1.5, 0.365), (0.3, 1.5, 0.2), (2.2, 1.5, 0.365)):
        v = impl.max_safe_speed_m_s(margin, dead, decel)
        assert impl.stopping_distance_m(v, dead, decel) == pytest.approx(margin)


def test_more_dead_time_means_a_slower_robot(impl):
    speeds = [impl.max_safe_speed_m_s(0.365, t, 1.5) for t in (0.2, 1.0, 2.0, 5.0)]
    assert speeds == sorted(speeds, reverse=True)
    assert speeds[-1] < 0.1, "a 5 s decision loop leaves a crawl"


def test_edge_cases(impl):
    assert impl.max_safe_speed_m_s(0.0, 0.2, 1.5) == 0.0
    assert impl.max_safe_speed_m_s(-0.1, 0.2, 1.5) == 0.0
    assert impl.max_safe_speed_m_s(0.4, 2.0, 0.0) == pytest.approx(0.2)
    assert impl.max_safe_speed_m_s(0.4, 0.0, 0.0) == float("inf")


def test_the_collision_monitor_allows_karmels_speed_limit_and_the_llm_does_not(impl):
    monitor = next(m for m in sc.stop_mechanisms() if m.name.startswith("collision_monitor"))
    agent = next(m for m in sc.stop_mechanisms() if m.layer == "offboard-software")
    assert impl.max_safe_speed_m_s(sc.STOP_MARGIN_M, monitor.dead_time_s, monitor.decel_m_s2) > 0.5
    assert impl.max_safe_speed_m_s(sc.STOP_MARGIN_M, agent.dead_time_s, agent.decel_m_s2) < 0.2


# --- open_hazards -----------------------------------------------------------------------------
def test_a_minor_hazard_with_a_tested_software_control_is_closed(impl):
    assert impl.open_hazards([hazard("H1", 2, ("onboard-software", "12.10-E4"))]) == []


def test_a_serious_hazard_needs_something_outside_software(impl):
    assert impl.open_hazards([hazard("H2", 3, ("onboard-software", "x"), ("firmware", "y"))]) == ["H2"]
    assert impl.open_hazards([hazard("H2", 3, ("onboard-software", "x"), ("hardware", "y"))]) == []


def test_a_severe_hazard_needs_two_independent_hardware_controls(impl):
    assert impl.open_hazards([hazard("H3", 4, ("hardware", "x"))]) == ["H3"]
    assert impl.open_hazards([hazard("H3", 4, ("hardware", "x"), ("hardware", "y"))]) == []


def test_an_untested_control_opens_the_hazard(impl):
    assert impl.open_hazards([hazard("H4", 1, ("hardware", ""))]) == ["H4"]
    assert impl.open_hazards([hazard("H4", 2, ("firmware", "y"), ("procedure", ""))]) == ["H4"]


def test_no_controls_at_all(impl):
    assert impl.open_hazards([hazard("H5", 1)]) == ["H5"]


def test_ids_are_unique_and_sorted(impl):
    many = [hazard("H9", 4, ("hardware", "")), hazard("H1", 3), hazard("H5", 1, ("procedure", "x"))]
    assert impl.open_hazards(many) == ["H1", "H9"]


def test_karmels_own_case_is_closed(impl):
    hazards = [{"id": h.id, "severity": h.severity,
                "controls": [{"layer": c.layer, "tested_by": c.tested_by} for c in h.controls]}
               for h in sc.karmel_hazards()]
    assert impl.open_hazards(hazards) == []


def test_removing_the_estop_opens_several_hazards(impl):
    _, stripped = sc.whatif("no-estop")
    hazards = [{"id": h.id, "severity": h.severity,
                "controls": [{"layer": c.layer, "tested_by": c.tested_by} for c in h.controls]}
               for h in stripped]
    opened = impl.open_hazards(hazards)
    assert "H3" in opened and len(opened) >= 4
