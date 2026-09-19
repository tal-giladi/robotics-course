"""Tests for the module-20 integration scripts.

    py -m pytest 20-final-robot/code

They pin the numbers the lessons quote, and the properties that make each script worth having:
the course architecture passes its own checks, the deliberately broken variants fail for the
stated reasons, and the scenario suite is deterministic.
"""

from __future__ import annotations

import math

import pytest

import bringup_health as bh
import compute_plan as cp
import demo_runbook as dr
import power_budget_v2 as pb
import safety_case as sc
import scenario_suite as ss
import system_contracts as sysc


# ---------------------------------------------------------------------------------------------
# 20.01 — interface contracts and compute placement
# ---------------------------------------------------------------------------------------------
def test_karmel_system_is_coherent():
    assert sysc.check_system(sysc.karmel_system()) == []


def test_broken_system_reports_exactly_the_four_planted_faults():
    rules = sorted(p.rule for p in sysc.check_system(sysc.broken_system()))
    assert rules == ["frame", "qos", "rate", "single-writer"]


def test_qos_request_offer_rules():
    assert sysc.qos_incompatibility(sysc.SENSOR_DATA, sysc.Qos()) is not None      # best_effort -> reliable
    assert sysc.qos_incompatibility(sysc.Qos(), sysc.SENSOR_DATA) is None          # reliable -> best_effort is fine
    assert sysc.qos_incompatibility(sysc.Qos(), sysc.LATCHED) is not None          # volatile -> transient_local


def test_camera_dominates_the_bandwidth_budget():
    total = sum(rate for _, rate in sysc.bandwidth(sysc.karmel_system()))
    top_topic, top_rate = sysc.bandwidth(sysc.karmel_system())[0]
    assert "camera" in top_topic
    assert top_rate / total > 0.8
    assert total * 8 / 1e6 == pytest.approx(6.2, abs=0.3)          # Mbit/s, quoted in 20.01


def test_latency_chains_fit_their_budgets():
    for chain in sysc.chains():
        assert chain.ok, f"{chain.name}: {chain.worst_case_s:.3f} s > {chain.budget_s} s"
    assert sysc.estop_chain().worst_case_s == pytest.approx(0.012)


def test_course_compute_plan_is_feasible_and_matches_the_rules():
    plan = cp.course_plan()
    assert cp.problems(plan) == []
    assert all(cp.required_host(job) == job.host for job in plan)
    assert cp.load_by_host(plan)[cp.PI].core_headroom == pytest.approx(0.25, abs=0.01)


def test_dropping_the_jetson_overloads_the_pi():
    _, moved = cp.whatif("no-jetson")
    assert any("cores of work" in p for p in cp.problems(moved))


# ---------------------------------------------------------------------------------------------
# 20.02 — power, wiring and mass
# ---------------------------------------------------------------------------------------------
def test_power_and_mass_budget_holds():
    loads, parts = pb.karmel_v2_loads(), pb.karmel_v2_parts()
    assert pb.check(loads, pb.karmel_v2_branches(loads), parts) == []


def test_every_actuator_is_behind_the_estop_relay():
    assert all(load.domain == pb.ACTUATOR for load in pb.karmel_v2_loads() if load.is_actuator)


def test_fuse_sizing_rule():
    assert pb.fuse_for(5.9, 18) == 7.5           # 1.25 x 5.9 = 7.4 -> the next standard value
    assert pb.fuse_for(1.71, 18) == 3.0
    with pytest.raises(ValueError):
        pb.fuse_for(12.0, 18)                    # 15 A fuse on 8 A wire: the wire is the fuse


def test_voltage_drop_counts_both_conductors():
    branch = pb.Branch("test", pb.ACTUATOR, 10.8, 5.0, 5.0, awg=18, length_m=1.0)
    assert branch.resistance_ohm == pytest.approx(2 * 0.02095)
    assert branch.drop_v == pytest.approx(5.0 * 2 * 0.02095)


def test_centre_of_gravity_and_tipping():
    cg = pb.centre_of_gravity(pb.karmel_v2_parts())
    assert cg.mass_kg == pytest.approx(2.66, abs=0.01)
    assert -0.05 < cg.x_m < 0.0                                  # behind the axle, as it must be
    assert pb.tipping_decel_m_s2(cg) == pytest.approx(pb.G * abs(cg.x_m) / cg.z_m)
    assert pb.tipping_decel_m_s2(cg) > 2.0                       # 2x the configured braking limit


def test_mounting_the_arm_at_the_front_tips_the_robot():
    _, loads, parts = pb.whatif("arm-forward")
    cg = pb.centre_of_gravity(parts)
    assert cg.x_m > 0.0
    assert pb.stability_margin_m(cg) < 0.0
    assert pb.tipping_decel_m_s2(cg) == 0.0
    assert any("nose" in p for p in pb.check(loads, pb.karmel_v2_branches(loads), parts))


def test_adding_a_jetson_breaks_the_5v_rail():
    _, loads, parts = pb.whatif("jetson")
    assert any("5 V rail" in p for p in pb.check(loads, pb.karmel_v2_branches(loads), parts))


def test_runtime_is_dominated_by_the_always_on_compute():
    loads = pb.karmel_v2_loads()
    mission = pb.session_power_w(loads)
    idle = pb.session_power_w([l for l in loads if not l.is_actuator])  # noqa: E741
    assert idle / mission > 0.6
    assert pb.runtime_min(3.35, 10.8, mission) == pytest.approx(107, abs=5)


# ---------------------------------------------------------------------------------------------
# 20.03 — bringup and health
# ---------------------------------------------------------------------------------------------
def test_start_waves_respect_dependencies():
    units = bh.karmel_units()
    waves = bh.start_waves(units)
    position = {name: i for i, wave in enumerate(waves) for name in wave}
    for unit in units:
        for dep in unit.requires:
            assert position[dep] < position[unit.name]


def test_start_waves_reject_a_cycle():
    units = [bh.Unit("a", ("b",)), bh.Unit("b", ("a",))]
    with pytest.raises(ValueError, match="cycle"):
        bh.start_waves(units)


def test_bringup_completes_in_16_seconds():
    ready = bh.ready_times(bh.karmel_units())
    assert max(ready.values()) == pytest.approx(16.0)
    assert ready["nav2"] == pytest.approx(13.5)


def test_a_dead_lidar_blocks_everything_downstream():
    ready = bh.ready_times(bh.karmel_units(), frozenset({"lidar"}))
    assert ready["lidar"] is None
    assert ready["nav2"] is None and ready["collision_monitor"] is None
    assert ready["arm"] is not None            # the arm does not depend on the LiDAR


def test_staleness_is_a_level_not_a_silence():
    assert bh.staleness_level(0.0, 2.0, period_s=1.0) is None
    assert bh.staleness_level(0.0, 4.0, period_s=1.0) is bh.Level.STALE


def test_robot_state_policy():
    units = bh.karmel_units()
    ok = {u.name: bh.Level.OK for u in units}
    assert bh.robot_state(units, ok) is bh.RobotState.READY
    assert bh.robot_state(units, ok, estop=True) is bh.RobotState.ESTOP
    assert bh.robot_state(units, ok, bringup_complete=False) is bh.RobotState.INIT
    assert bh.robot_state(units, {**ok, "camera": bh.Level.ERROR}) is bh.RobotState.DEGRADED
    assert bh.robot_state(units, {**ok, "lidar": bh.Level.STALE}) is bh.RobotState.FAULT
    assert bh.robot_state(units, {**ok, "agent_bridge": bh.Level.ERROR}) is bh.RobotState.READY


def test_estop_beats_a_fault():
    units = bh.karmel_units()
    levels = {u.name: bh.Level.STALE for u in units}
    assert bh.robot_state(units, levels, estop=True) is bh.RobotState.ESTOP


def test_simulated_session_transitions():
    states = [state for _, state, _ in bh.simulate(bh.karmel_units(), bh.default_faults())]
    assert states == [bh.RobotState.INIT, bh.RobotState.READY, bh.RobotState.DEGRADED,
                      bh.RobotState.FAULT, bh.RobotState.DEGRADED]


def test_every_state_has_a_mission_policy():
    for state in bh.RobotState:
        allowed, speed = bh.MISSION_POLICY[state]
        assert allowed and speed >= 0.0
    assert bh.MISSION_POLICY[bh.RobotState.DEGRADED][1] < bh.MISSION_POLICY[bh.RobotState.READY][1]


# ---------------------------------------------------------------------------------------------
# 20.04 — the safety case
# ---------------------------------------------------------------------------------------------
def test_the_safety_case_holds():
    assert sc.check_case(sc.karmel_hazards()) == []


def test_every_severe_hazard_has_a_control_that_survives_a_software_bug():
    for hazard in sc.karmel_hazards():
        if hazard.severity >= 3:
            assert any(c.independent_of_software for c in hazard.controls), hazard.id


def test_removing_the_estop_breaks_the_case():
    _, hazards = sc.whatif("no-estop")
    problems = sc.check_case(hazards)
    assert len(problems) >= 5
    assert any("H3" in p for p in problems)


def test_an_untested_control_is_not_a_control():
    _, hazards = sc.whatif("untested")
    assert any("wish" in p for p in sc.check_case(hazards))


def test_stopping_distance_formula():
    mechanism = sc.StopMechanism("test", 0.1, 0.1, 1.5, "firmware")
    assert mechanism.distance_m(0.3) == pytest.approx(0.3 * 0.2 + 0.09 / 3.0)


def test_the_estop_acts_fastest_and_stops_slowest():
    mechanisms = {m.name: m for m in sc.stop_mechanisms()}
    estop = next(m for n, m in mechanisms.items() if n.startswith("hardware e-stop"))
    monitor = next(m for n, m in mechanisms.items() if n.startswith("collision_monitor"))
    assert estop.dead_time_s < monitor.dead_time_s
    assert estop.decel_m_s2 < monitor.decel_m_s2          # power removed = coasting


def test_every_software_stop_fits_the_margin_at_walking_pace():
    for mechanism in sc.stop_mechanisms():
        if mechanism.layer in ("hardware", "firmware", "onboard-software"):
            assert mechanism.distance_m(0.3) < sc.STOP_MARGIN_M, mechanism.name


def test_the_llm_does_not_fit_the_margin():
    agent = next(m for m in sc.stop_mechanisms() if m.layer == "offboard-software")
    assert agent.distance_m(0.3) > sc.STOP_MARGIN_M


# ---------------------------------------------------------------------------------------------
# 20.05 — the scenario suite
# ---------------------------------------------------------------------------------------------
def test_the_shipping_build_passes_the_whole_suite():
    result = ss.run_suite(ss.builds()["shipping"])
    failures = {m.scenario: m.failures(s) for m, s in zip(result.runs, ss.suite(), strict=True)}
    assert all(not f for f in failures.values()), failures


def test_runs_are_deterministic():
    scenario = ss.suite()[0]
    build = ss.builds()["shipping"]
    first, second = ss.run_scenario(scenario, build, 0), ss.run_scenario(scenario, build, 0)
    assert first.duration_s == second.duration_s
    assert math.isclose(first.distance_m, second.distance_m, rel_tol=1e-12)


def test_disabling_the_collision_monitor_causes_a_collision():
    blocked = next(s for s in ss.suite() if s.id == "S6-blocked")
    assert ss.run_scenario(blocked, ss.builds()["no-monitor"]).collided
    assert not ss.run_scenario(blocked, ss.builds()["shipping"]).collided


def test_a_fail_open_monitor_drives_blind():
    blind = next(s for s in ss.suite() if s.id == "S7-blind")
    assert ss.run_scenario(blind, ss.builds()["fail-open"]).blind_distance_m > 0.5
    assert ss.run_scenario(blind, ss.builds()["shipping"]).blind_distance_m < 0.05


def test_compare_reports_the_regression():
    only = "S7"
    before = ss.run_suite(ss.builds()["shipping"], only=only)
    after = ss.run_suite(ss.builds()["fail-open"], only=only)
    lines = ss.compare(before, after)
    assert any(line.startswith("REGRESSION S7") for line in lines)
    assert ss.compare(before, before) == []


# ---------------------------------------------------------------------------------------------
# 20.06 — the demo
# ---------------------------------------------------------------------------------------------
def test_the_demo_plan_is_ready():
    assert dr.check(dr.demo()) == []


def test_the_demo_fits_the_battery_with_reserve():
    b = dr.budget(dr.demo())
    assert b["used_fraction"] < 1.0 - dr.RESERVE_FRACTION
    assert b["minutes"] <= 12.0


def test_every_segment_has_a_fallback_and_an_abort():
    for segment in dr.demo():
        assert segment.fallback and segment.abort, segment.name


def test_a_segment_without_a_fallback_fails_the_check():
    broken = [*dr.demo()[:-1], dr.Segment("8. Encore", "something", 1.0, 0.5, 0.5, "anything", "", "e-stop")]
    assert any("no fallback" in p for p in dr.check(broken))
