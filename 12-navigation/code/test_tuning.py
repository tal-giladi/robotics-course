"""Tests for 12.08 — the Nav2 tuning arithmetic."""

from __future__ import annotations

import math

import nav2_config
import pytest
import tuning

params = nav2_config.load_params()
INSCRIBED = (nav2_config.footprint_radii(nav2_config.footprint_of(params))[0]
             + params["local_costmap"]["footprint_padding"])


def test_inflation_cost_matches_the_hand_computed_case_from_12_04():
    # 12.04: cost 164 at 0.20 m with k = 5 and r = 0.115 m, because C++ truncates
    assert tuning.inflation_cost(0.20, 0.115, 5.0) == 164
    assert tuning.inflation_cost(0.0, 0.115, 5.0) == 254
    assert tuning.inflation_cost(0.115, 0.115, 5.0) == 253


def test_karmel_fits_through_a_standard_israeli_interior_door():
    door = tuning.Doorway(0.80, INSCRIBED, 5.0)
    assert door.geometrically_passable
    assert door.clearance_m == pytest.approx(0.285)
    assert door.centre_cost == 60


def test_a_door_narrower_than_the_inflated_robot_is_blocked():
    door = tuning.Doorway(0.20, INSCRIBED, 5.0)
    assert not door.geometrically_passable
    assert door.centre_cost == 253, "the centre line itself is inside the inscribed radius"


def test_cost_in_a_doorway_falls_as_cost_scaling_factor_rises():
    costs = [tuning.Doorway(0.70, INSCRIBED, k).centre_cost for k in (1.0, 3.0, 5.0, 10.0)]
    assert costs == sorted(costs, reverse=True)
    assert costs[0] > 190 and costs[-1] < 30


def test_corner_cut_is_the_familiar_fraction_of_the_lookahead():
    assert tuning.corner_cut_m(1.0) == pytest.approx(1 - 1 / math.sqrt(2), abs=1e-9)
    assert tuning.corner_cut_m(0.4) == pytest.approx(0.117, abs=5e-4)
    assert tuning.corner_cut_m(0.4, math.pi) == pytest.approx(0.4), "a U-turn cuts the whole lookahead"


def test_stopping_distance_splits_into_latency_and_braking():
    budget = tuning.StopBudget(0.25, 0.8, 0.1, 0.2, 0.05)
    assert budget.latency_s == pytest.approx(0.35)
    assert budget.blind_m == pytest.approx(0.0875)
    assert budget.braking_m == pytest.approx(0.0391, abs=1e-4)
    assert budget.total_m == pytest.approx(0.1266, abs=1e-4)


def test_latency_dominates_at_karmel_speeds_and_braking_at_car_speeds():
    slow = tuning.StopBudget(0.25, 0.8, 0.1, 0.2, 0.05)
    fast = tuning.StopBudget(2.0, 1.5, 0.1, 0.2, 0.05)
    assert slow.blind_m > slow.braking_m
    assert fast.braking_m > fast.blind_m


def test_karmels_two_metre_local_costmap_has_room_at_cruise_and_runs_out_around_1_m_s():
    assert tuning.min_local_costmap_width_m(tuning.StopBudget(0.25, 0.8, 0.1, 0.2, 0.05)) < 1.0
    assert tuning.min_local_costmap_width_m(tuning.StopBudget(0.5, 0.8, 0.1, 0.2, 0.05)) < 1.5
    assert tuning.min_local_costmap_width_m(tuning.StopBudget(1.0, 0.8, 0.1, 0.2, 0.05)) > 2.0


def test_replan_period_keeps_the_stale_path_budget_constant():
    assert tuning.replan_period_for_distance_s(0.25) == pytest.approx(1.0)
    assert tuning.replan_period_for_distance_s(0.5) == pytest.approx(0.5)


def test_goal_tolerance_covers_noise_and_overshoot():
    assert tuning.goal_tolerance_m(0.02, 0.088) == pytest.approx(0.158)
    assert tuning.goal_tolerance_m(0.0, 0.0) == pytest.approx(0.03)


def test_transform_tolerance_covers_three_periods():
    assert tuning.transform_tolerance_s(10.0) == pytest.approx(0.3)
    assert tuning.transform_tolerance_s(5.0) == pytest.approx(0.6)


# --- recommendations -------------------------------------------------------------------------------
def names(recs):
    return [r.parameter for r in recs]


def test_a_healthy_robot_needs_almost_nothing_changed():
    recs = tuning.recommend(tuning.Measurements(), params)
    assert names(recs) == ["FollowPath.transform_tolerance"], \
        "the shipped 0.2 s is tight for a 10 Hz LiDAR; everything else already fits"


def test_a_noisy_localizer_widens_the_goal_tolerance():
    recs = tuning.recommend(tuning.Measurements(amcl_std_m=0.08), params)
    assert "controller_server.general_goal_checker.xy_goal_tolerance" in names(recs)


def test_a_loaded_pi_gets_told_to_cut_particles_and_the_controller_rate():
    recs = tuning.recommend(tuning.Measurements(cpu_load=0.95, controller_hz=9.0), params)
    assert "amcl.max_particles" in names(recs)
    assert "controller_server.controller_frequency" in names(recs)


def test_a_narrow_door_is_reported_against_the_footprint():
    recs = tuning.recommend(tuning.Measurements(narrowest_door_m=0.20), params)
    assert "footprint / footprint_padding" in names(recs)


def test_a_tight_corridor_warns_about_the_lookahead():
    recs = tuning.recommend(tuning.Measurements(narrowest_door_m=0.40), params)
    assert any("lookahead_dist" in name for name in names(recs)), \
        "0.4 m lookahead cuts 11.7 cm; a 40 cm door leaves only 8.5 cm"


def test_a_robot_that_cannot_reach_its_cruise_speed_is_told_so():
    recs = tuning.recommend(tuning.Measurements(measured_max_speed_m_s=0.18), params)
    assert "FollowPath.desired_linear_vel" in names(recs)


def test_every_recommendation_explains_itself():
    recs = tuning.recommend(tuning.Measurements(cpu_load=0.95, amcl_std_m=0.08, scan_hz=6.0,
                                                controller_hz=9.0, narrowest_door_m=0.45,
                                                measured_max_speed_m_s=0.18), params)
    assert len(recs) >= 6
    for rec in recs:
        assert rec.reason and rec.current != rec.suggested
        assert str(rec).strip().startswith(rec.parameter)


def test_sim_vs_real_table_covers_the_six_things_that_bite():
    assert len(tuning.SIM_VS_REAL) == 6
    assert {what for what, _, _ in tuning.SIM_VS_REAL} == {
        "Clock", "LiDAR", "Wheel odometry", "Obstacles", "CPU", "Failure"}
