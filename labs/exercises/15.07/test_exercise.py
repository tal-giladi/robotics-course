"""Tests for 15.07 — grasp pose, approach waypoints, and the pre-flight checks."""

from __future__ import annotations

import math

import numpy as np
import pytest

BLOCK = (0.237, 0.059, 0.026)
YAW = math.radians(115.0)


# ---------------------------------------------------------------- the approach axis
def test_straight_down_is_straight_down(impl):
    for azimuth in (0.0, 1.0, -2.5):
        np.testing.assert_allclose(impl.approach_axis(math.pi / 2, azimuth),
                                   [0.0, 0.0, -1.0], atol=1e-12)


def test_horizontal_approach_points_along_the_azimuth(impl):
    a = impl.approach_axis(0.0, math.radians(30.0))
    np.testing.assert_allclose(a, [math.cos(math.radians(30)), math.sin(math.radians(30)), 0.0],
                               atol=1e-12)


def test_approach_axis_is_unit_and_matches_its_pitch(impl):
    for pitch_deg in (90, 80, 70, 45, 10):
        a = impl.approach_axis(math.radians(pitch_deg), 0.7)
        assert float(np.linalg.norm(a)) == pytest.approx(1.0)
        pitch = math.atan2(-a[2], math.hypot(a[0], a[1]))       # arm_kinematics.approach_pitch
        assert math.degrees(pitch) == pytest.approx(pitch_deg, abs=1e-9)


# ---------------------------------------------------------------- the tool pose
def test_tool_pose_is_a_proper_rotation(impl):
    for pitch_deg in (90, 80, 60, 45):
        T = impl.grasp_tool_pose(BLOCK, YAW, pitch_rad=math.radians(pitch_deg))
        R = T[:3, :3]
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        assert float(np.linalg.det(R)) == pytest.approx(1.0, abs=1e-12)
        np.testing.assert_allclose(T[:3, 3], BLOCK, atol=1e-15)
        np.testing.assert_allclose(T[3], [0, 0, 0, 1], atol=1e-15)


def test_tool_z_is_the_approach_axis(impl):
    T = impl.grasp_tool_pose(BLOCK, YAW, pitch_rad=math.radians(70.0))
    azimuth = math.atan2(BLOCK[1], BLOCK[0])
    np.testing.assert_allclose(T[:3, 2], impl.approach_axis(math.radians(70.0), azimuth),
                               atol=1e-12)


def test_jaw_yaw_round_trips_for_a_vertical_approach(impl):
    for yaw_deg in (0.0, 25.0, 50.0, 115.0, 179.0):
        T = impl.grasp_tool_pose(BLOCK, math.radians(yaw_deg))
        assert impl.jaw_yaw_of(T) == pytest.approx(impl.wrap_axis(math.radians(yaw_deg)), abs=1e-9)


def test_jaw_yaw_is_an_axis_not_a_direction(impl):
    a = impl.grasp_tool_pose(BLOCK, math.radians(30.0))
    b = impl.grasp_tool_pose(BLOCK, math.radians(210.0))
    assert impl.jaw_yaw_of(a) == pytest.approx(impl.jaw_yaw_of(b), abs=1e-9)


def test_tilted_approach_keeps_the_closing_direction_perpendicular(impl):
    T = impl.grasp_tool_pose(BLOCK, YAW, pitch_rad=math.radians(60.0))
    assert float(T[:3, 0] @ T[:3, 2]) == pytest.approx(0.0, abs=1e-12)


def test_parallel_closing_direction_is_rejected(impl):
    """Horizontal approach along the azimuth, closing along the same line: no valid frame."""
    with pytest.raises(ValueError):
        impl.grasp_tool_pose((0.25, 0.0, 0.05), 0.0, pitch_rad=0.0, azimuth_rad=0.0)


# ---------------------------------------------------------------- waypoints
def test_waypoint_names_and_order(impl):
    wps = impl.approach_waypoints(impl.grasp_tool_pose(BLOCK, YAW))
    assert [w.name for w in wps] == ["pre_grasp", "approach", "grasp", "lift"]


def test_vertical_standoff_positions(impl):
    T = impl.grasp_tool_pose(BLOCK, YAW)
    wps = impl.approach_waypoints(T, standoff_m=0.080, lift_m=0.060)
    np.testing.assert_allclose(wps[0].position, [0.237, 0.059, 0.106], atol=1e-12)
    np.testing.assert_allclose(wps[1].position, [0.237, 0.059, 0.046], atol=1e-12)
    np.testing.assert_allclose(wps[2].position, BLOCK, atol=1e-12)
    np.testing.assert_allclose(wps[3].position, [0.237, 0.059, 0.086], atol=1e-12)


def test_tilted_standoff_moves_back_along_the_approach_axis(impl):
    T = impl.grasp_tool_pose(BLOCK, YAW, pitch_rad=math.radians(60.0))
    wps = impl.approach_waypoints(T, standoff_m=0.100)
    offset = wps[0].position - np.asarray(BLOCK)
    np.testing.assert_allclose(offset, -0.100 * T[:3, 2], atol=1e-12)
    assert float(np.linalg.norm(offset)) == pytest.approx(0.100, abs=1e-12)


def test_the_lift_is_vertical_even_for_a_tilted_approach(impl):
    T = impl.grasp_tool_pose(BLOCK, YAW, pitch_rad=math.radians(60.0))
    wps = impl.approach_waypoints(T, lift_m=0.050)
    np.testing.assert_allclose(wps[3].position - np.asarray(BLOCK), [0.0, 0.0, 0.050], atol=1e-12)


def test_every_waypoint_keeps_the_grasp_orientation(impl):
    T = impl.grasp_tool_pose(BLOCK, YAW, pitch_rad=math.radians(70.0))
    for w in impl.approach_waypoints(T):
        np.testing.assert_allclose(w.T_base_tool[:3, :3], T[:3, :3], atol=1e-15)


def test_approach_is_slower_than_the_transit(impl):
    wps = impl.approach_waypoints(impl.grasp_tool_pose(BLOCK, YAW))
    by_name = {w.name: w for w in wps}
    assert by_name["approach"].speed_scale < by_name["pre_grasp"].speed_scale
    assert by_name["grasp"].speed_scale < by_name["pre_grasp"].speed_scale


def test_a_grasp_below_the_table_is_rejected(impl):
    T = impl.grasp_tool_pose((0.237, 0.059, 0.001), YAW)
    with pytest.raises(ValueError):
        impl.approach_waypoints(T, table_z=0.0, clearance_m=0.004)


# ---------------------------------------------------------------- the pre-flight check
def test_a_good_plan_has_no_problems(impl):
    wps = impl.approach_waypoints(impl.grasp_tool_pose(BLOCK, YAW))
    assert impl.check_plan(wps) == []


def test_out_of_reach_is_reported_and_names_the_waypoint(impl):
    far = (0.40, 0.0, 0.03)
    wps = impl.approach_waypoints(impl.grasp_tool_pose(far, 0.0))
    problems = impl.check_plan(wps, reach_max_m=0.32)
    assert problems
    assert any("grasp" in p for p in problems)


def test_too_close_to_the_base_is_reported(impl):
    near = (0.06, 0.0, 0.03)
    wps = impl.approach_waypoints(impl.grasp_tool_pose(near, 0.0), standoff_m=0.02, lift_m=0.02)
    assert impl.check_plan(wps, reach_min_m=0.12)


def test_out_of_order_waypoints_are_caught_first(impl):
    wps = impl.approach_waypoints(impl.grasp_tool_pose(BLOCK, YAW))
    problems = impl.check_plan([wps[2], wps[0], wps[1], wps[3]])
    assert len(problems) == 1
    assert "order" in problems[0]


def test_a_non_descending_approach_is_caught(impl):
    T = impl.grasp_tool_pose(BLOCK, YAW)
    wps = impl.approach_waypoints(T)
    broken = list(wps)
    bad = np.array(wps[0].T_base_tool, dtype=float)
    bad[2, 3] = wps[2].position[2] - 0.005                     # pre_grasp below the grasp
    broken[0] = impl.Waypoint("pre_grasp", bad, "broken", 1.0)
    assert any("descend" in p for p in impl.check_plan(broken))


# ---------------------------------------------------------------- the tolerance the budget must fit
def test_lateral_tolerance(impl):
    assert impl.lateral_tolerance_m(0.045, 0.030) == pytest.approx(0.0075)
    assert impl.lateral_tolerance_m(0.045, 0.020) == pytest.approx(0.0125)
    assert impl.lateral_tolerance_m(0.045, 0.040) == pytest.approx(0.0025)


def test_lateral_tolerance_is_never_negative(impl):
    assert impl.lateral_tolerance_m(0.045, 0.078) == pytest.approx(0.0)
