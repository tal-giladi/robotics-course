"""Tests for 12.07 — reading and checking karmel's Nav2 parameter file."""

from __future__ import annotations

import math

import nav2_config as nc
import pytest

from robotlab.config import load_config

params = nc.load_params()


def test_every_block_in_the_file_is_a_server_we_can_name():
    known = {name for name, _, _ in nc.SERVERS} | {"local_costmap", "global_costmap"}
    assert set(params) - known == set(), "an unexplained parameter block appeared"
    # map_server is the one server the tour knows about that karmel configures from the launch file
    assert set(known) - set(params) == {"map_server"}


def test_costmaps_are_flattened_out_of_their_double_nesting():
    for costmap in ("local_costmap", "global_costmap"):
        assert params[costmap]["resolution"] == 0.05
        assert "inflation_layer" in params[costmap]


def test_the_plugins_karmel_actually_runs():
    plugins = dict(nc.plugins_of(params))
    assert plugins["GridBased"] == "nav2_navfn_planner::NavfnPlanner"
    assert plugins["FollowPath"] == \
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    assert plugins["general_goal_checker"] == "nav2_controller::SimpleGoalChecker"
    assert plugins["progress_checker"] == "nav2_controller::SimpleProgressChecker"
    assert plugins["inflation_layer"] == "nav2_costmap_2d::InflationLayer"


def test_planner_uses_astar_as_lesson_12_02_teaches():
    assert params["planner_server"]["GridBased"]["use_astar"] is True


def test_footprint_radii_match_the_chassis():
    cfg = load_config()
    inscribed, circumscribed = nc.footprint_radii(nc.footprint_of(params))
    assert inscribed == pytest.approx((cfg.drive.wheel_separation_m + cfg.drive.wheel_width_m) / 2)
    assert circumscribed == pytest.approx(math.hypot(0.125, 0.105))


def test_footprint_radii_of_a_triangle_use_edges_not_only_vertices():
    inscribed, circumscribed = nc.footprint_radii([(0.2, 0.0), (-0.1, 0.15), (-0.1, -0.15)])
    # the nearest point of the polygon is on a slanted side, 0.03 / hypot(0.3, 0.15) away,
    # not on the flat back edge at x = -0.1
    assert inscribed == pytest.approx(0.03 / math.hypot(0.3, 0.15))
    assert circumscribed == pytest.approx(0.2)      # the nose


def test_all_consistency_checks_pass_for_the_shipped_file():
    failed = [c.name for c in nc.check_against_robot(params) if not c.ok]
    assert failed == []


def test_a_check_fails_when_the_footprint_stops_matching_the_robot(tmp_path):
    broken = nc.load_params()
    broken["global_costmap"] = dict(broken["global_costmap"])
    broken["global_costmap"]["footprint"] = "[[0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3]]"
    failed = [c.name for c in nc.check_against_robot(broken) if not c.ok]
    assert "footprint = chassis length x wheel span" in failed


def test_a_check_fails_when_rpp_and_the_costmap_disagree_about_inflation():
    broken = nc.load_params()
    broken["controller_server"]["FollowPath"]["inflation_cost_scaling_factor"] = 3.0
    failed = [c.name for c in nc.check_against_robot(broken) if not c.ok]
    assert any("inflation_cost_scaling_factor" in name for name in failed)


def test_key_numbers_are_the_ones_the_lesson_quotes():
    numbers = nc.key_numbers(params)
    assert numbers["controller_frequency_hz"] == 20.0
    assert numbers["desired_linear_vel_m_s"] == 0.25
    assert numbers["xy_goal_tolerance_m"] == 0.15
    assert numbers["local_costmap_width_m"] == 2
    assert numbers["max_velocity_m_s"] == 0.3
    assert numbers["bt_default_server_timeout_ms"] == 200


def test_loading_a_stock_style_file_without_karmel_extras(tmp_path):
    path = tmp_path / "small.yaml"
    path.write_text(
        "planner_server:\n"
        "  ros__parameters:\n"
        "    planner_plugins: ['GridBased']\n"
        "    GridBased:\n"
        "      plugin: 'nav2_navfn_planner::NavfnPlanner'\n",
        encoding="utf-8")
    small = nc.load_params(path)
    assert small["planner_server"]["planner_plugins"] == ["GridBased"]
    assert nc.plugins_of(small) == [("GridBased", "nav2_navfn_planner::NavfnPlanner")]
