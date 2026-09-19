"""Tests for 12.10 — the collision monitor and the costmap filters."""

from __future__ import annotations

import math

import numpy as np
import pytest
import safety as sf


def square(half: float = 0.2) -> np.ndarray:
    return np.array([[half, half], [half, -half], [-half, -half], [-half, half]])


# --- geometry ---------------------------------------------------------------------------------
def test_point_in_polygon_counts_only_the_points_inside():
    points = np.array([[0.0, 0.0], [0.1, 0.1], [0.5, 0.0], [-0.5, 0.0], [0.0, 0.19]])
    assert sf.points_in_polygon(square(), points) == 3


def test_an_empty_point_cloud_is_never_inside():
    assert sf.points_in_polygon(square(), np.zeros((0, 2))) == 0


def test_project_state_drives_a_straight_line_then_an_arc():
    straight = sf.project_state(0.5, (0.0, 0.0, 0.0), sf.Velocity(0.2, 0.0))
    assert straight == pytest.approx((0.1, 0.0, 0.0))
    turning = sf.project_state(0.5, (0.0, 0.0, 0.0), sf.Velocity(0.2, 1.0))
    assert turning == pytest.approx((0.1, 0.0, 0.5)), "Nav2 steps, then rotates"


def test_transform_points_expresses_obstacles_in_the_moved_robots_frame():
    points = np.array([[1.0, 0.0]])
    moved = sf.transform_points((0.5, 0.0, 0.0), points)
    assert moved == pytest.approx(np.array([[0.5, 0.0]]))
    rotated = sf.transform_points((0.0, 0.0, math.pi / 2), points)
    assert rotated == pytest.approx(np.array([[0.0, -1.0]]), abs=1e-9)


# --- the approach action ----------------------------------------------------------------------
@pytest.fixture(scope="module")
def approach():
    return sf.karmel_footprint_approach()


def test_karmels_footprint_approach_matches_nav2_params(approach):
    assert approach.action_type is sf.ActionType.APPROACH
    assert approach.min_points == 6
    assert approach.time_before_collision == 1.2
    assert approach.simulation_time_step == 0.1
    assert approach.points[:, 0].max() == pytest.approx(0.135), "0.25 m chassis + 2 x 1 cm padding"


def test_a_far_wall_does_not_slow_the_robot(approach):
    monitor = sf.CollisionMonitor([approach])
    action = monitor.process(sf.Velocity(0.25, 0.0), sf.wall_points(1.5))
    assert action.action_type is sf.ActionType.NONE
    assert action.velocity.v == pytest.approx(0.25)


def test_the_approach_action_scales_speed_by_time_to_collision(approach):
    monitor = sf.CollisionMonitor([approach])
    fast = monitor.process(sf.Velocity(0.25, 0.0), sf.wall_points(0.40)).velocity.v
    close = monitor.process(sf.Velocity(0.25, 0.0), sf.wall_points(0.20)).velocity.v
    assert 0.0 < close < fast < 0.25
    assert fast == pytest.approx(0.25 * 1.0 / 1.2, abs=0.01)


def test_a_wall_already_inside_the_footprint_stops_the_robot(approach):
    action = sf.CollisionMonitor([approach]).process(sf.Velocity(0.25, 0.0), sf.wall_points(0.10))
    assert action.velocity.v == 0.0 and action.action_type is sf.ActionType.APPROACH


def test_time_to_collision_is_discretized_by_simulation_time_step(approach):
    times = {sf.wall_points(d).shape[0]: approach.collision_time(sf.wall_points(d), sf.Velocity(0.25, 0.0))
             for d in (0.4,)}
    collision_time = list(times.values())[0]
    assert collision_time == pytest.approx(round(collision_time / 0.1) * 0.1, abs=1e-9)


def test_fewer_points_than_min_points_do_not_trigger(approach):
    sparse = np.array([[0.2, 0.0], [0.2, 0.02]])            # only 2 points, min_points is 6
    assert approach.collision_time(sparse, sf.Velocity(0.25, 0.0)) == -1.0


def test_a_slower_command_gives_more_time_and_less_scaling(approach):
    slow = approach.collision_time(sf.wall_points(0.4), sf.Velocity(0.10, 0.0))
    fast = approach.collision_time(sf.wall_points(0.4), sf.Velocity(0.25, 0.0))
    assert slow == -1.0, "at 0.10 m/s the wall is more than 1.2 s away"
    assert fast > 0.0


# --- the other three actions -------------------------------------------------------------------
def test_stop_zeroes_the_command():
    zone = sf.SafetyPolygon.rectangle("stop", sf.ActionType.STOP, 0.8, 0.5, min_points=4)
    action = sf.CollisionMonitor([zone]).process(sf.Velocity(0.25, 0.8), sf.wall_points(0.3))
    assert action.velocity == sf.Velocity(0.0, 0.0)
    assert action.action_type is sf.ActionType.STOP


def test_slowdown_scales_both_components():
    zone = sf.SafetyPolygon.rectangle("slow", sf.ActionType.SLOWDOWN, 0.9, 0.6,
                                      min_points=4, slowdown_ratio=0.4)
    action = sf.CollisionMonitor([zone]).process(sf.Velocity(0.25, 0.8), sf.wall_points(0.3))
    assert action.velocity.v == pytest.approx(0.10)
    assert action.velocity.w == pytest.approx(0.32)


def test_limit_clamps_each_component_and_keeps_the_sign():
    zone = sf.SafetyPolygon.circle("limit", sf.ActionType.LIMIT, 0.5, min_points=4,
                                   linear_limit=0.1, angular_limit=0.4)
    action = sf.CollisionMonitor([zone]).process(sf.Velocity(-0.25, 0.8), sf.wall_points(0.3))
    assert action.velocity.v == pytest.approx(-0.1)
    assert action.velocity.w == pytest.approx(0.4)


def test_the_slowest_requirement_of_all_polygons_wins():
    monitor = sf.CollisionMonitor([
        sf.SafetyPolygon.rectangle("stop", sf.ActionType.STOP, 0.55, 0.45, min_points=4),
        sf.SafetyPolygon.rectangle("slow", sf.ActionType.SLOWDOWN, 1.2, 0.8, min_points=4,
                                   slowdown_ratio=0.5),
    ])
    assert monitor.process(sf.Velocity(0.25, 0.0), sf.wall_points(0.5)).action_type is sf.ActionType.SLOWDOWN
    assert monitor.process(sf.Velocity(0.25, 0.0), sf.wall_points(0.25)).action_type is sf.ActionType.STOP


def test_a_disabled_polygon_does_nothing():
    zone = sf.SafetyPolygon.rectangle("stop", sf.ActionType.STOP, 0.8, 0.5, min_points=4, enabled=False)
    assert sf.CollisionMonitor([zone]).process(sf.Velocity(0.25, 0.0),
                                               sf.wall_points(0.3)).velocity.v == pytest.approx(0.25)


# --- costmap filters ----------------------------------------------------------------------------
def test_mask_values_convert_to_costs():
    assert sf.mask_cost(-1) == sf.NO_INFORMATION
    assert sf.mask_cost(0) == 0
    assert sf.mask_cost(100) == sf.LETHAL_OBSTACLE
    assert sf.mask_cost(50) == pytest.approx(127, abs=1)


def test_base_and_multiplier_move_the_mask_into_filter_space():
    assert sf.mask_cost(50, base=50.0, multiplier=1.0) == sf.LETHAL_OBSTACLE
    assert sf.mask_cost(100, base=0.0, multiplier=0.5) == pytest.approx(127, abs=1)


def test_keepout_only_raises_costs_and_leaves_unknown_mask_cells_alone():
    master = np.array([[0, 200, sf.NO_INFORMATION]], dtype=np.uint8)
    mask = np.array([[100, 10, -1]], dtype=np.int8)
    out = sf.apply_keepout(master, mask)
    assert out.tolist() == [[254, 200, int(sf.NO_INFORMATION)]]


def test_keepout_can_turn_unknown_space_into_known_cost():
    master = np.array([[sf.NO_INFORMATION]], dtype=np.uint8)
    out = sf.apply_keepout(master, np.array([[10]], dtype=np.int8))
    assert out[0, 0] == sf.mask_cost(10) < sf.NO_INFORMATION


def test_the_master_costmap_is_not_modified_in_place():
    master = np.zeros((2, 2), dtype=np.uint8)
    sf.apply_keepout(master, np.full((2, 2), 100, dtype=np.int8))
    assert master.max() == 0


def test_speed_filter_percentages():
    assert sf.speed_limit(0) == pytest.approx(0.3), "0 means no limit"
    assert sf.speed_limit(100) == pytest.approx(0.3)
    assert sf.speed_limit(30) == pytest.approx(0.09)
    # absolute mode (filter type 2): base and multiplier turn the 0..100 mask into m/s
    assert sf.speed_limit(50, base=0.0, multiplier=0.003, percentage=False) == pytest.approx(0.15)
    assert sf.speed_limit(100, base=0.0, multiplier=0.003, percentage=False) == pytest.approx(0.3)
    assert sf.speed_limit(50, percentage=False) == pytest.approx(0.3), "multiplier 1 -> 50 m/s, clamped"


# --- the apartment demo --------------------------------------------------------------------------
def test_a_keepout_zone_reroutes_the_global_plan():
    import costmap as cm
    import grid_planning as gp
    import nav_common
    from robotlab.config import load_config
    from robotlab.sim import World

    cfg = load_config()
    grid = World.apartment().to_occupancy_grid(nav_common.GRID_RESOLUTION)
    costs = cm.inflate(cm.static_layer(grid), grid.resolution, cfg.chassis.footprint_radius_m, 3.0, 0.8)
    mask = sf.rasterize(grid, sf.KEEPOUT_ZONES)
    assert (mask == 100).sum() == 448

    def length(costmap):
        blocked, extra = cm.cost_to_planner_penalty(costmap)
        result = gp.astar(blocked, grid.world_to_cell(*nav_common.START_XY),
                          grid.world_to_cell(*nav_common.STUDY_XY), cell_cost=extra)
        return gp.path_cost(result.path) * grid.resolution if result.path else math.inf

    assert length(costs) == pytest.approx(3.86, abs=0.02)
    assert length(sf.apply_keepout(costs, mask)) == pytest.approx(9.26, abs=0.05)
    assert length(sf.apply_keepout(costs, sf.rasterize(grid, sf.KEEPOUT_OVER_THE_GOAL))) == math.inf
