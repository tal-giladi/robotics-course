"""Checker for 12.10 — the collision monitor and the keepout filter.

Run: ``python course.py check 12.10`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robotlab.config import load_config


def wall(distance_m: float, half_width: float = 0.6, count: int = 60) -> np.ndarray:
    """A wall across the robot's path, ``distance_m`` ahead, as LiDAR points in base_link."""
    ys = np.linspace(-half_width, half_width, count)
    return np.column_stack([np.full(count, distance_m), ys])


def rectangle(length: float, width: float) -> np.ndarray:
    hl, hw = length / 2, width / 2
    return np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])


def karmel_footprint(impl):
    """karmel's shipped FootprintApproach polygon, from labs/config/karmel.yaml + 1 cm padding."""
    cfg = load_config()
    return impl.SafetyPolygon(
        "FootprintApproach",
        rectangle(cfg.chassis.length_m + 0.02, cfg.drive.wheel_separation_m + cfg.drive.wheel_width_m + 0.02),
        impl.ActionType.APPROACH, min_points=6, time_before_collision=1.2, simulation_time_step=0.1)


# --- points_in_polygon ------------------------------------------------------------------------
def test_counts_only_the_points_inside(impl):
    polygon = rectangle(0.4, 0.4)
    points = np.array([[0.0, 0.0], [0.1, 0.1], [0.5, 0.0], [-0.5, 0.0], [0.0, 0.19]])
    assert impl.points_in_polygon(polygon, points) == 3


def test_an_empty_cloud_is_never_inside(impl):
    assert impl.points_in_polygon(rectangle(1.0, 1.0), np.zeros((0, 2))) == 0


def test_a_triangle_is_handled_as_well_as_a_rectangle(impl):
    triangle = np.array([[0.3, 0.0], [-0.1, 0.2], [-0.1, -0.2]])
    assert impl.points_in_polygon(triangle, np.array([[0.0, 0.0]])) == 1
    assert impl.points_in_polygon(triangle, np.array([[0.25, 0.15]])) == 0


def test_all_wall_points_within_the_width_count(impl):
    # 61 points 2 cm apart from -0.60 to +0.60 m; 25 of them have |y| < 0.25 m
    assert impl.points_in_polygon(rectangle(1.0, 0.5), wall(0.1, half_width=0.6, count=61)) == 25


# --- collision_time ---------------------------------------------------------------------------
def test_a_far_wall_never_collides(impl):
    assert karmel_footprint(impl).collision_time(wall(1.5), impl.Velocity(0.25, 0.0)) == -1.0


def test_a_wall_inside_the_footprint_collides_now(impl):
    assert karmel_footprint(impl).collision_time(wall(0.10), impl.Velocity(0.25, 0.0)) == 0.0


def test_collision_time_is_a_multiple_of_the_simulation_step(impl):
    polygon = karmel_footprint(impl)
    for distance in (0.2, 0.3, 0.4):
        t = polygon.collision_time(wall(distance), impl.Velocity(0.25, 0.0))
        assert t >= 0.0 and t == pytest.approx(round(t / 0.1) * 0.1, abs=1e-9)


def test_collision_time_grows_with_distance(impl):
    polygon = karmel_footprint(impl)
    times = [polygon.collision_time(wall(d), impl.Velocity(0.25, 0.0)) for d in (0.2, 0.3, 0.4)]
    assert times == sorted(times)
    assert times == pytest.approx([0.2, 0.6, 1.0], abs=1e-9), \
        "0.25 m/s, footprint front at 0.135 m, one step of lag"


def test_a_slower_command_sees_no_collision_inside_the_horizon(impl):
    assert karmel_footprint(impl).collision_time(wall(0.4), impl.Velocity(0.10, 0.0)) == -1.0


def test_fewer_points_than_min_points_never_trigger(impl):
    sparse = np.array([[0.2, 0.0], [0.2, 0.02]])
    assert karmel_footprint(impl).collision_time(sparse, impl.Velocity(0.25, 0.0)) == -1.0


def test_a_turning_robot_sweeps_a_different_path(impl):
    """The projection follows the commanded arc, not a straight line."""
    polygon = karmel_footprint(impl)
    side_wall = np.column_stack([np.linspace(0.0, 0.6, 40), np.full(40, 0.30)])
    assert polygon.collision_time(side_wall, impl.Velocity(0.25, 0.0)) == -1.0, "straight: passes by"
    assert polygon.collision_time(side_wall, impl.Velocity(0.25, 0.8)) == -1.0, "gentle turn: still clear"
    assert polygon.collision_time(side_wall, impl.Velocity(0.25, 1.2)) == pytest.approx(1.1, abs=1e-9)
    assert polygon.collision_time(side_wall, impl.Velocity(0.25, 2.0)) == pytest.approx(0.9, abs=1e-9)


# --- the four actions ---------------------------------------------------------------------------
def test_no_obstacle_means_no_action(impl):
    monitor = impl.CollisionMonitor([karmel_footprint(impl)])
    action = monitor.process(impl.Velocity(0.25, 0.0), wall(1.5))
    assert action.action_type is impl.ActionType.NONE
    assert action.velocity.v == pytest.approx(0.25) and action.polygon_name == ""


def test_approach_scales_the_command_by_time_to_collision(impl):
    monitor = impl.CollisionMonitor([karmel_footprint(impl)])
    action = monitor.process(impl.Velocity(0.25, 0.0), wall(0.40))
    assert action.action_type is impl.ActionType.APPROACH
    assert action.velocity.v == pytest.approx(0.25 * 1.0 / 1.2)
    assert monitor.process(impl.Velocity(0.25, 0.0), wall(0.10)).velocity.v == 0.0


def test_stop_zeroes_both_components(impl):
    zone = impl.SafetyPolygon("stop", rectangle(0.8, 0.5), impl.ActionType.STOP, min_points=4)
    action = impl.CollisionMonitor([zone]).process(impl.Velocity(0.25, 0.8), wall(0.3))
    assert (action.velocity.v, action.velocity.w) == (0.0, 0.0)
    assert action.action_type is impl.ActionType.STOP and action.polygon_name == "stop"


def test_slowdown_scales_both_components(impl):
    zone = impl.SafetyPolygon("slow", rectangle(0.9, 0.6), impl.ActionType.SLOWDOWN,
                              min_points=4, slowdown_ratio=0.4)
    action = impl.CollisionMonitor([zone]).process(impl.Velocity(0.25, 0.8), wall(0.3))
    assert action.velocity.v == pytest.approx(0.10)
    assert action.velocity.w == pytest.approx(0.32)


def test_limit_clamps_each_component_and_keeps_the_sign(impl):
    zone = impl.SafetyPolygon("limit", rectangle(1.0, 1.0), impl.ActionType.LIMIT,
                              min_points=4, linear_limit=0.1, angular_limit=0.4)
    action = impl.CollisionMonitor([zone]).process(impl.Velocity(-0.25, 0.8), wall(0.3))
    assert action.velocity.v == pytest.approx(-0.1)
    assert action.velocity.w == pytest.approx(0.4)


def test_limit_does_not_speed_a_slow_command_up(impl):
    zone = impl.SafetyPolygon("limit", rectangle(1.0, 1.0), impl.ActionType.LIMIT,
                              min_points=4, linear_limit=0.4, angular_limit=1.0)
    action = impl.CollisionMonitor([zone]).process(impl.Velocity(0.05, 0.1), wall(0.3))
    assert action.velocity.v == pytest.approx(0.05) and action.velocity.w == pytest.approx(0.1)


def test_the_slowest_requirement_of_all_polygons_wins(impl):
    monitor = impl.CollisionMonitor([
        impl.SafetyPolygon("stop", rectangle(0.55, 0.45), impl.ActionType.STOP, min_points=4),
        impl.SafetyPolygon("slow", rectangle(1.2, 0.8), impl.ActionType.SLOWDOWN,
                           min_points=4, slowdown_ratio=0.5),
    ])
    assert monitor.process(impl.Velocity(0.25, 0.0), wall(0.5)).action_type is impl.ActionType.SLOWDOWN
    assert monitor.process(impl.Velocity(0.25, 0.0), wall(0.25)).action_type is impl.ActionType.STOP


def test_a_disabled_polygon_is_ignored(impl):
    zone = impl.SafetyPolygon("stop", rectangle(0.8, 0.5), impl.ActionType.STOP,
                              min_points=4, enabled=False)
    assert impl.CollisionMonitor([zone]).process(impl.Velocity(0.25, 0.0),
                                                 wall(0.3)).velocity.v == pytest.approx(0.25)


def test_an_empty_monitor_passes_the_command_through(impl):
    action = impl.CollisionMonitor([]).process(impl.Velocity(0.3, -0.4), wall(0.1))
    assert (action.velocity.v, action.velocity.w) == (0.3, -0.4)


# --- the keepout filter --------------------------------------------------------------------------
def test_mask_values_convert_to_costs(impl):
    assert impl.mask_cost(-1) == impl.NO_INFORMATION
    assert impl.mask_cost(0) == 0
    assert impl.mask_cost(100) == impl.LETHAL_OBSTACLE
    assert impl.mask_cost(50) == pytest.approx(127, abs=1)


def test_base_and_multiplier_move_the_mask_into_filter_space(impl):
    assert impl.mask_cost(50, base=50.0, multiplier=1.0) == impl.LETHAL_OBSTACLE
    assert impl.mask_cost(100, base=0.0, multiplier=0.5) == pytest.approx(127, abs=1)
    assert impl.mask_cost(500) == impl.LETHAL_OBSTACLE, "clamped to 100 %"


def test_keepout_raises_costs_and_respects_unknown_mask_cells(impl):
    master = np.array([[0, 200, impl.NO_INFORMATION]], dtype=np.uint8)
    mask = np.array([[100, 10, -1]], dtype=np.int8)
    assert impl.apply_keepout(master, mask).tolist() == [[254, 200, int(impl.NO_INFORMATION)]]


def test_keepout_can_turn_unknown_space_into_a_lower_known_cost(impl):
    out = impl.apply_keepout(np.array([[impl.NO_INFORMATION]], dtype=np.uint8),
                             np.array([[10]], dtype=np.int8))
    assert out[0, 0] == impl.mask_cost(10) < impl.NO_INFORMATION


def test_the_master_costmap_is_not_modified_in_place(impl):
    master = np.zeros((3, 3), dtype=np.uint8)
    impl.apply_keepout(master, np.full((3, 3), 100, dtype=np.int8))
    assert master.max() == 0


def test_a_keepout_zone_blocks_a_doorway_for_the_planner(impl):
    """A 2 m corridor with a 1 m door; a keepout over the door must make every door cell lethal."""
    master = np.zeros((40, 40), dtype=np.uint8)
    mask = np.zeros((40, 40), dtype=np.int8)
    mask[18:22, 10:30] = 100
    out = impl.apply_keepout(master, mask)
    assert (out[18:22, 10:30] == impl.LETHAL_OBSTACLE).all()
    assert (out[0:18] == 0).all() and (out[22:] == 0).all()


def test_the_whole_pipeline_on_karmels_numbers(impl):
    """Command 0.25 m/s at a wall; check where the robot first slows and where it stops."""
    monitor = impl.CollisionMonitor([karmel_footprint(impl)])
    speeds = {d: monitor.process(impl.Velocity(0.25, 0.0), wall(d)).velocity.v
              for d in (1.0, 0.5, 0.4, 0.3, 0.2, 0.15)}
    assert speeds[1.0] == pytest.approx(0.25) and speeds[0.5] == pytest.approx(0.25)
    assert speeds[0.15] == 0.0
    assert list(speeds.values()) == sorted(speeds.values(), reverse=True)
    assert speeds[0.3] == pytest.approx(0.125), "0.6 s to collision / 1.2 s horizon"


def test_the_monitor_never_makes_the_command_faster(impl):
    monitor = impl.CollisionMonitor([karmel_footprint(impl)])
    for distance in np.linspace(0.14, 1.2, 25):
        assert monitor.process(impl.Velocity(0.25, 0.0), wall(float(distance))).velocity.v <= 0.25 + 1e-12
    assert math.isclose(monitor.process(impl.Velocity(0.0, 0.0), wall(0.2)).velocity.v, 0.0)
