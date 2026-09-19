"""Tests for costmap.py (lesson 12.04)."""

import math

import numpy as np
import pytest

import costmap as cm
import nav_common
from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, OccupancyGrid, SensorParams, World

NAV2_FOOTPRINT = [(0.125, 0.105), (0.125, -0.105), (-0.125, -0.105), (-0.125, 0.105)]


def test_footprint_numbers_quoted_in_the_lesson():
    assert cm.footprint_radii(NAV2_FOOTPRINT) == pytest.approx((0.105, 0.16325), abs=1e-5)
    assert cm.footprint_radii(cm.pad_footprint(NAV2_FOOTPRINT, 0.01)) == pytest.approx((0.115, 0.17734), abs=1e-5)
    cfg = load_config()
    hl, hw = cfg.chassis.length_m / 2, cfg.chassis.width_m / 2
    assert cm.footprint_radii([(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)])[1] == pytest.approx(cfg.chassis.footprint_radius_m)


@pytest.mark.parametrize(("d", "csf", "cost"), [(0.0, 5, 254), (0.10, 5, 253), (0.15, 5, 211), (0.20, 5, 164), (0.25, 5, 128),
                                                (0.35, 5, 77), (0.15, 10, 177), (0.35, 10, 24), (0.45, 2, 128)])
def test_inflation_table(d, csf, cost):
    assert cm.inflation_cost(d, 0.115, csf) == cost


def test_distance_for_cost():
    assert cm.distance_for_cost(128, 0.115, 5.0) == pytest.approx(0.250, abs=1e-3)
    assert cm.distance_for_cost(10, 0.115, 10.0) == pytest.approx(0.438, abs=1e-3)


def test_inflate_single_cell_and_unknown_rule():
    master = np.zeros((1, 12), np.uint8)
    master[0, 0] = cm.LETHAL_OBSTACLE
    master[0, 1] = cm.NO_INFORMATION
    master[0, 5] = cm.NO_INFORMATION
    out = cm.inflate(master, 0.05, 0.1, 5.0, 0.33)
    assert list(out[0, :9]) == [254, 253, 253, 196, 152, 255, 92, 72, 0]


def test_obstacle_layer_sees_the_unmapped_basket_and_clears_free_space():
    cfg = load_config()
    world = World.apartment()
    clutter = World.from_segments(world.segments, np.vstack([world.circles, [[2.2, 1.5, 0.15]]]))
    sim = DiffDriveSim(clutter, DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg), pose=nav_common.START_POSE, seed=0)
    grid = OccupancyGrid(np.full((120, 140), -1, np.int8), 0.05, (-0.5, -0.5))
    layer = cm.obstacle_layer(grid, sim.lidar_pose, sim.lidar_scan())
    r, c = grid.world_to_cell(2.2 - 0.15, 1.5)  # the basket's near side
    assert (layer[r - 1 : r + 2, c - 1 : c + 2] == cm.LETHAL_OBSTACLE).any()
    assert layer[grid.world_to_cell(1.6, 1.4)] == cm.FREE_SPACE  # between robot and basket
    assert layer[grid.world_to_cell(-0.3, -0.3)] == cm.NO_INFORMATION  # outside the walls: never seen
    master = cm.combine_max(cm.static_layer(grid), layer)
    assert master[grid.world_to_cell(1.6, 1.4)] == cm.FREE_SPACE  # unknown in the map, cleared by the scan


def test_cost_aware_planning_keeps_away_from_walls():
    import grid_planning as gp

    cfg = load_config()
    world = World.apartment()
    grid = world.to_occupancy_grid(0.05)
    costs = cm.inflate(cm.static_layer(grid), 0.05, cfg.chassis.footprint_radius_m, 3.0, 0.8)
    blocked, extra = cm.cost_to_planner_penalty(costs)
    s, g = grid.world_to_cell(*nav_common.START_XY), grid.world_to_cell(*nav_common.KITCHEN_XY)
    plain, aware = gp.astar(blocked, s, g), gp.astar(blocked, s, g, cell_cost=extra)
    clearance = lambda res: world.distance_to_obstacles(gp.cells_to_world(grid, res.path)).min()  # noqa: E731
    assert gp.path_cost(aware.path) > gp.path_cost(plain.path)
    assert clearance(aware) > clearance(plain) + 0.1
    assert math.isfinite(aware.cost)
