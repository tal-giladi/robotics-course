"""Tests for grid_planning.py (lesson 12.02)."""

import math

import numpy as np
import pytest

import grid_planning as gp
import nav_common
from robotlab.config import load_config
from robotlab.sim import World

SQRT2 = math.sqrt(2.0)


def tiny():
    return np.array([[ch == "#" for ch in row] for row in gp.TINY_MAP])


def test_tiny_map_numbers_quoted_in_the_lesson():
    blocked = tiny()
    b, d, a = gp.bfs(blocked, (0, 2), (4, 8)), gp.dijkstra(blocked, (0, 2), (4, 8)), gp.astar(blocked, (0, 2), (4, 8))
    assert b.cost == pytest.approx(4 + 4 * SQRT2)  # 9.657: fewest moves, not shortest
    assert d.cost == pytest.approx(6 + 2 * SQRT2) == a.cost  # 8.828
    assert len(b.path) - 1 == len(a.path) - 1 == 8
    assert (b.expanded, d.expanded, a.expanded) == (53, 52, 14)
    assert gp.octile((0, 2), (4, 8)) == pytest.approx(7.657, abs=1e-3)


@pytest.fixture(scope="module")
def apartment():
    grid = World.apartment().to_occupancy_grid(nav_common.GRID_RESOLUTION)
    return grid, gp.configuration_space(grid, load_config().chassis.footprint_radius_m)


def test_apartment_planners(apartment):
    grid, blocked = apartment
    s, g = grid.world_to_cell(*nav_common.START_XY), grid.world_to_cell(*nav_common.KITCHEN_XY)
    d, a, w = gp.dijkstra(blocked, s, g), gp.astar(blocked, s, g), gp.astar(blocked, s, g, weight=2.0)
    assert a.cost == pytest.approx(d.cost)
    assert d.cost * grid.resolution == pytest.approx(4.394, abs=1e-3)
    assert a.expanded < d.expanded / 10
    assert w.cost >= a.cost
    smooth = gp.shortcut(blocked, a.path)
    assert gp.path_cost(smooth) < a.cost
    assert all(gp.line_of_sight(blocked, p, q) for p, q in zip(smooth, smooth[1:]))


def test_no_path_and_blocked_goal():
    blocked = np.zeros((5, 5), bool)
    blocked[1:4, 1:4] = True
    blocked[2, 2] = False
    assert gp.astar(blocked, (0, 0), (2, 2)).path is None
    assert gp.bfs(blocked, (0, 0), (1, 1)).path is None


def test_configuration_space_grows_obstacles(apartment):
    grid, blocked = apartment
    # a free cell 0.10 m from a wall is blocked for a 0.16 m robot; 0.5 m away is free
    assert blocked[grid.world_to_cell(0.10, 2.0)]
    assert not blocked[grid.world_to_cell(0.60, 2.0)]


def test_densify_spacing():
    pts = gp.densify(np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 0.3]]), 0.05)
    assert np.max(np.linalg.norm(np.diff(pts, axis=0), axis=1)) <= 0.05 + 1e-12
    assert pts[-1] == pytest.approx([1.0, 0.3])
