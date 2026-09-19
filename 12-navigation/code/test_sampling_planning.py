"""Tests for sampling_planning.py (lesson 12.03)."""

import numpy as np
import pytest

import nav_common
import sampling_planning as sp
from robotlab.config import load_config
from robotlab.sim import World


@pytest.fixture(scope="module")
def checker():
    return sp.DiskRobotChecker(World.apartment(), load_config().chassis.footprint_radius_m)


BOUNDS = np.array([[0.0, 6.0], [0.0, 5.0]])


def test_rrt_finds_a_collision_free_path(checker):
    start, goal = np.array(nav_common.START_XY), np.array(nav_common.KITCHEN_XY)
    res = sp.rrt(start, goal, BOUNDS, checker.is_free, checker.motion_free, np.random.default_rng(0))
    assert res.path is not None
    assert np.allclose(res.path[0], start) and np.linalg.norm(res.path[-1] - goal) <= 0.15
    assert all(checker.motion_free(a, b) for a, b in zip(res.path[:-1], res.path[1:]))
    assert sp.path_length(res.path) >= np.linalg.norm(goal - start)


def test_rrt_star_improves_on_rrt(checker):
    start, goal = np.array(nav_common.START_XY), np.array(nav_common.KITCHEN_XY)
    rrt = sp.rrt(start, goal, BOUNDS, checker.is_free, checker.motion_free, np.random.default_rng(0))
    star = sp.rrt(start, goal, BOUNDS, checker.is_free, checker.motion_free, np.random.default_rng(0),
                  star=True, keep_improving=True, max_iterations=1500)
    costs = [c for _, c in star.cost_history]
    assert costs == sorted(costs, reverse=True), "RRT* best cost never increases"
    assert costs[-1] < sp.path_length(rrt.path)
    assert costs[-1] == pytest.approx(sp.path_length(star.path))
    assert costs[-1] < 4.2  # the straight-line distance is 4.12 m


def test_start_in_collision_returns_no_path(checker):
    res = sp.rrt(np.array([0.05, 2.0]), np.array([1.0, 1.3]), BOUNDS, checker.is_free, checker.motion_free, np.random.default_rng(0))
    assert res.path is None


def test_two_link_arm_collision_model():
    arm = sp.TwoLinkArm()
    assert arm.is_free(np.radians([-100.0, 20.0]))
    assert not arm.is_free(np.radians([40.0, 0.0]))  # link 1 along 40 deg passes the obstacle at (0.30, 0.25)
    tip = arm.points(np.array([0.0, 0.0]))[-1]
    assert tip == pytest.approx([0.55, 0.0])


def test_grid_cells_needed():
    assert sp.grid_cells_needed(2, 5) == pytest.approx(72**2)
    assert sp.grid_cells_needed(6, 1) == pytest.approx(360.0**6)


def test_narrow_passage_world_geometry():
    radius = load_config().chassis.footprint_radius_m
    w = sp.narrow_passage_world(0.5)
    assert not w.collides(2.0, 1.5, radius)  # the corridor's centre line is free for karmel
    assert w.collides(2.0, 1.1, radius)
    assert w.raycast((0.5, 1.5), [0.0], 5.0)[0] == pytest.approx(3.5)  # straight through the corridor to the far wall
