"""Tests for the module-11 lesson code (fast: no full apartment tour).

    py -m pytest 11-slam/code
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from icp import (
    best_fit_transform,
    corridor_case,
    icp,
    icp_coarse_to_fine,
    icp_point_to_line,
    match_is_reliable,
    simulated_scan,
)
from map_representations import memory_table
from occupancy_grid_mapping import LogOddsGrid, bresenham, bresenham_many, compare_with_truth, logit
from pose_graph import Edge, edge_error, edge_jacobians, information_from_sigmas, optimize, optimize_with_scipy
from slam_common import SE2, World, record_tour, position_rmse, trajectory_array
from robotlab.sim import LaserScan, OccupancyGrid


# --- 11.01 -------------------------------------------------------------------------------------------
def test_memory_math_for_a_home_at_5_cm():
    table = {name.split(" (")[0]: nbytes for name, _, nbytes in memory_table(12.0, 10.0, 2.6, 0.05)}
    assert table["2D occupancy grid, int8"] == 48_000
    assert table["3D voxel grid, int8"] == 2_496_000
    assert table["raw 2D LiDAR points, 10 min at 10 Hz"] == 17_280_000


# --- 11.02 -------------------------------------------------------------------------------------------
def test_bresenham_examples():
    assert bresenham(0, 0, 5, 2) == [(0, 0), (1, 0), (2, 1), (3, 1), (4, 2), (5, 2)]
    assert bresenham(3, 3, 3, 3) == [(3, 3)]


def test_vectorized_rays_match_bresenham_almost_always():
    rng = np.random.default_rng(0)
    same = 0
    for _ in range(1000):
        x0, y0, x1, y1 = (int(v) for v in rng.integers(-40, 40, 4))
        xs, ys, _ = bresenham_many(x0, y0, np.array([x1]), np.array([y1]))
        same += bresenham(x0, y0, x1, y1) == list(zip(xs.tolist(), ys.tolist()))
    assert same >= 990


def test_logodds_grid_maps_a_room_and_round_trips_through_pgm(tmp_path):
    world = World.rectangle_room(3.0, 2.0)
    truth = world.to_occupancy_grid(0.05, 0.5)
    grid = LogOddsGrid.like(truth)
    angles = -math.pi + np.arange(360) * (2 * math.pi / 360)
    for x, y, th in [(1.0, 1.0, 0.0), (2.0, 0.7, 1.0), (2.2, 1.4, -2.0)]:
        r = world.raycast((x, y), angles + th, 12.0)
        grid.integrate_scan(SE2(x, y, th), LaserScan(-math.pi, 2 * math.pi / 360, 0.15, 12.0, r))
    occ = grid.to_occupancy_grid()
    q = compare_with_truth(occ, truth)
    assert q["occupied_on_a_wall"] > 0.99 and q["free_really_free"] > 0.95 and q["occupied_cells"] > 150
    assert grid.logodds.max() <= 4.0 and grid.logodds.min() >= -4.0
    assert logit(0.7) == pytest.approx(0.8473, abs=1e-4)
    path = occ.save(tmp_path / "room.yaml")
    assert np.array_equal(OccupancyGrid.load(path).data, occ.data)


# --- 11.04 -------------------------------------------------------------------------------------------
def test_kabsch_example():
    src = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]])
    true = SE2(1.0, 0.5, math.radians(30))
    T = best_fit_transform(src, true.apply(src))
    assert T.as_tuple() == pytest.approx(true.as_tuple(), abs=1e-12)


@pytest.mark.parametrize("method", ["plain", "coarse_to_fine", "point_to_line"])
def test_icp_variants_on_apartment_scans(method):
    world = World.apartment()
    a, b = SE2(1.0, 1.3, 0.0), SE2(1.35, 1.45, math.radians(12))
    truth = a.between(b)
    src, dst = simulated_scan(world, b, 0.01, seed=1), simulated_scan(world, a, 0.01, seed=2)
    run = {"plain": icp, "coarse_to_fine": icp_coarse_to_fine, "point_to_line": icp_point_to_line}[method]
    res = run(src, dst, SE2())
    tol = {"plain": 0.03, "coarse_to_fine": 0.01, "point_to_line": 0.01}[method]
    assert res.transform.distance_to(truth) < tol
    assert abs(res.transform.theta - truth.theta) < math.radians(0.5)
    assert match_is_reliable(res)


def test_coarse_to_fine_has_a_wide_basin():
    world = World.apartment()
    a, b = SE2(1.0, 1.3, 0.0), SE2(1.2, 1.4, math.radians(5))
    truth = a.between(b)
    guess = SE2(truth.x + 0.7, truth.y - 0.5, truth.theta + math.radians(35))
    res = icp_coarse_to_fine(simulated_scan(world, b), simulated_scan(world, a), guess)
    assert res.transform.distance_to(truth) < 0.02


def test_corridor_is_degenerate():
    cases = corridor_case()
    truth, res, ratio = cases["corridor"]
    assert abs(res.transform.x - truth.x) > 0.3, "along a featureless corridor ICP cannot see the motion"
    assert ratio < 0.1
    truth, res, ratio = cases["living room"]
    assert abs(res.transform.x - truth.x) < 0.03 and ratio > 0.5


# --- 11.05 -------------------------------------------------------------------------------------------
def test_edge_jacobians_numerically():
    rng = np.random.default_rng(3)
    xi, xj = rng.uniform(-2, 2, 3), rng.uniform(-2, 2, 3)
    z = SE2(0.3, -0.2, 0.4)
    A, B = edge_jacobians(xi, xj, z)
    h = 1e-6
    for k in range(3):
        d = np.eye(3)[k] * h
        assert A[:, k] == pytest.approx((edge_error(xi + d, xj, z) - edge_error(xi - d, xj, z)) / (2 * h), abs=1e-6)
        assert B[:, k] == pytest.approx((edge_error(xi, xj + d, z) - edge_error(xi, xj - d, z)) / (2 * h), abs=1e-6)


def test_gauss_newton_matches_scipy_and_closes_a_loop():
    truth = np.array([[0, 0, 0], [1, 0, math.pi / 2], [1, 1, math.pi], [0, 1, -math.pi / 2], [0, 0, 0]], dtype=float)
    rng = np.random.default_rng(0)
    edges, chain = [], [SE2(*truth[0])]
    for k in range(4):
        z = SE2(*truth[k]).between(SE2(*truth[k + 1]))
        z = SE2(z.x + rng.normal(0, 0.02), z.y + rng.normal(0, 0.02), z.theta + 0.08)
        edges.append(Edge(k, k + 1, z, information_from_sigmas(0.05, 0.05)))
        chain.append(chain[-1] @ z)
    edges.append(Edge(4, 0, SE2(), information_from_sigmas(0.01, 0.01), "loop"))
    initial = trajectory_array(chain)
    ours, history, _ = optimize(initial, edges)
    theirs = optimize_with_scipy(initial, edges)
    assert history[-1] < 0.05 * history[0]
    assert ours[:, :2] == pytest.approx(theirs[:, :2], abs=1e-4)
    assert position_rmse(ours, truth) < 0.5 * position_rmse(initial, truth)


def test_record_tour_in_a_small_room_is_deterministic():
    room = World.rectangle_room(3.0, 2.0)
    kwargs = dict(realistic=True, seed=5, waypoints=[(2.2, 0.6), (2.2, 1.4), (0.8, 1.0)], start=SE2(0.8, 0.6, 0.0), world=room)
    a, b = record_tour(**kwargs), record_tour(**kwargs)
    assert len(a.scans) == len(b.scans) > 20
    assert a.odom[-1] == b.odom[-1]
    assert a.truth[-1].distance_to(SE2(0.8, 1.0, 0.0)) < 0.1
    assert a.keyframes()[0] == 0 and a.keyframes()[-1] == len(a.scans) - 1
