"""Checker for 11.05 — Pose graphs and loop closure.

Run: ``python course.py check 11.05`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robotlab.config import load_config
from robotlab.geometry import SE2, angle_diff
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World, arc_update

approx = pytest.approx


def info(sigma_xy: float, sigma_theta: float) -> np.ndarray:
    return np.diag([sigma_xy**-2, sigma_xy**-2, sigma_theta**-2])


def rmse(estimate: np.ndarray, truth: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.sum((np.asarray(estimate)[:, :2] - truth[:, :2]) ** 2, axis=1))))


# --- relative pose and edge error --------------------------------------------------------------------
def test_relative_pose(impl):
    # j is 1 m "ahead" of i when i faces +y.
    assert impl.relative_pose(np.array([1.0, 2.0, math.pi / 2]), np.array([1.0, 3.0, math.pi / 2])) == approx([1.0, 0.0, 0.0], abs=1e-12)
    assert impl.relative_pose(np.array([0.0, 0.0, 0.0]), np.array([1.0, 1.0, math.pi / 2])) == approx([1.0, 1.0, math.pi / 2])
    # 170 deg -> -170 deg is a +20 deg turn, not -340.
    rel = impl.relative_pose(np.array([0.0, 0.0, math.radians(170)]), np.array([0.0, 0.0, math.radians(-170)]))
    assert rel[2] == approx(math.radians(20))


def test_edge_error_is_zero_for_a_consistent_edge(impl):
    xi, xj = np.array([0.3, -1.2, 2.5]), np.array([1.1, 0.4, -2.9])
    z = impl.relative_pose(xi, xj)
    assert impl.edge_error(xi, xj, z) == approx([0.0, 0.0, 0.0], abs=1e-12)


def test_edge_error_hand_examples(impl):
    # Measured 1.0 m ahead, actually 1.1 m ahead: 10 cm error along x.
    assert impl.edge_error(np.zeros(3), np.array([1.1, 0.0, 0.0]), np.array([1.0, 0.0, 0.0])) == approx([0.1, 0.0, 0.0])
    # The error is expressed in the MEASUREMENT frame: z = (1, 0, 90 deg), true j = (1, 1, 90 deg).
    # R(90)ᵀ · ((1, 1) - (1, 0)) = R(-90) · (0, 1) = (1, 0); angles agree.
    e = impl.edge_error(np.zeros(3), np.array([1.0, 1.0, math.pi / 2]), np.array([1.0, 0.0, math.pi / 2]))
    assert e == approx([1.0, 0.0, 0.0], abs=1e-12)
    # Angle error wraps: 179 deg measured, -179 deg actual = 2 deg error, not -358.
    e = impl.edge_error(np.zeros(3), np.array([0.0, 0.0, math.radians(-179)]), np.array([0.0, 0.0, math.radians(179)]))
    assert e[2] == approx(math.radians(2))


def test_jacobians_match_finite_differences(impl):
    rng = np.random.default_rng(5)
    for _ in range(10):
        xi = rng.uniform([-3, -3, -3], [3, 3, 3])
        xj = rng.uniform([-3, -3, -3], [3, 3, 3])
        z = impl.relative_pose(xi, xj) + rng.normal(0, 0.1, 3)
        A, B = impl.edge_jacobians(xi, xj, z)
        h = 1e-6
        for k in range(3):
            d = np.zeros(3)
            d[k] = h
            num_a = (impl.edge_error(xi + d, xj, z) - impl.edge_error(xi - d, xj, z)) / (2 * h)
            num_b = (impl.edge_error(xi, xj + d, z) - impl.edge_error(xi, xj - d, z)) / (2 * h)
            assert np.asarray(A)[:, k] == approx(num_a, abs=1e-5), f"column {k} of A (d e / d xi) is wrong"
            assert np.asarray(B)[:, k] == approx(num_b, abs=1e-5), f"column {k} of B (d e / d xj) is wrong"


# --- optimization ------------------------------------------------------------------------------------
def odometry_chain(impl, edges, start: np.ndarray) -> np.ndarray:
    """Dead reckoning: compose every odometry measurement onto the start pose."""
    poses = [np.asarray(start, dtype=float)]
    for edge in edges:
        x = poses[-1]
        t = x[:2] + np.array([[math.cos(x[2]), -math.sin(x[2])], [math.sin(x[2]), math.cos(x[2])]]) @ edge.z[:2]
        poses.append(np.array([t[0], t[1], math.atan2(math.sin(x[2] + edge.z[2]), math.cos(x[2] + edge.z[2]))]))
    return np.array(poses)


def square_loop(impl, seed: int = 0):
    """A 2 m square driven in 20 steps, odometry with a +2 deg/turn heading bias and noise, and one
    loop-closure edge from the last node back to the first (the robot is where it started)."""
    rng = np.random.default_rng(seed)
    truth = []
    for side in range(4):
        heading = side * math.pi / 2
        corner = np.array([[0, 0], [2, 0], [2, 2], [0, 2]][side], dtype=float)
        for k in range(5):
            p = corner + 0.4 * k * np.array([math.cos(heading), math.sin(heading)])
            truth.append([p[0], p[1], heading])
    truth.append([0.0, 0.0, 2 * math.pi])
    truth = np.array(truth)
    truth[:, 2] = np.arctan2(np.sin(truth[:, 2]), np.cos(truth[:, 2]))
    odom_edges = []
    for k in range(len(truth) - 1):
        z = impl.relative_pose(truth[k], truth[k + 1]) + rng.normal(0, [0.01, 0.01, 0.005])
        z[2] += 0.035 * abs(z[2]) / (math.pi / 2) + 0.005  # biased gyro/wheelbase: every turn over-counted
        odom_edges.append(impl.Edge(k, k + 1, z, info(0.05, math.radians(3))))
    loop = impl.Edge(len(truth) - 1, 0, impl.relative_pose(truth[-1], truth[0]), info(0.01, math.radians(0.5)))
    return truth, odom_edges, loop


def test_consistent_graph_does_not_move(impl):
    truth, _, _ = square_loop(impl)
    edges = [impl.Edge(k, k + 1, impl.relative_pose(truth[k], truth[k + 1]), info(0.05, 0.05)) for k in range(len(truth) - 1)]
    out = impl.optimize_pose_graph(truth.copy(), edges)
    assert np.asarray(out) == approx(truth, abs=1e-6)


def test_first_node_is_the_anchor_and_input_is_not_modified(impl):
    truth, odom, loop = square_loop(impl)
    initial = odometry_chain(impl, odom, truth[0])
    before = initial.copy()
    out = impl.optimize_pose_graph(initial, odom + [loop])
    assert np.array_equal(initial, before), "return a new array; do not modify the input poses"
    assert np.asarray(out)[0] == approx(truth[0], abs=1e-6), "node 0 must not move"


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_one_loop_closure_fixes_the_square(impl, seed):
    truth, odom, loop = square_loop(impl, seed)
    initial = odometry_chain(impl, odom, truth[0])
    without = impl.optimize_pose_graph(initial, odom)
    assert rmse(without, truth) == approx(rmse(initial, truth), abs=1e-6), "without a loop closure there is nothing to correct"
    optimized = impl.optimize_pose_graph(initial, odom + [loop])
    before, after = rmse(initial, truth), rmse(optimized, truth)
    assert impl.chi2(optimized, odom + [loop]) < 0.1 * impl.chi2(initial, odom + [loop])
    assert after < 0.5 * before, f"RMSE {before:.3f} m -> {after:.3f} m: the loop closure should at least halve it"
    assert np.linalg.norm(optimized[-1, :2] - truth[-1, :2]) < 0.05, "the last node must end up near the start"


# --- against the simulator ---------------------------------------------------------------------------
def drive_loop(seed: int):
    """Realistic karmel drives a 1.6 m x 1.0 m rectangle back to its start; returns keyframe truth
    poses and wheel-odometry poses (every ~0.3 m or 20 deg)."""
    cfg = load_config()
    r, b = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    rad_per_tick = 2 * math.pi / cfg.drive.ticks_per_wheel_rev
    sim = DiffDriveSim(World(), DiffDriveParams.realistic(cfg), SensorParams.ideal(cfg), pose=(0.0, 0.0, 0.0), seed=seed)
    goals = [(1.6, 0.0), (1.6, 1.0), (0.0, 1.0), (0.0, 0.0)]
    odom, ticks = SE2(), sim.ticks
    truth_keys, odom_keys = [sim.pose], [odom]
    while goals and sim.t < 60.0:
        gx, gy = goals[0]
        p = sim.pose
        dist = math.hypot(gx - p.x, gy - p.y)
        if dist < 0.05:
            goals.pop(0)
            continue
        err = angle_diff(math.atan2(gy - p.y, gx - p.x), p.theta)
        w = max(-1.5, min(1.5, 3.0 * err))
        v = 0.0 if abs(err) > 0.3 else min(0.3, dist + 0.05)
        sim.set_velocity((v - w * b / 2) / r, (v + w * b / 2) / r)
        sim.step(0.02)
        new = sim.ticks
        odom = arc_update(odom, (new[0] - ticks[0]) * rad_per_tick * r, (new[1] - ticks[1]) * rad_per_tick * r, b)
        ticks = new
        last = odom_keys[-1]
        if last.distance_to(odom) > 0.3 or abs(angle_diff(odom.theta, last.theta)) > math.radians(20):
            truth_keys.append(sim.pose)
            odom_keys.append(odom)
    truth_keys.append(sim.pose)
    odom_keys.append(odom)
    return np.array([p.as_tuple() for p in truth_keys]), np.array([p.as_tuple() for p in odom_keys])


@pytest.mark.parametrize("seed", [1, 2])
def test_loop_closure_on_simulated_robot(impl, seed):
    truth, odom = drive_loop(seed)
    n = len(truth)
    edges = [impl.Edge(k, k + 1, impl.relative_pose(odom[k], odom[k + 1]), info(0.03, math.radians(2))) for k in range(n - 1)]
    rng = np.random.default_rng(seed)
    # The loop closure a scan matcher would report: the true relative pose, 1 cm / 0.5 deg noise.
    z_loop = impl.relative_pose(truth[-1], truth[0]) + rng.normal(0, [0.01, 0.01, math.radians(0.5)])
    edges.append(impl.Edge(n - 1, 0, z_loop, info(0.01, math.radians(0.5))))
    optimized = impl.optimize_pose_graph(odom, edges)
    before, after = rmse(odom, truth), rmse(optimized, truth)
    assert before > 0.03, "the realistic robot's odometry should have drifted"
    assert after < 0.5 * before, f"optimized RMSE {after:.3f} m should be under half the odometry RMSE {before:.3f} m"
