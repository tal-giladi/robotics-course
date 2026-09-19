"""Tests for local_planning.py (lesson 12.05)."""

import math

import numpy as np
import pytest

import local_planning as lp
from robotlab.config import load_config
from robotlab.sim import World


def test_rollout_matches_the_arc_formulas():
    traj = lp.rollout(np.array([0.2, 0.2]), np.array([0.0, 1.0]), 1.0, 0.1)
    assert traj.shape == (2, 10, 3)
    assert traj[0, -1] == pytest.approx([0.2, 0.0, 0.0])
    assert traj[1, -1] == pytest.approx([0.2 * math.sin(1.0), 0.2 * (1 - math.cos(1.0)), 1.0])


def test_dynamic_window_numbers_quoted_in_the_lesson():
    assert lp.dynamic_window(0.20, 0.0, lp.Limits(), 0.1) == pytest.approx((0.14, 0.25, -0.2, 0.2))
    assert lp.dynamic_window(0.0, 1.1, lp.Limits(), 0.1) == pytest.approx((0.0, 0.06, 0.9, 1.2))


def test_pure_pursuit_curvature_and_lookahead():
    assert lp.pure_pursuit_curvature(0.3, 0.1) == pytest.approx(2.0)
    path = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]])
    point, i = lp.lookahead_point(path, np.array([0.9, 0.0]), 0.5, 0)
    assert point == pytest.approx([1.0, math.sqrt(0.24)]) and i == 1


def test_dwa_rejects_arcs_into_an_obstacle():
    path_r = np.column_stack([np.linspace(0, 1.5, 31), np.zeros(31)])
    wall = np.column_stack([np.full(21, 0.45), np.linspace(-0.1, 0.1, 21)])  # something 45 cm straight ahead
    v, w, dbg = lp.dwa_choose(0.2, 0.0, path_r, np.array([1.0, 0.0]), wall, 0.16, lp.Limits(), lp.DWAConfig(), 0.1)
    straight = int(np.argmin(np.abs(dbg["traj"][:, -1, 1]) + (dbg["traj"][:, -1, 0] < 0.3)))  # fastest straight arc
    assert not np.isfinite(dbg["cost"][straight])
    assert abs(w) > 0.0 or v < 0.2


def test_pure_pursuit_and_dwa_in_the_apartment():
    radius = load_config().chassis.footprint_radius_m
    path = lp.global_path()
    apartment = World.apartment()
    ctrl = lp.PurePursuit(path)
    log = lp.run_in_sim("pp", apartment, path, lambda pose, scan, v, w: (*ctrl.compute(pose), ctrl.done), control_every=1, use_scan=False)
    assert log.reached and not log.collided
    assert max(lp.cross_track_error(path, np.array(p[:2])) for p in log.poses) < 0.06

    basket = lp.point_along(path, 1.3)
    world = World.from_segments(apartment.segments, np.vstack([apartment.circles, [[basket[0], basket[1], 0.15]]]))

    def dwa(pose, scan, v, w):
        if np.linalg.norm(path[-1] - np.array(pose[:2])) <= 0.08:
            return 0.0, 0.0, True
        i = int(np.argmin(np.linalg.norm(path - np.array(pose[:2]), axis=1)))
        s = lp.path_distances(path)
        j = min(int(np.searchsorted(s, s[i] + 1.0)), len(path) - 1)
        nv, nw, _ = lp.dwa_choose(v, w, lp.to_robot_frame(pose, path[i : i + 40]), lp.to_robot_frame(pose, path[j])[0],
                                  lp.scan_points_in_base(scan), radius, lp.Limits(), lp.DWAConfig(), 0.1)
        return nv, nw, False

    log = lp.run_in_sim("dwa", world, path, dwa, control_every=5, timeout_s=45.0)
    assert log.reached and not log.collided
