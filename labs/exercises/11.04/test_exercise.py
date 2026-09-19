"""Checker for 11.04 — Scan matching with ICP.

Run: ``python course.py check 11.04`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robotlab.geometry import SE2, angle_diff
from robotlab.sim import World

approx = pytest.approx
ANGLES = -math.pi + np.arange(360) * (2 * math.pi / 360)


def scan_points(world: World, pose: SE2, noise_std: float = 0.0, seed: int = 0) -> np.ndarray:
    """What a 360-beam LiDAR at ``pose`` sees, as points in its own frame."""
    r = world.raycast((pose.x, pose.y), ANGLES + pose.theta, 12.0)
    if noise_std:
        r = r + np.random.default_rng(seed).normal(0.0, noise_std, r.size)
    ok = np.isfinite(r)
    return np.column_stack([r[ok] * np.cos(ANGLES[ok]), r[ok] * np.sin(ANGLES[ok])])


def pose_error(result, truth: SE2) -> tuple[float, float]:
    return math.hypot(result.x - truth.x, result.y - truth.y), abs(angle_diff(result.theta, truth.theta))


# --- best_fit_transform ------------------------------------------------------------------------------
def test_kabsch_three_point_example(impl):
    # The lesson's worked example: rotate 30 degrees, then move by (1.0, 0.5).
    src = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]])
    c, s = math.cos(math.radians(30)), math.sin(math.radians(30))
    dst = src @ np.array([[c, -s], [s, c]]).T + [1.0, 0.5]
    R, t = impl.best_fit_transform(src, dst)
    assert np.asarray(R) == approx(np.array([[c, -s], [s, c]]), abs=1e-9)
    assert np.asarray(t) == approx([1.0, 0.5], abs=1e-9)


def test_kabsch_identity(impl):
    pts = np.array([[1.0, 2.0], [3.0, -1.0], [0.5, 0.5], [-2.0, 1.0]])
    R, t = impl.best_fit_transform(pts, pts)
    assert np.asarray(R) == approx(np.eye(2), abs=1e-9)
    assert np.asarray(t) == approx([0.0, 0.0], abs=1e-9)


def test_kabsch_never_returns_a_reflection(impl):
    src = np.array([[1.0, 0.2], [0.0, 1.0], [-1.0, 0.0], [0.3, -1.5]])
    dst = src * [1.0, -1.0]  # mirror image: no rotation can produce it
    R, _ = impl.best_fit_transform(src, dst)
    assert np.linalg.det(R) == approx(1.0), "R must be a rotation (det +1), not a reflection (det -1)"
    assert np.asarray(R) @ np.asarray(R).T == approx(np.eye(2), abs=1e-9)


def test_kabsch_with_noisy_correspondences(impl):
    rng = np.random.default_rng(4)
    src = rng.uniform(-3.0, 3.0, (500, 2))
    truth = SE2(-0.4, 0.25, math.radians(-17))
    dst = truth.apply(src) + rng.normal(0.0, 0.01, src.shape)
    R, t = impl.best_fit_transform(src, dst)
    assert math.atan2(R[1, 0], R[0, 0]) == approx(truth.theta, abs=math.radians(0.1))
    assert np.asarray(t) == approx([truth.x, truth.y], abs=0.003)


# --- icp ---------------------------------------------------------------------------------------------
def test_icp_recovers_transform_of_identical_points(impl):
    # 60 scattered points, spaced ~0.5 m apart; the target is the same points moved, so an exact
    # answer exists and a small motion keeps every nearest neighbour correct.
    source = np.random.default_rng(3).uniform(-2.0, 2.0, (60, 2))
    truth = SE2(0.12, -0.08, math.radians(5))
    result = impl.icp(source, truth.apply(source))
    assert result.converged
    dp, dth = pose_error(result, truth)
    assert dp < 1e-4 and dth < math.radians(0.01), f"expected {truth}, got {result}"
    assert result.rmse < 1e-4 and result.inlier_fraction == approx(1.0)


def test_icp_uses_the_initial_guess(impl):
    rng = np.random.default_rng(0)
    source = rng.uniform(-2, 2, (300, 2))
    truth = SE2(2.0, 1.0, math.radians(120))  # far outside any basin from identity
    result = impl.icp(source, truth.apply(source), initial=(1.95, 1.05, math.radians(118)))
    dp, dth = pose_error(result, truth)
    assert dp < 1e-4 and dth < 1e-4, "starting next to the answer must converge to it"


@pytest.mark.parametrize(("noise", "seed"), [(0.0, 0), (0.01, 1), (0.01, 2)])
def test_icp_on_apartment_scans(impl, noise, seed):
    world = World.apartment()
    a, b = SE2(1.0, 1.3, 0.0), SE2(1.3, 1.45, math.radians(10))
    truth = a.between(b)  # the pose of b in a's frame = the transform from b's points to a's points
    source = scan_points(world, b, noise, seed)
    target = scan_points(world, a, noise, seed + 100)
    result = impl.icp(source, target, max_correspondence_distance=0.3)
    dp, dth = pose_error(result, truth)
    assert result.converged
    assert dp < 0.03, f"translation off by {dp * 100:.1f} cm (tolerance 3 cm)"
    assert dth < math.radians(1.0), f"rotation off by {math.degrees(dth):.2f} deg (tolerance 1 deg)"


def test_icp_rejects_outliers(impl):
    # 30% of the source points are clutter only one scan sees (a person, a chair moved).
    world = World.apartment()
    a, b = SE2(1.0, 1.3, 0.0), SE2(1.2, 1.35, math.radians(6))
    truth = a.between(b)
    rng = np.random.default_rng(7)
    clean = scan_points(world, b)
    clutter = rng.uniform([-0.6, -0.6], [0.6, 0.6], (len(clean) * 3 // 7, 2)) + [0.5, 0.9]
    source = np.vstack([clean, clutter])
    result = impl.icp(source, scan_points(world, a), max_correspondence_distance=0.2)
    dp, dth = pose_error(result, truth)
    assert dp < 0.03 and dth < math.radians(1.0), "gated ICP must ignore the clutter"
    assert result.inlier_fraction < 0.9, "clutter points have no partner: inlier_fraction must count them out"


def test_icp_stops_when_nothing_matches(impl):
    result = impl.icp(np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]), np.array([[50.0, 50.0], [51.0, 50.0], [50.0, 51.0]]))
    assert not result.converged
    assert result.inlier_fraction == 0.0


# --- match_is_reliable -------------------------------------------------------------------------------
def make_result(impl, rmse=0.02, inliers=0.9, converged=True):
    return impl.IcpResult(0.0, 0.0, 0.0, rmse, inliers, 10, converged)


def test_reliability_rules(impl):
    assert impl.match_is_reliable(make_result(impl)) is True
    assert impl.match_is_reliable(make_result(impl, converged=False)) is False
    assert impl.match_is_reliable(make_result(impl, inliers=0.5)) is False
    assert impl.match_is_reliable(make_result(impl, rmse=0.2)) is False
    assert impl.match_is_reliable(make_result(impl, rmse=0.2), max_rmse=0.3) is True


def test_rejects_a_match_between_different_rooms(impl):
    world = World.apartment()
    living_room, bedroom = SE2(1.0, 1.3, 0.0), SE2(4.9, 3.25, 0.0)
    wrong = impl.icp(scan_points(world, bedroom), scan_points(world, living_room), max_correspondence_distance=0.3)
    assert not impl.match_is_reliable(wrong), f"scans of two different rooms must not be accepted: {wrong}"
    right = impl.icp(scan_points(world, living_room @ SE2(0.2, 0.0, 0.1)), scan_points(world, living_room),
                     max_correspondence_distance=0.3)
    assert impl.match_is_reliable(right), f"a good match in the same room must be accepted: {right}"
