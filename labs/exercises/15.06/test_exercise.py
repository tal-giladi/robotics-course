"""Tests for 15.06 — visual servoing.

The important test is ``test_interaction_matrix_matches_numerical_derivative``: it compares your
analytic 2x6 against a finite difference of the actual projection, which is the only check that
can catch a sign error in a single column.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

Z0 = 0.25


def _block_points(long_m: float = 0.060, short_m: float = 0.030) -> np.ndarray:
    a, b = long_m / 2.0, short_m / 2.0
    return np.array([[-a, -b, 0.0], [a, -b, 0.0], [a, b, 0.0], [-a, b, 0.0]])


def _T(R: np.ndarray | None = None, t=(0.0, 0.0, 0.0)) -> np.ndarray:
    out = np.eye(4)
    if R is not None:
        out[:3, :3] = R
    out[:3, 3] = np.asarray(t, dtype=float)
    return out


def _points_in_camera(T_cam_obj, points_obj):
    T = np.asarray(T_cam_obj, dtype=float)
    return np.asarray(points_obj, dtype=float) @ T[:3, :3].T + T[:3, 3]


def _normalized(points_cam: np.ndarray) -> np.ndarray:
    p = np.asarray(points_cam, dtype=float)
    return np.column_stack((p[:, 0] / p[:, 2], p[:, 1] / p[:, 2]))


def _move_camera(points_cam: np.ndarray, v: np.ndarray, dt: float, impl) -> np.ndarray:
    """Where the (fixed) points end up in the camera frame after the camera moves by twist v."""
    R = impl.rot_of(np.asarray(v[3:]) * dt)
    # camera moves by (v, omega); points in the camera frame move by the inverse
    return (points_cam - np.asarray(v[:3]) * dt) @ R


# ---------------------------------------------------------------- interaction matrix
def test_interaction_matrix_point_shape_and_entries(impl):
    L = impl.interaction_matrix_point(0.2, -0.1, 0.5)
    assert L.shape == (2, 6)
    assert L[0, 0] == pytest.approx(-2.0)
    assert L[1, 1] == pytest.approx(-2.0)
    assert L[0, 1] == pytest.approx(0.0)
    assert L[0, 2] == pytest.approx(0.4)
    assert L[1, 2] == pytest.approx(-0.2)
    assert L[0, 4] == pytest.approx(-1.04)
    assert L[1, 5] == pytest.approx(-0.2)


def test_interaction_matrix_point_rejects_zero_depth(impl):
    with pytest.raises(ValueError):
        impl.interaction_matrix_point(0.0, 0.0, 0.0)


def test_interaction_matrix_stacks(impl):
    f = np.array([[0.1, 0.0], [-0.2, 0.15], [0.05, -0.05]])
    Z = np.array([0.3, 0.25, 0.4])
    L = impl.interaction_matrix(f, Z)
    assert L.shape == (6, 6)
    np.testing.assert_allclose(L[2:4], impl.interaction_matrix_point(-0.2, 0.15, 0.25))


def test_interaction_matrix_rejects_mismatched_depths(impl):
    with pytest.raises(ValueError):
        impl.interaction_matrix(np.zeros((3, 2)), np.ones(2))


def test_interaction_matrix_matches_numerical_derivative(impl):
    """ds/dv from the analytic matrix must equal ds/dv measured on the projection itself."""
    pts = _points_in_camera(_T(t=(0.02, -0.01, Z0)), _block_points())
    s0 = _normalized(pts)
    L = impl.interaction_matrix(s0, pts[:, 2])
    dt = 1e-6
    numeric = np.zeros_like(L)
    for k in range(6):
        v = np.zeros(6)
        v[k] = 1.0
        s1 = _normalized(_move_camera(pts, v, dt, impl))
        numeric[:, k] = ((s1 - s0) / dt).reshape(-1)
    np.testing.assert_allclose(L, numeric, atol=2e-5)


# ---------------------------------------------------------------- damped pseudo-inverse
def test_damped_pinv_matches_numpy_when_undamped(impl):
    rng = np.random.default_rng(0)
    for shape in ((8, 6), (4, 6), (6, 6)):
        M = rng.normal(size=shape)
        np.testing.assert_allclose(impl.damped_pinv(M, 0.0), np.linalg.pinv(M), atol=1e-8)


def test_damping_shrinks_the_solution(impl):
    """That is the whole point of damping: near a singularity, ask for less."""
    M = np.diag([1.0, 1e-4])
    e = np.array([1.0, 1.0])
    undamped = impl.damped_pinv(M, 0.0) @ e
    damped = impl.damped_pinv(M, 0.01) @ e
    assert np.linalg.norm(damped) < np.linalg.norm(undamped) / 10.0


# ---------------------------------------------------------------- the two control laws
def test_ibvs_velocity_is_zero_at_the_goal(impl):
    pts = _points_in_camera(_T(t=(0.0, 0.0, Z0)), _block_points())
    s = _normalized(pts)
    v = impl.ibvs_velocity(s, s, pts[:, 2])
    np.testing.assert_allclose(v, np.zeros(6), atol=1e-9)


def test_ibvs_step_reduces_the_image_error(impl):
    goal = _points_in_camera(_T(t=(0.0, 0.0, Z0)), _block_points())
    s_star = _normalized(goal)
    start = _points_in_camera(_T(impl.rot_of([0.0, 0.0, 0.3]), (0.05, -0.03, 0.33)),
                              _block_points())
    pts = start.copy()
    err0 = float(np.linalg.norm(_normalized(pts) - s_star))
    for _ in range(100):
        s = _normalized(pts)
        v = impl.ibvs_velocity(s, s_star, goal[:, 2], gain=0.8)
        pts = _move_camera(pts, v, 0.05, impl)
    err1 = float(np.linalg.norm(_normalized(pts) - s_star))
    assert err1 < 0.05 * err0


def test_ibvs_gain_scales_the_velocity(impl):
    goal = _points_in_camera(_T(t=(0.0, 0.0, Z0)), _block_points())
    start = _points_in_camera(_T(t=(0.04, 0.0, 0.30)), _block_points())
    s, s_star = _normalized(start), _normalized(goal)
    v1 = impl.ibvs_velocity(s, s_star, goal[:, 2], gain=0.5)
    v2 = impl.ibvs_velocity(s, s_star, goal[:, 2], gain=1.5)
    np.testing.assert_allclose(v2, 3.0 * v1, rtol=1e-9)


def test_pbvs_velocity_is_zero_at_the_goal(impl):
    T = _T(impl.rot_of([0.1, -0.2, 0.05]), (0.01, 0.02, Z0))
    np.testing.assert_allclose(impl.pbvs_velocity(T, T), np.zeros(6), atol=1e-9)


def test_pbvs_pure_rotation_gives_a_pure_rotation(impl):
    """The camera-retreat lesson in one assertion: PBVS never translates to fix a roll."""
    goal = _T(t=(0.0, 0.0, Z0))
    start = _T(impl.rot_of([0.0, 0.0, math.radians(120.0)]), (0.0, 0.0, Z0))
    v = impl.pbvs_velocity(start, goal, gain=1.0)
    np.testing.assert_allclose(v[:3], np.zeros(3), atol=1e-9)
    assert v[5] == pytest.approx(math.radians(120.0), abs=1e-9)


def test_pbvs_pure_translation_gives_a_pure_translation(impl):
    goal = _T(t=(0.0, 0.0, Z0))
    start = _T(t=(0.03, -0.02, 0.31))
    v = impl.pbvs_velocity(start, goal, gain=1.0)
    np.testing.assert_allclose(v, [0.03, -0.02, 0.06, 0.0, 0.0, 0.0], atol=1e-9)


# ---------------------------------------------------------------- camera twist -> tool twist
def test_pure_translation_is_only_rotated(impl):
    R = impl.rot_of([0.0, 0.0, math.pi / 2])          # camera yawed 90 deg from the tool
    T_base_tool = _T(t=(0.2, 0.0, 0.1))
    T_base_cam = _T(R, (0.2, 0.0, 0.14))
    v = impl.camera_twist_to_tool_twist([0.1, 0.0, 0.0, 0.0, 0.0, 0.0], T_base_cam, T_base_tool)
    np.testing.assert_allclose(v, [0.0, 0.1, 0.0, 0.0, 0.0, 0.0], atol=1e-9)


def test_rotation_adds_the_lever_arm_term(impl):
    """A camera 40 mm above the tool rotating at 1 rad/s drags the tool at 40 mm/s."""
    T_base_tool = _T(t=(0.2, 0.0, 0.10))
    T_base_cam = _T(t=(0.2, 0.0, 0.14))
    v = impl.camera_twist_to_tool_twist([0.0, 0.0, 0.0, 1.0, 0.0, 0.0], T_base_cam, T_base_tool)
    np.testing.assert_allclose(v[3:], [1.0, 0.0, 0.0], atol=1e-9)
    np.testing.assert_allclose(v[:3], [0.0, 0.04, 0.0], atol=1e-9)


def test_no_lever_arm_when_the_frames_coincide(impl):
    T = _T(t=(0.2, -0.05, 0.1))
    v_in = np.array([0.01, -0.02, 0.03, 0.4, -0.1, 0.2])
    np.testing.assert_allclose(impl.camera_twist_to_tool_twist(v_in, T, T), v_in, atol=1e-12)


def test_tool_twist_is_consistent_with_finite_differences(impl):
    """Move the camera by the twist for dt; the tool point must land where v_tool predicts."""
    R = impl.rot_of([0.3, -0.2, 0.5])
    T_base_cam = _T(R, (0.25, 0.03, 0.18))
    p_tool_in_cam = np.array([0.01, 0.04, -0.03])
    T_base_tool = _T(R, T_base_cam[:3, 3] + R @ p_tool_in_cam)
    v_cam = np.array([0.02, -0.01, 0.05, 0.3, 0.2, -0.4])
    v_tool = impl.camera_twist_to_tool_twist(v_cam, T_base_cam, T_base_tool)

    dt = 1e-7
    R_new = T_base_cam[:3, :3] @ impl.rot_of(np.asarray(v_cam[3:]) * dt)
    t_new = T_base_cam[:3, 3] + T_base_cam[:3, :3] @ (np.asarray(v_cam[:3]) * dt)
    tool_new = t_new + R_new @ p_tool_in_cam
    np.testing.assert_allclose((tool_new - T_base_tool[:3, 3]) / dt, v_tool[:3], atol=1e-5)
