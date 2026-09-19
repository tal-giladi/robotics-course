"""Tests for frames_math.py: py -m pytest 05-frames-and-transforms/code"""

from __future__ import annotations

import math

import numpy as np
import pytest

import frames_math as fm

rng = np.random.default_rng(5)


def random_se2() -> np.ndarray:
    x, y = rng.uniform(-5, 5, 2)
    return fm.se2(x, y, rng.uniform(-math.pi, math.pi))


def random_se3() -> np.ndarray:
    roll, yaw = rng.uniform(-math.pi, math.pi, 2)
    pitch = rng.uniform(-1.4, 1.4)
    return fm.se3_from_xyz_rpy(*rng.uniform(-3, 3, 3), roll, pitch, yaw)


# --- angles ---------------------------------------------------------------------------------
@pytest.mark.parametrize("angle, expected", [
    (0.0, 0.0), (math.pi, math.pi), (-math.pi, math.pi), (3 * math.pi, math.pi),
    (math.radians(270), math.radians(-90)), (math.radians(-190), math.radians(170)),
    (math.radians(720 + 30), math.radians(30)),
])
def test_wrap_angle(angle: float, expected: float) -> None:
    assert fm.wrap_angle(angle) == pytest.approx(expected)


def test_wrap_angle_matches_robotlab() -> None:
    geometry = pytest.importorskip("robotlab.geometry")
    for a in rng.uniform(-20, 20, 200):
        assert fm.wrap_angle(a) == pytest.approx(geometry.wrap_angle(a))


# --- SE(2) ----------------------------------------------------------------------------------
def test_se2_worked_example_bottle_2d() -> None:
    T_map_base = fm.se2(2.0, 1.0, math.radians(90))
    T_base_camera = fm.se2(0.10, 0.0, 0.0)
    p_camera = np.array([0.60, -0.05])
    p_base = fm.transform_points(T_base_camera, p_camera)
    p_map = fm.transform_points(T_map_base @ T_base_camera, p_camera)
    assert p_base == pytest.approx([0.70, -0.05])
    assert p_map == pytest.approx([2.05, 1.70])


def test_se2_inverse_worked_example() -> None:
    T_base_map = fm.se2_inverse(fm.se2(2.0, 1.0, math.radians(90)))
    assert fm.se2_params(T_base_map) == pytest.approx((-1.0, 2.0, math.radians(-90)))


def test_se2_inverse_and_params_roundtrip() -> None:
    for _ in range(50):
        T = random_se2()
        assert T @ fm.se2_inverse(T) == pytest.approx(np.eye(3))
        assert fm.se2_inverse(T) == pytest.approx(np.linalg.inv(T))
        assert fm.se2(*fm.se2_params(T)) == pytest.approx(T)


def test_se2_matches_robotlab_SE2() -> None:
    geometry = pytest.importorskip("robotlab.geometry")
    for _ in range(50):
        A, B = random_se2(), random_se2()
        a, b = geometry.SE2.from_matrix(A), geometry.SE2.from_matrix(B)
        assert (a @ b).as_matrix() == pytest.approx(A @ B)
        assert a.inverse().as_matrix() == pytest.approx(fm.se2_inverse(A))
        p = rng.uniform(-2, 2, (7, 2))
        assert a.apply(p) == pytest.approx(fm.transform_points(A, p))


def test_composition_is_not_commutative() -> None:
    A, B = fm.se2(1.0, 0.0, 0.0), fm.se2(0.0, 0.0, math.radians(90))
    assert fm.se2_params(A @ B) == pytest.approx((1.0, 0.0, math.pi / 2))
    assert fm.se2_params(B @ A) == pytest.approx((0.0, 1.0, math.pi / 2))


# --- rotations ------------------------------------------------------------------------------
def test_rpy_matches_scipy_fixed_xyz_and_intrinsic_ZYX() -> None:
    Rotation = pytest.importorskip("scipy.spatial.transform").Rotation
    for _ in range(50):
        r, p, y = rng.uniform(-3, 3), rng.uniform(-1.5, 1.5), rng.uniform(-3, 3)
        R = fm.rpy_to_matrix(r, p, y)
        assert R == pytest.approx(Rotation.from_euler("xyz", [r, p, y]).as_matrix())
        assert R == pytest.approx(Rotation.from_euler("ZYX", [y, p, r]).as_matrix())
        assert fm.is_rotation_matrix(R)


def test_matrix_to_rpy_roundtrip_and_gimbal_lock() -> None:
    for _ in range(50):
        rpy = (rng.uniform(-3, 3), rng.uniform(-1.5, 1.5), rng.uniform(-3, 3))
        assert fm.matrix_to_rpy(fm.rpy_to_matrix(*rpy)) == pytest.approx(rpy)
    R = fm.rpy_to_matrix(0.3, math.pi / 2, 0.5)          # gimbal lock: only yaw - roll matters
    r, p, y = fm.matrix_to_rpy(R)
    assert p == pytest.approx(math.pi / 2)
    assert fm.rpy_to_matrix(r, p, y) == pytest.approx(R)


def test_quaternions_match_scipy_xyzw() -> None:
    Rotation = pytest.importorskip("scipy.spatial.transform").Rotation
    for _ in range(50):
        R = fm.rpy_to_matrix(*rng.uniform(-3, 3, 3))
        q = fm.matrix_to_quat(R)
        q_scipy = Rotation.from_matrix(R).as_quat()          # scipy default is (x, y, z, w)
        assert q == pytest.approx(q_scipy if q_scipy[3] >= 0 else -q_scipy, abs=1e-9)
        assert fm.quat_to_matrix(q) == pytest.approx(R)
        assert fm.quat_to_matrix(-q) == pytest.approx(R)     # q and -q: same rotation


def test_quaternion_known_values() -> None:
    s = math.sqrt(0.5)
    assert fm.quat_from_yaw(math.radians(90)) == pytest.approx([0, 0, s, s])
    assert fm.quat_from_rpy(0, 0, math.radians(90)) == pytest.approx([0, 0, s, s])
    assert fm.quat_from_rpy(-math.pi / 2, 0, -math.pi / 2) == pytest.approx([-0.5, 0.5, -0.5, 0.5])
    assert fm.yaw_from_quat([0, 0, s, s]) == pytest.approx(math.pi / 2)
    assert fm.xyzw_to_wxyz([1, 2, 3, 4]) == pytest.approx([4, 1, 2, 3])
    assert fm.wxyz_to_xyzw([4, 1, 2, 3]) == pytest.approx([1, 2, 3, 4])


def test_quat_multiply_matches_matrix_product() -> None:
    for _ in range(30):
        q1 = fm.quat_from_rpy(*rng.uniform(-3, 3, 3))
        q2 = fm.quat_from_rpy(*rng.uniform(-3, 3, 3))
        assert fm.quat_to_matrix(fm.quat_multiply(q1, q2)) == pytest.approx(
            fm.quat_to_matrix(q1) @ fm.quat_to_matrix(q2))


# --- SE(3) ----------------------------------------------------------------------------------
def test_se3_inverse() -> None:
    for _ in range(50):
        T = random_se3()
        assert fm.se3_inverse(T) == pytest.approx(np.linalg.inv(T))


def test_points_vs_vectors() -> None:
    T = fm.se3_from_xyz_rpy(1.0, 2.0, 3.0, 0, 0, math.radians(90))
    assert fm.transform_points(T, [1.0, 0.0, 0.0]) == pytest.approx([1.0, 3.0, 3.0])
    assert fm.rotate_vectors(T, [1.0, 0.0, 0.0]) == pytest.approx([0.0, 1.0, 0.0])


def test_transform_points_batch_equals_loop() -> None:
    T = random_se3()
    pts = rng.uniform(-5, 5, (360, 3))
    looped = np.array([(T @ np.append(p, 1.0))[:3] for p in pts])
    assert fm.transform_points(T, pts) == pytest.approx(looped)


def test_transform_points_rejects_wrong_dimension() -> None:
    with pytest.raises(ValueError):
        fm.transform_points(np.eye(4), [1.0, 2.0])


def test_optical_rotation_constant() -> None:
    T = fm.karmel_static_transforms()[("camera_link", "camera_optical_frame")]
    assert T[:3, :3] == pytest.approx(fm.R_CAMERA_LINK_OPTICAL, abs=1e-12)
    # optical +z (forward) is camera_link +x; optical +x (right) is camera_link -y
    assert fm.rotate_vectors(T, [0, 0, 1]) == pytest.approx([1, 0, 0])
    assert fm.rotate_vectors(T, [1, 0, 0]) == pytest.approx([0, -1, 0])
    assert fm.rotate_vectors(T, [0, 1, 0]) == pytest.approx([0, 0, -1])


def test_bottle_worked_example_3d() -> None:
    s = fm.karmel_static_transforms()
    T_map_footprint = fm.se3_from_se2(fm.se2(2.0, 1.0, math.radians(90)))
    T_map_optical = fm.chain([
        ("map", "base_footprint", T_map_footprint),
        ("base_footprint", "base_link", s[("base_footprint", "base_link")]),
        ("base_link", "camera_link", s[("base_link", "camera_link")]),
        ("camera_link", "camera_optical_frame", s[("camera_link", "camera_optical_frame")]),
    ])
    T_base_optical = s[("base_link", "camera_link")] @ s[("camera_link", "camera_optical_frame")]
    bottle = [0.05, -0.02, 0.60]
    assert fm.transform_points(T_base_optical, bottle) == pytest.approx([0.70, -0.05, 0.12])
    assert fm.transform_points(T_map_optical, bottle) == pytest.approx([2.05, 1.70, 0.165])


def test_chain_detects_broken_chain() -> None:
    with pytest.raises(ValueError, match="broken chain"):
        fm.chain([("map", "base_link", np.eye(4)), ("camera_link", "camera_optical_frame", np.eye(4))])
