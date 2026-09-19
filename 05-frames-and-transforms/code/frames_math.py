"""SE(2) / SE(3) helpers for module 05 — frames and transforms, in plain numpy.

One notation, used in every lesson of module 05:

    T_a_b  =  the pose of frame b expressed in frame a
           =  the transform that maps point coordinates from frame b to frame a

    p_a   = T_a_b @ p_b            (apply)
    T_a_c = T_a_b @ T_b_c          (compose: the inner subscripts "cancel")
    T_b_a = inverse(T_a_b)         (invert: swap the subscripts)

2D transforms are 3x3 homogeneous matrices, 3D transforms are 4x4. Angles are radians,
REP-103 axes (x forward, y left, z up). Quaternions are (x, y, z, w) — the ROS and scipy order.

This module mirrors the course library ``labs/python/robotlab/geometry.py``: ``se2(x, y, th)``
is ``SE2(x, y, th).as_matrix()``, and ``SE2.__matmul__`` is the ``@`` below. The matrix form
is used here because it extends to 3D unchanged.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --------------------------------------------------------------------------------------------
# Angles
# --------------------------------------------------------------------------------------------
def wrap_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]. Same convention as robotlab.geometry.wrap_angle.

    Python's % always returns a value in [0, 2*pi) for a positive divisor, so
    pi - ((pi - a) % 2pi) lands in (-pi, pi]: +pi stays +pi, -pi becomes +pi.
    """
    return math.pi - ((math.pi - angle) % (2.0 * math.pi))


# --------------------------------------------------------------------------------------------
# SE(2): 3x3 homogeneous transforms
# --------------------------------------------------------------------------------------------
def rot2(theta: float) -> Array:
    """2x2 counter-clockwise rotation. Columns = the child's x and y axes seen in the parent."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def se2(x: float, y: float, theta: float) -> Array:
    """T_a_b for frame b at (x, y) with heading theta, all expressed in frame a."""
    T = np.eye(3)
    T[:2, :2] = rot2(theta)
    T[:2, 2] = (x, y)
    return T


def se2_params(T: ArrayLike) -> tuple[float, float, float]:
    """(x, y, theta) of a 3x3 transform, theta wrapped to (-pi, pi]."""
    M = np.asarray(T, dtype=float)
    return float(M[0, 2]), float(M[1, 2]), wrap_angle(math.atan2(M[1, 0], M[0, 0]))


def se2_inverse(T: ArrayLike) -> Array:
    """Closed-form inverse [R^T, -R^T t]. Exact for rigid transforms, cheaper than np.linalg.inv."""
    M = np.asarray(T, dtype=float)
    R, t = M[:2, :2], M[:2, 2]
    inv = np.eye(3)
    inv[:2, :2] = R.T
    inv[:2, 2] = -R.T @ t
    return inv


# --------------------------------------------------------------------------------------------
# SO(3) and SE(3): 3x3 rotations, 4x4 transforms
# --------------------------------------------------------------------------------------------
def rot_x(angle: float) -> Array:
    """Rotation about x (roll)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(angle: float) -> Array:
    """Rotation about y (pitch)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle: float) -> Array:
    """Rotation about z (yaw)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> Array:
    """ROS / URDF convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    Read right to left about the FIXED parent axes: roll about x, then pitch about y, then yaw
    about z. Equivalently, read left to right about the MOVING axes: yaw, then pitch, then roll.
    scipy: Rotation.from_euler("xyz", [roll, pitch, yaw]) (lowercase = fixed/extrinsic).
    """
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def matrix_to_rpy(R: ArrayLike) -> tuple[float, float, float]:
    """Inverse of rpy_to_matrix. Pitch is in [-pi/2, pi/2].

    At pitch = +-90 deg (gimbal lock) roll and yaw are not unique; this returns roll = 0 and
    puts the whole remaining rotation into yaw.
    """
    M = np.asarray(R, dtype=float)
    sin_pitch = -M[2, 0]
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    if abs(sin_pitch) > 1.0 - 1e-9:
        roll = 0.0
        yaw = math.atan2(-M[0, 1], M[1, 1])
    else:
        roll = math.atan2(M[2, 1], M[2, 2])
        yaw = math.atan2(M[1, 0], M[0, 0])
    return roll, pitch, yaw


def is_rotation_matrix(R: ArrayLike, tol: float = 1e-9) -> bool:
    """Orthonormal (R^T R = I) and right-handed (det = +1)."""
    M = np.asarray(R, dtype=float)
    return M.shape == (3, 3) and np.allclose(M.T @ M, np.eye(3), atol=tol) and abs(
        np.linalg.det(M) - 1.0) < tol


def se3(R: ArrayLike | None = None, t: ArrayLike = (0.0, 0.0, 0.0)) -> Array:
    """4x4 transform [[R, t], [0 0 0 1]]."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(t, dtype=float)
    return T


def se3_from_xyz_rpy(x: float, y: float, z: float,
                     roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> Array:
    """Same meaning as a URDF <origin xyz="x y z" rpy="roll pitch yaw"/>: T_parent_child."""
    return se3(rpy_to_matrix(roll, pitch, yaw), (x, y, z))


def se3_inverse(T: ArrayLike) -> Array:
    """Closed-form inverse of a 4x4 rigid transform."""
    M = np.asarray(T, dtype=float)
    R, t = M[:3, :3], M[:3, 3]
    inv = np.eye(4)
    inv[:3, :3] = R.T
    inv[:3, 3] = -R.T @ t
    return inv


def se3_from_se2(T2: ArrayLike, z: float = 0.0) -> Array:
    """Lift a planar 3x3 transform to 4x4 (rotation about z, optional height)."""
    x, y, theta = se2_params(T2)
    return se3(rot_z(theta), (x, y, z))


# --------------------------------------------------------------------------------------------
# Quaternions, (x, y, z, w) order
# --------------------------------------------------------------------------------------------
def quat_normalize(q: ArrayLike) -> Array:
    v = np.asarray(q, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-12:
        raise ValueError("zero-length quaternion is not a rotation")
    return v / n


def quat_to_matrix(q: ArrayLike) -> Array:
    """Rotation matrix of a quaternion given as (x, y, z, w)."""
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def matrix_to_quat(R: ArrayLike) -> Array:
    """(x, y, z, w) of a rotation matrix, with w >= 0 (q and -q are the same rotation)."""
    M = np.asarray(R, dtype=float)
    trace = M[0, 0] + M[1, 1] + M[2, 2]
    # Shepard's method: pick the largest diagonal term for numerical stability.
    candidates = [trace, M[0, 0], M[1, 1], M[2, 2]]
    i = int(np.argmax(candidates))
    if i == 0:
        w = math.sqrt(max(0.0, 1.0 + trace)) / 2.0
        x = (M[2, 1] - M[1, 2]) / (4 * w)
        y = (M[0, 2] - M[2, 0]) / (4 * w)
        z = (M[1, 0] - M[0, 1]) / (4 * w)
    elif i == 1:
        x = math.sqrt(max(0.0, 1.0 + 2 * M[0, 0] - trace)) / 2.0
        w = (M[2, 1] - M[1, 2]) / (4 * x)
        y = (M[0, 1] + M[1, 0]) / (4 * x)
        z = (M[0, 2] + M[2, 0]) / (4 * x)
    elif i == 2:
        y = math.sqrt(max(0.0, 1.0 + 2 * M[1, 1] - trace)) / 2.0
        w = (M[0, 2] - M[2, 0]) / (4 * y)
        x = (M[0, 1] + M[1, 0]) / (4 * y)
        z = (M[1, 2] + M[2, 1]) / (4 * y)
    else:
        z = math.sqrt(max(0.0, 1.0 + 2 * M[2, 2] - trace)) / 2.0
        w = (M[1, 0] - M[0, 1]) / (4 * z)
        x = (M[0, 2] + M[2, 0]) / (4 * z)
        y = (M[1, 2] + M[2, 1]) / (4 * z)
    q = quat_normalize((x, y, z, w))
    return -q if q[3] < 0 else q


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> Array:
    """(x, y, z, w) for the ROS roll-pitch-yaw convention."""
    return matrix_to_quat(rpy_to_matrix(roll, pitch, yaw))


def quat_from_yaw(yaw: float) -> Array:
    """(x, y, z, w) of a pure rotation about z: (0, 0, sin(yaw/2), cos(yaw/2))."""
    return np.array([0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)])


def yaw_from_quat(q: ArrayLike) -> float:
    """Heading of a quaternion (x, y, z, w) — what a planar robot needs from odometry."""
    x, y, z, w = quat_normalize(q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_multiply(q1: ArrayLike, q2: ArrayLike) -> Array:
    """Hamilton product q1 * q2 for (x, y, z, w). Same meaning as R(q1) @ R(q2)."""
    x1, y1, z1, w1 = np.asarray(q1, dtype=float)
    x2, y2, z2, w2 = np.asarray(q2, dtype=float)
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def xyzw_to_wxyz(q: ArrayLike) -> Array:
    x, y, z, w = np.asarray(q, dtype=float)
    return np.array([w, x, y, z])


def wxyz_to_xyzw(q: ArrayLike) -> Array:
    w, x, y, z = np.asarray(q, dtype=float)
    return np.array([x, y, z, w])


# --------------------------------------------------------------------------------------------
# Applying and chaining
# --------------------------------------------------------------------------------------------
def transform_points(T_a_b: ArrayLike, points_b: ArrayLike) -> Array:
    """Map points from frame b to frame a.

    T_a_b is 3x3 (2D) or 4x4 (3D). points_b has shape (d,) or (N, d) with d = 2 or 3.
    Returns the same shape. Vectorized: a 360-beam scan is one matrix multiply.
    """
    T = np.asarray(T_a_b, dtype=float)
    p = np.asarray(points_b, dtype=float)
    d = T.shape[0] - 1
    if T.shape not in ((3, 3), (4, 4)) or p.shape[-1] != d:
        raise ValueError(f"transform {T.shape} cannot map points of shape {p.shape}")
    return p @ T[:d, :d].T + T[:d, d]


def rotate_vectors(T_a_b: ArrayLike, vectors_b: ArrayLike) -> Array:
    """Map DIRECTIONS (velocities, normals, axes) from b to a: rotation only, no translation."""
    T = np.asarray(T_a_b, dtype=float)
    v = np.asarray(vectors_b, dtype=float)
    d = T.shape[0] - 1
    return v @ T[:d, :d].T


def compose(*transforms: ArrayLike) -> Array:
    """compose(T_a_b, T_b_c, T_c_d) == T_a_d. Order matters; matrices multiply left to right."""
    if not transforms:
        raise ValueError("compose() needs at least one transform")
    result = np.asarray(transforms[0], dtype=float)
    for T in transforms[1:]:
        result = result @ np.asarray(T, dtype=float)
    return result


def chain(named: Sequence[tuple[str, str, ArrayLike]]) -> Array:
    """Compose [(a, b, T_a_b), (b, c, T_b_c), ...] and CHECK that the frame names line up.

    >>> chain([("map", "base_link", T1), ("base_link", "laser", T2)])   # -> T_map_laser
    Raises ValueError on ("map", "base_link", ...), ("camera_link", ...): a broken chain is
    the most common transform bug and it never raises on its own.
    """
    if not named:
        raise ValueError("empty chain")
    for (_, child, _), (parent, _, _) in zip(named, named[1:]):
        if child != parent:
            raise ValueError(f"broken chain: ...{child}] @ [{parent}... do not match")
    return compose(*(T for _, _, T in named))


# --------------------------------------------------------------------------------------------
# karmel (labs/config/karmel.yaml, labs/ros2_ws/src/karmel_description/urdf/karmel.urdf.xacro)
# --------------------------------------------------------------------------------------------
KARMEL_WHEEL_RADIUS_M = 0.045

#: camera_link (x forward, y left, z up) -> camera_optical_frame (z forward, x right, y down).
#: Columns are the optical axes written in camera_link: x_opt = -y, y_opt = -z, z_opt = +x.
R_CAMERA_LINK_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def karmel_static_transforms() -> dict[tuple[str, str], Array]:
    """The fixed transforms of karmel's URDF, keyed by (parent, child)."""
    return {
        ("base_footprint", "base_link"): se3_from_xyz_rpy(0.0, 0.0, KARMEL_WHEEL_RADIUS_M),
        ("base_link", "laser"): se3_from_xyz_rpy(0.0, 0.0, 0.12),
        ("base_link", "imu_link"): se3_from_xyz_rpy(0.0, 0.0, 0.04),
        ("base_link", "range_front_link"): se3_from_xyz_rpy(0.125, 0.0, 0.05),
        ("base_link", "camera_link"): se3_from_xyz_rpy(0.10, 0.0, 0.10),
        ("camera_link", "camera_optical_frame"): se3_from_xyz_rpy(
            0.0, 0.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2),
    }


def _self_check() -> None:
    assert np.allclose(R_CAMERA_LINK_OPTICAL,
                       karmel_static_transforms()[("camera_link", "camera_optical_frame")][:3, :3])


if __name__ == "__main__":
    _self_check()
    s = karmel_static_transforms()
    T_map_footprint = se3_from_se2(se2(2.0, 1.0, math.radians(90)))
    T_map_optical = chain([
        ("map", "base_footprint", T_map_footprint),
        ("base_footprint", "base_link", s[("base_footprint", "base_link")]),
        ("base_link", "camera_link", s[("base_link", "camera_link")]),
        ("camera_link", "camera_optical_frame", s[("camera_link", "camera_optical_frame")]),
    ])
    T_base_optical = s[("base_link", "camera_link")] @ s[("camera_link", "camera_optical_frame")]
    bottle_optical = np.array([0.05, -0.02, 0.60])
    np.set_printoptions(precision=4, suppress=True)
    print("bottle in camera_optical_frame:", bottle_optical)
    print("bottle in base_link:           ", transform_points(T_base_optical, bottle_optical))
    print("bottle in map:                 ", transform_points(T_map_optical, bottle_optical))
