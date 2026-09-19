"""15.06 — reference solution: the three Jacobians of a visual servo loop.

See ``15-manipulation/code/visual_servo.py`` for the same functions inside a running simulation.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given ---------------------------------------------------------------------------------
def project(points_cam: ArrayLike, fx: float = 525.0, fy: float = 525.0,
            cx: float = 319.5, cy: float = 239.5) -> Array:
    p = np.asarray(points_cam, dtype=float).reshape(-1, 3)
    if np.any(p[:, 2] <= 1e-6):
        raise ValueError("a feature is at or behind the image plane (Z <= 0)")
    return np.column_stack((fx * p[:, 0] / p[:, 2] + cx, fy * p[:, 1] / p[:, 2] + cy))


def skew(v: ArrayLike) -> Array:
    x, y, z = np.asarray(v, dtype=float).reshape(3)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def rot_of(rotvec: ArrayLike) -> Array:
    r = np.asarray(rotvec, dtype=float).reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = skew(k)
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def inv_T(T: ArrayLike) -> Array:
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


# --- implement these -------------------------------------------------------------------------
def interaction_matrix_point(x: float, y: float, Z: float) -> Array:
    if Z <= 1e-9:
        raise ValueError("depth must be positive")
    return np.array([
        [-1.0 / Z, 0.0, x / Z, x * y, -(1.0 + x * x), y],
        [0.0, -1.0 / Z, y / Z, 1.0 + y * y, -x * y, -x],
    ])


def interaction_matrix(features: ArrayLike, depths: ArrayLike) -> Array:
    f = np.asarray(features, dtype=float).reshape(-1, 2)
    Z = np.asarray(depths, dtype=float).reshape(-1)
    if len(Z) != len(f):
        raise ValueError("need one depth per feature")
    return np.vstack([interaction_matrix_point(float(x), float(y), float(z))
                      for (x, y), z in zip(f, Z, strict=True)])


def damped_pinv(L: ArrayLike, damping: float = 0.0) -> Array:
    L = np.asarray(L, dtype=float)
    m, n = L.shape
    if m <= n:
        return L.T @ np.linalg.solve(L @ L.T + damping ** 2 * np.eye(m), np.eye(m))
    return np.linalg.solve(L.T @ L + damping ** 2 * np.eye(n), L.T)


def ibvs_velocity(features: ArrayLike, features_star: ArrayLike, depths: ArrayLike, *,
                  gain: float = 0.8, damping: float = 0.0) -> Array:
    e = (np.asarray(features, dtype=float) - np.asarray(features_star, dtype=float)).reshape(-1)
    L = interaction_matrix(features, depths)
    return -gain * (damped_pinv(L, damping) @ e)


def rotvec_of(R: ArrayLike) -> Array:
    R = np.asarray(R, dtype=float)
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    theta = math.acos(c)
    if theta < 1e-9:
        return np.zeros(3)
    if math.pi - theta < 1e-6:
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        k = int(np.argmax(axis))
        axis = A[:, k] / axis[k]
        return axis / float(np.linalg.norm(axis)) * theta
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (theta / (2.0 * math.sin(theta)))


def pbvs_velocity(T_cam_obj: ArrayLike, T_cam_obj_star: ArrayLike, *, gain: float = 0.8) -> Array:
    T_err = inv_T(T_cam_obj_star) @ np.asarray(T_cam_obj, dtype=float)
    return gain * np.concatenate([T_err[:3, 3], rotvec_of(T_err[:3, :3])])


def camera_twist_to_tool_twist(v_cam: ArrayLike, T_base_cam: ArrayLike,
                               T_base_tool: ArrayLike) -> Array:
    v = np.asarray(v_cam, dtype=float).reshape(6)
    Tbc = np.asarray(T_base_cam, dtype=float)
    Tbt = np.asarray(T_base_tool, dtype=float)
    R = Tbc[:3, :3]
    omega = R @ v[3:]
    lin = R @ v[:3] + np.cross(omega, Tbt[:3, 3] - Tbc[:3, 3])
    return np.concatenate([lin, omega])
