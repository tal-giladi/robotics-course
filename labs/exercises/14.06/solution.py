"""14.06 — reference solution. Read it after you have tried ``student.py`` yourself."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given ---------------------------------------------------------------------------------------
def planar_joint_points(lengths: Sequence[float], q: Sequence[float]) -> Array:
    if len(lengths) != len(q):
        raise ValueError(f"{len(lengths)} links but {len(q)} joint angles")
    pts = np.zeros((len(lengths) + 1, 2))
    phi = 0.0
    for i, (length, qi) in enumerate(zip(lengths, q)):
        phi += qi
        pts[i + 1] = pts[i] + length * np.array([np.cos(phi), np.sin(phi)])
    return pts


def planar_tip(lengths: Sequence[float], q: Sequence[float]) -> Array:
    return planar_joint_points(lengths, q)[-1]


# --- solution ------------------------------------------------------------------------------------
def planar_jacobian(lengths: Sequence[float], q: Sequence[float]) -> Array:
    pts = planar_joint_points(lengths, q)
    tip = pts[-1]
    J = np.zeros((2, len(q)))
    for j in range(len(q)):
        rx, ry = tip - pts[j]          # from joint j to the tip
        J[0, j] = -ry                  # z_hat x r  in the plane
        J[1, j] = rx
    return J


def singular_values(J: ArrayLike) -> Array:
    return np.linalg.svd(np.asarray(J, dtype=float), compute_uv=False)


def manipulability(J: ArrayLike) -> float:
    J = np.asarray(J, dtype=float)
    return float(math.sqrt(max(0.0, float(np.linalg.det(J @ J.T)))))


def dls_velocity(J: ArrayLike, v: ArrayLike, damping: float) -> Array:
    J = np.asarray(J, dtype=float)
    v = np.asarray(v, dtype=float)
    if damping < 0:
        raise ValueError("damping must be >= 0")
    if damping == 0.0:
        return np.linalg.pinv(J) @ v
    m = J.shape[0]
    return J.T @ np.linalg.solve(J @ J.T + (damping ** 2) * np.eye(m), v)


def scale_to_limits(qdot: ArrayLike, limit: float) -> Array:
    if limit <= 0:
        raise ValueError("limit must be positive")
    qdot = np.asarray(qdot, dtype=float)
    biggest = float(np.max(np.abs(qdot))) if qdot.size else 0.0
    if biggest <= limit:
        return qdot.copy()
    return qdot * (limit / biggest)


def payload_torques(J: ArrayLike, force: ArrayLike) -> Array:
    return np.asarray(J, dtype=float).T @ np.asarray(force, dtype=float)


def worst_case_joint_speed(J: ArrayLike, speed: float) -> float:
    sigma_min = float(singular_values(J)[-1])
    if sigma_min <= 1e-12:
        return float("inf")
    return float(speed) / sigma_min
