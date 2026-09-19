"""15.03 — reference solution. Same public names as student.py.

Mirrors ``15-manipulation/code/hand_eye.py`` (without the optional nonlinear refinement).
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


def rot_xyz(roll: float, pitch: float, yaw: float) -> Array:
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def make_T(R: ArrayLike, t: ArrayLike) -> Array:
    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_T(T: ArrayLike) -> Array:
    T = np.asarray(T, dtype=float)
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def rotation_angle_deg(R: ArrayLike) -> float:
    return math.degrees(float(np.linalg.norm(rotvec_of(R))))


def pose_error(T_est: ArrayLike, T_true: ArrayLike) -> tuple[float, float]:
    d = inv_T(np.asarray(T_true, float)) @ np.asarray(T_est, float)
    return float(np.linalg.norm(d[:3, 3]) * 1000.0), rotation_angle_deg(d[:3, :3])


def rotvec_of(R: ArrayLike) -> Array:
    R = np.asarray(R, dtype=float)
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    theta = math.acos(c)
    if theta < 1e-9:
        return np.zeros(3)
    if math.pi - theta < 1e-6:                      # near 180 deg: use the symmetric part
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        k = int(np.argmax(axis))
        axis = A[:, k] / axis[k] if axis[k] > 1e-9 else axis
        axis = axis / np.linalg.norm(axis)
        return axis * theta
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (theta / (2.0 * math.sin(theta)))


def rot_of(rotvec: ArrayLike) -> Array:
    r = np.asarray(rotvec, dtype=float).reshape(3)
    theta = float(np.linalg.norm(r))
    if theta < 1e-12:
        return np.eye(3)
    k = r / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def motion_pairs(T_base_grippers: list[Array], T_cam_targets: list[Array],
                 *, eye_in_hand: bool = True) -> tuple[list[Array], list[Array]]:
    n = len(T_base_grippers)
    if n != len(T_cam_targets) or n < 3:
        raise ValueError("need at least 3 matching pose pairs (use >= 10 in practice)")
    A, B = [], []
    for i in range(n):
        for j in range(i + 1, n):
            gi, gj = T_base_grippers[i], T_base_grippers[j]
            A.append(inv_T(gj) @ gi if eye_in_hand else gj @ inv_T(gi))
            B.append(T_cam_targets[j] @ inv_T(T_cam_targets[i]))
    return A, B


def solve_ax_xb_park(A: list[Array], B: list[Array]) -> Array:
    M = np.zeros((3, 3))
    for Ai, Bi in zip(A, B, strict=True):
        M += np.outer(rotvec_of(Bi[:3, :3]), rotvec_of(Ai[:3, :3]))
    w, V = np.linalg.eigh(M.T @ M)
    w = np.maximum(w, 1e-18)
    inv_sqrt = V @ np.diag(1.0 / np.sqrt(w)) @ V.T
    R_X = inv_sqrt @ M.T
    u, _, vt = np.linalg.svd(R_X)
    R_X = u @ vt
    if np.linalg.det(R_X) < 0:
        R_X = u @ np.diag([1.0, 1.0, -1.0]) @ vt

    C = np.zeros((3 * len(A), 3))
    d = np.zeros(3 * len(A))
    for k, (Ai, Bi) in enumerate(zip(A, B, strict=True)):
        C[3 * k:3 * k + 3, :] = Ai[:3, :3] - np.eye(3)
        d[3 * k:3 * k + 3] = R_X @ Bi[:3, 3] - Ai[:3, 3]
    t_X, *_ = np.linalg.lstsq(C, d, rcond=None)
    return make_T(R_X, t_X)


def calibrate_eye_in_hand(T_base_grippers: list[Array], T_cam_targets: list[Array]) -> Array:
    A, B = motion_pairs(T_base_grippers, T_cam_targets, eye_in_hand=True)
    return solve_ax_xb_park(A, B)


def calibrate_eye_to_hand(T_base_grippers: list[Array], T_cam_targets: list[Array]) -> Array:
    A, B = motion_pairs(T_base_grippers, T_cam_targets, eye_in_hand=False)
    return solve_ax_xb_park(A, B)


def consistency_residual(T_base_grippers: list[Array], T_cam_targets: list[Array],
                         X: Array) -> tuple[float, float]:
    poses = [T_bg @ X @ T_ct for T_bg, T_ct in zip(T_base_grippers, T_cam_targets, strict=True)]
    centre = np.mean([T[:3, 3] for T in poses], axis=0)
    t_rms = float(np.sqrt(np.mean([np.sum((T[:3, 3] - centre) ** 2) for T in poses])) * 1000.0)
    R0 = poses[0][:3, :3]
    r_rms = float(np.sqrt(np.mean([rotation_angle_deg(R0.T @ T[:3, :3]) ** 2 for T in poses])))
    return t_rms, r_rms


def rotation_spread_deg(T_base_grippers: list[Array]) -> float:
    n = len(T_base_grippers)
    angles = [rotation_angle_deg((inv_T(T_base_grippers[j]) @ T_base_grippers[i])[:3, :3])
              for i in range(n) for j in range(i + 1, n)]
    return float(np.mean(angles))
