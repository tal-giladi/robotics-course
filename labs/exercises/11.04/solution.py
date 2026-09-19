"""11.04 — Scan matching with point-to-point ICP (reference solution)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import KDTree

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class IcpResult:
    x: float
    y: float
    theta: float
    rmse: float
    inlier_fraction: float
    iterations: int
    converged: bool


def rotation(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def apply(pose: Pose, points: np.ndarray) -> np.ndarray:
    x, y, theta = pose
    return np.asarray(points, dtype=float) @ rotation(theta).T + np.array([x, y])


def compose(a: Pose, b: Pose) -> Pose:
    ax, ay, ath = a
    bx, by = rotation(ath) @ np.array(b[:2]) + np.array([ax, ay])
    return (float(bx), float(by), math.atan2(math.sin(ath + b[2]), math.cos(ath + b[2])))


def best_fit_transform(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    src = np.asarray(src, dtype=float).reshape(-1, 2)
    dst = np.asarray(dst, dtype=float).reshape(-1, 2)
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    return R, mu_d - R @ mu_s


def icp(
    source: np.ndarray,
    target: np.ndarray,
    initial: Pose = (0.0, 0.0, 0.0),
    max_iterations: int = 50,
    tolerance: float = 1e-6,
    max_correspondence_distance: float = 0.5,
) -> IcpResult:
    source = np.asarray(source, dtype=float).reshape(-1, 2)
    target = np.asarray(target, dtype=float).reshape(-1, 2)
    tree = KDTree(target)
    T: Pose = tuple(float(v) for v in initial)  # type: ignore[assignment]
    converged, iterations = False, 0
    for iterations in range(1, max_iterations + 1):
        moved = apply(T, source)
        dist, idx = tree.query(moved)
        keep = dist < max_correspondence_distance
        if keep.sum() < 3:
            break
        R, t = best_fit_transform(moved[keep], target[idx[keep]])
        step_theta = math.atan2(R[1, 0], R[0, 0])
        T = compose((float(t[0]), float(t[1]), step_theta), T)
        if math.hypot(t[0], t[1]) < tolerance and abs(step_theta) < tolerance:
            converged = True
            break
    dist, _ = tree.query(apply(T, source))
    keep = dist < max_correspondence_distance
    rmse = float(np.sqrt(np.mean(dist[keep] ** 2))) if keep.any() else math.inf
    return IcpResult(T[0], T[1], T[2], rmse, float(keep.mean()), iterations, converged)


def match_is_reliable(result: IcpResult, min_inlier_fraction: float = 0.7, max_rmse: float = 0.05) -> bool:
    return result.converged and result.inlier_fraction >= min_inlier_fraction and result.rmse <= max_rmse
