"""11.04 — Scan matching with point-to-point ICP.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 11.04``.
Uses numpy and ``scipy.spatial.KDTree`` (nearest-neighbour search).

Conventions: 2D points are ``(N, 2)`` arrays in meters. A pose / rigid transform is a tuple
``(x, y, theta)``; applying it to a point p gives ``R(theta) @ p + (x, y)``. ICP estimates the
transform that moves the SOURCE scan onto the TARGET scan: ``target ≈ apply(T, source)``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import KDTree  # noqa: F401  (you will need it in icp)

Pose = tuple[float, float, float]


@dataclass(frozen=True)
class IcpResult:
    x: float
    y: float
    theta: float  # the transform source -> target
    rmse: float  # RMS distance of the inlier pairs at the final estimate, m
    inlier_fraction: float  # share of source points whose nearest target point is within the gate
    iterations: int
    converged: bool  # True if the last correction was below the tolerance


# --- given helpers ----------------------------------------------------------------------------------
def rotation(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def apply(pose: Pose, points: np.ndarray) -> np.ndarray:
    """Transform ``(N, 2)`` points by ``pose``."""
    x, y, theta = pose
    return np.asarray(points, dtype=float) @ rotation(theta).T + np.array([x, y])


def compose(a: Pose, b: Pose) -> Pose:
    """``a ∘ b``: first apply b, then a (like matrix product A @ B)."""
    ax, ay, ath = a
    bx, by = rotation(ath) @ np.array(b[:2]) + np.array([ax, ay])
    return (float(bx), float(by), math.atan2(math.sin(ath + b[2]), math.cos(ath + b[2])))


# --- your code --------------------------------------------------------------------------------------
def best_fit_transform(src: np.ndarray, dst: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The rotation R (2x2, det = +1) and translation t (2,) minimizing Σ |R @ src_i + t - dst_i|².

    ``src[i]`` and ``dst[i]`` are known to correspond. Kabsch / Arun et al.:
      1. centroids μs, μd; centered point sets
      2. H = Σ (src_i - μs)(dst_i - μd)ᵀ   (2x2)
      3. U, S, Vᵀ = svd(H);  R = V Uᵀ
      4. if det(R) < 0 (a reflection): flip the sign of the last row of Vᵀ and recompute R
      5. t = μd - R μs
    """
    # TODO(student)
    raise NotImplementedError("best_fit_transform")


def icp(
    source: np.ndarray,
    target: np.ndarray,
    initial: Pose = (0.0, 0.0, 0.0),
    max_iterations: int = 50,
    tolerance: float = 1e-6,
    max_correspondence_distance: float = 0.5,
) -> IcpResult:
    """Point-to-point ICP starting from the guess ``initial``.

    Repeat up to ``max_iterations`` times:
      1. moved = apply(T, source)
      2. nearest target point for every moved point (build ONE KDTree(target) before the loop)
      3. keep only pairs closer than ``max_correspondence_distance`` (outlier rejection);
         stop (not converged) if fewer than 3 pairs remain
      4. R, t = best_fit_transform(moved[kept], target[partner[kept]])
      5. T = compose((t[0], t[1], atan2(R[1, 0], R[0, 0])), T)
      6. converged if |t| < tolerance and |step angle| < tolerance: stop
    Finally compute rmse and inlier_fraction at the final T with the same gate.
    """
    # TODO(student)
    raise NotImplementedError("icp")


def match_is_reliable(result: IcpResult, min_inlier_fraction: float = 0.7, max_rmse: float = 0.05) -> bool:
    """Accept a match only if ICP converged, enough points found a partner, and the partners are close."""
    # TODO(student)
    raise NotImplementedError("match_is_reliable")
