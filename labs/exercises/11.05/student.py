"""11.05 — Pose graphs and loop closure: a 2D pose-graph optimizer (Gauss-Newton on SE(2)).

Fill in every ``TODO(student)``. Check your work with ``python course.py check 11.05``.
Only numpy is needed.

Conventions: a pose is a numpy array ``[x, y, theta]`` (meters, radians, theta wrapped to
(-pi, pi]). ``poses`` is an ``(N, 3)`` array, one row per graph node. An edge from node i to node
j carries the measurement ``z = [x, y, theta]``: "node j, seen from node i's frame, is at z".
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Edge:
    i: int
    j: int
    z: np.ndarray  # (3,) measured pose of node j in the frame of node i
    information: np.ndarray  # (3, 3) inverse covariance of z


# --- given helpers ----------------------------------------------------------------------------------
def wrap(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def rot(theta: float) -> np.ndarray:
    """2x2 rotation matrix."""
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def chi2(poses: np.ndarray, edges: list[Edge]) -> float:
    """Total weighted squared error Σ eᵀ Ω e — what the optimizer minimizes."""
    total = 0.0
    for edge in edges:
        e = edge_error(poses[edge.i], poses[edge.j], edge.z)
        total += float(e @ edge.information @ e)
    return total


# --- your code --------------------------------------------------------------------------------------
def relative_pose(xi: np.ndarray, xj: np.ndarray) -> np.ndarray:
    """Pose of j expressed in the frame of i: [Riᵀ (tj - ti), wrap(θj - θi)]."""
    # TODO(student)
    raise NotImplementedError("relative_pose")


def edge_error(xi: np.ndarray, xj: np.ndarray, z: np.ndarray) -> np.ndarray:
    """Residual of one edge, expressed in the measurement frame:

        e_xy = R(θz)ᵀ · (R(θi)ᵀ · (tj - ti) - tz)
        e_θ  = wrap(θj - θi - θz)

    It is zero when the two poses agree exactly with the measurement.
    """
    # TODO(student)
    raise NotImplementedError("edge_error")


def edge_jacobians(xi: np.ndarray, xj: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """A = ∂e/∂xi and B = ∂e/∂xj, both 3x3:

        A = [ -Rzᵀ Riᵀ    Rzᵀ (dRiᵀ/dθi) (tj - ti) ]      B = [ Rzᵀ Riᵀ   0 ]
            [  0    0                -1            ]          [ 0    0    1 ]

    with dRᵀ/dθ = [[-sin θ, cos θ], [-cos θ, -sin θ]].
    """
    # TODO(student)
    raise NotImplementedError("edge_jacobians")


def optimize_pose_graph(poses: np.ndarray, edges: list[Edge], iterations: int = 20, tolerance: float = 1e-6) -> np.ndarray:
    """Gauss-Newton. Returns a NEW (N, 3) array; node 0 stays where it is (the anchor).

    Each iteration:
      1. H = zeros(3N, 3N), b = zeros(3N)
      2. for every edge: e, (A, B); with Ω = edge.information and blocks i = 3i:3i+3, j = 3j:3j+3
             H[i,i] += AᵀΩA   H[i,j] += AᵀΩB   H[j,i] += BᵀΩA   H[j,j] += BᵀΩB
             b[i]   += AᵀΩe   b[j]   += BᵀΩe
      3. anchor node 0: H[0:3, 0:3] += 1e9 * I   (otherwise H is singular: moving the whole
         map costs nothing)
      4. dx = solve(H, -b); poses += dx reshaped to (N, 3); wrap every theta
      5. stop when |dx| < tolerance
    """
    # TODO(student)
    raise NotImplementedError("optimize_pose_graph")
