"""11.05 — Pose graphs and loop closure (reference solution)."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Edge:
    i: int
    j: int
    z: np.ndarray
    information: np.ndarray


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def chi2(poses: np.ndarray, edges: list[Edge]) -> float:
    total = 0.0
    for edge in edges:
        e = edge_error(poses[edge.i], poses[edge.j], edge.z)
        total += float(e @ edge.information @ e)
    return total


def relative_pose(xi: np.ndarray, xj: np.ndarray) -> np.ndarray:
    xi, xj = np.asarray(xi, dtype=float), np.asarray(xj, dtype=float)
    t = rot(xi[2]).T @ (xj[:2] - xi[:2])
    return np.array([t[0], t[1], wrap(xj[2] - xi[2])])


def edge_error(xi: np.ndarray, xj: np.ndarray, z: np.ndarray) -> np.ndarray:
    xi, xj, z = (np.asarray(v, dtype=float) for v in (xi, xj, z))
    t = rot(z[2]).T @ (rot(xi[2]).T @ (xj[:2] - xi[:2]) - z[:2])
    return np.array([t[0], t[1], wrap(xj[2] - xi[2] - z[2])])


def edge_jacobians(xi: np.ndarray, xj: np.ndarray, z: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    xi, xj, z = (np.asarray(v, dtype=float) for v in (xi, xj, z))
    Rz_T, Ri_T = rot(z[2]).T, rot(xi[2]).T
    s, c = math.sin(xi[2]), math.cos(xi[2])
    dRi_T = np.array([[-s, c], [-c, -s]])
    A = np.zeros((3, 3))
    A[:2, :2] = -Rz_T @ Ri_T
    A[:2, 2] = Rz_T @ dRi_T @ (xj[:2] - xi[:2])
    A[2, 2] = -1.0
    B = np.zeros((3, 3))
    B[:2, :2] = Rz_T @ Ri_T
    B[2, 2] = 1.0
    return A, B


def optimize_pose_graph(poses: np.ndarray, edges: list[Edge], iterations: int = 20, tolerance: float = 1e-6) -> np.ndarray:
    x = np.array(poses, dtype=float).reshape(-1, 3).copy()
    n = len(x)
    for _ in range(iterations):
        H = np.zeros((3 * n, 3 * n))
        b = np.zeros(3 * n)
        for edge in edges:
            e = edge_error(x[edge.i], x[edge.j], edge.z)
            A, B = edge_jacobians(x[edge.i], x[edge.j], edge.z)
            om = edge.information
            si, sj = slice(3 * edge.i, 3 * edge.i + 3), slice(3 * edge.j, 3 * edge.j + 3)
            H[si, si] += A.T @ om @ A
            H[si, sj] += A.T @ om @ B
            H[sj, si] += B.T @ om @ A
            H[sj, sj] += B.T @ om @ B
            b[si] += A.T @ om @ e
            b[sj] += B.T @ om @ e
        H[:3, :3] += 1e9 * np.eye(3)
        dx = np.linalg.solve(H, -b)
        x += dx.reshape(-1, 3)
        x[:, 2] = np.arctan2(np.sin(x[:, 2]), np.cos(x[:, 2]))
        if np.linalg.norm(dx) < tolerance:
            break
    return x
