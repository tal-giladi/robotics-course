"""15.04 — reference solution. Same public names as student.py.

Mirrors ``15-manipulation/code/object_pose.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


@dataclass(frozen=True)
class Plane:
    normal: tuple[float, float, float]
    d: float
    n_inliers: int = 0

    def signed_distance(self, pts: ArrayLike) -> Array:
        return np.asarray(pts, dtype=float) @ np.asarray(self.normal, dtype=float) + self.d

    @property
    def height(self) -> float:
        return -self.d / self.normal[2] if abs(self.normal[2]) > 1e-9 else float("nan")


@dataclass(frozen=True)
class ObjectPose:
    centre: tuple[float, float, float]
    yaw: float
    extents: tuple[float, float, float]
    top_z: float
    n_points: int

    @property
    def grasp_width(self) -> float:
        return self.extents[1]


def fit_plane_ransac(points: ArrayLike, *, threshold_m: float = 0.006, iterations: int = 200,
                     rng: np.random.Generator | None = None, sample_size: int = 20_000,
                     refit_iterations: int = 2, up_hint: ArrayLike | None = (0.0, 0.0, 1.0),
                     max_tilt_deg: float = 25.0) -> Plane:
    pts = np.asarray(points, dtype=float)
    rng = np.random.default_rng(0) if rng is None else rng
    sample = pts if len(pts) <= sample_size else pts[rng.choice(len(pts), sample_size, replace=False)]
    up = None if up_hint is None else np.asarray(up_hint, dtype=float)
    best_n, best_d, best_count = None, 0.0, -1
    for _ in range(iterations):
        idx = rng.choice(len(sample), size=3, replace=False)
        a, b, c = sample[idx]
        n = np.cross(b - a, c - a)
        norm = float(np.linalg.norm(n))
        if norm < 1e-9:
            continue
        n = n / norm
        if up is not None and abs(float(n @ up)) < math.cos(math.radians(max_tilt_deg)):
            continue
        d = -float(n @ a)
        count = int((np.abs(sample @ n + d) < threshold_m).sum())
        if count > best_count:
            best_count, best_n, best_d = count, n, d
    if best_n is None:
        raise RuntimeError("RANSAC found no plane: check threshold_m and up_hint")

    n, d = best_n, best_d
    mask = np.abs(pts @ n + d) < threshold_m
    for _ in range(refit_iterations):
        inliers = pts[mask]
        centroid = inliers.mean(axis=0)
        _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
        n = vt[-1]
        if up is not None and float(n @ up) < 0:
            n = -n
        d = -float(n @ centroid)
        mask = np.abs(pts @ n + d) < threshold_m
    return Plane(tuple(n), d, int(mask.sum()))


def cluster_points(points: ArrayLike, *, voxel_m: float = 0.010, min_points: int = 150) -> list[Array]:
    pts = np.asarray(points, dtype=float)
    if len(pts) == 0:
        return []
    keys = np.floor(pts / voxel_m).astype(np.int64)
    uniq, inverse = np.unique(keys, axis=0, return_inverse=True)
    index = {tuple(k): i for i, k in enumerate(uniq)}

    label = np.full(len(uniq), -1, dtype=np.int64)
    offsets = [(dx, dy, dz) for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
               if (dx, dy, dz) != (0, 0, 0)]
    next_label = 0
    for start in range(len(uniq)):
        if label[start] >= 0:
            continue
        stack, label[start] = [start], next_label
        while stack:
            cur = stack.pop()
            k = uniq[cur]
            for off in offsets:
                nb = index.get((k[0] + off[0], k[1] + off[1], k[2] + off[2]))
                if nb is not None and label[nb] < 0:
                    label[nb] = next_label
                    stack.append(nb)
        next_label += 1

    clusters = [pts[label[inverse] == c] for c in range(next_label)]
    clusters = [c for c in clusters if len(c) >= min_points]
    return sorted(clusters, key=len, reverse=True)


def wrap_axis_angle(a: float) -> float:
    a = (a + math.pi / 2) % math.pi - math.pi / 2
    return a if a > -math.pi / 2 else a + math.pi


def pose_from_cluster(points: ArrayLike, table: Plane) -> ObjectPose:
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3:
        raise ValueError("need at least 3 points")
    heights = table.signed_distance(pts)
    xy = pts[:, :2] - pts[:, :2].mean(axis=0)
    cov = xy.T @ xy / len(xy)
    w, v = np.linalg.eigh(cov)
    long_axis = v[:, int(np.argmax(w))]
    yaw = wrap_axis_angle(math.atan2(long_axis[1], long_axis[0]))

    R = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    local = (pts[:, :2] - pts[:, :2].mean(axis=0)) @ R.T
    long_len = float(local[:, 0].max() - local[:, 0].min())
    short_len = float(local[:, 1].max() - local[:, 1].min())
    mid = pts[:, :2].mean(axis=0) + (np.array([local[:, 0].min() + long_len / 2,
                                               local[:, 1].min() + short_len / 2]) @ R)
    top = float(heights.max())
    return ObjectPose((float(mid[0]), float(mid[1]), float(table.height + top / 2.0)),
                      yaw, (long_len, short_len, top), table.height + top, len(pts))


def segment_tabletop(points: ArrayLike, *, plane_threshold_m: float = 0.006, voxel_m: float = 0.010,
                     min_points: int = 150, min_height_m: float = 0.005,
                     rng: np.random.Generator | None = None) -> tuple[Plane, list[ObjectPose]]:
    pts = np.asarray(points, dtype=float)
    table = fit_plane_ransac(pts, threshold_m=plane_threshold_m, rng=rng)
    above = pts[table.signed_distance(pts) > max(plane_threshold_m, min_height_m)]
    clusters = cluster_points(above, voxel_m=voxel_m, min_points=min_points)
    return table, [pose_from_cluster(c, table) for c in clusters]
