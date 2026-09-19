"""Lesson 15.04 — from a depth image to a graspable 6-DoF object pose.

The pipeline, in the order the code runs it:

    depth image  ->  point cloud (optical frame)  ->  base frame (hand-eye X from 15.03)
                 ->  RANSAC table plane          ->  remove the table
                 ->  cluster what is left        ->  one cluster per object
                 ->  PCA on the footprint        ->  centre, yaw, extents, top height

    py object_pose.py                 the full pipeline and the tables printed in the lesson
    py object_pose.py --only bias     just the partial-view bias experiment

Nothing here needs a camera, ROS or Open3D: it is numpy plus the tiny renderer in
``tabletop_scene.py``. On the robot you get the depth image from ``realsense2_camera`` or
``depth_image_proc`` and the transform from TF2 instead (13.15, 13.16).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

from tabletop_scene import (Box, Camera, Cylinder, Scene, Sphere, camera_above, cluttered_scene,
                            demo_scene, depth_to_points, noisy_depth, transform_points)

Array = NDArray[np.float64]


# ============================================================================== table plane
@dataclass(frozen=True)
class Plane:
    """n . p + d = 0 with a unit normal. ``signed_distance`` is positive on the normal's side."""

    normal: tuple[float, float, float]
    d: float
    n_inliers: int = 0

    def signed_distance(self, pts: ArrayLike) -> Array:
        return np.asarray(pts, dtype=float) @ np.asarray(self.normal, dtype=float) + self.d

    @property
    def height(self) -> float:
        """Height of the plane above the origin along its own normal (z of the table, if level)."""
        return -self.d / self.normal[2] if abs(self.normal[2]) > 1e-9 else float("nan")


def fit_plane_ransac(points: ArrayLike, *, threshold_m: float = 0.006, iterations: int = 200,
                     rng: np.random.Generator | None = None, sample_size: int = 20_000,
                     refit_iterations: int = 2, up_hint: ArrayLike | None = (0.0, 0.0, 1.0),
                     max_tilt_deg: float = 25.0) -> Plane:
    """RANSAC plane fit, then a least-squares refit on the inliers.

    ``up_hint`` rejects candidate planes that are not roughly horizontal — a cheap prior that stops
    RANSAC from latching onto a wall or the side of a big box. ``threshold_m`` must be larger than
    the depth noise at the working distance (2-6 mm at 0.3-0.5 m) and smaller than the shortest
    object you must not swallow.
    """
    pts = np.asarray(points, dtype=float)
    rng = np.random.default_rng(0) if rng is None else rng
    # Score candidates on a random subset: 20k points give the same winner as 200k, 10x faster.
    sample = pts if len(pts) <= sample_size else pts[rng.choice(len(pts), sample_size, replace=False)]
    up = None if up_hint is None else np.asarray(up_hint, dtype=float)
    best_mask, best_n, best_d = None, None, 0.0
    best_count = -1
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

    # Least-squares refit: the RANSAC normal comes from 3 noisy points, this one from ~10^5.
    # Two rounds, because the first inlier set is biased — with millimetre-quantised depth, the
    # plane that wins the RANSAC vote is the one aligned with a quantisation shell, ~2 mm off.
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


# ============================================================================== clustering
def cluster_points(points: ArrayLike, *, voxel_m: float = 0.010, min_points: int = 150) -> list[Array]:
    """Voxel-grid connected components: the poor man's DBSCAN, and fast enough on a Pi.

    Points are dropped into a ``voxel_m`` grid; occupied voxels that touch (26-neighbourhood) join
    the same cluster. Choose ``voxel_m`` larger than the gaps noise leaves in a surface and smaller
    than the gap between two objects you must separate. Returns clusters sorted by size.
    """
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


# ============================================================================== object pose
@dataclass(frozen=True)
class ObjectPose:
    """A graspable pose: where the object is, how it is turned, and how big it is.

    centre:   (x, y, z) of the *visible* bounding box centre, in the base frame
    yaw:      rotation of the long footprint axis about z, wrapped to (-pi/2, pi/2]
    extents:  (long, short, height) in metres — long/short are the footprint axes
    top_z:    height of the highest point above the table plane
    n_points: how many points the estimate is based on (a quality number worth logging)
    """

    centre: tuple[float, float, float]
    yaw: float
    extents: tuple[float, float, float]
    top_z: float
    n_points: int

    @property
    def grasp_width(self) -> float:
        """What the jaws must span for a top-down grasp across the short axis."""
        return self.extents[1]


def wrap_axis_angle(a: float) -> float:
    """Wrap an *axis* angle (not a direction) to (-pi/2, pi/2]: PCA cannot tell a from a + pi."""
    a = (a + math.pi / 2) % math.pi - math.pi / 2
    return a if a > -math.pi / 2 else a + math.pi


def pose_from_cluster(points: ArrayLike, table: Plane) -> ObjectPose:
    """Principal-axis pose of one cluster, assuming the object stands on ``table``.

    The footprint (x, y after removing the table normal) is what a top-down grasp cares about, so
    PCA runs on two dimensions, not three. The eigenvector with the larger eigenvalue is the long
    axis; its angle is the yaw, defined only up to 180 degrees.
    """
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


# ============================================================================== whole pipeline
def segment_tabletop(depth_m: Array, cam: Camera, T_base_optical: Array, *,
                     plane_threshold_m: float = 0.006, voxel_m: float = 0.010,
                     min_points: int = 150, min_height_m: float = 0.005,
                     rng: np.random.Generator | None = None) -> tuple[Plane, list[ObjectPose]]:
    """depth image -> (table plane, one ObjectPose per object), all in the robot base frame."""
    pts_optical, _ = depth_to_points(depth_m, cam)
    pts = transform_points(T_base_optical, pts_optical)
    table = fit_plane_ransac(pts, threshold_m=plane_threshold_m, rng=rng)
    above = pts[table.signed_distance(pts) > max(plane_threshold_m, min_height_m)]
    clusters = cluster_points(above, voxel_m=voxel_m, min_points=min_points)
    return table, [pose_from_cluster(c, table) for c in clusters]


def match_to_truth(poses: list[ObjectPose], scene: Scene) -> list[tuple[str, ObjectPose | None]]:
    """Nearest-centre matching of estimates to the scene's objects, for scoring only."""
    out = []
    for obj in scene.objects:
        c = np.array(truth_centre_xy(obj))
        best, best_d = None, 1e9
        for p in poses:
            dist = float(np.linalg.norm(np.array(p.centre[:2]) - c))
            if dist < best_d:
                best, best_d = p, dist
        out.append((obj.name, best if best_d < 0.06 else None))
    return out


def truth_centre_xy(obj) -> tuple[float, float]:
    if isinstance(obj, Cylinder):
        return obj.centre_xy
    return (obj.centre[0], obj.centre[1])


def truth_footprint(obj) -> tuple[float, float, float]:
    """(long, short, top_z) ground truth of an object's footprint."""
    if isinstance(obj, Box):
        sx, sy, sz = obj.size
        return (max(sx, sy), min(sx, sy), obj.centre[2] + sz / 2)
    if isinstance(obj, Cylinder):
        return (2 * obj.radius, 2 * obj.radius, obj.height)
    return (2 * obj.radius, 2 * obj.radius, obj.centre[2] + obj.radius)


# ============================================================================== demos
def demo_pipeline() -> None:
    cam, scene = Camera(), demo_scene()
    rng = np.random.default_rng(4)
    T_base_optical = camera_above(tilt_deg=60.0)
    depth = noisy_depth(scene.depth(cam, T_base_optical), rng)
    table, poses = segment_tabletop(depth, cam, T_base_optical, rng=rng)

    print(f"--- table: normal {np.round(table.normal, 4)}, height {table.height * 1000:+.1f} mm, "
          f"{table.n_inliers} inliers ---")
    print(f"found {len(poses)} objects (truth: {len(scene.objects)})\n")
    print(f"{'object':<14}{'pts':>6}{'dx mm':>8}{'dy mm':>8}{'yaw err':>9}"
          f"{'long mm':>10}{'short mm':>10}{'top mm':>9}")
    for name, est in match_to_truth(poses, scene):
        obj = next(o for o in scene.objects if o.name == name)
        tx, ty = truth_centre_xy(obj)
        tl, ts, ttop = truth_footprint(obj)
        if est is None:
            print(f"{name:<14}{'MISSED':>6}")
            continue
        yaw_err = "-" if not isinstance(obj, Box) else \
            f"{math.degrees(abs(wrap_axis_angle(est.yaw - obj.yaw))):.1f} deg"
        print(f"{name:<14}{est.n_points:>6}{(est.centre[0] - tx) * 1000:>8.1f}"
              f"{(est.centre[1] - ty) * 1000:>8.1f}{yaw_err:>9}"
              f"{f'{est.extents[0] * 1000:.0f}/{tl * 1000:.0f}':>10}"
              f"{f'{est.extents[1] * 1000:.0f}/{ts * 1000:.0f}':>10}"
              f"{f'{est.top_z * 1000:.0f}/{ttop * 1000:.0f}':>9}")
    print("  (estimate/truth for the size columns)\n")


def demo_bias() -> None:
    print("--- viewing angle: what the camera can and cannot measure (90 deg = straight down) ---")
    print(f"{'tilt':>5} {'object':<14}{'centre err mm':>14}{'width mm est/true':>20}{'height mm est/true':>20}")
    cam, scene = Camera(), demo_scene()
    for tilt in (30.0, 45.0, 60.0, 75.0, 90.0):
        T = camera_above(tilt_deg=tilt)
        clean = scene.depth(cam, T)
        rng = np.random.default_rng(11)
        table, poses = segment_tabletop(noisy_depth(clean, rng), cam, T, rng=rng)
        for name, est in match_to_truth(poses, scene):
            if name not in ("drinks can", "wooden block"):
                continue
            obj = next(o for o in scene.objects if o.name == name)
            tx, ty = truth_centre_xy(obj)
            _, ts, ttop = truth_footprint(obj)
            if est is None:
                print(f"{tilt:>5.0f} {name:<14}{'MISSED':>14}")
                continue
            err = math.hypot(est.centre[0] - tx, est.centre[1] - ty) * 1000
            print(f"{tilt:>5.0f} {name:<14}{err:>14.1f}"
                  f"{f'{est.extents[1] * 1000:.0f} / {ts * 1000:.0f}':>20}"
                  f"{f'{est.top_z * 1000:.0f} / {ttop * 1000:.0f}':>20}")
    print("  the footprint is well observed from any downward view, but a shallow view inflates the")
    print("  measured width (the visible side face plus noise), which costs you gripper clearance.\n")


def demo_parameters() -> None:
    cam = Camera()
    T = camera_above(tilt_deg=60.0)
    scene = demo_scene()
    clean = scene.depth(cam, T)
    truth = len(scene.objects)
    print(f"--- plane threshold: {truth} objects on the table, the flattest is the 10 mm phone ---")
    print(f"{'plane thr mm':>13}{'objects found':>15}  notes")
    for thr_mm in (2, 6, 12, 25):
        found = []
        for seed in range(3):
            rng = np.random.default_rng(seed)
            _, poses = segment_tabletop(noisy_depth(clean, rng), cam, T,
                                        plane_threshold_m=thr_mm / 1000, rng=rng)
            found.append(len(poses))
        n = float(np.mean(found))
        note = ("table fragments survive as fake objects" if n > truth + 0.2 else
                "flat objects are swallowed by the plane" if n < truth - 0.2 else "")
        print(f"{thr_mm:>13}{n:>15.1f}  {note}")
    print()

    print("--- clustering voxel: two 30 mm blocks with a 12 mm gap (should be 2 objects) ---")
    clutter = cluttered_scene(gap_m=0.012)
    clean2 = clutter.depth(cam, T)
    print(f"{'voxel mm':>9}{'objects found':>15}  notes")
    for voxel_mm in (4, 6, 10, 20):
        found = []
        for seed in range(3):
            rng = np.random.default_rng(seed)
            _, poses = segment_tabletop(noisy_depth(clean2, rng), cam, T,
                                        voxel_m=voxel_mm / 1000, rng=rng)
            found.append(len(poses))
        n = float(np.mean(found))
        note = ("the two blocks merged into one" if n < 1.8 else
                "one surface broke into pieces" if n > 2.2 else "")
        print(f"{voxel_mm:>9}{n:>15.1f}  {note}")
    print()


def demo_noise() -> None:
    print("--- how depth noise reaches the grasp (tilt 75 deg, drinks can, 8 seeds) ---")
    cam, scene = Camera(), demo_scene()
    T = camera_above(tilt_deg=75.0)
    clean = scene.depth(cam, T)
    print(f"{'k (sigma = k Z^2)':>18}{'sigma at 0.36 m':>17}{'centre std mm':>15}{'width std mm':>14}")
    for k in (0.0, 0.001, 0.002, 0.005, 0.010):
        cs, ws = [], []
        for seed in range(8):
            rng = np.random.default_rng(500 + seed)
            _, poses = segment_tabletop(noisy_depth(clean, rng, k=k), cam, T, rng=rng)
            for name, est in match_to_truth(poses, scene):
                if name == "drinks can" and est is not None:
                    cs.append(est.centre[:2])
                    ws.append(est.extents[1])
        cs = np.asarray(cs)
        print(f"{k:>18.3f}{f'{k * 0.36 ** 2 * 1000:.2f} mm':>17}"
              f"{float(np.hypot(*cs.std(axis=0))) * 1000:>15.2f}{float(np.std(ws)) * 1000:>14.2f}")
    print()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["pipeline", "bias", "parameters", "noise"])
    args = ap.parse_args(argv)
    if args.only in (None, "pipeline"):
        demo_pipeline()
    if args.only in (None, "bias"):
        demo_bias()
    if args.only in (None, "parameters"):
        demo_parameters()
    if args.only in (None, "noise"):
        demo_noise()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
