"""Lesson 13.08 — stereo disparity, depth images -> point clouds, RANSAC plane, objects on the table.

  py depth3d.py                  synthetic tabletop (numpy + OpenCV only)
  py depth3d.py --open3d         also run Open3D's segment_plane / DBSCAN if open3d is installed

Writes out/13.08-*.png and out/tabletop.ply (open it in MeshLab, CloudCompare or Open3D).
"""
from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

from camera_model import PinholeCamera, inv_T, karmel_camera, transform_points
from synthetic import TabletopWorld, noisy_depth, stereo_pair, tabletop, tabletop_camera_pose
from webcam import out_path


# ----------------------------------------------------------------------------- stereo
def disparity_sgbm(left: np.ndarray, right: np.ndarray, num_disp: int = 128, block: int = 5) -> np.ndarray:
    """Disparity in pixels (float32), <= 0 where invalid. num_disp must be divisible by 16."""
    sgbm = cv2.StereoSGBM_create(minDisparity=0, numDisparities=num_disp, blockSize=block,
                                 P1=8 * block * block, P2=32 * block * block, uniquenessRatio=10,
                                 speckleWindowSize=100, speckleRange=2, disp12MaxDiff=1,
                                 mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)
    return sgbm.compute(left, right).astype(np.float32) / 16.0      # SGBM returns fixed-point x16


def disparity_to_depth(disp: np.ndarray, f_px: float, baseline_m: float) -> np.ndarray:
    """Z = f * B / d; 0 where the disparity is invalid."""
    with np.errstate(divide="ignore"):
        z = np.where(disp > 0.5, f_px * baseline_m / disp, 0.0)
    return z.astype(np.float32)


# ----------------------------------------------------------------------------- point clouds
def depth_to_points(depth_m: np.ndarray, cam: PinholeCamera, stride: int = 1) -> np.ndarray:
    """(N,3) points in camera_optical from a depth image (Z per pixel, 0 = invalid). Vectorized."""
    d = depth_m[::stride, ::stride]
    v, u = np.nonzero(d > 0)
    z = d[v, u].astype(np.float64)
    u = u * stride
    v = v * stride
    x = (u - cam.cx) / cam.fx * z
    y = (v - cam.cy) / cam.fy * z
    return np.column_stack((x, y, z))


def depth_from_uint16_mm(raw: np.ndarray) -> np.ndarray:
    """RealSense / OAK / ROS 16UC1 depth: millimeters, 0 = no data."""
    return raw.astype(np.float32) / 1000.0


def write_ply(path, points: np.ndarray) -> None:
    header = (f"ply\nformat ascii 1.0\nelement vertex {len(points)}\n"
              "property float x\nproperty float y\nproperty float z\nend_header\n")
    with open(path, "w", encoding="ascii") as f:
        f.write(header)
        np.savetxt(f, points, fmt="%.4f")


# ----------------------------------------------------------------------------- RANSAC plane
@dataclass(frozen=True)
class Plane:
    normal: np.ndarray     # unit (3,)
    d: float               # plane: normal . p + d = 0
    inliers: np.ndarray    # boolean mask over the input points


def ransac_iterations(inlier_fraction: float, sample_size: int = 3, confidence: float = 0.99) -> int:
    return math.ceil(math.log(1 - confidence) / math.log(1 - inlier_fraction ** sample_size))


def fit_plane_svd(points: np.ndarray) -> tuple[np.ndarray, float]:
    c = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - c, full_matrices=False)
    n = vt[2]
    return n, float(-n @ c)


def ransac_plane(points: np.ndarray, threshold_m: float, iterations: int,
                 rng: np.random.Generator) -> Plane:
    """Largest plane: sample 3 points, count points within threshold, keep the best, refit with SVD."""
    best_count, best = -1, None
    n_pts = len(points)
    for _ in range(iterations):
        p0, p1, p2 = points[rng.choice(n_pts, 3, replace=False)]
        n = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(n)
        if norm < 1e-9:                       # collinear sample
            continue
        n /= norm
        dist = np.abs(points @ n - n @ p0)
        count = int(np.count_nonzero(dist < threshold_m))
        if count > best_count:
            best_count, best = count, (n, float(-n @ p0))
    n, d = best
    inl = np.abs(points @ n + d) < threshold_m
    n, d = fit_plane_svd(points[inl])
    inl = np.abs(points @ n + d) < threshold_m
    return Plane(n, d, inl)


# ----------------------------------------------------------------------------- objects on the table
@dataclass(frozen=True)
class TableObject:
    centroid_world: np.ndarray
    size_xy_m: tuple[float, float]
    height_m: float
    n_points: int


def objects_on_plane(points_world: np.ndarray, plane: Plane, min_h: float = 0.01, max_h: float = 0.4,
                     cell_m: float = 0.01, min_points: int = 50) -> list[TableObject]:
    """Points above the plane -> 2D occupancy grid on the table -> connected components = objects."""
    n = plane.normal if plane.normal[2] > 0 else -plane.normal    # normal pointing up
    d = plane.d if plane.normal[2] > 0 else -plane.d
    h = points_world @ n + d
    above = points_world[(h > min_h) & (h < max_h)]
    heights = h[(h > min_h) & (h < max_h)]
    if len(above) == 0:
        return []
    ij = np.floor(above[:, :2] / cell_m).astype(int)
    ij0 = ij.min(axis=0)
    ij -= ij0
    grid = np.zeros(ij.max(axis=0)[::-1] + 1, np.uint8)
    grid[ij[:, 1], ij[:, 0]] = 255
    grid = cv2.morphologyEx(grid, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    n_labels, labels = cv2.connectedComponents(grid)
    point_label = labels[ij[:, 1], ij[:, 0]]
    objects = []
    for k in range(1, n_labels):
        sel = point_label == k
        if np.count_nonzero(sel) < min_points:
            continue
        pts = above[sel]
        lo, hi = pts.min(axis=0), pts.max(axis=0)
        objects.append(TableObject(pts.mean(axis=0), (float(hi[0] - lo[0]), float(hi[1] - lo[1])),
                                   float(heights[sel].max()), int(np.count_nonzero(sel))))
    return sorted(objects, key=lambda o: -o.n_points)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--open3d", action="store_true")
    ap.add_argument("--baseline", type=float, default=0.075)
    args = ap.parse_args()
    cam = karmel_camera()
    rng = np.random.default_rng(3)
    world = TabletopWorld()
    T_wc = tabletop_camera_pose()

    # 1. stereo
    print(f"1) stereo, baseline {args.baseline * 100:.1f} cm, f = {cam.fx:.1f} px")
    left, right, z_true = stereo_pair(cam, T_wc, args.baseline, rng, world)
    t0 = time.perf_counter()
    disp = disparity_sgbm(left, right)
    ms = (time.perf_counter() - t0) * 1000
    z_st = disparity_to_depth(disp, cam.fx, args.baseline)
    valid = (z_st > 0) & (z_true > 0)
    err = np.abs(z_st - z_true)[valid]
    print(f"   SGBM {ms:.0f} ms, valid pixels {100 * np.count_nonzero(valid) / valid.size:.0f}%, "
          f"depth error median {np.median(err) * 1000:.1f} mm, 90th pct {np.percentile(err, 90) * 1000:.1f} mm")
    for zq in (0.5, 1.0, 2.0):
        d = cam.fx * args.baseline / zq
        dz = zq * zq / (cam.fx * args.baseline) * 0.25
        print(f"   at Z={zq:.1f} m: disparity {d:5.1f} px; a 0.25 px disparity error = {dz * 1000:5.1f} mm")
    cv2.imwrite(str(out_path("13.08-left.png")), left)
    cv2.imwrite(str(out_path("13.08-right.png")), right)
    cv2.imwrite(str(out_path("13.08-disparity.png")),
                cv2.normalize(np.maximum(disp, 0), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8))

    # 2. depth image -> point cloud (as a depth camera would give it: uint16 millimeters)
    print("2) depth image -> point cloud")
    z_clean, _ = tabletop(cam, T_wc, world)
    raw_mm = np.round(noisy_depth(z_clean, rng) * 1000).astype(np.uint16)
    depth = depth_from_uint16_mm(raw_mm)
    t0 = time.perf_counter()
    pts_cam = depth_to_points(depth, cam)
    ms = (time.perf_counter() - t0) * 1000
    pts_world = transform_points(T_wc, pts_cam)
    print(f"   {len(pts_cam)} points from a {depth.shape[1]}x{depth.shape[0]} depth image in {ms:.1f} ms")
    near = pts_world[np.linalg.norm(pts_cam, axis=1) < 1.5]
    write_ply(out_path("tabletop.ply"), near[::2])

    # 3. RANSAC plane on points within 1.5 m of the camera (table + objects + a bit of floor)
    print("3) RANSAC table plane")
    table_candidates = near
    it = ransac_iterations(0.6)
    print(f"   iterations for 60% inliers, 99% confidence: {it}")
    t0 = time.perf_counter()
    plane = ransac_plane(table_candidates, threshold_m=0.008, iterations=it, rng=rng)
    ms = (time.perf_counter() - t0) * 1000
    n = plane.normal if plane.normal[2] > 0 else -plane.normal
    tilt = math.degrees(math.acos(min(1.0, n[2])))
    print(f"   normal {np.round(n, 4)} (tilt from vertical {tilt:.2f} deg), "
          f"height {-(plane.d if plane.normal[2] > 0 else -plane.d) * 1000:+.1f} mm, "
          f"inliers {100 * plane.inliers.mean():.0f}%, {ms:.0f} ms")

    # 4. what's on the table?
    print("4) objects above the table")
    objs = objects_on_plane(table_candidates, plane)
    truths = {"box": (np.array(world.box_min) + np.array(world.box_max)) / 2, "ball": np.array(world.ball_center)}
    for o in objs:
        name, t = min(truths.items(), key=lambda kv: np.linalg.norm(kv[1][:2] - o.centroid_world[:2]))
        print(f"   object: centroid {np.round(o.centroid_world, 3)}  footprint {o.size_xy_m[0] * 100:.1f} x "
              f"{o.size_xy_m[1] * 100:.1f} cm  height {o.height_m * 100:.1f} cm  ({o.n_points} pts) "
              f"-> nearest truth: {name} center {np.round(t, 3)}")

    if args.open3d:
        try:
            import open3d as o3d
        except ImportError:
            print("open3d is not installed (pip install open3d; wheels may lag the newest Python)")
            return
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(table_candidates))
        pcd = pcd.voxel_down_sample(0.005)
        model, inl = pcd.segment_plane(distance_threshold=0.008, ransac_n=3, num_iterations=it * 5)
        print(f"   open3d plane {np.round(model, 4)}, {len(inl)} inliers")
        rest = pcd.select_by_index(inl, invert=True)
        labels = np.array(rest.cluster_dbscan(eps=0.02, min_points=20))
        print(f"   open3d DBSCAN clusters above/around the plane: {labels.max() + 1}")


if __name__ == "__main__":
    main()
