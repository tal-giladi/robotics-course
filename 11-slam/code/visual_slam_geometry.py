#!/usr/bin/env python3
"""The three numbers that decide whether visual SLAM works on your robot (lesson 11.08).

A LiDAR measures range directly. A camera measures only *bearing*: the pixel a point lands on.
Depth has to be triangulated from two views, and that is where visual SLAM's strengths and its
failure modes both come from. This script computes, for karmel's camera:

1. `depth_uncertainty` — how depth error grows with range, baseline and pixel noise. This is why
   a 6 cm stereo baseline is useless at 5 m and a moving monocular camera can do better.
2. `scale_drift` — a monocular map has no unit. A 1% error in the scale estimate compounds.
3. `bundle_adjustment` — a two-view bundle adjustment on synthetic data: minimize *reprojection*
   error in pixels over camera poses AND 3D points, the visual analogue of 11.05's pose graph.

    python 11-slam/code/visual_slam_geometry.py
    python 11-slam/code/visual_slam_geometry.py --seed 3

No ROS, no camera, no GPU: numpy + scipy only.
"""
from __future__ import annotations

import argparse
import math

import numpy as np
from scipy.optimize import least_squares

# karmel's camera (labs/config/karmel.yaml): 640 x 480, horizontal FOV 1.20 rad.
WIDTH_PX = 640
HEIGHT_PX = 480
HFOV_RAD = 1.20
FOCAL_PX = (WIDTH_PX / 2.0) / math.tan(HFOV_RAD / 2.0)


def intrinsics(focal: float = FOCAL_PX) -> np.ndarray:
    """Pinhole camera matrix K for karmel's camera (13.05)."""
    return np.array([[focal, 0.0, WIDTH_PX / 2.0],
                     [0.0, focal, HEIGHT_PX / 2.0],
                     [0.0, 0.0, 1.0]])


# --------------------------------------------------------------------------- 1. depth from bearing
def depth_uncertainty(depth_m: float, baseline_m: float, focal_px: float = FOCAL_PX,
                      pixel_noise_px: float = 0.5) -> tuple[float, float]:
    """Disparity and 1-sigma depth error for a point at `depth_m` seen from two views.

    Z = f * B / d, so dZ/dd = -Z^2 / (f * B) and sigma_Z = Z^2 * sigma_d / (f * B).
    Returns (disparity_px, sigma_Z_m).
    """
    disparity = focal_px * baseline_m / depth_m
    sigma_z = depth_m ** 2 * pixel_noise_px / (focal_px * baseline_m)
    return disparity, sigma_z


# --------------------------------------------------------------------------- 2. monocular scale
def scale_drift(true_length_m: float, scale_error: float, segments: int) -> float:
    """Length a monocular system reports after `segments` hand-offs, each off by `scale_error`.

    Monocular SLAM fixes the unit once (from the first baseline) and every later triangulation
    inherits it. A per-segment relative error compounds: (1 + e)^n.
    """
    return true_length_m * (1.0 + scale_error) ** segments


# --------------------------------------------------------------------------- 3. bundle adjustment
def project(points_xyz: np.ndarray, pose: np.ndarray, K: np.ndarray) -> np.ndarray:
    """Project world points into a camera at `pose` = (x, y, z, rx, ry, rz) (Rodrigues rotation).

    Camera looks down its +z axis, x right, y down (OpenCV convention, 13.05).
    """
    t = pose[:3]
    rvec = pose[3:6]
    theta = np.linalg.norm(rvec)
    if theta < 1e-12:
        R = np.eye(3)
    else:
        k = rvec / theta
        Kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
        R = np.eye(3) + math.sin(theta) * Kx + (1.0 - math.cos(theta)) * (Kx @ Kx)
    cam = (points_xyz - t) @ R          # world -> camera
    uv = cam @ K.T
    return uv[:, :2] / uv[:, 2:3]


BASELINE_M = 0.30          # how far the robot drove between the two views


def ba_residuals(params: np.ndarray, n_views: int, n_points: int, observations: list,
                 K: np.ndarray, fixed_pose: np.ndarray, scale_weight: float = 0.0) -> np.ndarray:
    """Stacked reprojection residuals in pixels. Camera 0 is fixed (the gauge, like 11.05's anchor).

    With `scale_weight > 0` one more residual pins the distance from camera 0 to camera 1 at
    BASELINE_M — what a stereo rig, an RGB-D camera, wheel odometry or an IMU would tell you.
    Without it the problem has one free direction (scale) that no pixel can observe.
    """
    n_free = n_views - 1
    poses = [fixed_pose] + [params[6 * k:6 * k + 6] for k in range(n_free)]
    pts = params[6 * n_free:].reshape(n_points, 3)
    out = [(project(pts, pose, K) - uv).ravel() for pose, uv in zip(poses, observations)]
    if scale_weight > 0.0:
        out.append(np.array([scale_weight * (float(np.linalg.norm(poses[1][:3])) - BASELINE_M)]))
    return np.concatenate(out)


def true_poses(n_views: int) -> list[np.ndarray]:
    """Camera 0 at the origin, then one BASELINE_M step to the right per view, yawing slightly."""
    return [np.array([BASELINE_M * k, 0.0, 0.0, 0.0, -0.05 * k, 0.0]) for k in range(n_views)]


def bundle_adjustment(seed: int = 0, n_views: int = 2, n_points: int = 40,
                      pixel_noise_px: float = 0.5, scale_known: bool = False,
                      verbose: bool = True) -> dict:
    """Views of a wall of points; perturb the poses and the structure, then optimize."""
    rng = np.random.default_rng(seed)
    K = intrinsics()

    # Truth: points on a rough wall 3 m in front of camera 0.
    pts_true = np.column_stack([rng.uniform(-1.2, 1.2, n_points),
                                rng.uniform(-0.8, 0.8, n_points),
                                3.0 + rng.uniform(-0.4, 0.4, n_points)])
    poses_true = true_poses(n_views)
    obs = [project(pts_true, p, K) + rng.normal(0, pixel_noise_px, (n_points, 2))
           for p in poses_true]

    # Initial guess: the front end's estimate — poses off by ~4 cm / 1 deg, points off by ~8 cm.
    bump = np.array([-0.04, 0.01, 0.02, 0.0, 0.02, 0.005])
    poses_guess = [poses_true[k] + bump * k for k in range(1, n_views)]
    pts_guess = pts_true + rng.normal(0, 0.08, pts_true.shape)
    x0 = np.concatenate([np.concatenate(poses_guess), pts_guess.ravel()])
    weight = 1000.0 if scale_known else 0.0

    args = (n_views, n_points, obs, K, poses_true[0], weight)
    before = ba_residuals(x0, *args)
    result = least_squares(ba_residuals, x0, args=args, method='trf', xtol=1e-12, ftol=1e-12)

    n_free = n_views - 1
    poses_opt = [result.x[6 * k:6 * k + 6] for k in range(n_free)]
    pts_opt = result.x[6 * n_free:].reshape(n_points, 3)
    n_pix = 2 * n_views * n_points

    def pose_err(poses):
        return float(np.mean([np.linalg.norm(p[:3] - t[:3])
                              for p, t in zip(poses, poses_true[1:])]))

    # A monocular reconstruction is only defined up to scale. Rescale it so that its first
    # baseline is BASELINE_M and *then* compare: that separates "wrong unit" from "wrong shape".
    gauge = BASELINE_M / max(np.linalg.norm(poses_opt[0][:3]), 1e-9)
    pts_gauged = pts_opt * gauge
    poses_gauged = [np.concatenate([p[:3] * gauge, p[3:]]) for p in poses_opt]

    stats = {
        'n_views': n_views, 'scale_known': scale_known,
        'unknowns': len(x0), 'residuals': len(before),
        'rmse_px_before': float(np.sqrt(np.mean(before[:n_pix] ** 2))),
        'rmse_px_after': float(np.sqrt(np.mean(result.fun[:n_pix] ** 2))),
        'pose_err_before_m': pose_err(poses_guess),
        'pose_err_after_m': pose_err(poses_opt),
        'point_err_before_m': float(np.mean(np.linalg.norm(pts_guess - pts_true, axis=1))),
        'point_err_after_m': float(np.mean(np.linalg.norm(pts_opt - pts_true, axis=1))),
        'pose_err_gauged_m': pose_err(poses_gauged),
        'point_err_gauged_m': float(np.mean(np.linalg.norm(pts_gauged - pts_true, axis=1))),
        'baseline_after_m': float(np.linalg.norm(poses_opt[0][:3])),
    }
    if verbose:
        label = 'baseline known' if scale_known else 'scale free    '
        print(f"  {n_views} views, {label} | {stats['unknowns']:3d} unknowns, "
              f"{n_pix:4d} residuals | "
              f"reproj {stats['rmse_px_before']:5.2f} -> {stats['rmse_px_after']:.2f} px | "
              f"baseline {stats['baseline_after_m']:6.3f} m | "
              f"points {stats['point_err_before_m'] * 100:5.1f} -> {stats['point_err_after_m'] * 100:7.1f} cm"
              f" ({stats['point_err_gauged_m'] * 100:5.1f} cm after rescaling)")
    return stats


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    print(f"karmel camera: {WIDTH_PX}x{HEIGHT_PX}, hfov {HFOV_RAD} rad "
          f"-> focal {FOCAL_PX:.1f} px, {math.degrees(HFOV_RAD / WIDTH_PX) * 60:.2f} arcmin/pixel")
    print()
    print("1. depth uncertainty (0.5 px matching noise)")
    print("   range   stereo B=6 cm            monocular B=30 cm (robot drove)")
    print("           disparity  sigma_Z       disparity  sigma_Z")
    for z in (0.5, 1.0, 2.0, 3.0, 5.0, 8.0):
        d6, s6 = depth_uncertainty(z, 0.06)
        d30, s30 = depth_uncertainty(z, 0.30)
        print(f"   {z:4.1f} m  {d6:7.1f} px  {s6 * 100:6.1f} cm     "
              f"{d30:7.1f} px  {s30 * 100:6.1f} cm")
    print()

    print("2. monocular scale drift (a 12 m hallway, mapped in 20 segments)")
    for e in (0.0, 0.005, 0.01, 0.02):
        print(f"   {e * 100:4.1f}% per segment -> reported length "
              f"{scale_drift(12.0, e, 20):6.2f} m "
              f"({(scale_drift(12.0, e, 20) / 12.0 - 1) * 100:+5.1f}%)")
    print()

    print("3. bundle adjustment: 40 points on a wall 3 m away, 0.5 px matching noise")
    for views in (2, 3, 5, 9):
        for known in (False, True):
            bundle_adjustment(seed=args.seed, n_views=views, scale_known=known)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
