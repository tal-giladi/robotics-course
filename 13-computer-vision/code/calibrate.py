"""Lesson 13.06 — lens distortion and checkerboard calibration with OpenCV.

  py calibrate.py                               synthetic: 20 views through a known lens, recover it
  py calibrate.py --views 4 --poses fronto      the classic mistake: few, flat, centered views
  py calibrate.py --capture 25 --camera 0       save webcam frames in which the board is found
  py calibrate.py --images "out/calib/*.png"    calibrate a real camera from saved frames

Board: 10 x 7 squares = 9 x 6 inner corners, 25 mm squares (print at 100%, measure it!).
Writes out/camera_info.yaml in the ROS sensor_msgs/CameraInfo YAML format (camera_info_manager).
"""
from __future__ import annotations

import argparse
import glob
import time

import cv2
import numpy as np

from camera_model import PinholeCamera, make_T, rot_x, rot_y
from synthetic import (checkerboard_image, checkerboard_object_points, random_board_poses,
                       undistort_normalized)
from webcam import open_camera, out_path

PATTERN = (9, 6)                   # inner corners per row, per column
SQUARE_M = 0.025
# The "true" lens for the synthetic experiment: a wide webcam lens with strong barrel distortion
# (exaggerated so you can see it; a C920 at 640x480 is usually milder -- calibrate yours).
TRUE_CAM = PinholeCamera(fx=471.3, fy=469.8, cx=324.6, cy=236.9, width=640, height=480)
TRUE_DIST = np.array([-0.28, 0.11, 0.0008, -0.0006, -0.02])


def find_corners(gray: np.ndarray, pattern=PATTERN) -> np.ndarray | None:
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not found:
        return None
    crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), crit)
    return corners.reshape(-1, 1, 2)       # OpenCV 4.x returns (N,1,2), 5.x (N,2): normalize


def calibrate(images: list[np.ndarray], pattern=PATTERN, square_m: float = SQUARE_M) -> dict:
    obj = checkerboard_object_points(pattern[0], pattern[1], square_m)
    obj_pts, img_pts, used = [], [], []
    for i, img in enumerate(images):
        gray = img if img.ndim == 2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners = find_corners(gray, pattern)
        if corners is not None:
            obj_pts.append(obj)
            img_pts.append(corners)
            used.append(i)
    if len(used) < 3:
        raise RuntimeError(f"board found in only {len(used)} images; need at least 3 (10-25 recommended)")
    h, w = images[0].shape[:2]
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_pts, img_pts, (w, h), None, None)
    per_view = []
    for o, ip, rv, tv in zip(obj_pts, img_pts, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, rv, tv, K, dist)
        per_view.append(float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - ip.reshape(-1, 2)) ** 2, axis=1)))))
    return {"rms": rms, "K": K, "dist": dist.ravel(), "per_view": per_view, "used": used,
            "size": (w, h), "img_pts": img_pts}


def fronto_poses(rng: np.random.Generator, n: int) -> list[np.ndarray]:
    """Board straight in front of the camera, centered, barely tilted: a bad calibration set."""
    center = checkerboard_object_points(*PATTERN, SQUARE_M).mean(axis=0)
    poses = []
    for _ in range(n):
        R = rot_x(rng.uniform(-0.05, 0.05)) @ rot_y(rng.uniform(-0.05, 0.05))
        target = np.array([rng.uniform(-0.01, 0.01), rng.uniform(-0.01, 0.01), rng.uniform(0.45, 0.5)])
        poses.append(make_T(R, target - R @ center))
    return poses


def camera_info_yaml(K: np.ndarray, dist: np.ndarray, size: tuple[int, int], name: str = "karmel_camera") -> str:
    """ROS camera_info_manager YAML. For a monocular camera P = [K | 0] (no rectification)."""
    def row(vals):
        return "[" + ", ".join(f"{v:.6f}" for v in vals) + "]"
    P = np.hstack([K, np.zeros((3, 1))])
    d5 = list(dist[:5]) + [0.0] * max(0, 5 - len(dist))
    return (f"image_width: {size[0]}\nimage_height: {size[1]}\ncamera_name: {name}\n"
            f"camera_matrix:\n  rows: 3\n  cols: 3\n  data: {row(K.ravel())}\n"
            f"distortion_model: plumb_bob\n"
            f"distortion_coefficients:\n  rows: 1\n  cols: 5\n  data: {row(d5)}\n"
            f"rectification_matrix:\n  rows: 3\n  cols: 3\n  data: {row(np.eye(3).ravel())}\n"
            f"projection_matrix:\n  rows: 3\n  cols: 4\n  data: {row(P.ravel())}\n")


def line_bend_px(corners: np.ndarray, pattern=PATTERN) -> float:
    """Worst deviation (px) of a corner from the straight line through its board row."""
    pts = corners.reshape(pattern[1], pattern[0], 2)
    worst = 0.0
    for r in pts:
        centered = r - r.mean(axis=0)
        _, _, vt = np.linalg.svd(centered)
        normal = vt[1]
        worst = max(worst, float(np.max(np.abs(centered @ normal))))
    return worst


def capture(camera: int, n: int) -> None:
    folder = out_path("calib")
    folder.mkdir(exist_ok=True)
    cap = open_camera(camera)
    saved, last = 0, 0.0
    print("move the board slowly: near/far, tilted, into every corner of the image")
    try:
        while saved < n:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("no frame")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if time.time() - last > 1.5 and find_corners(gray) is not None:
                cv2.imwrite(str(folder / f"view_{saved:02d}.png"), frame)
                saved, last = saved + 1, time.time()
                print(f"saved view {saved}/{n}")
    finally:
        cap.release()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--views", type=int, default=20)
    ap.add_argument("--poses", choices=["varied", "fronto"], default="varied")
    ap.add_argument("--images", type=str, default=None)
    ap.add_argument("--capture", type=int, default=None)
    ap.add_argument("--camera", type=int, default=0)
    args = ap.parse_args()

    if args.capture:
        capture(args.camera, args.capture)
        return

    if args.images:
        files = sorted(glob.glob(args.images))
        images = [cv2.imread(f) for f in files]
        if not images:
            raise SystemExit(f"no images match {args.images}")
        res = calibrate(images)
        print(f"board found in {len(res['used'])}/{len(images)} images")
    else:
        rng = np.random.default_rng(21)
        poses = (random_board_poses(TRUE_CAM, rng, args.views, *PATTERN, SQUARE_M) if args.poses == "varied"
                 else fronto_poses(rng, args.views))
        images = [checkerboard_image(TRUE_CAM, TRUE_DIST, T, rng, *PATTERN, SQUARE_M) for T in poses]
        for i, img in enumerate(images[:3]):
            cv2.imwrite(str(out_path(f"13.06-view-{i}.png")), img)
        res = calibrate(images)

    K, dist = res["K"], res["dist"]
    np.set_printoptions(precision=4, suppress=True)
    print(f"views used: {len(res['used'])}   RMS reprojection error: {res['rms']:.3f} px")
    print(f"per-view RMS: min {min(res['per_view']):.3f}  max {max(res['per_view']):.3f} px "
          f"(worst view index {res['used'][int(np.argmax(res['per_view']))]})")
    print(f"fx={K[0, 0]:.1f} fy={K[1, 1]:.1f} cx={K[0, 2]:.1f} cy={K[1, 2]:.1f}")
    print(f"dist (k1 k2 p1 p2 k3) = {dist[:5]}")
    if not args.images:
        t = TRUE_CAM
        print(f"truth: fx={t.fx:.1f} fy={t.fy:.1f} cx={t.cx:.1f} cy={t.cy:.1f}  dist = {TRUE_DIST}")
        # how far does the *estimated* model misplace pixels, compared with the true lens?
        grid = np.stack(np.meshgrid(np.linspace(0, 639, 17), np.linspace(0, 479, 13)), -1).reshape(-1, 2)
        xn, yn = (grid[:, 0] - t.cx) / t.fx, (grid[:, 1] - t.cy) / t.fy
        xu, yu = undistort_normalized(xn, yn, TRUE_DIST)
        rays = np.column_stack((xu, yu, np.ones_like(xu)))
        proj, _ = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), K, dist)
        err = np.linalg.norm(proj.reshape(-1, 2) - grid, axis=1)
        inner = (np.abs(grid[:, 0] - 320) <= 256) & (np.abs(grid[:, 1] - 240) <= 192)
        print(f"model error vs. true lens: central 80% of the image mean {err[inner].mean():.2f} px "
              f"max {err[inner].max():.2f} px; outer border max {err[~inner].max():.2f} px")

    w, h = res["size"]
    xu, yu = undistort_normalized(np.array([-K[0, 2] / K[0, 0]]), np.array([-K[1, 2] / K[1, 1]]), dist)
    shift = float(np.hypot(xu[0] * K[0, 0] + K[0, 2], yu[0] * K[1, 1] + K[1, 2]))
    print(f"distortion: the top-left pixel (0,0) shows what an ideal pinhole camera would put at "
          f"({xu[0] * K[0, 0] + K[0, 2]:.1f}, {yu[0] * K[1, 1] + K[1, 2]:.1f}) -> {shift:.1f} px shift")

    # undistortion straightens lines
    k = int(np.argmax([line_bend_px(c) for c in res["img_pts"]]))       # the most bent view
    img0 = images[res["used"][k]]
    gray0 = img0 if img0.ndim == 2 else cv2.cvtColor(img0, cv2.COLOR_BGR2GRAY)
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, (w, h), alpha=0.0)
    und = cv2.undistort(gray0, K, dist, None, newK)
    before = res["img_pts"][k]
    after = find_corners(und)
    msg = f"{line_bend_px(after):.2f} px" if after is not None else "board not found"
    print(f"row straightness, view {res['used'][k]}: before undistort {line_bend_px(before):.2f} px, after {msg}")
    cv2.imwrite(str(out_path("13.06-undistorted.png")), und)
    path = out_path("camera_info.yaml")
    path.write_text(camera_info_yaml(K, dist, (w, h)), encoding="utf-8")
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
