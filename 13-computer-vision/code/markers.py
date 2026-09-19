"""Lesson 13.07 — AprilTag / ArUco detection and 6-DoF pose with solvePnP (IPPE_SQUARE).

  py markers.py                                  synthetic: the charging-dock tag seen by karmel
  py markers.py --print-tag                      write a printable tag36h11 id 0 (100 mm) PNG
  py markers.py --camera 0 --calib out/camera_info.yaml --tag-size 0.10   live poses

Works on OpenCV 4.6 (Ubuntu 24.04 apt) and >= 4.7 / 5.x through aruco_compat.py.
"""
from __future__ import annotations

import argparse
import math

import cv2
import numpy as np

from aruco_compat import (detect_markers, estimate_marker_pose, generate_marker, get_dictionary)
from camera_model import inv_T, karmel_camera, karmel_T_base_optical, make_T, rot_z
from synthetic import marker_scene
from webcam import open_camera, out_path

TAG_DICT = cv2.aruco.DICT_APRILTAG_36h11
DOCK_TAG_ID = 0
TAG_SIZE_M = 0.10            # edge of the black square, measured on the printout

# Tag frame when it faces the robot squarely: x = robot's right (-y), y = up (+z), z = toward robot (-x)
R_BASE_TAG_FACING = np.array([[0.0, 0.0, -1.0],
                              [-1.0, 0.0, 0.0],
                              [0.0, 1.0, 0.0]])


def dock_tag_pose(x: float, y: float, z: float, yaw_rad: float) -> np.ndarray:
    """T_base_tag for a tag at (x, y, z) in base_footprint, rotated yaw_rad about the vertical."""
    return make_T(rot_z(yaw_rad) @ R_BASE_TAG_FACING, (x, y, z))


def tag_yaw_in_base(T_base_tag: np.ndarray) -> float:
    """Heading the robot must face to look straight at the tag = direction of -z_tag."""
    zt = T_base_tag[:3, 2]
    return math.atan2(-zt[1], -zt[0])


def docking_goal(T_base_tag: np.ndarray, standoff_m: float = 0.30) -> tuple[float, float, float]:
    """(x, y, heading) in base_footprint: standoff_m in front of the tag, facing it."""
    p = T_base_tag[:3, 3] + standoff_m * T_base_tag[:3, 2]
    return float(p[0]), float(p[1]), tag_yaw_in_base(T_base_tag)


def detect_and_estimate(gray: np.ndarray, K: np.ndarray, dist, tag_size_m: float = TAG_SIZE_M):
    """[(id, T_optical_tag, reprojection errors, ambiguity ratio, corners)] for every detected tag."""
    dictionary = get_dictionary(TAG_DICT)
    corners, ids, _ = detect_markers(gray, dictionary)
    out = []
    if ids is None:
        return out
    for c, i in zip(corners, ids.ravel()):
        T, errs, ratio = estimate_marker_pose(c, tag_size_m, K, dist)
        out.append((int(i), T, errs, ratio, c.reshape(4, 2)))
    return out


def rotation_angle_deg(R: np.ndarray) -> float:
    return math.degrees(math.acos(np.clip((np.trace(R) - 1) / 2, -1, 1)))


def synthetic_demo() -> None:
    cam = karmel_camera()
    K = cam.K
    T_bo = karmel_T_base_optical()
    rng = np.random.default_rng(7)

    T_base_tag = dock_tag_pose(1.2, 0.25, 0.15, math.radians(-20))
    T_opt_tag = inv_T(T_bo) @ T_base_tag
    img = marker_scene(cam, T_opt_tag, rng, DOCK_TAG_ID, TAG_SIZE_M, TAG_DICT)
    cv2.imwrite(str(out_path("13.07-dock-view.png")), img)

    found = detect_and_estimate(img, K, None)
    print(f"detected ids: {[f[0] for f in found]}")
    if not found:
        return
    _, T_est, errs, ratio, corners = found[0]
    T_base_est = T_bo @ T_est
    pos_err = np.linalg.norm(T_base_est[:3, 3] - T_base_tag[:3, 3])
    rot_err = rotation_angle_deg(T_base_est[:3, :3].T @ T_base_tag[:3, :3])
    print(f"tag in camera_optical: t = {np.round(T_est[:3, 3], 3)} m")
    print(f"tag in base_footprint: est {np.round(T_base_est[:3, 3], 3)}  truth {T_base_tag[:3, 3]}")
    print(f"position error {pos_err * 1000:.1f} mm, rotation error {rot_err:.2f} deg, "
          f"reprojection err best/second {errs[0]:.3f}/{errs[1]:.3f} px (ratio {ratio:.1f})")
    gx, gy, gth = docking_goal(T_base_est)
    tx, ty, tth = docking_goal(T_base_tag)
    print(f"docking goal 0.30 m in front of the tag: x={gx:.3f} y={gy:.3f} heading={math.degrees(gth):.1f} deg "
          f"(truth {tx:.3f}, {ty:.3f}, {math.degrees(tth):.1f} deg)")
    print(f"range {math.hypot(T_base_est[0, 3], T_base_est[1, 3]):.3f} m, "
          f"bearing {math.degrees(math.atan2(T_base_est[1, 3], T_base_est[0, 3])):.1f} deg")

    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    rvec, _ = cv2.Rodrigues(T_est[:3, :3])
    cv2.drawFrameAxes(vis, K, np.zeros(5), rvec, T_est[:3, 3], 0.05)
    cv2.imwrite(str(out_path("13.07-dock-axes.png")), vis)

    print("\nhow pose quality degrades with distance (tag facing the camera, 3 noise seeds each)")
    print(f"{'range m':>8} {'side px':>8} {'pos err mm':>11} {'rot err deg':>12} {'ambiguity ratio':>16}")
    for d in (0.5, 1.0, 2.0, 3.0, 4.0):
        pe, re, ra, side = [], [], [], 0.0
        for seed in range(3):
            truth = dock_tag_pose(d, 0.0, 0.145, math.radians(10))
            T_ot = inv_T(T_bo) @ truth
            im = marker_scene(cam, T_ot, np.random.default_rng(seed), DOCK_TAG_ID, TAG_SIZE_M, TAG_DICT)
            f = detect_and_estimate(im, K, None)
            if not f:
                continue
            _, Te, _, r, c = f[0]
            side = float(np.linalg.norm(c[0] - c[1]))
            Tb = T_bo @ Te
            pe.append(np.linalg.norm(Tb[:3, 3] - truth[:3, 3]) * 1000)
            re.append(rotation_angle_deg(Tb[:3, :3].T @ truth[:3, :3]))
            ra.append(r)
        if pe:
            print(f"{d:8.1f} {side:8.1f} {np.mean(pe):11.1f} {np.max(re):12.2f} {np.min(ra):16.2f}")
        else:
            print(f"{d:8.1f}   not detected")


def write_printable_tag(tag_id: int = DOCK_TAG_ID, tag_mm: float = 100.0, dpi: int = 300) -> None:
    dictionary = get_dictionary(TAG_DICT)
    cells = dictionary.markerSize + 2
    px = int(round(tag_mm / 25.4 * dpi / cells)) * cells       # whole pixels per cell
    marker = generate_marker(dictionary, tag_id, px, border_bits=1)
    margin = 2 * px // cells
    page = cv2.copyMakeBorder(marker, margin, margin, margin, margin, cv2.BORDER_CONSTANT, value=255)
    path = out_path(f"tag36h11_id{tag_id}_{tag_mm:.0f}mm_{dpi}dpi.png")
    cv2.imwrite(str(path), page)
    print(f"wrote {path}: black square {px} px = {px / dpi * 25.4:.1f} mm at {dpi} dpi. "
          f"Print at 100% scale and MEASURE the black square.")


def live(camera: int, calib: str, tag_size: float, frames: int) -> None:
    import yaml
    info = yaml.safe_load(open(calib, encoding="utf-8"))
    K = np.array(info["camera_matrix"]["data"], dtype=float).reshape(3, 3)
    dist = np.array(info["distortion_coefficients"]["data"], dtype=float)
    T_bo = karmel_T_base_optical()
    cap = open_camera(camera, info["image_width"], info["image_height"])
    try:
        for _ in range(frames):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("no frame")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            for tag_id, T, errs, ratio, _ in detect_and_estimate(gray, K, dist, tag_size):
                Tb = T_bo @ T
                print(f"id {tag_id}: optical t={np.round(T[:3, 3], 3)}  base x={Tb[0, 3]:.3f} y={Tb[1, 3]:.3f} "
                      f"yaw-to-face={math.degrees(tag_yaw_in_base(Tb)):.1f} deg  err={errs[0]:.2f}px "
                      f"ratio={ratio:.1f}")
    finally:
        cap.release()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--print-tag", action="store_true")
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--calib", type=str, default="out/camera_info.yaml")
    ap.add_argument("--tag-size", type=float, default=TAG_SIZE_M)
    ap.add_argument("--frames", type=int, default=300)
    args = ap.parse_args()
    if args.print_tag:
        write_printable_tag()
    elif args.camera is not None:
        live(args.camera, args.calib, args.tag_size, args.frames)
    else:
        synthetic_demo()


if __name__ == "__main__":
    main()
