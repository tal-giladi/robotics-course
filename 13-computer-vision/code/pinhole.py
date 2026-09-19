"""Lesson 13.05 — the pinhole model: K, extrinsics, projection, back-projection, distance to the ball.

  py pinhole.py
"""
from __future__ import annotations

import math
import textwrap

import cv2
import numpy as np

from camera_model import (distance_from_apparent_size, ground_point_from_pixel, inv_T,
                          karmel_camera, karmel_T_base_optical, transform_points)
from find_ball import BallDetectorConfig, find_ball
from synthetic import BALL_RADIUS_M, ball_scene


def main() -> None:
    cam = karmel_camera()
    K = cam.K
    print("1) intrinsics from the field of view (640x480, HFOV 1.20 rad)")
    print(f"   f = (640/2) / tan(1.20/2) = {cam.fx:.1f} px")
    print(textwrap.indent("K =\n" + np.array2string(K, precision=1, suppress_small=True), "   "))
    print(f"   vertical FOV = 2*atan(240/f) = {2 * math.atan(240 / cam.fy):.3f} rad")

    print("2) extrinsics: base_footprint -> camera_optical_frame")
    T_bo = karmel_T_base_optical()
    T_ob = inv_T(T_bo)
    print(textwrap.indent("T_base_optical =\n" + np.array2string(T_bo, precision=3, suppress_small=True), "   "))

    print("3) project the ball center (base_footprint x=1.1, y=0.1, z=0.035)")
    p_base = np.array([[1.1, 0.1, BALL_RADIUS_M]])
    p_opt = transform_points(T_ob, p_base)
    uv, _ = cam.project(p_opt)
    rvec, _ = cv2.Rodrigues(T_ob[:3, :3])
    uv_cv, _ = cv2.projectPoints(p_base, rvec, T_ob[:3, 3], K, None)
    print(f"   in optical frame: X={p_opt[0, 0]:.3f} Y={p_opt[0, 1]:.3f} Z={p_opt[0, 2]:.3f} m")
    print(f"   numpy:  u={uv[0, 0]:.2f} v={uv[0, 1]:.2f}   cv2.projectPoints: "
          f"u={uv_cv[0, 0, 0]:.2f} v={uv_cv[0, 0, 1]:.2f}")

    print("4) back-project that pixel with the known depth Z -> the same 3D point")
    back = cam.back_project(uv, p_opt[0, 2])
    print(f"   back-projected: {np.round(back[0], 4)}   (a pixel alone is only a ray)")

    print("5) distance to a detected ball: apparent size vs. ground-plane ray")
    print(f"   {'true x':>7} {'r px':>6} {'size-based x':>13} {'err':>7} {'ray-based x':>12} {'err':>7}")
    rng = np.random.default_rng(11)
    cfg = BallDetectorConfig(hsv_lo=(5, 160, 25), min_area_px=30)
    for x in (0.5, 1.0, 1.5, 2.0, 3.0):
        img, truth = ball_scene(rng, ball_xy=(x, 0.0))
        det = find_ball(img, cfg)
        if det is None:
            print(f"   {x:7.2f}  not detected")
            continue
        # size: Z along the optical axis, then to base x (camera sits 0.10 m ahead of base)
        z_size = distance_from_apparent_size(det.radius_px, BALL_RADIUS_M, cam.fx)
        x_size = z_size + T_bo[0, 3]
        # ray: intersect the center ray with the plane of ball centers (z = ball radius)
        g = ground_point_from_pixel(cam, T_bo, (det.u, det.v), plane_z=BALL_RADIUS_M)
        x_ray = g[0] if g is not None else float("nan")
        print(f"   {x:7.2f} {det.radius_px:6.1f} {x_size:13.3f} {x_size - x:+7.3f} {x_ray:12.3f} {x_ray - x:+7.3f}")

    print("6) sensitivity at 3 m: what does a 1 px mistake cost?")
    Z = 3.0 - T_bo[0, 3]
    r = cam.fx * BALL_RADIUS_M / Z
    dz_size = Z / r
    h = T_bo[2, 3] - BALL_RADIUS_M
    dz_ray = Z * Z / (cam.fy * h)
    print(f"   radius {r:.2f} px -> 1 px radius error = {dz_size:.2f} m")
    print(f"   center {cam.fy * h / Z:.2f} px below the horizon -> 1 px row error = {dz_ray:.2f} m")
    tilt = math.radians(2.0)
    g = ground_point_from_pixel(cam, karmel_T_base_optical(pitch_down_rad=tilt),
                                cam.project(transform_points(T_ob, [[3.0, 0.0, BALL_RADIUS_M]]))[0][0],
                                plane_z=BALL_RADIUS_M)
    print(f"   camera really level, but you assume 2 deg pitch-down: ray-based x = {g[0]:.2f} m (truth 3.00)")


if __name__ == "__main__":
    main()
