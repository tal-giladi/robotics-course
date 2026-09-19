"""05.04 — homogeneous transforms: karmel's bottle through the full 3D chain.

    python 05-frames-and-transforms/code/homogeneous_demo.py

Chain (every factor is T_parent_child, names cancel left to right):
    T_map_camera_optical_frame = T_map_base_footprint @ T_base_footprint_base_link
                                 @ T_base_link_camera_link @ T_camera_link_camera_optical_frame
"""

from __future__ import annotations

import math
import time

import numpy as np

import frames_math as fm


def karmel_bottle_chain() -> dict[str, np.ndarray]:
    s = fm.karmel_static_transforms()
    T_map_base_footprint = fm.se3_from_se2(fm.se2(2.0, 1.0, math.radians(90)))  # robot pose
    T_map_optical = fm.chain([
        ("map", "base_footprint", T_map_base_footprint),
        ("base_footprint", "base_link", s[("base_footprint", "base_link")]),
        ("base_link", "camera_link", s[("base_link", "camera_link")]),
        ("camera_link", "camera_optical_frame", s[("camera_link", "camera_optical_frame")]),
    ])
    T_base_link_optical = s[("base_link", "camera_link")] @ s[("camera_link", "camera_optical_frame")]
    return {
        "T_map_base_link": T_map_base_footprint @ s[("base_footprint", "base_link")],
        "T_base_link_optical": T_base_link_optical,
        "T_map_optical": T_map_optical,
        "T_base_link_laser": s[("base_link", "laser")],
    }


def fake_scan(n: int = 360, range_m: float = 2.0) -> np.ndarray:
    """A LiDAR scan of a 2 m circle, as (N, 3) points in the laser frame."""
    bearings = np.linspace(-math.pi, math.pi, n, endpoint=False)
    return np.column_stack([range_m * np.cos(bearings), range_m * np.sin(bearings), np.zeros(n)])


def main() -> None:
    np.set_printoptions(precision=4, suppress=True)
    T = karmel_bottle_chain()
    bottle_optical = np.array([0.05, -0.02, 0.60])

    print("T_base_link_optical =\n", T["T_base_link_optical"])
    print("T_map_optical =\n", T["T_map_optical"])

    # homogeneous coordinates: append w = 1 for a point, one matrix product does rotate + translate
    p_h = np.append(bottle_optical, 1.0)
    print("bottle in base_link:", (T["T_base_link_optical"] @ p_h)[:3])
    print("bottle in map:      ", (T["T_map_optical"] @ p_h)[:3])

    # inverse: closed form vs generic
    T_optical_map = fm.se3_inverse(T["T_map_optical"])
    print("closed-form inverse == np.linalg.inv:", np.allclose(T_optical_map, np.linalg.inv(T["T_map_optical"])))
    print("map origin in camera_optical_frame:", fm.transform_points(T_optical_map, [0.0, 0.0, 0.0]))

    # points (w = 1) vs directions (w = 0)
    v_base = np.array([0.2, 0.0, 0.0])             # karmel drives forward at 0.2 m/s
    print("velocity in map (w=0):", (T["T_map_base_link"] @ np.append(v_base, 0.0))[:3])
    print("same numbers as a point (w=1), WRONG for a velocity:", (T["T_map_base_link"] @ np.append(v_base, 1.0))[:3])

    # a whole scan at once
    scan_laser = fake_scan()
    T_map_laser = T["T_map_base_link"] @ T["T_base_link_laser"]
    t0 = time.perf_counter()
    looped = np.array([(T_map_laser @ np.append(p, 1.0))[:3] for p in scan_laser])
    t_loop = time.perf_counter() - t0
    fm.transform_points(T_map_laser, scan_laser)            # warm-up (first call allocates)
    t0 = time.perf_counter()
    for _ in range(100):
        batched = fm.transform_points(T_map_laser, scan_laser)
    t_batch = (time.perf_counter() - t0) / 100
    print(f"360 points: loop {t_loop * 1e3:.2f} ms, batched {t_batch * 1e3:.3f} ms, "
          f"equal: {np.allclose(looped, batched)}")
    print("beam 0 (bearing -180 deg) in map:", batched[0], " beam 180 (bearing 0) in map:", batched[180])

    # numerical drift: 100 000 small relative rotations (like integrating a gyro at 100 Hz for 17 min)
    R = np.eye(3)
    step = fm.rot_z(1e-3) @ fm.rot_x(1e-3)
    for _ in range(100_000):
        R = R @ step
    print(f"after 1e5 products: max |R^T R - I| = {np.abs(R.T @ R - np.eye(3)).max():.1e}, det = {np.linalg.det(R):.12f}")
    U, _, Vt = np.linalg.svd(R)
    R_fixed = U @ Vt
    print(f"after SVD re-orthonormalization:   max |R^T R - I| = {np.abs(R_fixed.T @ R_fixed - np.eye(3)).max():.1e}")


if __name__ == "__main__":
    main()
