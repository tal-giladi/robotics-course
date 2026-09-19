"""Pinhole camera model and frame helpers used by lessons 13.03-13.08.

Pure numpy (OpenCV only where noted), so the same math runs on a laptop and on the robot.

Frames (REP-103 / REP-105):
  base_footprint   x forward, y left, z up, on the floor under the wheel axle midpoint
  camera_optical   z forward (out of the lens), x right, y down  -- the frame OpenCV and
                   sensor_msgs/CameraInfo use. Image frame_id = camera_optical_frame.

The karmel defaults come from labs/config/karmel.yaml: camera at x=0.10 m, z=0.10 m above
base_link, and base_link is one wheel radius (0.045 m) above base_footprint -> the lens is
0.145 m above the floor. 640x480, horizontal field of view 1.20 rad.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Optical frame axes expressed in the body frame (x fwd, y left, z up):
#   x_optical (right) = -y_body, y_optical (down) = -z_body, z_optical (forward) = +x_body
R_BODY_OPTICAL = np.array([[0.0, 0.0, 1.0],
                           [-1.0, 0.0, 0.0],
                           [0.0, -1.0, 0.0]])


@dataclass(frozen=True)
class PinholeCamera:
    """Intrinsics of an ideal (undistorted) pinhole camera, in pixels."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov_rad: float) -> "PinholeCamera":
        """Square pixels, principal point at the image center."""
        f = (width / 2.0) / math.tan(hfov_rad / 2.0)
        return cls(fx=f, fy=f, cx=width / 2.0, cy=height / 2.0, width=width, height=height)

    @classmethod
    def from_K(cls, K: np.ndarray, width: int, height: int) -> "PinholeCamera":
        return cls(float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2]), width, height)

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]])

    def scaled(self, s: float) -> "PinholeCamera":
        """Same camera at s times the resolution (used for supersampled rendering)."""
        return PinholeCamera(self.fx * s, self.fy * s, (self.cx + 0.5) * s - 0.5,
                             (self.cy + 0.5) * s - 0.5, int(self.width * s), int(self.height * s))

    def project(self, p_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """(N,3) points in camera_optical -> (N,2) pixels and a (N,) mask of points in front."""
        p = np.atleast_2d(np.asarray(p_cam, dtype=float))
        z = p[:, 2]
        in_front = z > 1e-9
        zs = np.where(in_front, z, np.nan)
        u = self.fx * p[:, 0] / zs + self.cx
        v = self.fy * p[:, 1] / zs + self.cy
        return np.column_stack((u, v)), in_front

    def back_project(self, uv: np.ndarray, depth_z: np.ndarray | float) -> np.ndarray:
        """(N,2) pixels + depth Z (meters along the optical axis) -> (N,3) points in camera_optical."""
        uv = np.atleast_2d(np.asarray(uv, dtype=float))
        z = np.broadcast_to(np.asarray(depth_z, dtype=float), (uv.shape[0],))
        x = (uv[:, 0] - self.cx) / self.fx * z
        y = (uv[:, 1] - self.cy) / self.fy * z
        return np.column_stack((x, y, z))

    def rays(self, uv: np.ndarray) -> np.ndarray:
        """(N,2) pixels -> (N,3) ray directions with z = 1 (not unit length)."""
        return self.back_project(uv, 1.0)


def karmel_camera() -> PinholeCamera:
    """The course robot's USB webcam as configured in labs/config/karmel.yaml."""
    return PinholeCamera.from_hfov(640, 480, 1.20)


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return make_T(R.T, -R.T @ t)


def transform_points(T_a_b: np.ndarray, p_b: np.ndarray) -> np.ndarray:
    """Express (N,3) points given in frame b in frame a."""
    p = np.atleast_2d(np.asarray(p_b, dtype=float))
    return p @ T_a_b[:3, :3].T + T_a_b[:3, 3]


def karmel_T_base_optical(pitch_down_rad: float = 0.0,
                          xyz=(0.10, 0.0, 0.145)) -> np.ndarray:
    """T_base_footprint_camera_optical. pitch_down_rad > 0 tilts the camera toward the floor."""
    R = rot_y(pitch_down_rad) @ R_BODY_OPTICAL
    return make_T(R, xyz)


def look_at(eye, target, up=(0.0, 0.0, 1.0)) -> np.ndarray:
    """T_world_optical for a camera at `eye` looking at `target` (world z up)."""
    eye = np.asarray(eye, dtype=float)
    f = np.asarray(target, dtype=float) - eye
    f /= np.linalg.norm(f)
    right = np.cross(f, np.asarray(up, dtype=float))
    right /= np.linalg.norm(right)
    down = np.cross(f, right)
    return make_T(np.column_stack((right, down, f)), eye)


def distance_from_apparent_size(radius_px: float, radius_m: float, f_px: float) -> float:
    """Range to a sphere's center from its image radius (small-angle pinhole approximation)."""
    return f_px * radius_m / radius_px


def ground_point_from_pixel(cam: PinholeCamera, T_base_optical: np.ndarray, uv,
                            plane_z: float = 0.0) -> np.ndarray | None:
    """Intersect the pixel's viewing ray with the horizontal plane z = plane_z in base_footprint.

    Returns the (3,) point in base_footprint, or None if the ray points at or above the horizon.
    """
    d_opt = cam.rays(np.asarray(uv, dtype=float).reshape(1, 2))[0]
    R, c = T_base_optical[:3, :3], T_base_optical[:3, 3]
    d = R @ d_opt
    if d[2] >= -1e-9:
        return None
    s = (plane_z - c[2]) / d[2]
    return c + s * d
