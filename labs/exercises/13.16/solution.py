"""Reference solution for 13.16 — from a detection box to a 3D position in the map frame.

Same public names and signatures as ``student.py``.

Every 3D point here is in ``camera_optical_frame`` (REP-103): **z forward (depth), x right,
y down**. Angles in radians, distances in metres, pixels in pixels.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

# Real heights of karmel's target objects: (minimum, typical, maximum) in metres.
OBJECT_HEIGHT_M: dict[str, tuple[float, float, float]] = {
    "bottle": (0.15, 0.24, 0.35),
    "cup": (0.07, 0.10, 0.15),
}

# chi-square, 3 degrees of freedom: the 95 % and 99 % association gates.
GATE_95 = 7.815
GATE_99 = 11.345


# --- given ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Intrinsics:
    """A pinhole camera's K, together with the image size it belongs to."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov_rad: float) -> Intrinsics:
        f = (width / 2) / math.tan(hfov_rad / 2)
        return cls(f, f, width / 2, height / 2, width, height)


@dataclass(frozen=True)
class Box:
    """An axis-aligned detection box in pixels (x1, y1) top-left, (x2, y2) bottom-right."""

    x1: float
    y1: float
    x2: float
    y2: float

    @classmethod
    def from_center_size(cls, cx: float, cy: float, size_x: float, size_y: float) -> Box:
        """The vision_msgs/BoundingBox2D convention: centre plus size."""
        return cls(cx - size_x / 2, cy - size_y / 2, cx + size_x / 2, cy + size_y / 2)

    @property
    def center(self) -> tuple[float, float]:
        return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    def shrunk(self, factor: float) -> Box:
        """The central ``factor`` of the box (0.5 = the middle half in each dimension)."""
        cx, cy = self.center
        return Box.from_center_size(cx, cy, self.width * factor, self.height * factor)


# --- implement -----------------------------------------------------------------------------------
def scale_intrinsics(intr: Intrinsics, width: int, height: int) -> Intrinsics:
    """The same camera's K after the image is resized to ``width`` x ``height``."""
    sx, sy = width / intr.width, height / intr.height
    return Intrinsics(intr.fx * sx, intr.fy * sy, intr.cx * sx, intr.cy * sy, width, height)


def back_project(intr: Intrinsics, u: float, v: float, z: float) -> NDArray[np.float64]:
    """Pixel (u, v) at depth ``z`` metres -> a 3D point (x, y, z) in the optical frame."""
    return np.array([(u - intr.cx) * z / intr.fx, (v - intr.cy) * z / intr.fy, float(z)])


def depth_from_depth_image(depth_m: NDArray[np.float64], box: Box, inner: float = 0.5,
                           min_valid: int = 10) -> float | None:
    """Median of the VALID depths inside the central ``inner`` of the box, or None."""
    b = box.shrunk(inner)
    x1, y1 = max(int(math.floor(b.x1)), 0), max(int(math.floor(b.y1)), 0)
    x2 = min(int(math.ceil(b.x2)), depth_m.shape[1])
    y2 = min(int(math.ceil(b.y2)), depth_m.shape[0])
    if x2 <= x1 or y2 <= y1:
        return None
    patch = np.asarray(depth_m[y1:y2, x1:x2], dtype=float).ravel()
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size < min_valid:
        return None
    return float(np.median(valid))


def depth_from_known_size(box: Box, intr: Intrinsics, real_height_m: float) -> float:
    """Depth of an object whose real height is known: h_px = fy * H / Z."""
    if box.height <= 0:
        raise ValueError("box has no height")
    return intr.fy * real_height_m / box.height


def depth_from_ground_plane(box: Box, intr: Intrinsics, camera_height_m: float,
                            pitch_rad: float = 0.0) -> float | None:
    """Depth of an object standing on the floor, from the ray through its BOTTOM edge centre."""
    u, v = box.center[0], box.y2
    ray = np.array([(u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy, 1.0])
    normal = np.array([0.0, math.cos(pitch_rad), math.sin(pitch_rad)])
    denom = float(normal @ ray)
    if denom <= 1e-9:
        return None
    return float(camera_height_m / denom * ray[2])


def depth_from_range_sensor(range_m: float, box: Box, intr: Intrinsics) -> float:
    """Depth from a 1D range reading along the box's bearing."""
    bearing = math.atan2(box.center[0] - intr.cx, intr.fx)
    return float(range_m * math.cos(bearing))


def position_covariance(box: Box, intr: Intrinsics, depth_m: float, sigma_depth_m: float,
                        sigma_pixel: float = 2.0) -> NDArray[np.float64]:
    """3x3 covariance of the back-projected box centre, in the optical frame."""
    u, v = box.center
    j = np.array([
        [depth_m / intr.fx, 0.0, (u - intr.cx) / intr.fx],
        [0.0, depth_m / intr.fy, (v - intr.cy) / intr.fy],
        [0.0, 0.0, 1.0],
    ])
    inputs = np.diag([sigma_pixel ** 2, sigma_pixel ** 2, sigma_depth_m ** 2])
    return j @ inputs @ j.T


def plausible_size(class_name: str, box: Box, intr: Intrinsics, depth_m: float,
                   tolerance: float = 0.25) -> tuple[bool, float]:
    """The 16.01 gate: does this box at this depth imply a believable real height?"""
    lo, _typical, hi = OBJECT_HEIGHT_M[class_name]
    implied = box.height * depth_m / intr.fy
    return bool(lo * (1 - tolerance) <= implied <= hi * (1 + tolerance)), float(implied)


def mahalanobis2(p1: NDArray[np.float64], cov1: NDArray[np.float64],
                 p2: NDArray[np.float64], cov2: NDArray[np.float64]) -> float:
    """Squared Mahalanobis distance between two uncertain positions."""
    d = np.asarray(p2, float) - np.asarray(p1, float)
    s = np.asarray(cov1, float) + np.asarray(cov2, float)
    return float(d @ np.linalg.solve(s, d))


def fuse(p1: NDArray[np.float64], cov1: NDArray[np.float64], p2: NDArray[np.float64],
         cov2: NDArray[np.float64], min_sigma_m: float = 0.02
         ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Kalman update of a static object with a new measurement."""
    p1, c1 = np.asarray(p1, float), np.asarray(cov1, float)
    p2, c2 = np.asarray(p2, float), np.asarray(cov2, float)
    k = c1 @ np.linalg.inv(c1 + c2)
    position = p1 + k @ (p2 - p1)
    covariance = (np.eye(3) - k) @ c1
    np.fill_diagonal(covariance, np.maximum(np.diag(covariance), min_sigma_m ** 2))
    return position, covariance
