"""13.16 — from a detection box to a 3D position in the map frame.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 13.16``.
Only the standard library and numpy are needed.

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
    """The same camera's K after the image is resized to ``width`` x ``height``.

    All four of fx, fy, cx, cy scale with the image; a 2x downscale halves every one of them.
    """
    # TODO(student): implement.
    raise NotImplementedError("scale_intrinsics")


def back_project(intr: Intrinsics, u: float, v: float, z: float) -> NDArray[np.float64]:
    """Pixel (u, v) at depth ``z`` metres -> a 3D point (x, y, z) in the optical frame."""
    # TODO(student): implement.
    raise NotImplementedError("back_project")


def depth_from_depth_image(depth_m: NDArray[np.float64], box: Box, inner: float = 0.5,
                           min_valid: int = 10) -> float | None:
    """Median of the VALID depths inside the central ``inner`` of the box, or None.

    ``depth_m`` is in metres; 0 and NaN mean "no reading" and must be ignored. Return None when
    fewer than ``min_valid`` pixels are valid — a missing answer beats a confident wrong one.
    Clip the sampling window to the image.
    """
    # TODO(student): implement.
    raise NotImplementedError("depth_from_depth_image")


def depth_from_known_size(box: Box, intr: Intrinsics, real_height_m: float) -> float:
    """Depth of an object whose real height is known: h_px = fy * H / Z."""
    # TODO(student): implement.
    raise NotImplementedError("depth_from_known_size")


def depth_from_ground_plane(box: Box, intr: Intrinsics, camera_height_m: float,
                            pitch_rad: float = 0.0) -> float | None:
    """Depth of an object standing on the floor, from the ray through its BOTTOM edge centre.

    Intersect that ray with the floor plane. In the optical frame the floor normal of a level
    camera is (0, 1, 0) ("down" is +y); pitching the camera down by ``pitch_rad`` rotates it about
    the optical x axis into (0, cos, sin). Return None when the ray points at or above the horizon.
    For a level camera the result must reduce to Z = h * fy / (v - cy).
    """
    # TODO(student): implement.
    raise NotImplementedError("depth_from_ground_plane")


def depth_from_range_sensor(range_m: float, box: Box, intr: Intrinsics) -> float:
    """Depth from a 1D range reading along the box's bearing.

    The sensor reports a slant range; the depth (the optical z) is its forward component.
    """
    # TODO(student): implement.
    raise NotImplementedError("depth_from_range_sensor")


def position_covariance(box: Box, intr: Intrinsics, depth_m: float, sigma_depth_m: float,
                        sigma_pixel: float = 2.0) -> NDArray[np.float64]:
    """3x3 covariance of the back-projected box centre, in the optical frame.

    First-order propagation of (u, v, Z) noise: Sigma = J diag(su^2, sv^2, sZ^2) J^T with J the
    Jacobian of (X, Y, Z) with respect to (u, v, Z). Pixel noise is independent in u and v.
    """
    # TODO(student): implement.
    raise NotImplementedError("position_covariance")


def plausible_size(class_name: str, box: Box, intr: Intrinsics, depth_m: float,
                   tolerance: float = 0.25) -> tuple[bool, float]:
    """The 16.01 gate: does this box at this depth imply a believable real height?

    Return (accepted, implied height in metres). Accept when the implied height lies inside the
    class's (min, max) range widened by ``tolerance`` on each side.
    """
    # TODO(student): implement.
    raise NotImplementedError("plausible_size")


def mahalanobis2(p1: NDArray[np.float64], cov1: NDArray[np.float64],
                 p2: NDArray[np.float64], cov2: NDArray[np.float64]) -> float:
    """Squared Mahalanobis distance between two uncertain positions: d^T (C1 + C2)^-1 d."""
    # TODO(student): implement.
    raise NotImplementedError("mahalanobis2")


def fuse(p1: NDArray[np.float64], cov1: NDArray[np.float64], p2: NDArray[np.float64],
         cov2: NDArray[np.float64], min_sigma_m: float = 0.02
         ) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Kalman update of a static object with a new measurement.

    K = C1 (C1 + C2)^-1 ;  p = p1 + K (p2 - p1) ;  C = (I - K) C1, with every diagonal entry of
    the result floored at ``min_sigma_m ** 2`` so the estimate never claims to be better than the
    calibration underneath it.
    """
    # TODO(student): implement.
    raise NotImplementedError("fuse")
