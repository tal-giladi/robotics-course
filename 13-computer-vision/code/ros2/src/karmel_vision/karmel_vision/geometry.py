"""13.16 — turning a 2D detection box into a 3D point, with no ROS imports.

Four ways to recover the missing depth of a bounding box, the pinhole back-projection that
follows, and the uncertainty each way carries. Pure numpy so it is unit-tested on any laptop
(`pytest 13-computer-vision/code/ros2/src/karmel_vision/tests`) and imported by the ROS nodes.

Conventions (REP-103 / 05.06): all 3D points here are in the CAMERA OPTICAL frame —
z forward (depth), x right (image columns), y down (image rows).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

# Real-world sizes of the objects karmel looks for, in metres (min, typical, max).
# Measured on the objects in the course's flat; yours differ — measure them.
OBJECT_HEIGHT_M: dict[str, tuple[float, float, float]] = {
    "bottle": (0.15, 0.24, 0.35),
    "cup": (0.07, 0.10, 0.15),
}


@dataclass(frozen=True)
class Intrinsics:
    """A pinhole camera's K matrix (13.05), plus the image size it belongs to."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_hfov(cls, width: int, height: int, hfov_rad: float) -> "Intrinsics":
        """Ideal camera from a horizontal field of view (karmel.yaml gives 1.20 rad)."""
        f = (width / 2) / math.tan(hfov_rad / 2)
        return cls(f, f, width / 2, height / 2, width, height)

    @classmethod
    def from_camera_info_k(cls, k, width: int, height: int) -> "Intrinsics":
        """From `sensor_msgs/CameraInfo.k` (row-major 3x3)."""
        k = np.asarray(k, dtype=float).reshape(3, 3)
        return cls(float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2]), width, height)

    def scaled_to(self, width: int, height: int) -> "Intrinsics":
        """K for the SAME camera after the image is resized. Resizing without this is the
        single most common 3D-vision bug: a 2x downscale halves fx, fy, cx and cy."""
        sx, sy = width / self.width, height / self.height
        return Intrinsics(self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy, width, height)

    def back_project(self, u: float, v: float, z: float) -> np.ndarray:
        """Pixel + depth -> 3D point in the optical frame."""
        return np.array([(u - self.cx) * z / self.fx, (v - self.cy) * z / self.fy, z])

    def project(self, point: np.ndarray) -> tuple[float, float]:
        """3D point in the optical frame -> pixel (the inverse of back_project)."""
        x, y, z = (float(c) for c in point)
        if z <= 0:
            raise ValueError("point is behind the camera")
        return self.cx + self.fx * x / z, self.cy + self.fy * y / z


@dataclass(frozen=True)
class Box:
    """An axis-aligned detection box in pixels of the image the intrinsics belong to."""

    x1: float
    y1: float
    x2: float
    y2: float

    @classmethod
    def from_center_size(cls, cx: float, cy: float, size_x: float, size_y: float) -> "Box":
        """vision_msgs/BoundingBox2D order (center + size), which is what a Detection2D carries."""
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

    def shrunk(self, factor: float) -> "Box":
        """The central `factor` of the box: the part most likely to be object, not background."""
        cx, cy = self.center
        return Box.from_center_size(cx, cy, self.width * factor, self.height * factor)

    def touches_border(self, intr: Intrinsics, margin: float = 1.0) -> bool:
        """True if the object is cut off by the image edge — then its pixel height is a lower
        bound and the known-size method silently over-estimates the distance."""
        return (self.x1 <= margin or self.y1 <= margin
                or self.x2 >= intr.width - 1 - margin or self.y2 >= intr.height - 1 - margin)


# --------------------------------------------------------------------------- depth, four ways
def depth_from_depth_image(depth_m: np.ndarray, box: Box, inner: float = 0.5,
                           min_valid: int = 10) -> float | None:
    """Method 1 — a depth camera. Median of the valid depths in the central `inner` of the box.

    `depth_m` is metres, with 0 or NaN meaning "no reading" (what every RGB-D camera returns for
    shiny, dark, transparent and too-close surfaces). The median, not the mean: a box always
    contains some background pixels, and one wall at 4 m ruins a mean.
    """
    b = box.shrunk(inner)
    x1, y1 = max(int(math.floor(b.x1)), 0), max(int(math.floor(b.y1)), 0)
    x2, y2 = min(int(math.ceil(b.x2)), depth_m.shape[1]), min(int(math.ceil(b.y2)), depth_m.shape[0])
    if x2 <= x1 or y2 <= y1:
        return None
    patch = np.asarray(depth_m[y1:y2, x1:x2], dtype=float).ravel()
    valid = patch[np.isfinite(patch) & (patch > 0.0)]
    if valid.size < min_valid:
        return None
    return float(np.median(valid))


def depth_from_known_size(box: Box, intr: Intrinsics, real_height_m: float) -> float:
    """Method 2 — the object's real height. h_px = fy * H / Z  =>  Z = fy * H / h_px."""
    if box.height <= 0:
        raise ValueError("box has no height")
    return intr.fy * real_height_m / box.height


def depth_from_ground_plane(box: Box, intr: Intrinsics, camera_height_m: float,
                            pitch_rad: float = 0.0) -> float | None:
    """Method 3 — the object stands on the floor, and we know how high the camera is.

    Intersect the ray through the box's BOTTOM edge centre with the floor plane. For a level
    camera this collapses to Z = h * fy / (v - cy), which is why the horizon (v = cy) means
    "infinitely far" and why a 2-pixel error matters enormously for distant objects.
    `pitch_rad` > 0 means the camera is tilted DOWN (the usual way to mount it on a small robot).
    """
    u, v = box.center[0], box.y2
    ray = np.array([(u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy, 1.0])
    # Floor normal in the optical frame: "down" is +y for a level camera; pitching the camera
    # down by theta rotates it about the optical x axis, tilting the normal into +z (forward).
    normal = np.array([0.0, math.cos(pitch_rad), math.sin(pitch_rad)])
    denom = float(normal @ ray)
    if denom <= 1e-9:                      # the ray points at or above the horizon
        return None
    t = camera_height_m / denom
    return float(t * ray[2])               # the z component is the depth


def depth_from_range_sensor(range_m: float, box: Box, intr: Intrinsics) -> float:
    """Method 4 — a 1D range reading (LiDAR beam or the ToF sensor) along the box's bearing.

    The sensor reports a slant range r along its beam; the depth we want is the z component,
    r * cos(bearing). Correct only when the beam really hit the same object: karmel's LiDAR
    scans at z = 0.12 m, so it sees the bottle's belly but flies straight over a mug.
    """
    u = box.center[0]
    bearing = math.atan2(u - intr.cx, intr.fx)
    return range_m * math.cos(bearing)


# --------------------------------------------------------------------------- position + uncertainty
def position_from_box(box: Box, intr: Intrinsics, depth_m: float) -> np.ndarray:
    """The object's centre in the optical frame. The box centre, NOT the bottom edge: the bottom
    edge is where the object touches the floor, the centre is what a gripper aims at."""
    u, v = box.center
    return intr.back_project(u, v, depth_m)


def position_covariance(box: Box, intr: Intrinsics, depth_m: float, sigma_depth_m: float,
                        sigma_pixel: float = 2.0) -> np.ndarray:
    """First-order propagation of depth and pixel noise into a 3x3 covariance (optical frame).

    X = (u - cx) Z / fx  =>  dX = (Z/fx) du + ((u - cx)/fx) dZ, and the same for Y. The lateral
    error is therefore dominated by depth error for off-centre objects and by pixel error for
    centred ones; the depth error passes into Z untouched.
    """
    u, v = box.center
    j = np.array([                              # d(X, Y, Z) / d(u, v, Z)
        [depth_m / intr.fx, 0.0, (u - intr.cx) / intr.fx],
        [0.0, depth_m / intr.fy, (v - intr.cy) / intr.fy],
        [0.0, 0.0, 1.0],
    ])
    inputs = np.diag([sigma_pixel ** 2, sigma_pixel ** 2, sigma_depth_m ** 2])
    return j @ inputs @ j.T


def known_size_depth_sigma(depth_m: float, box: Box, class_name: str,
                           sigma_pixel: float = 2.0) -> float:
    """How wrong the known-size depth can be: relative errors add in quadrature,
    sigma_Z/Z = sqrt((sigma_H/H)^2 + (sigma_h/h)^2), with sigma_H from the class's size spread."""
    lo, typical, hi = OBJECT_HEIGHT_M[class_name]
    sigma_h_rel = (hi - lo) / (2 * 1.96) / typical          # treat (lo, hi) as a 95 % interval
    sigma_px_rel = math.sqrt(2) * sigma_pixel / box.height  # two edges, each uncertain
    return depth_m * math.hypot(sigma_h_rel, sigma_px_rel)


def plausible_size(class_name: str, box: Box, intr: Intrinsics, depth_m: float,
                   tolerance: float = 0.25) -> tuple[bool, float]:
    """The engineered gate from 16.01, now with a real depth: does the implied real height make
    sense for the class? Returns (accept, implied height in metres)."""
    lo, _typical, hi = OBJECT_HEIGHT_M[class_name]
    implied = box.height * depth_m / intr.fy
    return bool(lo * (1 - tolerance) <= implied <= hi * (1 + tolerance)), float(implied)


CAMERA_HEIGHT_M = 0.145            # karmel.yaml: camera z 0.10 above base_link, which is 0.045 up


def synthetic_box(intr: Intrinsics, depth_m: float, lateral_m: float, class_name: str = "bottle",
                  camera_height_m: float = CAMERA_HEIGHT_M, diameter_m: float = 0.07) -> Box:
    """The box a perfect detector would draw around an object of known size standing on the floor.
    Used by the tests and by the lesson's worked example, so every number below is consistent."""
    height_m = OBJECT_HEIGHT_M[class_name][1]
    top = intr.project(np.array([lateral_m, camera_height_m - height_m, depth_m]))
    bottom = intr.project(np.array([lateral_m, camera_height_m, depth_m]))
    half_w = intr.fx * (diameter_m / 2) / depth_m
    return Box(bottom[0] - half_w, top[1], bottom[0] + half_w, bottom[1])


if __name__ == "__main__":                                  # the lesson's worked example
    intr = Intrinsics.from_hfov(640, 480, 1.20)
    print(f"karmel 640x480, hfov 1.20 rad: fx = fy = {intr.fx:.1f} px, (cx, cy) = ({intr.cx:.0f}, {intr.cy:.0f})")
    print(f"downscaled to 320x240:         fx = {intr.scaled_to(320, 240).fx:.1f} px")
    box = synthetic_box(intr, depth_m=1.0, lateral_m=0.17)
    print(f"a 0.24 m bottle 1.00 m away, 0.17 m to the right: box x {box.x1:.1f}..{box.x2:.1f} "
          f"y {box.y1:.1f}..{box.y2:.1f}  ({box.width:.1f} x {box.height:.1f} px)")
    methods = (("depth camera", 1.00),
               ("known size", depth_from_known_size(box, intr, OBJECT_HEIGHT_M["bottle"][1])),
               ("ground plane", depth_from_ground_plane(box, intr, CAMERA_HEIGHT_M) or float("nan")),
               ("LiDAR range", depth_from_range_sensor(math.hypot(1.0, 0.17), box, intr)))
    for name, z in methods:
        p = position_from_box(box, intr, z)
        print(f"{name:14s} Z = {z:.3f} m -> optical (x, y, z) = ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:.3f})")
    sigma = known_size_depth_sigma(1.0, box, "bottle")
    cov = position_covariance(box, intr, 1.0, sigma)
    print(f"known-size 1 sigma at 1 m: depth {sigma * 100:.1f} cm, lateral {math.sqrt(cov[0, 0]) * 100:.1f} cm")
