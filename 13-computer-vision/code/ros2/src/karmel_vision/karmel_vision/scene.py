"""The synthetic world behind `fake_camera`: where the objects are and what the camera sees.

No ROS and no OpenCV imports — just the geometry, so the numbers in the lesson can be checked
without starting anything. karmel's extrinsics come from `labs/config/karmel.yaml`:
camera at (0.10, 0, 0.10) in base_link, base_link 0.045 m (one wheel radius) above the floor.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .geometry import OBJECT_HEIGHT_M, Box, Intrinsics

CAMERA_X_M = 0.10
CAMERA_Z_M = 0.10
WHEEL_RADIUS_M = 0.045
CAMERA_HEIGHT_M = CAMERA_Z_M + WHEEL_RADIUS_M          # 0.145 m above the floor


@dataclass(frozen=True)
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class SceneObject:
    """An object standing on the floor at a known map position."""

    class_name: str
    x: float
    y: float
    diameter_m: float = 0.07

    @property
    def height_m(self) -> float:
        return OBJECT_HEIGHT_M[self.class_name][1]

    @property
    def centre_map(self) -> np.ndarray:
        """The ground truth the 3D pipeline is graded against: the object's centre in `map`."""
        return np.array([self.x, self.y, self.height_m / 2])


def to_optical(pose: RobotPose, point_map: np.ndarray) -> np.ndarray:
    """A point in `map` expressed in `camera_optical_frame` (z forward, x right, y down).

    The same five-transform chain as 05.10, collapsed into arithmetic because the robot is planar.
    """
    cam_x = pose.x + CAMERA_X_M * math.cos(pose.yaw)
    cam_y = pose.y + CAMERA_X_M * math.sin(pose.yaw)
    dx, dy = point_map[0] - cam_x, point_map[1] - cam_y
    forward = dx * math.cos(pose.yaw) + dy * math.sin(pose.yaw)
    left = -dx * math.sin(pose.yaw) + dy * math.cos(pose.yaw)
    return np.array([-left, CAMERA_HEIGHT_M - point_map[2], forward])


def visible_box(pose: RobotPose, obj: SceneObject, intr: Intrinsics) -> Box | None:
    """The box a perfect detector would draw, or None when the object is out of frame."""
    base = to_optical(pose, np.array([obj.x, obj.y, 0.0]))
    if base[2] < 0.15:                                   # behind the camera or unrealistically close
        return None
    top = to_optical(pose, np.array([obj.x, obj.y, obj.height_m]))
    u_c, v_bottom = intr.project(base)
    _, v_top = intr.project(top)
    half_w = intr.fx * (obj.diameter_m / 2) / base[2]
    box = Box(u_c - half_w, v_top, u_c + half_w, v_bottom)
    cx, cy = box.center
    if not (0 <= cx < intr.width and 0 <= cy < intr.height):
        return None                                      # centre outside the image: call it invisible
    return box


def depth_map(pose: RobotPose, objects: list[SceneObject], intr: Intrinsics,
              max_range_m: float = 4.0) -> np.ndarray:
    """A depth image (metres, 0 = no reading) of a flat floor with the objects standing on it.

    The floor's depth comes from the ground-plane formula Z = h * fy / (v - cy); pixels above the
    horizon get 0, exactly like a real depth camera looking at a far wall it cannot measure.
    """
    vs = np.arange(intr.height, dtype=float)[:, None]
    with np.errstate(divide="ignore", invalid="ignore"):
        floor = CAMERA_HEIGHT_M * intr.fy / (vs - intr.cy)
    floor = np.where((vs > intr.cy + 0.5) & (floor <= max_range_m), floor, 0.0)
    depth = np.repeat(floor, intr.width, axis=1)
    for obj in sorted(objects, key=lambda o: -to_optical(pose, o.centre_map)[2]):   # far to near
        box = visible_box(pose, obj, intr)
        if box is None:
            continue
        z = float(to_optical(pose, obj.centre_map)[2])
        x1, y1 = max(int(box.x1), 0), max(int(box.y1), 0)
        x2, y2 = min(int(math.ceil(box.x2)), intr.width), min(int(math.ceil(box.y2)), intr.height)
        if x2 > x1 and y2 > y1:
            depth[y1:y2, x1:x2] = z if z <= max_range_m else 0.0
    return depth
