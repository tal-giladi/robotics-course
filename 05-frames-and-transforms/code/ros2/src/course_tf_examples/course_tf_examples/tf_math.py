"""Pure-Python helpers used by the TF2 nodes. No rclpy import, so they are unit-testable anywhere.

Notation (module 05): T_a_b is the pose of frame b expressed in frame a; p_a = T_a_b @ p_b.
tf2's ``buffer.lookup_transform(target_frame=a, source_frame=b, time)`` returns exactly T_a_b:
``header.frame_id == a``, ``child_frame_id == b``.

Quaternions are (x, y, z, w) — the field order of geometry_msgs/Quaternion.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) for ROS roll-pitch-yaw (fixed axes x, y, z = R = Rz(yaw) Ry(pitch) Rx(roll))."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError('zero quaternion (did you forget to set w = 1.0?)')
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_msg_to_matrix(transform: Any) -> np.ndarray:
    """4x4 T_parent_child from a geometry_msgs/Transform (or anything with .translation/.rotation)."""
    t, q = transform.translation, transform.rotation
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(q.x, q.y, q.z, q.w)
    T[:3, 3] = (t.x, t.y, t.z)
    return T


def apply(T_a_b: np.ndarray, p_b: tuple[float, float, float]) -> np.ndarray:
    """p_a = T_a_b @ p_b for one 3D point."""
    return T_a_b[:3, :3] @ np.asarray(p_b, dtype=float) + T_a_b[:3, 3]


def circle_pose(t: float, linear_speed: float, angular_speed: float) -> tuple[float, float, float]:
    """Exact (x, y, yaw) after driving at constant (v, omega) for t seconds from the origin."""
    yaw = angular_speed * t
    if abs(angular_speed) < 1e-9:
        return linear_speed * t, 0.0, 0.0
    r = linear_speed / angular_speed
    return r * math.sin(yaw), r * (1.0 - math.cos(yaw)), yaw


#: karmel's fixed transforms (labs/config/karmel.yaml, karmel.urdf.xacro):
#: (parent, child, x, y, z, roll, pitch, yaw). robot_state_publisher replaces this in 05.08.
KARMEL_STATIC = [
    ('base_footprint', 'base_link', 0.0, 0.0, 0.045, 0.0, 0.0, 0.0),
    ('base_link', 'laser', 0.0, 0.0, 0.12, 0.0, 0.0, 0.0),
    ('base_link', 'imu_link', 0.0, 0.0, 0.04, 0.0, 0.0, 0.0),
    ('base_link', 'range_front_link', 0.125, 0.0, 0.05, 0.0, 0.0, 0.0),
    ('base_link', 'camera_link', 0.10, 0.0, 0.10, 0.0, 0.0, 0.0),
    ('camera_link', 'camera_optical_frame', 0.0, 0.0, 0.0, -math.pi / 2, 0.0, -math.pi / 2),
]
