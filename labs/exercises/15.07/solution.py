"""15.07 — reference solution: grasp pose, approach waypoints, and the checks before you move.

The same functions live inside a running pipeline at ``15-manipulation/code/reach_pipeline.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


@dataclass(frozen=True)
class Waypoint:
    name: str
    T_base_tool: Array
    why: str
    speed_scale: float = 1.0

    @property
    def position(self) -> Array:
        return np.asarray(self.T_base_tool, dtype=float)[:3, 3]


def se3(R: ArrayLike, t: ArrayLike) -> Array:
    out = np.eye(4)
    out[:3, :3] = np.asarray(R, dtype=float)
    out[:3, 3] = np.asarray(t, dtype=float).reshape(3)
    return out


def wrap_axis(a: float) -> float:
    a = (a + math.pi / 2) % math.pi - math.pi / 2
    return a if a > -math.pi / 2 else a + math.pi


def approach_axis(pitch_rad: float, azimuth_rad: float) -> Array:
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    return np.array([c * math.cos(azimuth_rad), c * math.sin(azimuth_rad), -s])


def grasp_tool_pose(position: ArrayLike, jaw_yaw: float, *, pitch_rad: float = math.pi / 2,
                    azimuth_rad: float | None = None) -> Array:
    p = np.asarray(position, dtype=float).reshape(3)
    azimuth = math.atan2(p[1], p[0]) if azimuth_rad is None else azimuth_rad
    z = approach_axis(pitch_rad, azimuth)
    d = np.array([math.cos(jaw_yaw), math.sin(jaw_yaw), 0.0])
    x = d - float(d @ z) * z
    n = float(np.linalg.norm(x))
    if n < 1e-6:
        raise ValueError("closing direction is parallel to the approach axis")
    x = x / n
    y = np.cross(z, x)
    return se3(np.column_stack((x, y, z)), p)


def jaw_yaw_of(T_base_tool: ArrayLike) -> float:
    x = np.asarray(T_base_tool, dtype=float)[:3, 0]
    return wrap_axis(math.atan2(x[1], x[0]))


def approach_waypoints(T_grasp: ArrayLike, *, standoff_m: float = 0.080, lift_m: float = 0.060,
                       table_z: float = 0.0, clearance_m: float = 0.004) -> list[Waypoint]:
    T = np.asarray(T_grasp, dtype=float)
    p, z = T[:3, 3], T[:3, 2]
    if p[2] < table_z + clearance_m:
        raise ValueError(f"grasp is {(table_z - p[2]) * 1000:.1f} mm into the table")
    pre = T.copy()
    pre[:3, 3] = p - z * standoff_m
    mid = T.copy()
    mid[:3, 3] = p - z * 0.020
    lift = T.copy()
    lift[:3, 3] = p + np.array([0.0, 0.0, lift_m])
    return [
        Waypoint("pre_grasp", pre, f"{standoff_m * 1000:.0f} mm back along the approach axis", 1.0),
        Waypoint("approach", mid, "20 mm out, slow: the last segment that can collide", 0.25),
        Waypoint("grasp", T, "pads at the closing height from 15.05", 0.25),
        Waypoint("lift", lift, f"straight up {lift_m * 1000:.0f} mm, before any lateral motion", 0.5),
    ]


def check_plan(waypoints: list[Waypoint], *, table_z: float = 0.0, clearance_m: float = 0.004,
               reach_min_m: float = 0.12, reach_max_m: float = 0.32) -> list[str]:
    names = [w.name for w in waypoints]
    problems: list[str] = []
    if names != ["pre_grasp", "approach", "grasp", "lift"]:
        problems.append(f"waypoints out of order: {names}")
        return problems
    for w in waypoints:
        p = w.position
        if p[2] < table_z + clearance_m:
            problems.append(f"{w.name} is {(table_z - p[2]) * 1000:.1f} mm into the table")
        r = float(np.linalg.norm(p))
        if r > reach_max_m:
            problems.append(f"{w.name} is {r * 1000:.0f} mm from the base, past the {reach_max_m * 1000:.0f} mm reach")
        if r < reach_min_m:
            problems.append(f"{w.name} is {r * 1000:.0f} mm from the base, inside the {reach_min_m * 1000:.0f} mm dead zone")
    pre, mid, grasp, lift = waypoints
    if not (pre.position[2] > mid.position[2] > grasp.position[2]):
        problems.append("the approach does not descend monotonically")
    if lift.position[2] <= grasp.position[2]:
        problems.append("the lift does not go up")
    if abs(lift.position[0] - grasp.position[0]) > 1e-9 or abs(lift.position[1] - grasp.position[1]) > 1e-9:
        problems.append("the lift is not vertical")
    return problems


def lateral_tolerance_m(stroke_m: float, object_width_m: float) -> float:
    return max(0.0, (stroke_m - object_width_m) / 2.0)
