"""15.10 — reference solution: support polygon, centre of mass, margin, frames, decision.

The same functions live inside a running analysis at ``15-manipulation/code/mobile_manip.py``.

Five functions, and together they answer the three questions that only exist once the arm is on
a moving base:

    support_polygon + convex_hull_2d   what is holding the robot up?
    combined_com                       where is the mass, with the arm out and something in the jaws?
    stability_margin                   how close is it to tipping?
    object_in_arm_frame                where is the object, from the ARM's point of view?
    should_use_arm                     move the arm, or move the base?

Frames: the map frame and the robot's base_link are both x forward, y left, z up (REP-103). The
arm is bolted to the base at ``mount_xyz`` with no rotation.

The reference implementation lives at ``15-manipulation/code/mobile_manip.py``.
"""

from __future__ import annotations

import math

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- implemented ------------------------------------------------------------------------------
def convex_hull_2d(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def support_polygon(wheel_separation_m: float, caster_x_m: float,
                    front_caster_x_m: float | None = None) -> list[tuple[float, float]]:
    half = wheel_separation_m / 2.0
    contacts = [(0.0, +half), (0.0, -half), (caster_x_m, 0.0)]
    if front_caster_x_m is not None:
        contacts += [(front_caster_x_m, +half), (front_caster_x_m, -half)]
    return convex_hull_2d(contacts)


def combined_com(parts: list[tuple[float, tuple[float, float, float]]]
                 ) -> tuple[float, Array]:
    if not parts:
        raise ValueError("no parts")
    total = sum(m for m, _ in parts)
    if total <= 0.0:
        raise ValueError("total mass must be positive")
    moment = np.zeros(3)
    for m, p in parts:
        moment += m * np.asarray(p, dtype=float)
    return float(total), moment / total


def stability_margin(support: list[tuple[float, float]], com_xy: ArrayLike) -> float:
    p = np.asarray(com_xy, dtype=float)[:2]
    n = len(support)
    best = float("inf")
    inside = True
    for i in range(n):
        a = np.asarray(support[i], dtype=float)
        b = np.asarray(support[(i + 1) % n], dtype=float)
        edge = b - a
        length = float(np.linalg.norm(edge))
        if length < 1e-12:
            continue
        normal = np.array([-edge[1], edge[0]]) / length
        signed = float(normal @ (p - a))
        inside &= signed >= 0.0
        best = min(best, abs(signed))
    return best if inside else -best


def object_in_arm_frame(base_xy_yaw: tuple[float, float, float],
                        mount_xyz: tuple[float, float, float],
                        object_map_xyz: ArrayLike) -> Array:
    x, y, yaw = base_xy_yaw
    c, s = math.cos(yaw), math.sin(yaw)
    Rz_T = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    p = np.asarray(object_map_xyz, dtype=float).reshape(3) - np.array([x, y, 0.0])
    return Rz_T @ p - np.asarray(mount_xyz, dtype=float)


def should_use_arm(manipulability: float, margin_m: float, *,
                   min_manipulability: float = 0.010,
                   min_margin_m: float = 0.030) -> bool:
    return manipulability >= min_manipulability and margin_m >= min_margin_m
