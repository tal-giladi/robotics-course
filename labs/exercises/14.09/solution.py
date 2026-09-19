"""14.09 — reference solution. Read it after you have tried ``student.py`` yourself.

ORIGINAL DOCSTRING:
14.09 — Collision checking, path validity and the resolution that decides whether it works.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 14.09``.
Only the standard library and numpy are needed — no ROS, no MoveIt.

This is, in miniature, what ``move_group`` calls tens of thousands of times per plan:

* links are **capsules** (a segment with a radius), obstacles are **spheres** — the same
  primitives MoveIt uses for its cheapest checks;
* a **state** is in collision when any capsule overlaps any sphere;
* a **path** is checked by interpolating between states and checking each one, which is safe only
  when the interpolation step is smaller than the smallest thing you care about hitting.

Geometry: a 3-DOF arm, pan about z then two lifts in the vertical plane. Its forward kinematics
are given; everything else is yours. Units: metres and radians throughout.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]

# The toy arm: base height, upper arm, forearm.
BASE_HEIGHT = 0.06
L1 = 0.116
L2 = 0.135
LINK_RADIUS = 0.02
JOINT_LIMITS = np.array([[-1.91986, 1.91986], [-1.74533, 1.74533], [-1.69, 1.69]])


# --- given ---------------------------------------------------------------------------------------
def arm_points(q: Sequence[float]) -> Array:
    """(4, 3): base, shoulder, elbow and tip of the toy arm, in the base frame.

    q = (pan, lift, elbow). The arm lies in the vertical plane rotated by ``pan`` about z;
    ``lift`` is measured from horizontal and ``elbow`` is relative to the upper arm.
    """
    pan, lift, elbow = (float(v) for v in q)
    forward = np.array([math.cos(pan), math.sin(pan), 0.0])
    up = np.array([0.0, 0.0, 1.0])
    base = np.zeros(3)
    shoulder = np.array([0.0, 0.0, BASE_HEIGHT])
    a1 = lift
    a2 = lift + elbow
    elbow_point = shoulder + L1 * (math.cos(a1) * forward + math.sin(a1) * up)
    tip = elbow_point + L2 * (math.cos(a2) * forward + math.sin(a2) * up)
    return np.array([base, shoulder, elbow_point, tip])


def link_segments(q: Sequence[float]) -> list[tuple[Array, Array]]:
    """The arm's capsule axes: [(base, shoulder), (shoulder, elbow), (elbow, tip)]."""
    p = arm_points(q)
    return [(p[i], p[i + 1]) for i in range(len(p) - 1)]


def max_jacobian_column(q: Sequence[float]) -> float:
    """How far the tip moves per radian of the most effective joint, at this pose [m/rad].

    Used by ``required_segment_fraction``: it converts joint motion into tip motion.
    """
    p = arm_points(q)
    tip = p[-1]
    pan = float(q[0])
    z = np.array([0.0, 0.0, 1.0])
    lift_axis = np.array([-math.sin(pan), math.cos(pan), 0.0])    # both lift joints turn about it
    columns = [np.cross(z, tip - p[0]),                           # pan, about the base axis
               np.cross(lift_axis, tip - p[1]),                   # lift, about the shoulder
               np.cross(lift_axis, tip - p[2])]                   # elbow
    return float(max(np.linalg.norm(c) for c in columns))




# --- solution ------------------------------------------------------------------------------------
def closest_point_on_segment(a: ArrayLike, b: ArrayLike, p: ArrayLike) -> Array:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    p = np.asarray(p, dtype=float)
    ab = b - a
    length2 = float(ab @ ab)
    if length2 <= 0.0:
        return a.copy()
    t = float((p - a) @ ab) / length2
    t = min(1.0, max(0.0, t))                     # the clamp: a SEGMENT, not a line
    return a + t * ab


def capsule_sphere_clearance(a: ArrayLike, b: ArrayLike, capsule_radius: float,
                             center: ArrayLike, sphere_radius: float) -> float:
    center = np.asarray(center, dtype=float)
    nearest = closest_point_on_segment(a, b, center)
    return float(np.linalg.norm(center - nearest)) - capsule_radius - sphere_radius


def state_clearance(q: Sequence[float], obstacles: Sequence[tuple[ArrayLike, float]],
                    link_radius: float = LINK_RADIUS) -> float:
    if not obstacles:
        return float("inf")
    worst = float("inf")
    for a, b in link_segments(q):
        for center, radius in obstacles:
            worst = min(worst, capsule_sphere_clearance(a, b, link_radius, center, radius))
    return worst


def interpolate_states(q0: ArrayLike, q1: ArrayLike, max_step: float) -> Array:
    if max_step <= 0:
        raise ValueError("max_step must be positive")
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    if q0.shape != q1.shape:
        raise ValueError(f"q0 and q1 must have the same shape, got {q0.shape} and {q1.shape}")
    biggest = float(np.max(np.abs(q1 - q0))) if q0.size else 0.0
    if biggest == 0.0:
        return q0.reshape(1, -1).copy()
    segments = int(math.ceil(biggest / max_step))
    ts = np.linspace(0.0, 1.0, segments + 1)          # +1: the goal state must be included
    return q0 + ts[:, None] * (q1 - q0)


def path_clearance(q0: ArrayLike, q1: ArrayLike, obstacles: Sequence[tuple[ArrayLike, float]],
                   max_step: float, link_radius: float = LINK_RADIUS) -> tuple[float, int]:
    states = interpolate_states(q0, q1, max_step)
    clearances = [state_clearance(q, obstacles, link_radius) for q in states]
    worst = int(np.argmin(clearances))
    return float(clearances[worst]), worst


def required_segment_fraction(joint_limits: ArrayLike, max_jacobian_column_norm: float,
                              tool_step_m: float) -> float:
    if tool_step_m <= 0:
        raise ValueError("tool_step_m must be positive")
    limits = np.asarray(joint_limits, dtype=float)
    extent = float(np.sum(limits[:, 1] - limits[:, 0]))
    return float(tool_step_m) / (extent * float(max_jacobian_column_norm))
