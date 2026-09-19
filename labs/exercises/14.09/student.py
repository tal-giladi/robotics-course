"""14.09 — Collision checking, path validity and the resolution that decides whether it works.

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


# --- TODO(student) -------------------------------------------------------------------------------
def closest_point_on_segment(a: ArrayLike, b: ArrayLike, p: ArrayLike) -> Array:
    """The point of the SEGMENT ab nearest to p.

    Project p onto the line, then CLAMP the parameter to [0, 1]. Forgetting the clamp gives you
    the nearest point on the infinite line, so the arm appears to collide with objects that are
    behind its own base. A degenerate segment (a == b) must return a.
    """
    raise NotImplementedError  # TODO(student)


def capsule_sphere_clearance(a: ArrayLike, b: ArrayLike, capsule_radius: float,
                             center: ArrayLike, sphere_radius: float) -> float:
    """Signed clearance [m] between a capsule (segment ab, radius) and a sphere.

    Positive = a gap of that size. Negative = that much overlap. Zero = exactly touching.
    Both radii come off the centre-to-axis distance.
    """
    raise NotImplementedError  # TODO(student)


def state_clearance(q: Sequence[float], obstacles: Sequence[tuple[ArrayLike, float]],
                    link_radius: float = LINK_RADIUS) -> float:
    """The worst clearance between any of the arm's links and any obstacle at pose ``q``.

    ``obstacles`` is a sequence of (center, radius). With no obstacles, return ``float("inf")``:
    nothing to hit.
    """
    raise NotImplementedError  # TODO(student)


def interpolate_states(q0: ArrayLike, q1: ArrayLike, max_step: float) -> Array:
    """Straight-line interpolation in joint space with no joint moving more than ``max_step``.

    Returns an (N, n) array that ALWAYS includes both endpoints. The number of segments is
    ``ceil(largest joint motion / max_step)``, so the number of states is that plus one — the
    off-by-one that silently drops the goal state is the most dangerous bug in this file.
    ``q0 == q1`` returns a single state. ``max_step`` must be positive.
    """
    raise NotImplementedError  # TODO(student)


def path_clearance(q0: ArrayLike, q1: ArrayLike, obstacles: Sequence[tuple[ArrayLike, float]],
                   max_step: float, link_radius: float = LINK_RADIUS) -> tuple[float, int]:
    """Check the straight-line path from q0 to q1.

    Returns ``(worst clearance found, index of the worst state)``. This is a LOWER BOUND on the
    path's true safety only when ``max_step`` is small enough — that is what
    ``required_segment_fraction`` is for, and what exercise 14.09-E2 demonstrates.
    """
    raise NotImplementedError  # TODO(student)


def required_segment_fraction(joint_limits: ArrayLike, max_jacobian_column_norm: float,
                              tool_step_m: float) -> float:
    """MoveIt's ``longest_valid_segment_fraction`` for a wanted tool resolution.

    MoveIt's joint-space state space has maximum extent = the SUM of the joint ranges, and the
    tip moves at most ``max_jacobian_column_norm`` metres per radian of joint motion, so

        fraction = tool_step_m / (extent * max_jacobian_column_norm)

    ``joint_limits`` is an (n, 2) array of (lower, upper). Raise ``ValueError`` for a
    non-positive ``tool_step_m``.
    """
    raise NotImplementedError  # TODO(student)
