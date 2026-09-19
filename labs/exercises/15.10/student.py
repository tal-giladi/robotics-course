"""15.10 — mobile manipulation: the support polygon, the centre of mass, and the frame chain.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.10``.
Only the standard library and numpy are needed — no arm, no Nav2, no MoveIt.

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


# --- implement these ---------------------------------------------------------------------------
def convex_hull_2d(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """The counter-clockwise convex hull of a set of 2-D points (monotone chain is fine).

    The support polygon is the hull of the ground contacts: a wheel *inside* the hull carries
    load but contributes nothing to stability, and must not appear in the output.

    Sort the unique points, build the lower and upper chains with a cross-product test, and
    concatenate them without repeating the endpoints. Fewer than three distinct points returns
    them sorted.
    """
    raise NotImplementedError("convex_hull_2d")  # TODO(student)


def support_polygon(wheel_separation_m: float, caster_x_m: float,
                    front_caster_x_m: float | None = None) -> list[tuple[float, float]]:
    """The ground contacts of a differential-drive base, hulled, in the base frame.

    The two drive wheels sit on the axle at ``x = 0``, at ``y = +/- wheel_separation_m / 2``.
    ``caster_x_m`` is the rear caster (negative: behind the axle). ``front_caster_x_m``, when
    given, adds **two** contacts at that x and the same +/- y as the wheels.

    Return ``convex_hull_2d`` of those contacts. Look hard at what you get back with no front
    caster — that shape is the whole lesson.
    """
    raise NotImplementedError("support_polygon")  # TODO(student)


def combined_com(parts: list[tuple[float, tuple[float, float, float]]]
                 ) -> tuple[float, Array]:
    """(total mass, centre of mass) of a list of ``(mass_kg, (x, y, z))`` parts.

    The CoM is the mass-weighted mean: ``sum(m_i p_i) / sum(m_i)``. Raise ``ValueError`` for an
    empty list or a non-positive total mass — a silent NaN here becomes a robot that thinks it is
    stable.
    """
    raise NotImplementedError("combined_com")  # TODO(student)


def stability_margin(support: list[tuple[float, float]], com_xy: ArrayLike) -> float:
    """Distance from the CoM's ground projection to the nearest edge of the support polygon.

    **Positive inside** (stable, by that many metres), **negative outside** (it tips). For each
    edge of the counter-clockwise polygon, the inward normal is the edge's left normal
    ``(-dy, dx) / |edge|``; the signed distance is ``normal . (p - a)``. The point is inside when
    every signed distance is >= 0, and the margin is the smallest magnitude either way.

    This is the static criterion. It says nothing about the inertial term when the arm
    *accelerates* outward, which is why the practical rule is "margin > 30 mm **and** move slowly".
    """
    raise NotImplementedError("stability_margin")  # TODO(student)


def object_in_arm_frame(base_xy_yaw: tuple[float, float, float],
                        mount_xyz: tuple[float, float, float],
                        object_map_xyz: ArrayLike) -> Array:
    """Where an object in the map is, expressed in the ARM's base frame.

    The chain is ``map -> base_link -> arm base_link``. The base is at ``(x, y)`` with heading
    ``yaw`` on the ground; the arm is bolted on at ``mount_xyz`` with no rotation. So:

        p_arm = Rz(yaw)^T (p_map - base_xyz) - mount_xyz

    where ``base_xyz = (x, y, 0)``. Getting the transpose the wrong way round gives an answer that
    is right whenever yaw = 0, which is every test you will write by hand.
    """
    raise NotImplementedError("object_in_arm_frame")  # TODO(student)


def should_use_arm(manipulability: float, margin_m: float, *,
                   min_manipulability: float = 0.010,
                   min_margin_m: float = 0.030) -> bool:
    """Move the arm (True) or re-dock the base (False)?

    Both conditions must hold: the arm must be far enough from a singularity
    (``manipulability >= min_manipulability``, the sqrt(det(J J^T)) of 14.06) **and** the robot
    must be far enough from tipping (``margin_m >= min_margin_m``). Either one failing means
    re-dock: 5–15 seconds and a fresh perception cycle is cheaper than a tipped robot.
    """
    raise NotImplementedError("should_use_arm")  # TODO(student)
