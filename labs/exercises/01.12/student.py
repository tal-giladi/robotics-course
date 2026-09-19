"""01.12 — Ticks to distances and angles, and back.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 01.12``.
Only the Python standard library is needed (``math``).

Conventions (REP-103): metres, radians, x forward, y left, counter-clockwise positive.
A *tick* is one count of the quadrature encoder; karmel has 2464 of them per wheel
revolution (11 pulses x 56 gear ratio x 4 quadrature edges). Tick counts are **signed**:
forward is positive for both wheels, so a left turn in place is (negative, positive).
"""

from __future__ import annotations

import math

TickPair = tuple[int, int]  # (left_ticks, right_ticks)


def meters_per_tick(wheel_radius_m: float, ticks_per_rev: int) -> float:
    """How far the wheel rim travels in one tick.

    One revolution moves the rim by the wheel circumference, and takes ``ticks_per_rev`` ticks.
    """
    # TODO(student): circumference / ticks per revolution.
    raise NotImplementedError("meters_per_tick")


def ticks_to_distance(ticks: int, wheel_radius_m: float, ticks_per_rev: int) -> float:
    """Distance in metres that one wheel travelled for ``ticks`` ticks (negative = backwards)."""
    # TODO(student): use meters_per_tick.
    raise NotImplementedError("ticks_to_distance")


def distance_to_ticks(distance_m: float, wheel_radius_m: float, ticks_per_rev: int) -> int:
    """Ticks one wheel must turn to travel ``distance_m``, rounded to the nearest whole tick.

    Use ``round()`` — Python rounds halves to even, which is fine here, and the residual error
    is at most half a tick (0.06 mm on karmel).
    """
    # TODO(student): the inverse of ticks_to_distance, rounded to an int.
    raise NotImplementedError("distance_to_ticks")


def drive_straight_ticks(distance_m: float, wheel_radius_m: float, ticks_per_rev: int) -> TickPair:
    """Tick targets for driving straight: both wheels travel the same signed distance."""
    # TODO(student): return the same value for both wheels.
    raise NotImplementedError("drive_straight_ticks")


def turn_in_place_ticks(
    angle_rad: float, wheel_radius_m: float, wheel_separation_m: float, ticks_per_rev: int
) -> TickPair:
    """Tick targets for turning ``angle_rad`` on the spot (positive = left / counter-clockwise).

    Turning in place spins the robot about the midpoint between the wheels, so each wheel
    follows a circle of radius ``wheel_separation_m / 2``. Turning by ``angle_rad`` moves each
    rim along an arc of ``angle_rad * wheel_separation_m / 2`` — the **left** wheel backwards
    and the **right** wheel forwards for a left turn.
    """
    # TODO(student): arc length per wheel, then distance_to_ticks, with opposite signs.
    raise NotImplementedError("turn_in_place_ticks")


def arc_ticks(
    radius_m: float,
    angle_rad: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
    ticks_per_rev: int,
) -> TickPair:
    """Tick targets for a curve: the robot's centre follows a circle of radius ``radius_m``.

    ``radius_m`` is measured from the centre of the circle to the midpoint between the wheels
    and is always >= 0. ``angle_rad`` is the turn angle, positive counter-clockwise (the centre
    of the circle is then to the robot's **left**), so:

        inner wheel travels (radius_m - wheel_separation_m / 2) * angle_rad
        outer wheel travels (radius_m + wheel_separation_m / 2) * angle_rad

    For a left turn the left wheel is the inner one. ``radius_m = 0`` must give the same answer
    as :func:`turn_in_place_ticks`; a radius smaller than half the track makes the inner wheel
    run backwards, which is correct (the robot pivots around a point between the wheels).
    """
    # TODO(student): left = (radius - b/2) * angle, right = (radius + b/2) * angle, then to ticks.
    raise NotImplementedError("arc_ticks")


def distance_from_ticks(
    left_ticks: int, right_ticks: int, wheel_radius_m: float, ticks_per_rev: int
) -> float:
    """How far the robot's centre moved, from a pair of tick *deltas*: the mean of the wheels."""
    # TODO(student): mean of the two wheel distances.
    raise NotImplementedError("distance_from_ticks")


def heading_change_from_ticks(
    left_ticks: int,
    right_ticks: int,
    wheel_radius_m: float,
    wheel_separation_m: float,
    ticks_per_rev: int,
) -> float:
    """Heading change in radians from a pair of tick *deltas* (positive = turned left).

    The wheels are ``wheel_separation_m`` apart, so a difference in how far they rolled is an
    angle: ``dtheta = (d_right - d_left) / wheel_separation_m``. Do **not** wrap the result —
    three full left turns must read +6*pi, not 0 (a turn controller needs to know).
    """
    # TODO(student): difference of the wheel distances divided by the track width.
    raise NotImplementedError("heading_change_from_ticks")


def square_plan(
    side_m: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
    ticks_per_rev: int,
    clockwise: bool = False,
) -> list[TickPair]:
    """The eight segments of a square: straight, turn, straight, turn, ... (counter-clockwise).

    Returns tick targets *per segment* (deltas, not cumulative), starting with a straight side.
    With ``clockwise=True`` the turns are right turns (-90 degrees).
    """
    # TODO(student): four times [drive_straight_ticks(side), turn_in_place_ticks(+-pi/2)].
    raise NotImplementedError("square_plan")
