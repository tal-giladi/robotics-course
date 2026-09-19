"""01.12 — Ticks to distances and angles, and back. Reference solution.

Don't read this before you have tried ``student.py``. Same public names, same conventions.
"""

from __future__ import annotations

import math

TickPair = tuple[int, int]  # (left_ticks, right_ticks)


def meters_per_tick(wheel_radius_m: float, ticks_per_rev: int) -> float:
    """How far the wheel rim travels in one tick."""
    return 2.0 * math.pi * wheel_radius_m / ticks_per_rev


def ticks_to_distance(ticks: int, wheel_radius_m: float, ticks_per_rev: int) -> float:
    """Distance in metres that one wheel travelled for ``ticks`` ticks (negative = backwards)."""
    return ticks * meters_per_tick(wheel_radius_m, ticks_per_rev)


def distance_to_ticks(distance_m: float, wheel_radius_m: float, ticks_per_rev: int) -> int:
    """Ticks one wheel must turn to travel ``distance_m``, rounded to the nearest whole tick."""
    return round(distance_m / meters_per_tick(wheel_radius_m, ticks_per_rev))


def drive_straight_ticks(distance_m: float, wheel_radius_m: float, ticks_per_rev: int) -> TickPair:
    """Tick targets for driving straight: both wheels travel the same signed distance."""
    ticks = distance_to_ticks(distance_m, wheel_radius_m, ticks_per_rev)
    return ticks, ticks


def turn_in_place_ticks(
    angle_rad: float, wheel_radius_m: float, wheel_separation_m: float, ticks_per_rev: int
) -> TickPair:
    """Tick targets for turning ``angle_rad`` on the spot (positive = left / counter-clockwise)."""
    arc_m = angle_rad * wheel_separation_m / 2.0
    ticks = distance_to_ticks(arc_m, wheel_radius_m, ticks_per_rev)
    return -ticks, ticks


def arc_ticks(
    radius_m: float,
    angle_rad: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
    ticks_per_rev: int,
) -> TickPair:
    """Tick targets for a curve whose centre-of-robot radius is ``radius_m``."""
    half_track = wheel_separation_m / 2.0
    left_m = (radius_m - half_track) * angle_rad
    right_m = (radius_m + half_track) * angle_rad
    return (
        distance_to_ticks(left_m, wheel_radius_m, ticks_per_rev),
        distance_to_ticks(right_m, wheel_radius_m, ticks_per_rev),
    )


def distance_from_ticks(
    left_ticks: int, right_ticks: int, wheel_radius_m: float, ticks_per_rev: int
) -> float:
    """How far the robot's centre moved, from a pair of tick deltas."""
    left_m = ticks_to_distance(left_ticks, wheel_radius_m, ticks_per_rev)
    right_m = ticks_to_distance(right_ticks, wheel_radius_m, ticks_per_rev)
    return 0.5 * (left_m + right_m)


def heading_change_from_ticks(
    left_ticks: int,
    right_ticks: int,
    wheel_radius_m: float,
    wheel_separation_m: float,
    ticks_per_rev: int,
) -> float:
    """Heading change in radians from a pair of tick deltas (positive = turned left)."""
    left_m = ticks_to_distance(left_ticks, wheel_radius_m, ticks_per_rev)
    right_m = ticks_to_distance(right_ticks, wheel_radius_m, ticks_per_rev)
    return (right_m - left_m) / wheel_separation_m


def square_plan(
    side_m: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
    ticks_per_rev: int,
    clockwise: bool = False,
) -> list[TickPair]:
    """The eight segments of a square: straight, turn, straight, turn, ..."""
    turn = -math.pi / 2 if clockwise else math.pi / 2
    plan: list[TickPair] = []
    for _ in range(4):
        plan.append(drive_straight_ticks(side_m, wheel_radius_m, ticks_per_rev))
        plan.append(turn_in_place_ticks(turn, wheel_radius_m, wheel_separation_m, ticks_per_rev))
    return plan
