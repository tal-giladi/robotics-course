"""09.02 — cmd_vel to wheel speeds (with saturation).

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 09.02``.
Only the Python standard library is needed (``math``).

Conventions: meters, radians, seconds. ``v`` is forward speed (m/s), ``omega`` is the yaw rate
(rad/s, counter-clockwise = left turn = positive). Wheel speeds are wheel *angular* speeds in
rad/s, positive = rolling the robot forward. ``wheel_separation_m`` is the distance between the
two wheel contact points (the track width, "L" in the lesson).
"""

from __future__ import annotations

import math


def twist_to_wheel_speeds(
    v: float, omega: float, wheel_radius_m: float, wheel_separation_m: float
) -> tuple[float, float]:
    """Body twist (v [m/s], omega [rad/s]) -> wheel angular speeds (left, right) [rad/s].

    Each wheel's rim speed is v minus/plus omega times half the wheel separation;
    divide a rim speed by the wheel radius to get rad/s.
    """
    # TODO(student): inverse kinematics of a differential drive.
    raise NotImplementedError("twist_to_wheel_speeds")


def wheel_speeds_to_twist(
    left_rad_s: float, right_rad_s: float, wheel_radius_m: float, wheel_separation_m: float
) -> tuple[float, float]:
    """Wheel angular speeds [rad/s] -> body twist (v [m/s], omega [rad/s]) (forward kinematics, 09.01)."""
    # TODO(student): v is the mean rim speed; omega is the rim-speed difference over the separation.
    raise NotImplementedError("wheel_speeds_to_twist")


def limit_twist(v: float, omega: float, max_linear: float, max_angular: float) -> tuple[float, float]:
    """Enforce |v| <= max_linear and |omega| <= max_angular by scaling BOTH with one factor.

    Clamping each on its own would change the ratio v/omega, i.e. the radius of the arc the
    planner asked for. Return the inputs unchanged when both are already within the limits.
    """
    # TODO(student): find the most restrictive scale factor (<= 1) and apply it to both.
    raise NotImplementedError("limit_twist")


def scale_to_limit(left_rad_s: float, right_rad_s: float, max_wheel_rad_s: float) -> tuple[float, float]:
    """If either wheel is faster than ``max_wheel_rad_s``, scale BOTH by the same factor.

    The faster wheel ends up exactly at the limit and the ratio left/right is unchanged, so the
    robot follows the same arc, just more slowly.
    """
    # TODO(student): compare the larger |speed| with the limit; scale both if needed.
    raise NotImplementedError("scale_to_limit")


def keep_rotation_to_limit(left_rad_s: float, right_rad_s: float, max_wheel_rad_s: float) -> tuple[float, float]:
    """If a wheel is over the limit, keep the turning part and reduce the forward part instead.

    Split the command into mean = (left + right) / 2 (forward) and diff = (right - left) / 2
    (turning), so left = mean - diff and right = mean + diff.
      * both wheels within the limit -> return them unchanged
      * |diff| >= limit -> even turning alone is too fast: spin in place at the limit,
        in the direction of diff
      * otherwise clamp mean to +-(limit - |diff|) and rebuild left and right
    """
    # TODO(student): implement the three cases.
    raise NotImplementedError("keep_rotation_to_limit")


def cmd_vel_to_wheels(
    v: float,
    omega: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
    max_wheel_rad_s: float,
    max_linear: float = math.inf,
    max_angular: float = math.inf,
    keep_rotation: bool = False,
) -> tuple[float, float]:
    """The whole pipeline a base driver runs for every cmd_vel message.

    1. limit_twist  2. twist_to_wheel_speeds  3. keep_rotation_to_limit if ``keep_rotation``
    else scale_to_limit.
    """
    # TODO(student): chain the functions above.
    raise NotImplementedError("cmd_vel_to_wheels")
