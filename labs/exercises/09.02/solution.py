"""Reference solution for 09.02 — cmd_vel to wheel speeds (with saturation).

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math


def twist_to_wheel_speeds(
    v: float, omega: float, wheel_radius_m: float, wheel_separation_m: float
) -> tuple[float, float]:
    """Body twist (v [m/s], omega [rad/s]) -> wheel angular speeds (left, right) [rad/s]."""
    half = omega * wheel_separation_m / 2.0
    return (v - half) / wheel_radius_m, (v + half) / wheel_radius_m


def wheel_speeds_to_twist(
    left_rad_s: float, right_rad_s: float, wheel_radius_m: float, wheel_separation_m: float
) -> tuple[float, float]:
    """Wheel angular speeds [rad/s] -> body twist (v [m/s], omega [rad/s])."""
    v = wheel_radius_m * (right_rad_s + left_rad_s) / 2.0
    omega = wheel_radius_m * (right_rad_s - left_rad_s) / wheel_separation_m
    return v, omega


def limit_twist(v: float, omega: float, max_linear: float, max_angular: float) -> tuple[float, float]:
    """Scale v and omega by ONE factor so |v| <= max_linear and |omega| <= max_angular."""
    scale = 1.0
    if abs(v) > max_linear:
        scale = min(scale, max_linear / abs(v))
    if abs(omega) > max_angular:
        scale = min(scale, max_angular / abs(omega))
    return v * scale, omega * scale


def scale_to_limit(left_rad_s: float, right_rad_s: float, max_wheel_rad_s: float) -> tuple[float, float]:
    """If a wheel is over the limit, scale BOTH wheels by the same factor (keeps the path curvature)."""
    largest = max(abs(left_rad_s), abs(right_rad_s))
    if largest <= max_wheel_rad_s:
        return left_rad_s, right_rad_s
    scale = max_wheel_rad_s / largest
    return left_rad_s * scale, right_rad_s * scale


def keep_rotation_to_limit(left_rad_s: float, right_rad_s: float, max_wheel_rad_s: float) -> tuple[float, float]:
    """If a wheel is over the limit, keep the turning rate and give up forward speed instead."""
    if max(abs(left_rad_s), abs(right_rad_s)) <= max_wheel_rad_s:
        return left_rad_s, right_rad_s
    mean = (left_rad_s + right_rad_s) / 2.0  # the forward part
    diff = (right_rad_s - left_rad_s) / 2.0  # the turning part
    if abs(diff) >= max_wheel_rad_s:  # even a pure spin is too fast: spin at the limit
        spin = math.copysign(max_wheel_rad_s, diff)
        return -spin, spin
    room = max_wheel_rad_s - abs(diff)  # forward speed still available
    mean = max(-room, min(room, mean))
    return mean - diff, mean + diff


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
    """The whole pipeline a base driver runs on every cmd_vel message."""
    v, omega = limit_twist(v, omega, max_linear, max_angular)
    left, right = twist_to_wheel_speeds(v, omega, wheel_radius_m, wheel_separation_m)
    if keep_rotation:
        return keep_rotation_to_limit(left, right, max_wheel_rad_s)
    return scale_to_limit(left, right, max_wheel_rad_s)
