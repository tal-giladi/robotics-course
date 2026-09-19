"""Differential-drive math used by base_node (pure Python, unit tested).

    v = R (ωr + ωl) / 2          ωl = (v - ω·L/2) / R
    ω = R (ωr - ωl) / L          ωr = (v + ω·L/2) / R

R = wheel radius, L = wheel separation, ωl/ωr = wheel angular velocities (rad/s).
"""
from __future__ import annotations

from dataclasses import dataclass
import math


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def twist_to_wheels(v: float, w: float, wheel_radius: float, wheel_separation: float,
                    max_wheel_speed: float) -> tuple[float, float]:
    """Body twist (m/s, rad/s) -> wheel speeds (rad/s), scaled down together if either wheel saturates.

    Scaling both wheels by the same factor keeps the path curvature (v/ω) the robot was asked to
    follow; clipping each wheel on its own would turn an arc into a different arc.
    """
    left = (v - w * wheel_separation / 2.0) / wheel_radius
    right = (v + w * wheel_separation / 2.0) / wheel_radius
    largest = max(abs(left), abs(right))
    if max_wheel_speed > 0.0 and largest > max_wheel_speed:
        scale = max_wheel_speed / largest
        left *= scale
        right *= scale
    return left, right


def wheels_to_twist(left: float, right: float, wheel_radius: float, wheel_separation: float) -> tuple[float, float]:
    """Wheel speeds (rad/s) -> body twist (m/s, rad/s)."""
    v = wheel_radius * (right + left) / 2.0
    w = wheel_radius * (right - left) / wheel_separation
    return v, w


def ramp(current: float, target: float, max_rate: float, dt: float) -> float:
    """Move current toward target by at most max_rate*dt (max_rate <= 0 disables the limit)."""
    if max_rate <= 0.0 or dt <= 0.0:
        return target
    step = max_rate * dt
    return current + clamp(target - current, -step, step)


@dataclass
class Pose2D:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


def wrap_angle(a: float) -> float:
    """Wrap to (-π, π]."""
    a = math.fmod(a + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


def integrate_odometry(pose: Pose2D, d_left: float, d_right: float, wheel_separation: float) -> Pose2D:
    """Advance a pose by wheel travel distances (m) using the exact arc (falls back to a line when straight)."""
    ds = (d_right + d_left) / 2.0
    dtheta = (d_right - d_left) / wheel_separation
    if abs(dtheta) < 1e-9:
        # midpoint rule == exact for a straight segment
        x = pose.x + ds * math.cos(pose.theta)
        y = pose.y + ds * math.sin(pose.theta)
    else:
        radius = ds / dtheta
        x = pose.x + radius * (math.sin(pose.theta + dtheta) - math.sin(pose.theta))
        y = pose.y - radius * (math.cos(pose.theta + dtheta) - math.cos(pose.theta))
    return Pose2D(x, y, wrap_angle(pose.theta + dtheta))


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    """(x, y, z, w) quaternion for a rotation about z."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
