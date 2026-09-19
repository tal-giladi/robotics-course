"""Reference solution for 09.04 — Build an odometry system from scratch.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math

Pose = tuple[float, float, float]  # (x [m], y [m], theta [rad] in (-pi, pi])


def tick_delta(previous: int, current: int, encoder_bits: int | None = None) -> int:
    """Signed tick change between two encoder readings.

    With ``encoder_bits`` set, the counter wraps like an N-bit hardware counter, so a jump from
    near the maximum to near the minimum is a small forward step, not a huge backward one.
    """
    delta = current - previous
    if encoder_bits is None:
        return delta
    modulus = 1 << encoder_bits
    half = modulus // 2
    return (delta + half) % modulus - half


def ticks_to_distance(ticks: int, wheel_radius_m: float, ticks_per_rev: int) -> float:
    """Distance the wheel rim travels for ``ticks`` encoder ticks."""
    return 2.0 * math.pi * wheel_radius_m * ticks / ticks_per_rev


def integrate_pose(pose: Pose, d_left: float, d_right: float, wheel_separation_m: float) -> Pose:
    """Advance ``pose`` by the wheel travels, along the exact arc (straight line if no turn)."""
    x, y, theta = pose
    ds = (d_left + d_right) / 2.0
    dtheta = (d_right - d_left) / wheel_separation_m
    if abs(dtheta) < 1e-9:
        x += ds * math.cos(theta)
        y += ds * math.sin(theta)
    else:
        radius = ds / dtheta
        x += radius * (math.sin(theta + dtheta) - math.sin(theta))
        y -= radius * (math.cos(theta + dtheta) - math.cos(theta))
    theta = math.atan2(math.sin(theta + dtheta), math.cos(theta + dtheta))
    return (x, y, theta)


class Odometry:
    """Wheel odometry: feed cumulative encoder counts, get the robot pose."""

    def __init__(
        self,
        wheel_radius_m: float,
        wheel_separation_m: float,
        ticks_per_rev: int,
        encoder_bits: int | None = None,
        initial_pose: Pose = (0.0, 0.0, 0.0),
    ) -> None:
        self.wheel_radius_m = wheel_radius_m
        self.wheel_separation_m = wheel_separation_m
        self.ticks_per_rev = ticks_per_rev
        self.encoder_bits = encoder_bits
        self.pose: Pose = initial_pose
        self._last_ticks: tuple[int, int] | None = None

    def update(self, left_ticks: int, right_ticks: int) -> Pose:
        """Integrate the motion since the previous call and return the new pose.

        The first call only remembers the counts (they need not start at zero).
        """
        if self._last_ticks is None:
            self._last_ticks = (left_ticks, right_ticks)
            return self.pose
        d_left_ticks = tick_delta(self._last_ticks[0], left_ticks, self.encoder_bits)
        d_right_ticks = tick_delta(self._last_ticks[1], right_ticks, self.encoder_bits)
        self._last_ticks = (left_ticks, right_ticks)
        d_left = ticks_to_distance(d_left_ticks, self.wheel_radius_m, self.ticks_per_rev)
        d_right = ticks_to_distance(d_right_ticks, self.wheel_radius_m, self.ticks_per_rev)
        self.pose = integrate_pose(self.pose, d_left, d_right, self.wheel_separation_m)
        return self.pose
