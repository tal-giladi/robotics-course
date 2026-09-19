"""09.04 — Build an odometry system from scratch.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 09.04``.
Only the Python standard library is needed (``math``).

Conventions: meters, radians, heading ``theta`` wrapped to (-pi, pi], x forward, y left,
counter-clockwise positive. A pose is a plain tuple ``(x, y, theta)``.
"""

from __future__ import annotations

import math

Pose = tuple[float, float, float]  # (x [m], y [m], theta [rad] in (-pi, pi])


def tick_delta(previous: int, current: int, encoder_bits: int | None = None) -> int:
    """Signed tick change between two encoder readings.

    If ``encoder_bits`` is None the counter never wraps: return ``current - previous``.
    Otherwise the counter is an N-bit signed hardware counter that wraps around; e.g. with
    8 bits, going from 127 to -128 is a change of +1, not -255. Assume the true change is
    always smaller than half the counter range.
    """
    # TODO(student): handle the non-wrapping case, then the wrap-around case.
    #   Hint: the modulus is 2**encoder_bits; Python's % always returns a non-negative result.
    raise NotImplementedError("tick_delta")


def ticks_to_distance(ticks: int, wheel_radius_m: float, ticks_per_rev: int) -> float:
    """Distance (m) the wheel rim travels for ``ticks`` encoder ticks (negative = backwards)."""
    # TODO(student): one revolution = ticks_per_rev ticks = one wheel circumference.
    raise NotImplementedError("ticks_to_distance")


def integrate_pose(pose: Pose, d_left: float, d_right: float, wheel_separation_m: float) -> Pose:
    """Advance ``pose`` by the distances each wheel travelled, along the exact arc.

    1. ds = mean of the wheel distances, dtheta = (d_right - d_left) / wheel_separation_m.
    2. If |dtheta| is tiny (< 1e-9) the robot drove straight: move ds along theta.
       Otherwise it drove along an arc of radius R = ds / dtheta:
           x += R * (sin(theta + dtheta) - sin(theta))
           y -= R * (cos(theta + dtheta) - cos(theta))
    3. Wrap the new heading to (-pi, pi] (math.atan2(sin, cos) does it).
    """
    # TODO(student): implement the straight-line case and the exact-arc case.
    raise NotImplementedError("integrate_pose")


class Odometry:
    """Wheel odometry: feed cumulative encoder counts, get the robot pose.

    >>> odom = Odometry(wheel_radius_m=0.045, wheel_separation_m=0.2, ticks_per_rev=2464)
    >>> odom.update(0, 0)          # first reading: nothing to integrate yet
    (0.0, 0.0, 0.0)
    """

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
        # TODO(student): remember that no encoder reading has been seen yet.

    def update(self, left_ticks: int, right_ticks: int) -> Pose:
        """Integrate the motion since the previous call and return the new pose.

        The encoder counts are cumulative and need not start at zero, so the *first* call can
        only remember them and return the current pose unchanged.
        """
        # TODO(student):
        #   1. first call: store the counts, return self.pose
        #   2. tick deltas per wheel (tick_delta, with self.encoder_bits)
        #   3. deltas -> distances (ticks_to_distance)
        #   4. self.pose = integrate_pose(...); store the counts; return self.pose
        raise NotImplementedError("Odometry.update")

