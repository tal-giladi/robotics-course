"""08.11 — Reference solution: trapezoidal profile + wrap-safe heading controller.

Don't read this before you have tried ``student.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

TWO_PI = 2.0 * math.pi


def wrap(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    wrapped = (angle + math.pi) % TWO_PI - math.pi
    return math.pi if wrapped == -math.pi else wrapped


def heading_error(target_rad: float, measured_rad: float) -> float:
    return wrap(target_rad - measured_rad)


@dataclass(frozen=True)
class TrapezoidalProfile:
    distance: float
    v_max: float
    a_max: float

    @property
    def _sign(self) -> float:
        return 1.0 if self.distance >= 0 else -1.0

    @property
    def peak_v(self) -> float:
        if self.distance == 0:
            return 0.0
        # the triangular case: accelerate over half the distance, v^2 = 2 * a * (|d| / 2)
        return min(self.v_max, math.sqrt(self.a_max * abs(self.distance)))

    @property
    def t_accel(self) -> float:
        return self.peak_v / self.a_max if self.a_max else 0.0

    @property
    def t_cruise(self) -> float:
        if not self.peak_v:
            return 0.0
        return max(abs(self.distance) - self.peak_v * self.t_accel, 0.0) / self.peak_v

    @property
    def duration(self) -> float:
        return 2.0 * self.t_accel + self.t_cruise

    def velocity(self, t: float) -> float:
        ta, tc = self.t_accel, self.t_cruise
        if t <= 0.0 or t >= self.duration:
            return 0.0
        if t < ta:
            return self._sign * self.a_max * t
        if t < ta + tc:
            return self._sign * self.peak_v
        return self._sign * self.a_max * (self.duration - t)

    def position(self, t: float) -> float:
        ta, tc, v = self.t_accel, self.t_cruise, self.peak_v
        if t <= 0.0:
            return 0.0
        if t >= self.duration:
            return self.distance
        if t < ta:
            return self._sign * 0.5 * self.a_max * t * t
        if t < ta + tc:
            return self._sign * (0.5 * v * ta + v * (t - ta))
        left = self.duration - t
        return self._sign * (abs(self.distance) - 0.5 * self.a_max * left * left)


@dataclass
class TurnController:
    profile: TrapezoidalProfile
    start_heading_rad: float
    kp: float = 4.0
    ki: float = 1.0
    max_yaw_rate_rad_s: float = 2.5
    integral: float = 0.0
    error: float = 0.0
    feedforward: float = 0.0
    output: float = 0.0

    def target(self, t: float) -> float:
        return wrap(self.start_heading_rad + self.profile.position(t))

    def update(self, t: float, measured_rad: float, dt: float) -> float:
        self.error = heading_error(self.target(t), measured_rad)
        self.feedforward = self.profile.velocity(t)
        others = self.feedforward + self.kp * self.error
        candidate = self.integral + self.ki * self.error * dt
        low, high = -self.max_yaw_rate_rad_s, self.max_yaw_rate_rad_s
        if others + candidate > high and self.error > 0:
            self.integral = max(self.integral, high - others)
        elif others + candidate < low and self.error < 0:
            self.integral = min(self.integral, low - others)
        else:
            self.integral = candidate
        self.integral = min(max(self.integral, low), high)
        self.output = min(max(others + self.integral, low), high)
        return self.output

    def finished(self, t: float, measured_rad: float, tolerance_rad: float = math.radians(1.0)) -> bool:
        if t < self.profile.duration:
            return False
        final = wrap(self.start_heading_rad + self.profile.distance)
        return abs(heading_error(final, measured_rad)) <= tolerance_rad
