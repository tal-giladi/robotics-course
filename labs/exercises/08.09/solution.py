"""08.09 — Reference solution: model feedforward for wheel speed, plus a small PI.

Don't read this before you have tried ``student.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def wheel_speed_for(ground_speed_m_s: float, wheel_radius_m: float) -> float:
    """v = r * w, so w = v / r."""
    if wheel_radius_m <= 0:
        raise ValueError("wheel_radius_m must be positive")
    return ground_speed_m_s / wheel_radius_m


@dataclass(frozen=True)
class FeedforwardModel:
    """The 08.02 motor model, inverted: duty = deadband + |w|/top_speed * (1 - deadband)."""

    max_wheel_speed_rad_s: float
    deadband: float
    nominal_v: float
    use_battery: bool = True

    def top_speed(self, battery_v: float | None = None) -> float:
        if not self.use_battery or battery_v is None or battery_v <= 0:
            return self.max_wheel_speed_rad_s
        return self.max_wheel_speed_rad_s * battery_v / self.nominal_v

    def duty(self, setpoint_rad_s: float, battery_v: float | None = None) -> float:
        if setpoint_rad_s == 0:
            return 0.0  # exactly zero: never send the deadband duty, the wheel would creep
        fraction = min(abs(setpoint_rad_s) / self.top_speed(battery_v), 1.0)
        duty = self.deadband + fraction * (1.0 - self.deadband)
        return math.copysign(min(duty, 1.0), setpoint_rad_s)


@dataclass
class FeedforwardPI:
    """duty = feedforward + kp*e + integral, clamped; anti-windup as in 08.05 / velocity.py."""

    model: FeedforwardModel
    kp: float
    ki: float
    output_min: float = -1.0
    output_max: float = 1.0
    integral: float = 0.0
    feedforward: float = 0.0
    output: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.feedforward = 0.0
        self.output = 0.0

    def update(self, setpoint_rad_s: float, measured_rad_s: float, dt: float,
               battery_v: float | None = None) -> float:
        error = setpoint_rad_s - measured_rad_s
        self.feedforward = self.model.duty(setpoint_rad_s, battery_v)
        others = self.feedforward + self.kp * error  # everything except the integral
        candidate = self.integral + self.ki * error * dt
        if others + candidate > self.output_max and error > 0:
            self.integral = max(self.integral, self.output_max - others)
        elif others + candidate < self.output_min and error < 0:
            self.integral = min(self.integral, self.output_min - others)
        else:
            self.integral = candidate
        self.integral = min(max(self.integral, self.output_min), self.output_max)
        self.output = min(max(others + self.integral, self.output_min), self.output_max)
        return self.output
