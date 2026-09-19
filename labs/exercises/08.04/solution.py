"""Reference solution for 08.04 — Proportional control: hold a wheel speed.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math


def steady_state_speed(setpoint: float, kp: float, slope: float, deadband: float) -> float:
    """Where a P-only speed loop settles, for a motor with a deadband (lesson 08.04, Level 3).

    Motor (08.02):  w = slope * (u - deadband)  for u > deadband, else 0;  saturates at u = 1.
    Controller:     u = kp * (setpoint - w)
    Substitute and solve for w:  w = slope * (kp * r - deadband) / (1 + kp * slope)
    """
    r = abs(setpoint)
    if r == 0.0 or kp * r <= deadband:
        return 0.0  # the command at w = 0 cannot even overcome the deadband: the wheel never starts
    w = slope * (kp * r - deadband) / (1.0 + kp * slope)
    w = min(w, slope * (1.0 - deadband))  # the output can't exceed duty 1.0
    return math.copysign(w, setpoint)


class PController:
    """u = kp * (setpoint - measured), clamped to [-output_limit, +output_limit]."""

    def __init__(self, kp: float, output_limit: float = 1.0) -> None:
        self.kp = kp
        self.output_limit = output_limit
        self.error = 0.0
        self.output = 0.0

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        self.error = setpoint - measured
        u = self.kp * self.error
        self.output = max(-self.output_limit, min(self.output_limit, u))
        return self.output

    def reset(self) -> None:
        self.error = 0.0
        self.output = 0.0
