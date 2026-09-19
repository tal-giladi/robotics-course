"""Reference solution for 08.05 — Integral control and anti-windup.

Don't read this until you have made an honest attempt at ``student.py``.
Same formulation as ``labs/firmware/pico/velocity.py`` (without feedforward and D).
"""

from __future__ import annotations


class PIController:
    """u = kp * e + integral,  integral += ki * e * dt,  u clamped to [output_min, output_max]."""

    def __init__(self, kp: float, ki: float, output_min: float = -1.0, output_max: float = 1.0,
                 anti_windup: bool = True) -> None:
        self.kp = kp
        self.ki = ki
        self.output_min = output_min
        self.output_max = output_max
        self.anti_windup = anti_windup
        self.integral = 0.0  # already multiplied by ki: in duty units
        self.output = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.output = 0.0

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        error = setpoint - measured
        proportional = self.kp * error
        candidate = self.integral + self.ki * error * dt
        if self.anti_windup:
            # 1) conditional integration: if the new integral would push the output past its limit in
            #    the direction the error pushes, keep the old one (same test as velocity.py)
            unclamped = proportional + candidate
            pushing_up = unclamped > self.output_max and error > 0
            pushing_down = unclamped < self.output_min and error < 0
            if not (pushing_up or pushing_down):
                self.integral = candidate
            # 2) clamping: the integral alone may never exceed the output range
            self.integral = min(max(self.integral, self.output_min), self.output_max)
        else:
            self.integral = candidate
        self.output = min(max(proportional + self.integral, self.output_min), self.output_max)
        return self.output
