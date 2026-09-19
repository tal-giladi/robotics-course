"""Reference solution for 08.06 — Derivative control: damping, noise and derivative kick.

Don't read this until you have made an honest attempt at ``student.py``.
Same formulation as ``labs/firmware/pico/velocity.py`` (P, ki-scaled integral with anti-windup,
derivative on the measurement), plus the first-order filter on the derivative the firmware omits.
"""

from __future__ import annotations


class PIDController:
    """u = kp*e + integral + d_term, clamped.  d_term = -kd * lowpass(d measured / dt)."""

    def __init__(self, kp: float, ki: float, kd: float, derivative_filter_tau_s: float = 0.0,
                 output_min: float = -1.0, output_max: float = 1.0) -> None:
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.derivative_filter_tau_s = derivative_filter_tau_s
        self.output_min = output_min
        self.output_max = output_max
        self.reset()

    def reset(self) -> None:
        self.integral = 0.0
        self.d_term = 0.0
        self.output = 0.0
        self._rate = 0.0  # filtered d(measured)/dt
        self._last_measured: float | None = None

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        error = setpoint - measured
        proportional = self.kp * error

        # Derivative on the MEASUREMENT (no kick when the setpoint jumps), low-pass filtered.
        if self._last_measured is not None and dt > 0:
            raw_rate = (measured - self._last_measured) / dt
            alpha = dt / (self.derivative_filter_tau_s + dt)  # tau = 0 -> alpha = 1 -> no filtering
            self._rate += alpha * (raw_rate - self._rate)
        self._last_measured = measured
        self.d_term = -self.kd * self._rate

        # Anti-windup exactly as in 08.05, with the D term included in the unclamped output.
        candidate = self.integral + self.ki * error * dt
        unclamped = proportional + candidate + self.d_term
        pushing_up = unclamped > self.output_max and error > 0
        pushing_down = unclamped < self.output_min and error < 0
        if not (pushing_up or pushing_down):
            self.integral = candidate
        self.integral = min(max(self.integral, self.output_min), self.output_max)

        output = proportional + self.integral + self.d_term
        self.output = min(max(output, self.output_min), self.output_max)
        return self.output
