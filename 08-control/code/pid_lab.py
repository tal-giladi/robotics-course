"""Reference controllers and small models for lessons 08.07-08.12.

The exercises of 08.04-08.06 are where YOU write P, PI and PID. This module holds the finished
tools the later lessons build on, in the same formulation as the firmware
(`labs/firmware/pico/velocity.py`):

* the integral is stored already multiplied by ki (duty units);
* anti-windup = conditional integration that fills the remaining headroom, then a clamp;
* derivative on the measurement, optionally low-pass filtered.

Nothing here talks to hardware; the scripts pass these objects to `control_lab.run_loop`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from control_lab import ROOT  # noqa: F401  (puts labs/python on sys.path)
from robotlab.config import KarmelConfig, load_config


# --- controllers ----------------------------------------------------------------------------------
@dataclass
class PositionalPID:
    """u = feedforward + kp*e + integral + d_term, clamped to [output_min, output_max].

    anti_windup: "fill" (firmware: integrate only up to the limit), "back_calculation"
    (integral += (u_clamped - u_unclamped) * dt / tracking_time_s) or "none".
    """

    kp: float
    ki: float
    kd: float = 0.0
    derivative_filter_tau_s: float = 0.0
    output_min: float = -1.0
    output_max: float = 1.0
    anti_windup: str = "fill"
    tracking_time_s: float = 0.05
    integral: float = 0.0
    d_term: float = 0.0
    output: float = 0.0
    _rate: float = field(default=0.0, repr=False)
    _last_measured: float | None = field(default=None, repr=False)

    def reset(self) -> None:
        self.integral = self.d_term = self.output = self._rate = 0.0
        self._last_measured = None

    def update(self, setpoint: float, measured: float, dt: float, feedforward: float = 0.0) -> float:
        error = setpoint - measured
        if self._last_measured is not None and dt > 0:
            alpha = dt / (self.derivative_filter_tau_s + dt)
            self._rate += alpha * ((measured - self._last_measured) / dt - self._rate)
        self._last_measured = measured
        self.d_term = -self.kd * self._rate
        others = feedforward + self.kp * error + self.d_term
        candidate = self.integral + self.ki * error * dt
        if self.anti_windup == "fill":
            if others + candidate > self.output_max and error > 0:
                self.integral = max(self.integral, self.output_max - others)
            elif others + candidate < self.output_min and error < 0:
                self.integral = min(self.integral, self.output_min - others)
            else:
                self.integral = candidate
            self.integral = min(max(self.integral, self.output_min), self.output_max)
        elif self.anti_windup == "back_calculation":
            unclamped = others + candidate
            clamped = min(max(unclamped, self.output_min), self.output_max)
            self.integral = candidate + (clamped - unclamped) * dt / self.tracking_time_s
        else:
            self.integral = candidate
        self.output = min(max(others + self.integral, self.output_min), self.output_max)
        return self.output


@dataclass
class VelocityFormPID:
    """The incremental ("velocity") form: compute the CHANGE of the output each period.

        du_k = kp (e_k - e_{k-1}) + ki e_k dt - kd (y_k - 2 y_{k-1} + y_{k-2}) / dt
        u_k  = clamp(u_{k-1} + du_k)

    The only state is the last output, so clamping it is all the anti-windup it needs.
    """

    kp: float
    ki: float
    kd: float = 0.0
    output_min: float = -1.0
    output_max: float = 1.0
    output: float = 0.0
    _e1: float | None = field(default=None, repr=False)
    _y1: float | None = field(default=None, repr=False)
    _y2: float | None = field(default=None, repr=False)

    def reset(self) -> None:
        self.output = 0.0
        self._e1 = self._y1 = self._y2 = None

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        error = setpoint - measured
        du = self.ki * error * dt
        if self._e1 is not None:
            du += self.kp * (error - self._e1)
        else:
            du += self.kp * error  # first call: the P term appears from nothing
        if self._y2 is not None and dt > 0:
            du -= self.kd * (measured - 2 * self._y1 + self._y2) / dt
        self._e1, self._y2, self._y1 = error, self._y1, measured
        self.output = min(max(self.output + du, self.output_min), self.output_max)
        return self.output


# --- the motor model (08.02) ----------------------------------------------------------------------
@dataclass(frozen=True)
class MotorModel:
    """Steady state: speed = slope * (battery_v / nominal_v) * (duty - deadband) above the deadband.

    slope = max_wheel_speed / (1 - deadband) at the nominal voltage (karmel.yaml: 17 / 0.88 = 19.3).
    """

    max_wheel_speed_rad_s: float
    deadband: float
    tau_s: float
    nominal_v: float

    @classmethod
    def from_config(cls, cfg: KarmelConfig | None = None) -> MotorModel:
        cfg = cfg or load_config()
        d = cfg.drive
        return cls(d.max_wheel_speed_rad_s, d.duty_deadband, d.motor_time_constant_s, cfg.battery.nominal_v)

    @property
    def slope(self) -> float:
        return self.max_wheel_speed_rad_s / (1.0 - self.deadband)

    def top_speed(self, battery_v: float | None = None) -> float:
        """Speed at duty 1.0 for this battery voltage (nominal if unknown)."""
        v = battery_v if battery_v and battery_v > 0 else self.nominal_v
        return self.max_wheel_speed_rad_s * v / self.nominal_v

    def compensate_deadband(self, u: float) -> float:
        """Map u in [-1, 1] onto the duties that move the wheel (firmware motors.compensate_deadband).
        After this the wheel speed is simply top_speed * u: a linear plant for the linear theory."""
        if u == 0:
            return 0.0
        return math.copysign(self.deadband + min(abs(u), 1.0) * (1.0 - self.deadband), u)


# --- a sampled-loop model for predictions (08.07) -------------------------------------------------
def simulate_pi_loop(K: float, tau: float, kp: float, ki: float, setpoint: float = 8.0, loop_dt: float = 0.02,
                     measurement_lag_s: float = 0.0, delay_s: float = 0.0, seconds: float = 2.0,
                     h: float = 0.0005) -> tuple[list[float], list[float]]:
    """Linear model: K/(tau s + 1) motor, first-order measurement lag, extra dead time, PI sampled every
    loop_dt with the output held (ZOH). Returns the MEASURED speed at each loop sample (like a log)."""
    delay_n = round(delay_s / h)
    queue = [0.0] * delay_n
    y = measured = integral = u = 0.0
    next_sample = 0.0
    t_log: list[float] = []
    y_log: list[float] = []
    for k in range(round(seconds / h)):
        now = k * h
        if now >= next_sample - 1e-12:
            t_log.append(now)
            y_log.append(measured)
            error = setpoint - measured
            integral += ki * error * loop_dt
            u = kp * error + integral
            next_sample += loop_dt
        queue.append(u)
        applied = queue.pop(0)
        y += (K * applied - y) * (1.0 - math.exp(-h / tau))
        measured += (y - measured) * ((1.0 - math.exp(-h / measurement_lag_s)) if measurement_lag_s > 0 else 1.0)
    return t_log, y_log


def lambda_phase_margin_deg(tau_cl: float, delay_s: float) -> float:
    """Loop 1/(tau_cl s) * exp(-delay s): crossover at 1/tau_cl, phase margin = 90 deg - delay/tau_cl rad."""
    return 90.0 - math.degrees(delay_s / tau_cl)
