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
from typing import Any

import numpy as np

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

    def duty_for_speed(self, speed_rad_s: float, battery_v: float | None = None) -> float:
        """Invert the model: the duty that should give this wheel speed (08.09's feedforward).

        ``compensate_deadband(speed / top_speed(battery))`` — the deadband is added back and the
        slope is corrected for the measured battery voltage.
        """
        if speed_rad_s == 0:
            return 0.0
        return self.compensate_deadband(speed_rad_s / self.top_speed(battery_v))


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


# --- tuning: cost functions, model fit and rule tables (08.08) -------------------------------------
def iae(t: Any, y: Any, setpoint: float) -> float:
    """Integral of |error| dt: every unit of error costs the same, whenever it happens."""
    t = np.asarray(t, dtype=float)
    return float(np.trapezoid(np.abs(setpoint - np.asarray(y, dtype=float)), t))


def itae(t: Any, y: Any, setpoint: float, t_step: float = 0.0) -> float:
    """Integral of t*|error| dt: errors that are still there late cost much more (kills ringing)."""
    t = np.asarray(t, dtype=float)
    e = np.abs(setpoint - np.asarray(y, dtype=float))
    return float(np.trapezoid(np.maximum(t - t_step, 0.0) * e, t))


def fit_first_order_plus_delay(t: Any, y: Any, duty_step: float, t_step: float) -> tuple[float, float, float]:
    """Fit ``y(t) = K*duty_step*(1 - exp(-(t - t_step - theta)/tau))`` to a logged OPEN-LOOP step.

    Returns ``(K, tau, theta)``: gain in rad/s per unit of duty *actually applied*, the time
    constant and the apparent dead time (loop period + estimator lag + real transport delay).
    This is the only model the tuning rules of 08.08 need.
    """
    from scipy.optimize import curve_fit

    t = np.asarray(t, dtype=float)
    y = np.asarray(y, dtype=float)

    def model(tt: Any, gain: float, tau: float, theta: float) -> Any:
        s = np.maximum(tt - t_step - theta, 0.0)
        return gain * duty_step * (1.0 - np.exp(-s / max(tau, 1e-4)))

    final = float(np.mean(y[t >= t[-1] - 0.3]))
    guess = (final / duty_step if duty_step else 1.0, 0.1, 0.02)
    bounds = ([0.0, 1e-3, 0.0], [np.inf, 5.0, 0.5])
    (gain, tau, theta), _ = curve_fit(model, t, y, p0=guess, bounds=bounds, maxfev=20000)
    return float(gain), float(tau), float(theta)


def ziegler_nichols_open_loop(K: float, tau: float, theta: float, kind: str = "PI") -> tuple[float, float, float]:
    """The 1942 reaction-curve rules from a first-order-plus-delay fit. Returns (kp, ki, kd).

    Written with the reaction rate ``R = K/tau`` and the lag ``L = theta``:
    P   kp = 1/(R L);  PI  kp = 0.9/(R L), Ti = L/0.3;  PID kp = 1.2/(R L), Ti = 2L, Td = 0.5L.
    They aim for quarter-amplitude decay, i.e. a deliberately oscillatory loop: expect overshoot.
    """
    theta = max(theta, 1e-4)
    base = tau / (K * theta)  # = 1 / (R * L)
    if kind == "P":
        return base, 0.0, 0.0
    if kind == "PI":
        kp = 0.9 * base
        return kp, kp / (theta / 0.3), 0.0
    if kind == "PID":
        kp = 1.2 * base
        return kp, kp / (2.0 * theta), kp * 0.5 * theta
    raise ValueError(f"kind must be P, PI or PID, not {kind!r}")


def ziegler_nichols_ultimate(ku: float, tu: float, kind: str = "PI") -> tuple[float, float, float]:
    """The closed-loop ("ultimate gain") rules. Returns (kp, ki, kd).

    P kp = 0.5 Ku; PI kp = 0.45 Ku, Ti = Tu/1.2; PID kp = 0.6 Ku, Ti = Tu/2, Td = Tu/8.
    "no_overshoot" is the classic conservative variant: kp = 0.2 Ku, Ti = Tu/2, Td = Tu/3.
    """
    if kind == "P":
        return 0.5 * ku, 0.0, 0.0
    if kind == "PI":
        kp = 0.45 * ku
        return kp, kp / (tu / 1.2), 0.0
    if kind == "PID":
        kp = 0.6 * ku
        return kp, kp / (tu / 2.0), kp * tu / 8.0
    if kind == "no_overshoot":
        kp = 0.2 * ku
        return kp, kp / (tu / 2.0), kp * tu / 3.0
    raise ValueError(f"unknown kind {kind!r}")


def relay_ultimate_gain(relay_amplitude: float, oscillation_amplitude: float, period_s: float) -> tuple[float, float]:
    """Åström & Hägglund's relay experiment: ``Ku = 4 d / (pi a)``, ``Tu = the observed period``.

    ``d`` is the relay's half-amplitude (duty), ``a`` the half-amplitude of the resulting
    oscillation in the measurement. The describing-function approximation keeps only the first
    harmonic of the square wave, which is why Ku comes out within ~10 % of the real one.
    """
    if oscillation_amplitude <= 0:
        return math.inf, period_s
    return 4.0 * relay_amplitude / (math.pi * oscillation_amplitude), period_s


# --- motion profiles and heading (08.10, 08.11) ---------------------------------------------------
@dataclass(frozen=True)
class TrapezoidalProfile:
    """A time-optimal position profile with bounded speed and acceleration.

    Accelerate at ``a_max`` up to ``v_max``, cruise, decelerate at ``a_max`` to a stop exactly at
    ``distance``. If the move is too short to reach ``v_max`` the profile is triangular and the
    peak speed is ``sqrt(a_max * distance)``. Units are whatever you pass in: rad and rad/s for a
    turn (08.11), m and m/s for a straight move.
    """

    distance: float
    v_max: float
    a_max: float

    @property
    def sign(self) -> float:
        return 1.0 if self.distance >= 0 else -1.0

    @property
    def peak_v(self) -> float:
        """The speed actually reached (v_max, or less when the move is triangular)."""
        return min(self.v_max, math.sqrt(self.a_max * abs(self.distance))) if self.distance else 0.0

    @property
    def t_accel(self) -> float:
        return self.peak_v / self.a_max

    @property
    def t_cruise(self) -> float:
        cruise_distance = abs(self.distance) - self.peak_v * self.t_accel
        return max(cruise_distance, 0.0) / self.peak_v if self.peak_v else 0.0

    @property
    def duration(self) -> float:
        return 2.0 * self.t_accel + self.t_cruise

    def velocity(self, t: float) -> float:
        """Commanded speed at time t (0 before the start and after the end)."""
        ta, tc = self.t_accel, self.t_cruise
        if t <= 0.0 or t >= self.duration:
            return 0.0
        if t < ta:
            return self.sign * self.a_max * t
        if t < ta + tc:
            return self.sign * self.peak_v
        return self.sign * self.a_max * (self.duration - t)

    def position(self, t: float) -> float:
        """Commanded position at time t (0 before the start, ``distance`` after the end)."""
        ta, tc, v = self.t_accel, self.t_cruise, self.peak_v
        if t <= 0.0:
            return 0.0
        if t >= self.duration:
            return self.distance
        if t < ta:
            return self.sign * 0.5 * self.a_max * t * t
        if t < ta + tc:
            return self.sign * (0.5 * v * ta + v * (t - ta))
        remaining = self.duration - t
        return self.sign * (abs(self.distance) - 0.5 * self.a_max * remaining * remaining)


def wheel_speeds_for(v_rad_s: float, yaw_rate_rad_s: float, wheel_radius_m: float,
                     wheel_separation_m: float) -> tuple[float, float]:
    """Wheel angular speeds (rad/s) for a body speed and a yaw rate — the one bit of
    differential-drive kinematics 08.10/08.11 need (the full treatment is 09.01/09.02).

        v = r (wL + wR) / 2          yaw_rate = r (wR - wL) / b
    """
    differential = yaw_rate_rad_s * wheel_separation_m / (2.0 * wheel_radius_m)
    centre = v_rad_s
    return centre - differential, centre + differential
