"""Small building blocks of the simulated robot: the battery and the firmware's velocity PID.

They are kept out of ``robot.py`` so :meth:`DiffDriveSim.step` stays short enough to read in
one sitting.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


@dataclass
class Battery:
    """A deliberately simple battery: open-circuit voltage linear in state of charge,
    minus an internal-resistance sag proportional to the current drawn.

    ``v = v_empty + soc * (v_full - v_empty) - current * internal_resistance``
    """

    full_v: float
    empty_v: float
    capacity_ah: float
    internal_resistance_ohm: float
    idle_current_a: float
    motor_current_a: float  # per motor, at |duty| = 1
    soc: float = 1.0  # state of charge 0..1
    voltage: float = field(init=False)

    def __post_init__(self) -> None:
        self.voltage = self.open_circuit_v - self.idle_current_a * self.internal_resistance_ohm

    @property
    def open_circuit_v(self) -> float:
        return self.empty_v + self.soc * (self.full_v - self.empty_v)

    def update(self, duty: NDArray[np.floating], dt: float) -> float:
        """Drain the pack for ``dt`` seconds at the given motor duties; return the new voltage."""
        current = self.idle_current_a + self.motor_current_a * float(np.abs(duty).sum())
        self.soc = max(0.0, self.soc - current * dt / (3600.0 * self.capacity_ah))
        self.voltage = self.open_circuit_v - current * self.internal_resistance_ohm
        return self.voltage


@dataclass
class WheelVelocityController:
    """The Pico firmware's velocity PID (``labs/firmware/pico/velocity.py``), for both wheels at
    once (arrays of 2). Same formulas, so gains tuned in simulation carry over:

    * ``duty = feedforward + kp*e + integral + derivative``, clamped to +-1
    * feedforward inverts the motor model: ``ff * sign(sp) * (deadband + |sp|/max * (1 - deadband))``
    * the integral is stored already multiplied by ``ki``; while the error pushes the output past
      a limit it only grows until the output sits exactly at that limit (anti-windup that fills
      the headroom, never beyond) and it is itself clamped to +-1
    * the derivative acts on the measurement, not the error (no derivative kick)
    """

    kp: float
    ki: float
    kd: float
    ff: float  # 1.0 = full model inversion, 0.0 = pure PID
    max_wheel_speed_rad_s: float
    deadband: float
    _integral: NDArray[np.floating] = field(default_factory=lambda: np.zeros(2), repr=False)
    _last_measured: NDArray[np.floating] | None = field(default=None, repr=False)

    def reset(self) -> None:
        self._integral = np.zeros(2)
        self._last_measured = None

    def update(self, setpoint: NDArray[np.floating], measured: NDArray[np.floating], dt: float) -> NDArray[np.floating]:
        error = setpoint - measured
        fraction = np.minimum(np.abs(setpoint) / self.max_wheel_speed_rad_s, 1.0)
        feedforward = self.ff * np.sign(setpoint) * (self.deadband + fraction * (1.0 - self.deadband))
        derivative = np.zeros(2)
        if self._last_measured is not None and dt > 0:
            derivative = -self.kd * (measured - self._last_measured) / dt
        self._last_measured = np.array(measured, dtype=float)
        candidate = self._integral + self.ki * error * dt
        others = feedforward + self.kp * error + derivative
        unclamped = others + candidate
        up = (unclamped > 1.0) & (error > 0)  # would push past +1: fill the headroom only
        down = (unclamped < -1.0) & (error < 0)
        integral = np.where(up, np.maximum(self._integral, 1.0 - others), candidate)
        integral = np.where(down, np.minimum(self._integral, -1.0 - others), integral)
        self._integral = np.clip(integral, -1.0, 1.0)
        return np.clip(feedforward + self.kp * error + self._integral + derivative, -1.0, 1.0)


def low_pass(previous: NDArray[np.floating], raw: NDArray[np.floating], alpha: float) -> NDArray[np.floating]:
    """First-order low-pass filter step: ``previous + alpha * (raw - previous)`` (alpha 1 = off)."""
    return previous + alpha * (raw - previous)


def deadband_speed(duty: NDArray[np.floating], deadband: float, max_speed: float) -> NDArray[np.floating]:
    """Steady-state wheel speed for a duty: zero inside the deadband, linear up to ``max_speed``."""
    duty = np.clip(duty, -1.0, 1.0)
    effective = np.maximum(np.abs(duty) - deadband, 0.0) / (1.0 - deadband)
    return np.sign(duty) * effective * max_speed


def first_order_alpha(dt: float, time_constant: float) -> float:
    """Exact discretization of a first-order lag: ``y += (u - y) * alpha``."""
    return 1.0 - math.exp(-dt / time_constant) if time_constant > 0 else 1.0
