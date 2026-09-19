"""Hardware abstraction layer: the one interface every robot program talks to.

The simulator (:class:`robotlab.sim.SimBase`), the real robot over serial and test fakes all
implement :class:`DifferentialBase`, so odometry, PID or teleop code runs unchanged everywhere.
See ``labs/README.md`` for the contract and the Pi <-> Pico protocol.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, runtime_checkable

# Telemetry flag bits — identical to the protocol's ``T`` message ``flags`` field.
FLAG_WATCHDOG = 1
"""No valid M/V command for ``watchdog_ms``: the motors were stopped."""
FLAG_LOW_BATTERY = 2
"""Battery voltage is below ``battery.low_warning_v``."""
FLAG_RANGE_ERROR = 4
"""The front range sensor reported an error."""
FLAG_VELOCITY_MODE = 8
"""The closed-loop wheel velocity controller is active (last command was V)."""


@dataclass(frozen=True)
class BaseState:
    """One telemetry sample from a differential-drive base."""

    t: float  # seconds (monotonic, robot clock)
    left_ticks: int  # cumulative signed encoder ticks
    right_ticks: int
    left_rad_s: float  # wheel angular velocity estimate
    right_rad_s: float
    battery_v: float | None
    range_m: float | None  # front distance sensor, None if no reading
    flags: int  # bitfield, see FLAG_* constants


@runtime_checkable
class DifferentialBase(Protocol):
    """A two-wheeled base: command the wheels, read telemetry."""

    def set_wheel_duty(self, left: float, right: float) -> None:
        """Open-loop motor duty cycle, each in -1.0..1.0."""
        ...

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        """Wheel angular velocity setpoints, closed loop on the microcontroller."""
        ...

    def stop(self) -> None:
        """Brake both motors."""
        ...

    def read(self) -> BaseState:
        """Return the latest telemetry sample."""
        ...

    def close(self) -> None:
        """Stop the motors and release the connection."""
        ...
