"""08.09 — Feedforward from the motor model, plus a small PI that cleans up what the model got wrong.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 08.09``.
Only the standard library is needed.

The motor model you fitted in 08.02:

    wheel speed = top_speed(battery_v) * (|duty| - deadband) / (1 - deadband) * sign(duty)
    top_speed(battery_v) = max_wheel_speed_rad_s * battery_v / nominal_v

Feedforward is that model solved for duty instead of for speed:

    duty = sign(w) * (deadband + |w| / top_speed(battery_v) * (1 - deadband))

The PI part is the one you wrote in 08.05, in the firmware's formulation
(``labs/firmware/pico/velocity.py``): the integral is stored already multiplied by ki, anti-windup
is conditional integration that fills the remaining headroom, then a clamp. The feedforward is part
of the output, so it must also be part of the anti-windup check — otherwise the integral winds up
while the feedforward alone already saturates the motor.
"""

from __future__ import annotations

from dataclasses import dataclass


def wheel_speed_for(ground_speed_m_s: float, wheel_radius_m: float) -> float:
    """Wheel angular speed (rad/s) that makes the rim travel ``ground_speed_m_s`` over the floor.

    A wheel of radius r turning at w rad/s lays down r*w metres of floor per second (no slip).
    """
    # TODO(student)
    raise NotImplementedError("wheel_speed_for")


@dataclass(frozen=True)
class FeedforwardModel:
    """The inverted motor model: "what duty should give me this wheel speed?".

    ``use_battery=False`` ignores the measured voltage and always assumes ``nominal_v`` — the
    version most robots ship with, and the one that slows down as the pack empties.
    """

    max_wheel_speed_rad_s: float
    deadband: float
    nominal_v: float
    use_battery: bool = True

    def top_speed(self, battery_v: float | None = None) -> float:
        """Wheel speed at duty 1.0 for this battery voltage."""
        # TODO(student): nominal_v when use_battery is False, or battery_v is None/<= 0.
        raise NotImplementedError("FeedforwardModel.top_speed")

    def duty(self, setpoint_rad_s: float, battery_v: float | None = None) -> float:
        """The feedforward duty for this wheel-speed setpoint, clamped to [-1, 1].

        A setpoint of exactly 0 must give exactly 0 (never the deadband: that would creep).
        """
        # TODO(student)
        raise NotImplementedError("FeedforwardModel.duty")


@dataclass
class FeedforwardPI:
    """``duty = feedforward + kp*e + integral``, clamped, with the 08.05 anti-windup.

    The integral is stored in duty units (already multiplied by ki). While the error pushes the
    output past a limit, the integral may only grow until the output sits exactly AT that limit
    (fill the headroom, never beyond), and it is itself clamped to the output range.
    """

    model: FeedforwardModel
    kp: float
    ki: float
    output_min: float = -1.0
    output_max: float = 1.0
    integral: float = 0.0
    feedforward: float = 0.0  # the last feedforward duty (for logging)
    output: float = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.feedforward = 0.0
        self.output = 0.0

    def update(self, setpoint_rad_s: float, measured_rad_s: float, dt: float,
               battery_v: float | None = None) -> float:
        """One control step; returns the duty to send to the motor."""
        # TODO(student)
        raise NotImplementedError("FeedforwardPI.update")
