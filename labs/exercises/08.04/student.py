"""08.04 — Proportional control: hold a wheel speed.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 08.04``.
Only the standard library is needed.

Units: speeds in rad/s, duty (the controller output) in -1..1, time in seconds.
"""

from __future__ import annotations

import math  # noqa: F401  (math.copysign is handy)


def steady_state_speed(setpoint: float, kp: float, slope: float, deadband: float) -> float:
    """Predict the speed a P-only loop settles at (it never quite reaches the setpoint).

    The motor model you measured in 08.02: a steady duty ``u`` gives the speed
    ``w = slope * (u - deadband)`` when ``u > deadband``, and 0 inside the deadband.
    The controller sends ``u = kp * (setpoint - w)``.

    1. Substitute one equation into the other and solve for ``w``.
    2. If ``kp * |setpoint| <= deadband`` the wheel never starts: return 0.0.
    3. The duty can't exceed 1.0, so ``|w|`` can't exceed ``slope * (1 - deadband)``.
    4. A negative setpoint gives the mirror image (negative speed).
    """
    # TODO(student): implement the formula from the lesson (Level 3) with the three edge cases.
    raise NotImplementedError("steady_state_speed")


class PController:
    """A proportional controller for one wheel.

    c = PController(kp=0.1)
    c.update(setpoint=8.0, measured=5.0, dt=0.02)    # -> 0.3  (0.1 * 3.0 rad/s of error)
    c.update(setpoint=8.0, measured=-20.0, dt=0.02)  # -> 1.0  (2.8 clamped to the output limit)
    """

    def __init__(self, kp: float, output_limit: float = 1.0) -> None:
        self.kp = kp
        self.output_limit = output_limit
        self.error = 0.0  # the last error, for logging
        self.output = 0.0  # the last output, for logging

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        """One control step: return the duty to send. ``dt`` is unused by P (I and D will need it).

        error = setpoint - measured; output = kp * error, clamped to [-output_limit, +output_limit].
        Store both in ``self.error`` and ``self.output``.
        """
        # TODO(student): three lines.
        raise NotImplementedError("PController.update")

    def reset(self) -> None:
        """Forget the last error and output (call before a new run)."""
        # TODO(student)
        raise NotImplementedError("PController.reset")
