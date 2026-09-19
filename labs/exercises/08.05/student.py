"""08.05 — Integral control: kill the steady-state error, then stop integral windup.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 08.05``.
Only the standard library is needed.

The formulation is the one the Pico firmware uses (labs/firmware/pico/velocity.py), so what you
build here is exactly what runs on the robot in 08.12:

* the integral is stored ALREADY MULTIPLIED by ki:  integral += ki * error * dt
  (it is in duty units, and changing ki while running doesn't make the output jump)
* output = kp * error + integral, clamped to [output_min, output_max]
"""

from __future__ import annotations


class PIController:
    """A PI controller for one wheel, with optional anti-windup.

    With ``anti_windup=True`` apply BOTH protections, in this order, every update:

    1. Conditional integration. Compute ``candidate = integral + ki * error * dt`` and the
       unclamped output ``kp * error + candidate``. If that output is above ``output_max`` while the
       error is positive (or below ``output_min`` while the error is negative), the actuator is
       already saturated in the direction the error pushes: keep the OLD integral. Otherwise
       accept the candidate. (An error of the opposite sign may always unwind the integral.)
    2. Clamping. Clamp the integral itself to [output_min, output_max].

    With ``anti_windup=False`` the integral simply accumulates (that's the bug to demonstrate).
    """

    def __init__(self, kp: float, ki: float, output_min: float = -1.0, output_max: float = 1.0,
                 anti_windup: bool = True) -> None:
        self.kp = kp
        self.ki = ki
        self.output_min = output_min
        self.output_max = output_max
        self.anti_windup = anti_windup
        self.integral = 0.0  # ki * accumulated error*dt, in duty units
        self.output = 0.0  # the last output, for logging

    def reset(self) -> None:
        """Forget the integral (call before every new run: an old integral is a hidden command)."""
        # TODO(student)
        raise NotImplementedError("PIController.reset")

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        """One control step; returns the duty. Store it in ``self.output`` too."""
        # TODO(student):
        #   error, proportional term, candidate integral
        #   if anti_windup: conditional integration, then clamp the integral
        #   else: accept the candidate
        #   output = proportional + integral, clamped
        raise NotImplementedError("PIController.update")
