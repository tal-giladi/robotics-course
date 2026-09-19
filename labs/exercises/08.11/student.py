"""08.11 — A trapezoidal motion profile and a wrap-safe heading controller, for an exact 90° turn.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 08.11``.
Only the standard library is needed.

Everything here is in radians and seconds, and every angle is wrapped to (-pi, pi].

The profile answers "where should the robot be *right now*?" instead of "where should it end up?".
Accelerate at ``a_max`` until the speed reaches ``v_max``, cruise, then decelerate at ``a_max`` so
that the robot arrives at ``distance`` with zero speed. If the turn is too small to ever reach
``v_max`` the profile is triangular and the peak speed is ``sqrt(a_max * |distance|)``.

    speed
      v_max |      _______________
            |     /               \\
            |    /                 \\
          0 |___/___________________\\____  time
             t_accel   t_cruise   t_accel

The controller then tracks that profile: ``yaw_rate = profile.velocity(t) + kp * error + integral``,
where ``error`` is the *wrapped* difference between the profile's position and the measured heading.
The profile velocity is a feedforward term (08.09): it does the work, the feedback only corrects.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def heading_error(target_rad: float, measured_rad: float) -> float:
    """The shortest signed rotation from ``measured_rad`` to ``target_rad``, in (-pi, pi].

    A robot at +179 deg told to go to -179 deg must turn +2 deg, not -358 deg. This one function
    is the difference between a 90 deg turn and a robot that spins in circles.
    (``robotlab.geometry.angle_diff`` does the same thing; write it yourself once.)
    """
    # TODO(student)
    raise NotImplementedError("heading_error")


@dataclass(frozen=True)
class TrapezoidalProfile:
    """Time-optimal position profile with bounded speed and acceleration.

    ``distance`` may be negative (turn the other way); ``v_max`` and ``a_max`` are positive.
    """

    distance: float
    v_max: float
    a_max: float

    @property
    def peak_v(self) -> float:
        """The speed actually reached: ``v_max``, or less when the move is too short (triangular)."""
        # TODO(student)
        raise NotImplementedError("TrapezoidalProfile.peak_v")

    @property
    def t_accel(self) -> float:
        """Seconds spent accelerating (and, symmetrically, decelerating)."""
        # TODO(student)
        raise NotImplementedError("TrapezoidalProfile.t_accel")

    @property
    def t_cruise(self) -> float:
        """Seconds at ``peak_v`` (0 for a triangular profile)."""
        # TODO(student)
        raise NotImplementedError("TrapezoidalProfile.t_cruise")

    @property
    def duration(self) -> float:
        """Total time of the move."""
        # TODO(student)
        raise NotImplementedError("TrapezoidalProfile.duration")

    def velocity(self, t: float) -> float:
        """Commanded speed at time ``t``: 0 before the start and after the end."""
        # TODO(student)
        raise NotImplementedError("TrapezoidalProfile.velocity")

    def position(self, t: float) -> float:
        """Commanded position at time ``t``: 0 before the start, ``distance`` after the end.

        This must be the exact integral of :meth:`velocity`, or the feedforward and the feedback
        will fight each other.
        """
        # TODO(student)
        raise NotImplementedError("TrapezoidalProfile.position")


@dataclass
class TurnController:
    """Track a heading profile: feedforward the profile's yaw rate, correct the rest with PI.

        yaw_rate = profile.velocity(t) + kp * error + integral,   clamped to +-max_yaw_rate_rad_s
        error    = heading_error(start + profile.position(t), measured)

    ``start_heading_rad`` is the heading when the turn began, so the profile's position is relative
    and the whole thing keeps working across the +-pi wrap.
    """

    profile: TrapezoidalProfile
    start_heading_rad: float
    kp: float = 4.0
    ki: float = 1.0
    max_yaw_rate_rad_s: float = 2.5
    integral: float = 0.0
    error: float = 0.0
    feedforward: float = 0.0
    output: float = 0.0

    def target(self, t: float) -> float:
        """The heading the robot should have at time ``t``, wrapped to (-pi, pi]."""
        # TODO(student)
        raise NotImplementedError("TurnController.target")

    def update(self, t: float, measured_rad: float, dt: float) -> float:
        """One control step; returns the yaw rate to command (rad/s).

        Anti-windup as in 08.05: while the error pushes the output past a limit, the integral may
        only grow until the output sits exactly at that limit.
        """
        # TODO(student)
        raise NotImplementedError("TurnController.update")

    def finished(self, t: float, measured_rad: float, tolerance_rad: float = math.radians(1.0)) -> bool:
        """True once the profile has run out AND the heading is inside ``tolerance_rad``."""
        # TODO(student)
        raise NotImplementedError("TurnController.finished")
