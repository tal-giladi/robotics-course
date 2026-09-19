"""09.03 — Dead reckoning: Euler, midpoint and exact-arc integration.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 09.03``.
Only the Python standard library is needed (``math``).

Conventions: meters, radians, seconds; x forward, y left, counter-clockwise positive.
A pose is a plain tuple ``(x, y, theta)`` with theta wrapped to (-pi, pi].
A "step" function has the signature ``step(pose, v, omega, dt) -> pose``: the robot drives at
forward speed ``v`` (m/s) and yaw rate ``omega`` (rad/s), both constant for ``dt`` seconds.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable

Pose = tuple[float, float, float]  # (x [m], y [m], theta [rad] in (-pi, pi])
Step = Callable[[Pose, float, float, float], Pose]


def wrap(theta: float) -> float:
    """Wrap an angle to (-pi, pi]. Given — use it."""
    return math.atan2(math.sin(theta), math.cos(theta))


def euler_step(pose: Pose, v: float, omega: float, dt: float) -> Pose:
    """Forward Euler: move v*dt along the heading at the START of the step, then add omega*dt."""
    # TODO(student): x += v dt cos(theta), y += v dt sin(theta), theta += omega dt (wrapped).
    raise NotImplementedError("euler_step")


def midpoint_step(pose: Pose, v: float, omega: float, dt: float) -> Pose:
    """Midpoint (RK2): move v*dt along the heading at the MIDDLE of the step, theta + omega*dt/2."""
    # TODO(student): like euler_step, but with the mid-step heading for the x/y update.
    raise NotImplementedError("midpoint_step")


def exact_step(pose: Pose, v: float, omega: float, dt: float) -> Pose:
    """Exact arc for constant (v, omega).

    dtheta = omega * dt. If |dtheta| < 1e-9 drive straight (same as Euler). Otherwise, with
    R = v / omega:
        x += R * (sin(theta + dtheta) - sin(theta))
        y -= R * (cos(theta + dtheta) - cos(theta))
    """
    # TODO(student): straight-line case and arc case; wrap the heading.
    raise NotImplementedError("exact_step")


def integrate(step: Step, pose: Pose, twists: Iterable[tuple[float, float]], dt: float) -> Pose:
    """Starting at ``pose``, apply ``step`` once for each (v, omega) in ``twists``; return the final pose."""
    # TODO(student): a loop.
    raise NotImplementedError("integrate")


def constant_twist_error(step: Step, v: float, omega: float, duration: float, dt: float) -> float:
    """How wrong is ``step`` for this time step?

    Integrate round(duration / dt) steps of constant (v, omega) from (0, 0, 0) with ``step``.
    The truth is ONE exact_step over the whole duration (a constant twist traces one arc).
    Return the Euclidean distance between the two final positions (ignore theta).
    """
    # TODO(student): use integrate() and exact_step().
    raise NotImplementedError("constant_twist_error")
