"""Reference solution for 09.03 — Dead reckoning: Euler, midpoint and exact-arc integration.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable

Pose = tuple[float, float, float]  # (x [m], y [m], theta [rad] in (-pi, pi])
Step = Callable[[Pose, float, float, float], Pose]


def wrap(theta: float) -> float:
    return math.atan2(math.sin(theta), math.cos(theta))


def euler_step(pose: Pose, v: float, omega: float, dt: float) -> Pose:
    """Move along the heading held at the START of the step, then turn."""
    x, y, theta = pose
    return (x + v * dt * math.cos(theta), y + v * dt * math.sin(theta), wrap(theta + omega * dt))


def midpoint_step(pose: Pose, v: float, omega: float, dt: float) -> Pose:
    """Move along the heading at the MIDDLE of the step (second-order Runge-Kutta)."""
    x, y, theta = pose
    mid = theta + omega * dt / 2.0
    return (x + v * dt * math.cos(mid), y + v * dt * math.sin(mid), wrap(theta + omega * dt))


def exact_step(pose: Pose, v: float, omega: float, dt: float) -> Pose:
    """Move along the exact circular arc driven at constant (v, omega)."""
    x, y, theta = pose
    dtheta = omega * dt
    if abs(dtheta) < 1e-9:
        return (x + v * dt * math.cos(theta), y + v * dt * math.sin(theta), wrap(theta + dtheta))
    radius = v / omega
    return (
        x + radius * (math.sin(theta + dtheta) - math.sin(theta)),
        y - radius * (math.cos(theta + dtheta) - math.cos(theta)),
        wrap(theta + dtheta),
    )


def integrate(step: Step, pose: Pose, twists: Iterable[tuple[float, float]], dt: float) -> Pose:
    """Apply ``step`` once per (v, omega) sample, each held for ``dt`` seconds."""
    for v, omega in twists:
        pose = step(pose, v, omega, dt)
    return pose


def constant_twist_error(step: Step, v: float, omega: float, duration: float, dt: float) -> float:
    """Position error (m) after ``duration`` s at constant (v, omega) from the origin, vs the exact arc."""
    n = round(duration / dt)
    estimate = integrate(step, (0.0, 0.0, 0.0), [(v, omega)] * n, dt)
    truth = exact_step((0.0, 0.0, 0.0), v, omega, n * dt)
    return math.hypot(estimate[0] - truth[0], estimate[1] - truth[1])
