"""14.07 — reference solution. Read it after you have tried ``student.py`` yourself."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Profile:
    q0: float
    q1: float
    v_peak: float
    accel: float
    t_accel: float
    t_cruise: float

    @property
    def duration(self) -> float:
        return 2.0 * self.t_accel + self.t_cruise

    @property
    def is_triangular(self) -> bool:
        return self.t_cruise <= 1e-12

    @property
    def distance(self) -> float:
        return abs(self.q1 - self.q0)

    @property
    def direction(self) -> float:
        return 1.0 if self.q1 >= self.q0 else -1.0


# --- solution ------------------------------------------------------------------------------------
def trapezoid_profile(q0: float, q1: float, v_max: float, a_max: float) -> Profile:
    if v_max <= 0 or a_max <= 0:
        raise ValueError("v_max and a_max must be positive")
    D = abs(q1 - q0)
    if D == 0.0:
        return Profile(q0, q1, 0.0, a_max, 0.0, 0.0)
    if D >= v_max * v_max / a_max:
        t_a = v_max / a_max
        t_c = (D - v_max * v_max / a_max) / v_max
        return Profile(q0, q1, v_max, a_max, t_a, t_c)
    v_peak = math.sqrt(D * a_max)
    return Profile(q0, q1, v_peak, a_max, v_peak / a_max, 0.0)


def profile_with_duration(q0: float, q1: float, duration: float, a_max: float) -> Profile:
    if a_max <= 0:
        raise ValueError("a_max must be positive")
    if duration < 0:
        raise ValueError("duration must be >= 0")
    D = abs(q1 - q0)
    if D == 0.0:
        return Profile(q0, q1, 0.0, a_max, 0.0, duration)
    disc = a_max * a_max * duration * duration - 4.0 * a_max * D
    if disc < -1e-12:
        raise ValueError(f"cannot move {D:.4f} in {duration:.3f} s with a_max = {a_max}")
    v = (a_max * duration - math.sqrt(max(0.0, disc))) / 2.0      # the smaller root
    t_a = v / a_max
    return Profile(q0, q1, v, a_max, t_a, max(0.0, duration - 2.0 * t_a))


def sample_profile(profile: Profile, t: ArrayLike) -> tuple[Array, Array, Array]:
    t = np.clip(np.asarray(t, dtype=float), 0.0, profile.duration)
    a, v, ta, tc = profile.accel, profile.v_peak, profile.t_accel, profile.t_cruise
    T, D, sign = profile.duration, profile.distance, profile.direction
    pos = np.empty_like(t)
    vel = np.empty_like(t)
    acc = np.empty_like(t)
    ramp_up = t <= ta
    cruise = (t > ta) & (t <= ta + tc)
    ramp_down = t > ta + tc
    pos[ramp_up] = 0.5 * a * t[ramp_up] ** 2
    vel[ramp_up] = a * t[ramp_up]
    acc[ramp_up] = a
    d_a = 0.5 * a * ta * ta
    pos[cruise] = d_a + v * (t[cruise] - ta)
    vel[cruise] = v
    acc[cruise] = 0.0
    remaining = T - t[ramp_down]
    pos[ramp_down] = D - 0.5 * a * remaining ** 2
    vel[ramp_down] = a * remaining
    acc[ramp_down] = -a
    if D == 0.0:
        pos[:] = 0.0
        vel[:] = 0.0
        acc[:] = 0.0
    return profile.q0 + sign * pos, sign * vel, sign * acc


def cubic_coefficients(q0: float, q1: float, T: float, v0: float = 0.0, v1: float = 0.0) -> Array:
    if T <= 0:
        raise ValueError("duration must be positive")
    d = q1 - q0
    return np.array([
        q0,
        v0,
        3.0 * d / T ** 2 - (2.0 * v0 + v1) / T,
        -2.0 * d / T ** 3 + (v0 + v1) / T ** 2,
    ])


def polynomial_sample(coefficients: ArrayLike, t: ArrayLike) -> tuple[Array, Array, Array]:
    c = np.asarray(coefficients, dtype=float)
    t = np.asarray(t, dtype=float)
    dc = np.polynomial.polynomial.polyder(c)
    ddc = np.polynomial.polynomial.polyder(dc)
    ones = np.ones_like(t)
    pos = np.polynomial.polynomial.polyval(t, c) * ones
    vel = np.polynomial.polynomial.polyval(t, dc) * ones
    acc = np.polynomial.polynomial.polyval(t, ddc) * ones
    return pos, vel, acc


def min_duration_cubic(distance: float, v_max: float, a_max: float) -> float:
    D = abs(distance)
    return max(1.5 * D / v_max, math.sqrt(6.0 * D / a_max))


def min_duration_quintic(distance: float, v_max: float, a_max: float) -> float:
    D = abs(distance)
    return max(15.0 * D / (8.0 * v_max), math.sqrt(10.0 / math.sqrt(3.0) * D / a_max))


def synchronize(q0: Sequence[float], q1: Sequence[float], v_max: Sequence[float],
                a_max: Sequence[float]) -> list[Profile]:
    fastest = [trapezoid_profile(a, b, v, acc) for a, b, v, acc in zip(q0, q1, v_max, a_max)]
    T = max(p.duration for p in fastest)
    return [profile_with_duration(a, b, T, acc) for a, b, acc in zip(q0, q1, a_max)]


def violations(t: ArrayLike, qd: ArrayLike, qdd: ArrayLike, v_max: ArrayLike, a_max: ArrayLike,
               tolerance: float = 1e-6) -> list[str]:
    qd = np.atleast_2d(np.asarray(qd, dtype=float))
    qdd = np.atleast_2d(np.asarray(qdd, dtype=float))
    n = qd.shape[1]
    v_max = np.broadcast_to(np.asarray(v_max, dtype=float), (n,))
    a_max = np.broadcast_to(np.asarray(a_max, dtype=float), (n,))
    out: list[str] = []
    peak_v = np.abs(qd).max(axis=0)
    peak_a = np.abs(qdd).max(axis=0)
    for j in range(n):
        if peak_v[j] > v_max[j] + tolerance:
            out.append(f"joint {j}: peak |qd| {peak_v[j]:.4f} > v_max {v_max[j]:.4f}")
    for j in range(n):
        if peak_a[j] > a_max[j] + tolerance:
            out.append(f"joint {j}: peak |qdd| {peak_a[j]:.4f} > a_max {a_max[j]:.4f}")
    return out
