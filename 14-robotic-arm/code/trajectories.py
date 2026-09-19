"""trajectories — time-parameterized joint and Cartesian motions for an arm (numpy only).

Profiles for one scalar coordinate q (a joint angle, or the path parameter s of a Cartesian line):

* ``Trapezoid``: accelerate at a_max, cruise at v_max, decelerate. Becomes a triangle when the
  move is too short to reach v_max. Fastest profile under velocity AND acceleration limits,
  but the acceleration jumps (infinite jerk).
* ``cubic_coefficients``: q(t) = a0 + a1 t + a2 t^2 + a3 t^3 with given start/end position and
  velocity. Smooth velocity, acceleration jumps at start and end.
* ``quintic_coefficients``: adds zero start/end acceleration, so the acceleration is continuous.

``min_duration_*`` time-scale a rest-to-rest move so it respects |qd| <= v_max and |qdd| <= a_max;
``synchronized_trapezoids`` makes several joints start and finish together.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# ============================================================================================
# Trapezoidal velocity profile
# ============================================================================================
@dataclass(frozen=True)
class Trapezoid:
    """Rest-to-rest trapezoidal profile from q0 to q1.

    Build it with ``Trapezoid.fastest(q0, q1, v_max, a_max)`` or
    ``Trapezoid.with_duration(q0, q1, duration, a_max)``.
    """

    q0: float
    q1: float
    v_peak: float       # cruise speed actually used (>= 0)
    accel: float        # acceleration actually used (>= 0)
    t_accel: float      # duration of the acceleration (and of the deceleration) phase
    t_cruise: float     # duration of the constant-velocity phase (0 for a triangle)

    @property
    def duration(self) -> float:
        return 2.0 * self.t_accel + self.t_cruise

    @property
    def is_triangular(self) -> bool:
        return self.t_cruise <= 1e-12

    @classmethod
    def fastest(cls, q0: float, q1: float, v_max: float, a_max: float) -> Trapezoid:
        """Minimum-time profile under |v| <= v_max, |a| <= a_max.

        Distance D = |q1 - q0|. Reaching v_max takes t_a = v_max / a_max and covers
        v_max^2 / a_max during accel + decel. If D is shorter than that, the profile is a triangle
        with peak speed sqrt(D * a_max).
        """
        if v_max <= 0 or a_max <= 0:
            raise ValueError("v_max and a_max must be positive")
        D = abs(q1 - q0)
        if D == 0.0:
            return cls(q0, q1, 0.0, a_max, 0.0, 0.0)
        if D >= v_max * v_max / a_max:
            t_a = v_max / a_max
            t_c = (D - v_max * v_max / a_max) / v_max
            return cls(q0, q1, v_max, a_max, t_a, t_c)
        v_peak = math.sqrt(D * a_max)
        return cls(q0, q1, v_peak, a_max, v_peak / a_max, 0.0)

    @classmethod
    def with_duration(cls, q0: float, q1: float, duration: float, a_max: float) -> Trapezoid:
        """Profile that takes exactly ``duration`` seconds using acceleration a_max.

        With t_a = v / a and D = v (T - v / a):  v = (a T - sqrt(a^2 T^2 - 4 a D)) / 2.
        Needs a T^2 >= 4 D (otherwise even a triangle at a_max is too slow).
        """
        D = abs(q1 - q0)
        if D == 0.0:
            return cls(q0, q1, 0.0, a_max, 0.0, duration)
        disc = a_max * a_max * duration * duration - 4.0 * a_max * D
        if disc < -1e-12:
            raise ValueError(f"cannot move {D:.4f} in {duration:.3f} s with a_max = {a_max}")
        v = (a_max * duration - math.sqrt(max(0.0, disc))) / 2.0
        t_a = v / a_max
        return cls(q0, q1, v, a_max, t_a, max(0.0, duration - 2.0 * t_a))

    def sample(self, t: ArrayLike) -> tuple[Array, Array, Array]:
        """Position, velocity, acceleration at times t (clamped to [0, duration])."""
        t = np.clip(np.asarray(t, dtype=float), 0.0, self.duration)
        sign = 1.0 if self.q1 >= self.q0 else -1.0
        a, v, ta, tc = self.accel, self.v_peak, self.t_accel, self.t_cruise
        T = self.duration
        pos = np.empty_like(t)
        vel = np.empty_like(t)
        acc = np.empty_like(t)
        p1 = t <= ta
        p2 = (t > ta) & (t <= ta + tc)
        p3 = t > ta + tc
        pos[p1] = 0.5 * a * t[p1] ** 2
        vel[p1] = a * t[p1]
        acc[p1] = a
        d_a = 0.5 * a * ta * ta
        pos[p2] = d_a + v * (t[p2] - ta)
        vel[p2] = v
        acc[p2] = 0.0
        tr = T - t[p3]
        D = abs(self.q1 - self.q0)
        pos[p3] = D - 0.5 * a * tr ** 2
        vel[p3] = a * tr
        acc[p3] = -a
        if D == 0.0:
            acc[:] = 0.0
        return self.q0 + sign * pos, sign * vel, sign * acc


def synchronized_trapezoids(q0: Sequence[float], q1: Sequence[float], v_max: Sequence[float],
                            a_max: Sequence[float]) -> list[Trapezoid]:
    """One trapezoid per joint, all with the duration of the slowest joint.

    Every joint then starts and stops together, so a straight-ish joint-space move doesn't turn
    into a "one joint finishes early, the others keep going" motion.
    """
    fastest = [Trapezoid.fastest(a, b, v, acc) for a, b, v, acc in zip(q0, q1, v_max, a_max)]
    T = max(p.duration for p in fastest)
    return [Trapezoid.with_duration(a, b, T, acc) for a, b, acc in zip(q0, q1, a_max)]


# ============================================================================================
# Polynomial profiles
# ============================================================================================
def cubic_coefficients(q0: float, q1: float, T: float, v0: float = 0.0, v1: float = 0.0) -> Array:
    """[a0, a1, a2, a3] with q(0)=q0, q(T)=q1, qd(0)=v0, qd(T)=v1."""
    if T <= 0:
        raise ValueError("duration must be positive")
    d = q1 - q0
    return np.array([
        q0,
        v0,
        3.0 * d / T ** 2 - (2.0 * v0 + v1) / T,
        -2.0 * d / T ** 3 + (v0 + v1) / T ** 2,
    ])


def quintic_coefficients(q0: float, q1: float, T: float, v0: float = 0.0, v1: float = 0.0,
                         a0: float = 0.0, a1: float = 0.0) -> Array:
    """[c0..c5] matching position, velocity and acceleration at t = 0 and t = T."""
    if T <= 0:
        raise ValueError("duration must be positive")
    M = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T ** 2, T ** 3, T ** 4, T ** 5],
        [0, 1, 2 * T, 3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
        [0, 0, 2, 6 * T, 12 * T ** 2, 20 * T ** 3],
    ], dtype=float)
    return np.linalg.solve(M, np.array([q0, v0, a0, q1, v1, a1], dtype=float))


def polynomial_sample(coefficients: ArrayLike, t: ArrayLike) -> tuple[Array, Array, Array]:
    """Position, velocity, acceleration of sum_k c_k t^k at times t."""
    c = np.asarray(coefficients, dtype=float)
    t = np.asarray(t, dtype=float)
    pos = np.polynomial.polynomial.polyval(t, c)
    dc = np.polynomial.polynomial.polyder(c)
    vel = np.polynomial.polynomial.polyval(t, dc)
    acc = np.polynomial.polynomial.polyval(t, np.polynomial.polynomial.polyder(dc))
    return pos, vel * np.ones_like(t), acc * np.ones_like(t)


def min_duration_cubic(distance: float, v_max: float, a_max: float) -> float:
    """Shortest rest-to-rest cubic: peak |qd| = 1.5 D / T, peak |qdd| = 6 D / T^2."""
    D = abs(distance)
    return max(1.5 * D / v_max, math.sqrt(6.0 * D / a_max))


def min_duration_quintic(distance: float, v_max: float, a_max: float) -> float:
    """Shortest rest-to-rest quintic: peak |qd| = 15 D / (8 T), peak |qdd| = (10 / sqrt 3) D / T^2."""
    D = abs(distance)
    return max(15.0 * D / (8.0 * v_max), math.sqrt(10.0 / math.sqrt(3.0) * D / a_max))


# ============================================================================================
# Multi-joint and Cartesian helpers
# ============================================================================================
def joint_space_trajectory(q_start: ArrayLike, q_goal: ArrayLike, v_max: ArrayLike, a_max: ArrayLike,
                           dt: float = 0.02, kind: str = "trapezoid") -> tuple[Array, Array, Array, Array]:
    """Synchronized rest-to-rest move of all joints. Returns t, q, qd, qdd (shape (N,), (N, n)...).

    kind: "trapezoid", "cubic" or "quintic". Every joint shares the duration of the slowest one.
    """
    q_start = np.asarray(q_start, dtype=float)
    q_goal = np.asarray(q_goal, dtype=float)
    v_max = np.broadcast_to(np.asarray(v_max, dtype=float), q_start.shape)
    a_max = np.broadcast_to(np.asarray(a_max, dtype=float), q_start.shape)
    if kind == "trapezoid":
        profiles = synchronized_trapezoids(q_start, q_goal, v_max, a_max)
        T = profiles[0].duration
        t = np.arange(0.0, T + dt / 2, dt)
        t[-1] = T
        parts = [p.sample(t) for p in profiles]
    elif kind in ("cubic", "quintic"):
        fn = min_duration_cubic if kind == "cubic" else min_duration_quintic
        coef = cubic_coefficients if kind == "cubic" else quintic_coefficients
        T = max(fn(g - s, v, a) for s, g, v, a in zip(q_start, q_goal, v_max, a_max)) or dt
        t = np.arange(0.0, T + dt / 2, dt)
        t[-1] = T
        parts = [polynomial_sample(coef(s, g, T), t) for s, g in zip(q_start, q_goal)]
    else:
        raise ValueError(f"unknown kind {kind!r}")
    q = np.stack([p[0] for p in parts], axis=1)
    qd = np.stack([p[1] for p in parts], axis=1)
    qdd = np.stack([p[2] for p in parts], axis=1)
    return t, q, qd, qdd


def straight_line_trajectory(
    p_start: ArrayLike, p_goal: ArrayLike, v_max: float, a_max: float, dt: float,
    ik: Callable[[Array, Array], Array], q_start: ArrayLike,
) -> tuple[Array, Array, Array]:
    """Cartesian straight line with a trapezoidal speed profile along the path.

    The path parameter s goes 0 -> |p_goal - p_start| with a Trapezoid; each sample's point is
    solved with ``ik(point, q_previous)``, warm-started from the previous solution so the arm
    stays on one IK branch. Returns t, points (N, 3), q (N, n).
    """
    p_start = np.asarray(p_start, dtype=float)
    p_goal = np.asarray(p_goal, dtype=float)
    L = float(np.linalg.norm(p_goal - p_start))
    prof = Trapezoid.fastest(0.0, L, v_max, a_max)
    t = np.arange(0.0, prof.duration + dt / 2, dt)
    t[-1] = prof.duration
    s, _, _ = prof.sample(t)
    direction = (p_goal - p_start) / L if L > 0 else np.zeros(3)
    points = p_start + s[:, None] * direction
    q_prev = np.asarray(q_start, dtype=float)
    qs = []
    for p in points:
        q_prev = ik(p, q_prev)
        qs.append(q_prev)
    return t, points, np.array(qs)


if __name__ == "__main__":
    prof = Trapezoid.fastest(0.0, math.radians(90), v_max=math.radians(60), a_max=math.radians(120))
    print(f"90 deg at 60 deg/s, 120 deg/s^2: t_accel {prof.t_accel:.3f} s, cruise {prof.t_cruise:.3f} s, "
          f"total {prof.duration:.3f} s, triangular={prof.is_triangular}")
    short = Trapezoid.fastest(0.0, math.radians(20), v_max=math.radians(60), a_max=math.radians(120))
    print(f"20 deg: triangle={short.is_triangular}, peak {math.degrees(short.v_peak):.1f} deg/s, "
          f"total {short.duration:.3f} s")
    D, vm, am = math.radians(90), math.radians(60), math.radians(120)
    print(f"cubic min duration {min_duration_cubic(D, vm, am):.3f} s, "
          f"quintic {min_duration_quintic(D, vm, am):.3f} s")
