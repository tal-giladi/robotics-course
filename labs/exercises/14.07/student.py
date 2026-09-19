"""14.07 — Time-parameterised trajectories: trapezoids, polynomials and synchronisation.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 14.07``.
Only the standard library and numpy are needed.

Everything here works on ONE scalar coordinate at a time — a joint angle in radians, or the
distance travelled along a Cartesian path in metres — except ``synchronize``, which coordinates
several of them. Units: whatever you put in (rad, rad/s, rad/s^2), consistently.

A profile is "rest to rest": zero velocity at both ends.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Profile:
    """A trapezoidal (or triangular) velocity profile from ``q0`` to ``q1``.

    ``t_cruise == 0`` means the move is too short to reach ``v_peak`` before it must brake again:
    a triangle. ``accel`` is the magnitude used on both ramps.
    """

    q0: float
    q1: float
    v_peak: float       # the cruise speed actually reached, >= 0
    accel: float        # the acceleration magnitude actually used, > 0
    t_accel: float      # duration of the acceleration ramp (and of the braking ramp)
    t_cruise: float     # duration of the constant-velocity phase, 0 for a triangle

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


# --- TODO(student) -------------------------------------------------------------------------------
def trapezoid_profile(q0: float, q1: float, v_max: float, a_max: float) -> Profile:
    """The minimum-time rest-to-rest profile under |v| <= v_max and |a| <= a_max.

    Reaching v_max takes ``v_max / a_max`` seconds and covers ``v_max**2 / (2 * a_max)``, so the
    two ramps together need ``v_max**2 / a_max`` of the distance D. If D is at least that, the
    profile is a trapezoid; otherwise it is a triangle whose peak speed is ``sqrt(D * a_max)``.

    A zero-distance move is legal and has duration 0. ``v_max`` and ``a_max`` must be positive.
    """
    raise NotImplementedError  # TODO(student)


def profile_with_duration(q0: float, q1: float, duration: float, a_max: float) -> Profile:
    """A profile that takes EXACTLY ``duration`` seconds, still accelerating at ``a_max``.

    This is what makes synchronisation possible: stretch a fast joint's move to the duration of
    the slowest one by cruising more slowly.

    With t_a = v / a and D = v * (T - v / a), the cruise speed solves a quadratic; take the
    SMALLER root (the larger one accelerates past the goal and comes back). Raise ``ValueError``
    when even a full-acceleration triangle is too slow, i.e. when ``a * T**2 < 4 * D``.
    """
    raise NotImplementedError  # TODO(student)


def sample_profile(profile: Profile, t: ArrayLike) -> tuple[Array, Array, Array]:
    """Position, velocity and acceleration at times ``t``, clamped to [0, duration].

    Returns three arrays shaped like ``t``. Outside the move the position holds at the endpoint
    and the velocity and acceleration are zero — sampling at exactly ``t = duration`` must give
    exactly ``q1``, not a value that has drifted past it.
    """
    raise NotImplementedError  # TODO(student)


def cubic_coefficients(q0: float, q1: float, T: float, v0: float = 0.0, v1: float = 0.0) -> Array:
    """[a0, a1, a2, a3] of q(t) = a0 + a1 t + a2 t^2 + a3 t^3 matching position and velocity
    at t = 0 and t = T. ``T`` must be positive."""
    raise NotImplementedError  # TODO(student)


def polynomial_sample(coefficients: ArrayLike, t: ArrayLike) -> tuple[Array, Array, Array]:
    """Position, velocity and acceleration of sum_k c_k t^k at times ``t`` (no clamping)."""
    raise NotImplementedError  # TODO(student)


def min_duration_cubic(distance: float, v_max: float, a_max: float) -> float:
    """Shortest rest-to-rest cubic duration: peak |qd| = 1.5 D / T, peak |qdd| = 6 D / T^2."""
    raise NotImplementedError  # TODO(student)


def min_duration_quintic(distance: float, v_max: float, a_max: float) -> float:
    """Shortest rest-to-rest quintic duration.

    A rest-to-rest quintic with zero end accelerations has peak |qd| = 15 D / (8 T) and
    peak |qdd| = (10 / sqrt(3)) D / T^2.
    """
    raise NotImplementedError  # TODO(student)


def synchronize(q0: Sequence[float], q1: Sequence[float], v_max: Sequence[float],
                a_max: Sequence[float]) -> list[Profile]:
    """One profile per joint, all sharing the duration of the slowest joint.

    Every joint then starts and finishes together, instead of the arm passing through poses nobody
    planned. Joints that do not move still get a profile of that duration.
    """
    raise NotImplementedError  # TODO(student)


def violations(t: ArrayLike, qd: ArrayLike, qdd: ArrayLike, v_max: ArrayLike, a_max: ArrayLike,
               tolerance: float = 1e-6) -> list[str]:
    """Every limit a sampled trajectory breaks — the check to run before anything moves.

    ``qd`` and ``qdd`` are (N, n) arrays (N samples, n joints); ``v_max`` and ``a_max`` are
    per-joint limits (or scalars). Return a list of human-readable strings, one per offending
    joint, naming the joint index, the measured peak and the limit. An empty list means the
    trajectory is legal. Report velocity violations before acceleration violations.
    """
    raise NotImplementedError  # TODO(student)
