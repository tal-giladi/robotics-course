"""14.06 — The Jacobian toolkit: velocities, singularities, damping and statics.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 14.06``.
Only the standard library and numpy are needed.

The arm here is the planar model of the SO-101's upper arm + forearm from 14.04–14.06:
``n`` revolute joints in a vertical plane, link lengths ``lengths`` (metres), joint angles ``q``
(radians, each relative to the previous link). ``planar_joint_points`` is given — it returns the
base, every joint and the tip, which is all the geometry a Jacobian needs.

Units, everywhere: metres, radians, seconds, newtons, newton-metres. A linear Jacobian entry is
metres per radian.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given ---------------------------------------------------------------------------------------
def planar_joint_points(lengths: Sequence[float], q: Sequence[float]) -> Array:
    """(n+1, 2) array: the base, every joint, and the tip. Link i points along q1 + ... + qi."""
    if len(lengths) != len(q):
        raise ValueError(f"{len(lengths)} links but {len(q)} joint angles")
    pts = np.zeros((len(lengths) + 1, 2))
    phi = 0.0
    for i, (length, qi) in enumerate(zip(lengths, q)):
        phi += qi
        pts[i + 1] = pts[i] + length * np.array([np.cos(phi), np.sin(phi)])
    return pts


def planar_tip(lengths: Sequence[float], q: Sequence[float]) -> Array:
    """The tip position (x, y) — the function whose derivative you are about to write."""
    return planar_joint_points(lengths, q)[-1]


# --- TODO(student) -------------------------------------------------------------------------------
def planar_jacobian(lengths: Sequence[float], q: Sequence[float]) -> Array:
    """The (2, n) linear Jacobian of the tip: ``v_tip = planar_jacobian(...) @ qdot``.

    Joint j rotates everything beyond it about the point ``pts[j]``, so the tip's velocity
    contribution is ``z_hat x (tip - pts[j])``. In 2D that cross product with the out-of-plane
    unit vector is simply ``(-r_y, r_x)``.

    No finite differences: build it from the geometry. The tests compare you against a
    central-difference derivative, so a finite-difference implementation would pass the first test
    and teach you nothing.
    """
    raise NotImplementedError  # TODO(student)


def singular_values(J: ArrayLike) -> Array:
    """Singular values of ``J``, largest first (the semi-axes of the velocity ellipse, m/rad)."""
    raise NotImplementedError  # TODO(student)


def manipulability(J: ArrayLike) -> float:
    """Yoshikawa's measure w = sqrt(det(J J^T)) — the volume of the velocity ellipse.

    Zero exactly at a singularity. For the 2-link arm it must equal l1 * l2 * |sin q2|.
    Never let a floating-point negative determinant reach ``sqrt``.
    """
    raise NotImplementedError  # TODO(student)


def dls_velocity(J: ArrayLike, v: ArrayLike, damping: float) -> Array:
    """Joint velocities for a desired tip velocity, by damped least squares.

        qdot = J^T (J J^T + damping^2 I)^-1 v

    ``damping = 0`` must reproduce the exact/pseudo-inverse solution. A positive damping must keep
    ``qdot`` bounded by ``|v| / (2 * damping)`` even at an exact singularity, where the plain
    inverse does not exist at all.

    Build the m x m system (m = rows of J), not the n x n one: it is smaller, and it is the form
    that still works when the arm has more joints than task dimensions.
    """
    raise NotImplementedError  # TODO(student)


def scale_to_limits(qdot: ArrayLike, limit: float) -> Array:
    """Scale the WHOLE vector so that ``max |qdot_i| <= limit``, preserving its direction.

    Returns ``qdot`` unchanged when it is already legal. Clipping joints individually would change
    the direction of the resulting tip motion, which is worse than being slow.
    ``limit`` must be positive.
    """
    raise NotImplementedError  # TODO(student)


def payload_torques(J: ArrayLike, force: ArrayLike) -> Array:
    """Joint torques [N.m] that hold a force [N] applied at the tip: tau = J^T f.

    ``force`` is the force the environment applies to the tip (a payload of mass m hanging under
    gravity applies ``(0, -m * 9.81)``). The joints must supply ``tau`` to stay put.
    """
    raise NotImplementedError  # TODO(student)


def worst_case_joint_speed(J: ArrayLike, speed: float) -> float:
    """The largest ``|qdot|`` (Euclidean norm) an exact inverse could demand for a tip speed of
    ``speed`` m/s, maximised over all directions of the commanded velocity.

    That worst case is ``speed / sigma_min``: the direction in which the arm is least able to move.
    It upper-bounds every individual joint speed too, so it is the number to compare against your
    servo's limit. Return ``float("inf")`` at a singularity — treat any ``sigma_min <= 1e-12``
    as zero, because the SVD of a singular matrix rarely returns an exact 0.0.
    """
    raise NotImplementedError  # TODO(student)
