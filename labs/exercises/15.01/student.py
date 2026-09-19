"""15.01 — the contact model: friction cones, grip force, torsion and force closure.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.01``.
Only the standard library, numpy and scipy are needed.

Everything is planar (2D). Units: SI (N, m, kg, rad).

Conventions
-----------
* A ``Contact`` is a point on the object's surface plus the **inward** unit normal — the direction
  the finger pushes (from the surface *into* the object).
* Coulomb friction: the contact force must stay inside the friction cone, ``|f_t| <= mu * f_n``.
* A planar wrench is ``(fx, fy, tau / L)``: the force plus the torque it makes about the object's
  centre of mass, divided by a characteristic length so the three numbers are commensurate.

The reference implementation lives at ``15-manipulation/code/grasp_physics.py`` — don't read it
until you've tried.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial import ConvexHull

Array = NDArray[np.float64]

G = 9.81  # m/s^2


# --- given -------------------------------------------------------------------------------------
def cross2(a: ArrayLike, b: ArrayLike) -> float:
    """Scalar cross product of two planar vectors: a_x b_y - a_y b_x."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return float(a[0] * b[1] - a[1] * b[0])


@dataclass(frozen=True)
class Contact:
    """A planar point contact with friction.

    position: (x, y) on the object surface, in the object frame (metres).
    normal:   inward unit normal (points from the surface into the object); need not be normalised.
    mu:       Coulomb friction coefficient at this contact.
    """

    position: tuple[float, float]
    normal: tuple[float, float]
    mu: float = 0.5

    @property
    def p(self) -> Array:
        return np.asarray(self.position, dtype=float)

    @property
    def n(self) -> Array:
        n = np.asarray(self.normal, dtype=float)
        norm = float(np.linalg.norm(n))
        if norm < 1e-12:
            raise ValueError("contact normal must be non-zero")
        return n / norm

    def cone_edges(self) -> tuple[Array, Array]:
        """The two unit vectors bounding the friction cone (left edge, right edge)."""
        a = friction_cone_half_angle(self.mu)
        n = self.n
        rot = lambda t: np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])  # noqa: E731
        return rot(+a) @ n, rot(-a) @ n

    def force_is_feasible(self, force: ArrayLike, tol: float = 1e-9) -> bool:
        """True if ``force`` (applied by the finger on the object) lies inside this friction cone.

        Two conditions, in this order:
        1. the finger must **push**, not pull: the component along the inward normal is >= 0;
        2. the tangential component must satisfy |f_t| <= mu * f_n.
        """
        raise NotImplementedError("Contact.force_is_feasible")  # TODO(student)


def evenly_spaced_contacts(n: int, radius_m: float, mu: float) -> list[Contact]:
    """``n`` fingers spread evenly around a disc of ``radius_m``, all pushing toward its centre."""
    out = []
    for k in range(n):
        a = 2.0 * math.pi * k / n
        out.append(Contact((radius_m * math.cos(a), radius_m * math.sin(a)),
                           (-math.cos(a), -math.sin(a)), mu))
    return out


# --- implement these ---------------------------------------------------------------------------
def friction_cone_half_angle(mu: float) -> float:
    """Half-angle of the Coulomb friction cone, in radians.

    ``alpha = atan(mu)``. Raise ``ValueError`` for a negative ``mu``.
    """
    raise NotImplementedError("friction_cone_half_angle")  # TODO(student)


def required_normal_force(
    mass_kg: float,
    mu: float,
    *,
    n_contacts: int = 2,
    accel_m_s2: float = 0.0,
    safety: float = 1.0,
    g: float = G,
) -> float:
    """Normal force per finger needed to hold ``mass_kg`` against slip in a pinch grasp.

    The object hangs in the friction of ``n_contacts`` fingers, each pressing with ``F`` normal to
    a vertical surface, so the load is carried purely by tangential friction:

        n_contacts * mu * F  >=  safety * m * (g + a)

    Raise ``ValueError`` if ``mu <= 0`` (a frictionless pinch holds nothing) or ``n_contacts < 1``.
    """
    raise NotImplementedError("required_normal_force")  # TODO(student)


def max_payload_kg(
    normal_force_n: float,
    mu: float,
    *,
    n_contacts: int = 2,
    accel_m_s2: float = 0.0,
    safety: float = 1.0,
    g: float = G,
) -> float:
    """Inverse of :func:`required_normal_force`: the heaviest object this grip can hold, in kg."""
    raise NotImplementedError("max_payload_kg")  # TODO(student)


def torsional_friction_moment(normal_force_n: float, mu: float, patch_radius_m: float) -> float:
    """Max torque a *soft* circular contact patch resists about its own normal, N*m.

    Uniform pressure over a disc of radius ``r``: ``tau = (2/3) * mu * F * r``. A point contact has
    r = 0 and resists no torsion at all.
    """
    raise NotImplementedError("torsional_friction_moment")  # TODO(student)


def is_antipodal(c1: Contact, c2: Contact, tol: float = 1e-9) -> bool:
    """True if the segment joining the two contacts lies inside **both** friction cones.

    The classic two-finger force-closure test in the plane. Let ``d`` be the unit vector from
    ``c1.p`` to ``c2.p``. The grasp is antipodal when the angle between ``d`` and ``c1.n`` and the
    angle between ``-d`` and ``c2.n`` are both within the respective cone half-angles.

    Two coincident contacts are not antipodal (return False rather than dividing by zero).
    """
    raise NotImplementedError("is_antipodal")  # TODO(student)


def primitive_wrenches(
    contacts: list[Contact],
    com: ArrayLike = (0.0, 0.0),
    *,
    length_scale_m: float = 0.05,
    edges: int = 2,
) -> Array:
    """Unit wrenches ``(fx, fy, tau / L)`` from the friction-cone edges of every contact.

    For each contact, take ``edges`` unit force directions spread evenly from ``-alpha`` to
    ``+alpha`` about the inward normal (``edges=2`` gives exactly the two cone boundaries, which is
    exact in 2D). For each, the torque about ``com`` is ``cross2(p - com, f)``, and the row is
    ``[fx, fy, tau / length_scale_m]``.

    Returns an ``(edges * len(contacts), 3)`` array, contacts in order.
    """
    raise NotImplementedError("primitive_wrenches")  # TODO(student)


def epsilon_quality(
    contacts: list[Contact],
    com: ArrayLike = (0.0, 0.0),
    *,
    length_scale_m: float = 0.05,
    edges: int = 2,
) -> float:
    """Ferrari-Canny epsilon metric with an L1 bound on the total contact force.

    The grasp wrench space is the convex hull of :func:`primitive_wrenches`. ``epsilon`` is the
    radius of the largest wrench ball centred on the origin that still fits inside it.

    Return exactly ``0.0`` when there is no force closure: fewer than 4 wrenches, a degenerate
    (not full-dimensional) hull — ``scipy.spatial.ConvexHull`` raises in that case — or the origin
    lying outside the hull.

    Hint: ``ConvexHull.equations`` holds rows ``[n_x, n_y, n_z, offset]`` with **unit** normals, and
    a point ``x`` is inside when ``n . x + offset <= 0`` for every row. So for the origin, the
    distance to the nearest facet is ``-max(offsets)``, and ``max(offsets) >= 0`` means outside.
    """
    raise NotImplementedError("epsilon_quality")  # TODO(student)


def has_force_closure(contacts: list[Contact], com: ArrayLike = (0.0, 0.0), **kw) -> bool:
    """True when the grasp can resist any planar wrench (epsilon > 1e-9)."""
    raise NotImplementedError("has_force_closure")  # TODO(student)
