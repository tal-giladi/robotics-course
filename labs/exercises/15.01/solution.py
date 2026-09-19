"""15.01 — reference solution. Same public names as student.py.

Mirrors ``15-manipulation/code/grasp_physics.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial import ConvexHull

Array = NDArray[np.float64]

G = 9.81  # m/s^2


def cross2(a: ArrayLike, b: ArrayLike) -> float:
    """Scalar cross product of two planar vectors: a_x b_y - a_y b_x."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return float(a[0] * b[1] - a[1] * b[0])


@dataclass(frozen=True)
class Contact:
    """A planar point contact with friction."""

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
        a = friction_cone_half_angle(self.mu)
        n = self.n
        rot = lambda t: np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])  # noqa: E731
        return rot(+a) @ n, rot(-a) @ n

    def force_is_feasible(self, force: ArrayLike, tol: float = 1e-9) -> bool:
        f = np.asarray(force, dtype=float)
        fn = float(f @ self.n)
        if fn < -tol:
            return False  # pulling, not pushing
        ft = float(np.linalg.norm(f - fn * self.n))
        return ft <= self.mu * fn + tol


def evenly_spaced_contacts(n: int, radius_m: float, mu: float) -> list[Contact]:
    out = []
    for k in range(n):
        a = 2.0 * math.pi * k / n
        out.append(Contact((radius_m * math.cos(a), radius_m * math.sin(a)),
                           (-math.cos(a), -math.sin(a)), mu))
    return out


def friction_cone_half_angle(mu: float) -> float:
    if mu < 0.0:
        raise ValueError("mu must be >= 0")
    return math.atan(mu)


def required_normal_force(
    mass_kg: float,
    mu: float,
    *,
    n_contacts: int = 2,
    accel_m_s2: float = 0.0,
    safety: float = 1.0,
    g: float = G,
) -> float:
    if mu <= 0.0:
        raise ValueError("a frictionless pinch grasp cannot hold anything: mu must be > 0")
    if n_contacts < 1:
        raise ValueError("n_contacts must be >= 1")
    return safety * mass_kg * (g + accel_m_s2) / (mu * n_contacts)


def max_payload_kg(
    normal_force_n: float,
    mu: float,
    *,
    n_contacts: int = 2,
    accel_m_s2: float = 0.0,
    safety: float = 1.0,
    g: float = G,
) -> float:
    return n_contacts * mu * normal_force_n / (safety * (g + accel_m_s2))


def torsional_friction_moment(normal_force_n: float, mu: float, patch_radius_m: float) -> float:
    return (2.0 / 3.0) * mu * normal_force_n * patch_radius_m


def is_antipodal(c1: Contact, c2: Contact, tol: float = 1e-9) -> bool:
    d = c2.p - c1.p
    dist = float(np.linalg.norm(d))
    if dist < 1e-12:
        return False
    d = d / dist
    a1 = math.acos(max(-1.0, min(1.0, float(d @ c1.n))))
    a2 = math.acos(max(-1.0, min(1.0, float(-d @ c2.n))))
    return a1 <= friction_cone_half_angle(c1.mu) + tol and a2 <= friction_cone_half_angle(c2.mu) + tol


def primitive_wrenches(
    contacts: list[Contact],
    com: ArrayLike = (0.0, 0.0),
    *,
    length_scale_m: float = 0.05,
    edges: int = 2,
) -> Array:
    com = np.asarray(com, dtype=float)
    rows: list[list[float]] = []
    for c in contacts:
        a = friction_cone_half_angle(c.mu)
        angles = np.linspace(-a, a, max(2, edges))
        for t in angles:
            rot = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
            f = rot @ c.n
            f = f / float(np.linalg.norm(f))
            tau = cross2(c.p - com, f)
            rows.append([f[0], f[1], tau / length_scale_m])
    return np.asarray(rows, dtype=float)


def epsilon_quality(
    contacts: list[Contact],
    com: ArrayLike = (0.0, 0.0),
    *,
    length_scale_m: float = 0.05,
    edges: int = 2,
) -> float:
    w = primitive_wrenches(contacts, com, length_scale_m=length_scale_m, edges=edges)
    if len(w) < 4:
        return 0.0
    try:
        hull = ConvexHull(w)
    except Exception:  # noqa: BLE001 — degenerate (coplanar) wrench set: not full dimensional
        return 0.0
    offsets = hull.equations[:, -1]  # normal . x + offset <= 0 inside, normals are unit
    if float(offsets.max()) >= 0.0:
        return 0.0  # the origin is outside the hull -> not force closure
    return float(-offsets.max())


def has_force_closure(contacts: list[Contact], com: ArrayLike = (0.0, 0.0), **kw) -> bool:
    return epsilon_quality(contacts, com, **kw) > 1e-9
