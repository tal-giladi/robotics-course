"""Lesson 15.01 — the physics of grasping: friction cones, grip force, force closure.

Everything here is planar (2D) unless a docstring says otherwise: a planar grasp is enough to
see every effect that matters, and you can draw it. Units: SI (N, m, kg, rad).

    py grasp_physics.py            the worked examples and tables printed in the lesson

Conventions
-----------
* A ``Contact`` is a point on the object's surface plus the **inward** unit normal (pointing from
  the surface into the object) — the direction the finger pushes.
* Coulomb friction: the contact force must stay inside the friction cone, ``|f_t| <= mu * f_n``.
* Wrenches are ``(fx, fy, tau)`` about the object's centre of mass, with the torque divided by a
  characteristic length so the three numbers have comparable magnitudes.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial import ConvexHull

Array = NDArray[np.float64]

G = 9.81  # m/s^2


# ============================================================================== friction basics
def friction_cone_half_angle(mu: float) -> float:
    """Half-angle of the Coulomb friction cone, radians: alpha = atan(mu)."""
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
    """Normal force per finger needed to hold ``mass_kg`` against slip in a pinch grasp.

    The object hangs in the friction of ``n_contacts`` fingers, each pressing with ``F`` normal to
    a vertical surface, so the load is carried purely by tangential friction:

        n_contacts * mu * F  >=  safety * m * (g + a)

    ``accel_m_s2`` is the extra vertical acceleration of the gripper (a fast lift or an emergency
    stop). ``safety`` is the margin you keep for a wrong mu, a wet surface or a bumpy drive.
    """
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
    """Inverse of :func:`required_normal_force`: the heaviest object this grip can hold."""
    return n_contacts * mu * normal_force_n / (safety * (g + accel_m_s2))


def torsional_friction_moment(normal_force_n: float, mu: float, patch_radius_m: float) -> float:
    """Max torque a *soft* circular contact patch resists about its own normal, N*m.

    Uniform pressure over a disc of radius ``r``: tau = (2/3) * mu * F * r. A point contact has
    r = 0 and resists no torsion at all — which is why hard fingers let an off-centre object spin.
    """
    return (2.0 / 3.0) * mu * normal_force_n * patch_radius_m


def suction_force_n(gauge_pressure_pa: float, cup_diameter_m: float, efficiency: float = 1.0) -> float:
    """Ideal lift force of one suction cup: F = dP * A * efficiency (used again in 15.02)."""
    area = math.pi * (cup_diameter_m / 2.0) ** 2
    return gauge_pressure_pa * area * efficiency


# ============================================================================== contacts
@dataclass(frozen=True)
class Contact:
    """A planar point contact with friction.

    position: (x, y) on the object surface, in the object frame (metres).
    normal:   inward unit normal (points from the surface into the object).
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
        """True if ``force`` (applied by the finger on the object) lies inside this friction cone."""
        f = np.asarray(force, dtype=float)
        fn = float(f @ self.n)
        if fn < -tol:
            return False  # pulling, not pushing
        ft = float(np.linalg.norm(f - fn * self.n))
        return ft <= self.mu * fn + tol


def cross2(a: ArrayLike, b: ArrayLike) -> float:
    """Scalar cross product of two planar vectors: a_x b_y - a_y b_x."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return float(a[0] * b[1] - a[1] * b[0])


# ============================================================================== antipodal test
def is_antipodal(c1: Contact, c2: Contact, tol: float = 1e-9) -> bool:
    """True if the segment joining the two contacts lies inside **both** friction cones.

    This is the classic two-finger force-closure test in the plane: a pinch is secure exactly when
    each finger can push along the grasp line without the tangential force exceeding mu * f_n.
    """
    d = c2.p - c1.p
    dist = float(np.linalg.norm(d))
    if dist < 1e-12:
        return False
    d = d / dist
    a1 = math.acos(max(-1.0, min(1.0, float(d @ c1.n))))
    a2 = math.acos(max(-1.0, min(1.0, float(-d @ c2.n))))
    return a1 <= friction_cone_half_angle(c1.mu) + tol and a2 <= friction_cone_half_angle(c2.mu) + tol


# ============================================================================== grasp wrench space
def primitive_wrenches(
    contacts: list[Contact],
    com: ArrayLike = (0.0, 0.0),
    *,
    length_scale_m: float = 0.05,
    edges: int = 2,
) -> Array:
    """Unit wrenches (fx, fy, tau/L) from the friction-cone edges of every contact.

    ``edges=2`` uses the two cone boundaries, which is exact in 2D. ``L = length_scale_m`` makes the
    torque commensurate with the forces (otherwise 'distance to the origin' mixes N and N*m).
    """
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
    """Ferrari-Canny epsilon metric with an L1 bound on the total contact force.

    The grasp wrench space is the convex hull of the primitive wrenches. ``epsilon`` is the radius of
    the largest wrench ball centred on the origin that still fits inside it — the weakest
    disturbance direction, in newtons per unit of total contact force. ``0.0`` means no force
    closure: some disturbance cannot be resisted at all.
    """
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


def evenly_spaced_contacts(n: int, radius_m: float, mu: float) -> list[Contact]:
    """``n`` fingers spread evenly around a disc of ``radius_m``, all pushing toward its centre."""
    out = []
    for k in range(n):
        a = 2.0 * math.pi * k / n
        out.append(Contact((radius_m * math.cos(a), radius_m * math.sin(a)),
                           (-math.cos(a), -math.sin(a)), mu))
    return out


def has_force_closure(contacts: list[Contact], com: ArrayLike = (0.0, 0.0), **kw) -> bool:
    """True when the grasp can resist any planar wrench (epsilon > 0)."""
    return epsilon_quality(contacts, com, **kw) > 1e-9


# ============================================================================== the lesson demos
def demo_grip_force() -> None:
    print("--- how hard must the SO-101 squeeze? (two flat pads, vertical lift) ---")
    print(f"{'object':<26}{'m kg':>7}{'mu':>6}{'a m/s2':>8}{'safety':>8}{'F per pad N':>13}")
    rows = [
        ("empty 330 ml can", 0.015, 0.6, 0.0, 2.0),
        ("full 330 ml can", 0.345, 0.6, 0.0, 2.0),
        ("full can, 2 m/s^2 lift", 0.345, 0.6, 2.0, 2.0),
        ("full can, wet/slick pad", 0.345, 0.25, 2.0, 2.0),
        ("250 g mug, rubber pad", 0.250, 0.8, 1.0, 2.0),
        ("250 g mug, bare plastic", 0.250, 0.30, 1.0, 2.0),
    ]
    for name, m, mu, a, s in rows:
        f = required_normal_force(m, mu, accel_m_s2=a, safety=s)
        print(f"{name:<26}{m:>7.3f}{mu:>6.2f}{a:>8.1f}{s:>8.1f}{f:>13.2f}")
    print()
    print("--- the other direction: what can a given squeeze hold? ---")
    for f_n in (5.0, 10.0, 20.0):
        for mu in (0.25, 0.6, 1.0):
            m = max_payload_kg(f_n, mu, accel_m_s2=1.0, safety=2.0)
            print(f"  F = {f_n:4.1f} N per pad, mu = {mu:4.2f}  ->  max {m * 1000:6.0f} g")
    print()


def demo_friction_cone() -> None:
    print("--- friction cone half-angles ---")
    for mu in (0.1, 0.25, 0.4, 0.6, 0.8, 1.0, 1.5):
        print(f"  mu = {mu:4.2f}  ->  alpha = {math.degrees(friction_cone_half_angle(mu)):5.1f} deg")
    print()
    print("--- off-centre grasp: torsion a point contact cannot resist ---")
    m, d = 0.250, 0.02  # 250 g mug grasped 20 mm off its centre of mass
    tau_needed = m * G * d
    print(f"  m = {m} kg grasped {d * 1000:.0f} mm off the CoM -> gravity torque {tau_needed:.4f} N*m")
    for r_mm in (0.0, 4.0, 8.0, 12.0):
        tau = 2.0 * torsional_friction_moment(6.0, 0.8, r_mm / 1000.0)  # two pads
        verdict = "holds" if tau >= tau_needed else "SLIPS (object rotates)"
        print(f"  pad radius {r_mm:4.1f} mm, 6 N, mu 0.8 -> tau_max {tau:.4f} N*m  {verdict}")
    print()


def demo_force_closure() -> None:
    print("--- a pinch on a 60 mm wide box, contacts on the two vertical faces ---")
    print(f"{'case':<34}{'antipodal':>10}{'epsilon':>10}")
    half = 0.030
    cases = [
        ("flat faces, mu 0.6, aligned", 0.0, 0.6, 0.0),
        ("contacts 15 mm out of line", 0.015, 0.6, 0.0),
        ("contacts 30 mm out of line", 0.030, 0.6, 0.0),
        ("faces tilted 20 deg, mu 0.6", 0.0, 0.6, math.radians(20)),
        ("faces tilted 40 deg, mu 0.6", 0.0, 0.6, math.radians(40)),
        ("faces tilted 40 deg, mu 1.0", 0.0, 1.0, math.radians(40)),
        ("slippery mu 0.1, aligned", 0.0, 0.1, 0.0),
    ]
    for name, offset, mu, tilt in cases:
        left = Contact((-half, +offset), (math.cos(tilt), -math.sin(tilt)), mu)
        right = Contact((+half, -offset), (-math.cos(tilt), math.sin(tilt)), mu)
        eps = epsilon_quality([left, right])
        print(f"{name:<34}{str(is_antipodal(left, right)):>10}{eps:>10.4f}")
    print()
    print("--- how many fingers? contacts spread evenly around a 60 mm disc, mu 0.6 ---")
    for n in (2, 3, 4, 5):
        print(f"  {n} fingers: epsilon = {epsilon_quality(evenly_spaced_contacts(n, 0.030, 0.6)):.4f}")
    print()
    print("--- epsilon vs. friction, 2 fingers on the 60 mm disc ---")
    for mu in (0.1, 0.25, 0.4, 0.6, 0.8, 1.0):
        print(f"  mu = {mu:4.2f}: epsilon = {epsilon_quality(evenly_spaced_contacts(2, 0.030, mu)):.4f}")
    print()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["grip", "cone", "closure"], help="run one demo")
    args = ap.parse_args(argv)
    if args.only in (None, "grip"):
        demo_grip_force()
    if args.only in (None, "cone"):
        demo_friction_cone()
    if args.only in (None, "closure"):
        demo_force_closure()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
