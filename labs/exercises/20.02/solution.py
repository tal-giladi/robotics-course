"""20.02 — the integration arithmetic (reference solution): wire, fuse, and where the mass ended up.

Four functions decide whether the hardware you just bolted together works:

* ``wire_drop_v``      — how much voltage the harness eats at the current that matters;
* ``fuse_rating_a``    — the standard fuse that protects the wire without nuisance-blowing;
* ``centre_of_gravity``— where the mass ended up after you added an arm and a LiDAR mast;
* ``tipping_accel``    — the acceleration that sits the robot back on its tail.

Fill in every ``TODO(student)``; check with ``python course.py check 20.02``.
Standard library only (``math`` if you want it).
"""

from __future__ import annotations

from collections.abc import Sequence

#: Copper resistance at 20 C, ohm per metre (PowerStream AWG table).
AWG_OHM_PER_M: dict[int, float] = {
    22: 0.05296, 20: 0.03331, 18: 0.02095, 16: 0.01318, 14: 0.008286, 12: 0.005211,
}
#: Standard automotive blade fuse values, amps.
BLADE_FUSE_A: tuple[float, ...] = (3.0, 5.0, 7.5, 10.0, 15.0, 20.0, 25.0, 30.0)
#: Standard gravity, m/s^2.
G = 9.81


def wire_drop_v(awg: int, length_m: float, amps: float) -> float:
    """Voltage lost in a run of ``length_m`` (ONE way) carrying ``amps``.

    The current goes out along one conductor and comes back along another, so the resistance in
    the loop is *twice* the one-way resistance. Forgetting the return path is the classic way to
    halve your own voltage-drop estimate and brown out the Pi under load.

    >>> round(wire_drop_v(18, 1.0, 5.0), 4)
    0.2095
    >>> wire_drop_v(18, 0.0, 5.0)
    0.0
    """
    return 2.0 * length_m * AWG_OHM_PER_M[awg] * amps


def fuse_rating_a(continuous_a: float, ampacity_a: float) -> float:
    """The smallest standard blade fuse that protects this branch, or raise ``ValueError``.

    Two constraints, and a fuse has to satisfy both:

    * **at least** ``1.25 * continuous_a`` — a fuse sized at the working current blows every time
      the motors accelerate, and a robot that dies at random is a robot you stop fusing;
    * **at most** ``ampacity_a`` — above the wire's rating the fuse never opens first, so the
      wire becomes the fuse. That is the failure mode the fuse existed to prevent.

    Raise ``ValueError`` when no value in :data:`BLADE_FUSE_A` satisfies both (the message is
    yours; the tests only check that it raises).

    >>> fuse_rating_a(5.9, 8.0)
    7.5
    >>> fuse_rating_a(1.71, 8.0)
    3.0
    """
    target = 1.25 * continuous_a
    for value in BLADE_FUSE_A:
        if value >= target:
            if value > ampacity_a:
                raise ValueError(f"{value:g} A fuse above the {ampacity_a:g} A ampacity: "
                                 "the wire would be the fuse - use thicker wire")
            return value
    raise ValueError(f"no standard blade fuse reaches {target:.1f} A")


def centre_of_gravity(parts: Sequence[tuple[float, float, float, float]]
                      ) -> tuple[float, float, float, float]:
    """``(total_mass_kg, x, y, z)`` of a list of ``(mass_kg, x, y, z)`` parts, in base_link.

    The mass-weighted mean: $x_{cg} = \\sum m_i x_i / \\sum m_i$, and the same for y and z.
    Raise ``ValueError`` on an empty list (a robot with no mass is a modelling error, not a
    division by zero).

    >>> centre_of_gravity([(1.0, 0.0, 0.0, 0.0), (1.0, 0.2, 0.0, 0.1)])
    (2.0, 0.1, 0.0, 0.05)
    """
    if not parts:
        raise ValueError("a robot with no parts has no centre of gravity")
    total = sum(p[0] for p in parts)
    if total <= 0.0:
        raise ValueError("total mass must be positive")
    return (total,
            sum(p[0] * p[1] for p in parts) / total,
            sum(p[0] * p[2] for p in parts) / total,
            sum(p[0] * p[3] for p in parts) / total)


def tipping_accel(x_cg_m: float, z_cg_m: float) -> float:
    """Forward acceleration (m/s^2) at which karmel rears up over its wheel axle.

    karmel touches the ground at two drive wheels on the axle (x = 0) and one ball caster
    100 mm in FRONT of them, so there is nothing holding the tail down behind. Accelerating
    makes inertia act backwards at the CG height; the robot tips when that moment beats
    gravity's:

    $$m\\,a\\,z_{cg} > m\\,g\\,x_{cg} \\quad\\Rightarrow\\quad a_\\text{tip} = \\frac{g\\,x_{cg}}{z_{cg}}$$

    Return ``0.0`` when ``x_cg_m <= 0`` (the CG is already on or behind the axle: the robot
    is resting back on its tail, and any acceleration at all tips it) and when ``z_cg_m <= 0``.

    >>> round(tipping_accel(0.021, 0.070), 2)
    2.94
    >>> tipping_accel(-0.012, 0.074)
    0.0
    """
    if x_cg_m <= 0.0 or z_cg_m <= 0.0:
        return 0.0
    return G * x_cg_m / z_cg_m
