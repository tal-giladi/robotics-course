"""15.02 — gripper models: parallel jaw, suction, and the selection logic.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.02``.
Only the standard library is needed (no numpy).

Units: SI (N, m, kg, Pa, N*m). The physics comes from 15.01, reimplemented here in two lines so
this exercise stands alone.

The reference implementation lives at ``15-manipulation/code/grippers.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.81  # m/s^2


# --- given (the 15.01 physics, so this exercise is self-contained) ------------------------------
def required_normal_force(mass_kg: float, mu: float, *, n_contacts: int = 2,
                          accel_m_s2: float = 0.0, safety: float = 1.0) -> float:
    """Normal force per finger to hold ``mass_kg`` against slip: s*m*(g+a) / (n*mu)."""
    if mu <= 0.0:
        raise ValueError("mu must be > 0")
    return safety * mass_kg * (G + accel_m_s2) / (mu * n_contacts)


def payload_from_force(normal_force_n: float, mu: float, *, n_contacts: int = 2,
                       accel_m_s2: float = 0.0, safety: float = 1.0) -> float:
    """The inverse: the heaviest object ``normal_force_n`` per finger can hold, in kg."""
    return n_contacts * mu * normal_force_n / (safety * (G + accel_m_s2))


@dataclass(frozen=True)
class TargetObject:
    """An object you want to pick.

    width_m:   the dimension the jaws must straddle
    surface:   "smooth-rigid" | "textured" | "porous" | "soft" | "irregular"
    flat_face: is there a flat patch at least a cup diameter across, for suction?
    """

    name: str
    width_m: float
    mass_kg: float
    surface: str
    flat_face: bool = True
    fragile: bool = False


MU_TABLE: dict[str, float] = {  # rubber pad against ..., measured the way 15.01-E4 measures it
    "smooth-rigid": 0.45,
    "textured": 0.75,
    "porous": 0.70,
    "soft": 0.90,
    "irregular": 0.55,
}


# --- implement these ----------------------------------------------------------------------------
def kgcm_to_nm(kg_cm: float) -> float:
    """Servo datasheets quote torque in kg*cm: 1 kg*cm = 0.01 m * 9.81 N = 0.0981 N*m."""
    raise NotImplementedError("kgcm_to_nm")  # TODO(student)


def servo_jaw_force(stall_torque_nm: float, lever_arm_m: float, *,
                    torque_limit: float = 0.30, efficiency: float = 0.7) -> float:
    """Normal force per pad from a servo-driven jaw.

        F = torque_limit * stall_torque * efficiency / lever_arm

    ``torque_limit`` is the fraction of stall you actually command (14.11: never run a hobby bus
    servo at stall). ``efficiency`` covers the linkage. Raise ``ValueError`` on a non-positive
    ``lever_arm_m``.
    """
    raise NotImplementedError("servo_jaw_force")  # TODO(student)


def suction_force_n(gauge_pressure_pa: float, cup_diameter_m: float, efficiency: float = 1.0) -> float:
    """Ideal lift force of one suction cup: F = dP * A * efficiency, with A = pi (d/2)^2."""
    raise NotImplementedError("suction_force_n")  # TODO(student)


@dataclass(frozen=True)
class JawGripper:
    """A two-finger gripper described by the four numbers that decide what it can pick.

    stroke_m:        maximum opening between the pads
    max_force_n:     normal force per pad at the commanded torque limit
    pad_mu:          friction coefficient of the pad against a typical object
    pad_radius_m:    effective radius of the contact patch (0 for a hard point contact)
    finger_length_m: pad centre to the last joint
    """

    name: str
    stroke_m: float
    max_force_n: float
    pad_mu: float = 0.6
    pad_radius_m: float = 0.006
    finger_length_m: float = 0.05

    def fits(self, object_width_m: float, clearance_m: float = 0.010) -> bool:
        """True if the object fits between the open pads **with room to approach**.

        The jaws must open wider than the object so they can descend around it without brushing
        it over, so the test is on ``object_width_m + clearance_m``, not on the width alone.
        """
        raise NotImplementedError("JawGripper.fits")  # TODO(student)

    def max_payload_kg(self, *, accel_m_s2: float = 1.0, safety: float = 2.0,
                       mu: float | None = None) -> float:
        """The heaviest object this jaw can hold. ``mu`` overrides ``pad_mu`` when given."""
        raise NotImplementedError("JawGripper.max_payload_kg")  # TODO(student)


@dataclass(frozen=True)
class SuctionGripper:
    """``n_cups`` cups of ``cup_diameter_m``, fed by a pump reaching ``gauge_pressure_pa`` below
    atmosphere. ``efficiency`` is the fraction of the ideal area force that survives cup
    deformation and lip leakage (0.6-0.8 on flat, smooth, clean surfaces)."""

    name: str
    cup_diameter_m: float
    n_cups: int = 1
    gauge_pressure_pa: float = 60_000.0
    efficiency: float = 0.7
    lip_mu: float = 0.5

    def normal_force_n(self) -> float:
        """Pull-off force along the cup axis, summed over all cups."""
        raise NotImplementedError("SuctionGripper.normal_force_n")  # TODO(student)

    def shear_force_n(self) -> float:
        """Force the cup resists *along* the surface: friction of the lip under the suction load,
        i.e. ``lip_mu * normal_force_n()``."""
        raise NotImplementedError("SuctionGripper.shear_force_n")  # TODO(student)

    def max_payload_kg(self, *, accel_m_s2: float = 1.0, safety: float = 2.0,
                       vertical_pull: bool = True) -> float:
        """Payload in kg: ``F / (safety * (g + a))`` with F the pull-off force when
        ``vertical_pull`` and the shear force otherwise."""
        raise NotImplementedError("SuctionGripper.max_payload_kg")  # TODO(student)


def evaluate(obj: TargetObject, jaw: JawGripper, suction: SuctionGripper) -> dict[str, str]:
    """Verdict per gripper type for one object. Strings, because the answer is a sentence.

    Returns a dict with exactly the keys ``"parallel jaw"``, ``"suction"`` and ``"compliant jaw"``.
    The tests do not check the wording; they check that each verdict **starts with** ``"yes"``,
    ``"no"``, ``"risky"`` or ``"works"``, and that the right one is chosen. The decision order
    matters and is pinned by the tests:

    parallel jaw
        1. does not fit (``jaw.fits`` is False)                     -> "no: ..."
        2. needs more force than ``jaw.max_force_n`` (use MU_TABLE
           for the surface, accel 1.0 m/s^2, safety 2.0)            -> "no: ..."
        3. fragile and ``pad_radius_m < 0.004``                     -> "risky: ..."
        4. otherwise                                                -> "yes: ..."

    suction
        1. porous surface (checked FIRST — a porous object with a
           flat face still cannot hold a vacuum)                    -> "no: ..."
        2. no flat face                                             -> "no: ..."
        3. capacity below the object's mass                         -> "no: ..."
        4. otherwise                                                -> "yes: ..."

    compliant jaw
        1. does not fit (same stroke limit as the rigid jaw)        -> "no: ..."
        2. surface is "irregular" or "soft", or fragile,
           or there is no flat face                                 -> "yes: ..."
        3. otherwise                                                -> "works, but ..."
    """
    raise NotImplementedError("evaluate")  # TODO(student)
