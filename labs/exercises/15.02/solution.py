"""15.02 — reference solution. Same public names as student.py.

Mirrors ``15-manipulation/code/grippers.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

G = 9.81  # m/s^2


def required_normal_force(mass_kg: float, mu: float, *, n_contacts: int = 2,
                          accel_m_s2: float = 0.0, safety: float = 1.0) -> float:
    if mu <= 0.0:
        raise ValueError("mu must be > 0")
    return safety * mass_kg * (G + accel_m_s2) / (mu * n_contacts)


def payload_from_force(normal_force_n: float, mu: float, *, n_contacts: int = 2,
                       accel_m_s2: float = 0.0, safety: float = 1.0) -> float:
    return n_contacts * mu * normal_force_n / (safety * (G + accel_m_s2))


@dataclass(frozen=True)
class TargetObject:
    name: str
    width_m: float
    mass_kg: float
    surface: str
    flat_face: bool = True
    fragile: bool = False


MU_TABLE: dict[str, float] = {
    "smooth-rigid": 0.45,
    "textured": 0.75,
    "porous": 0.70,
    "soft": 0.90,
    "irregular": 0.55,
}


def kgcm_to_nm(kg_cm: float) -> float:
    return kg_cm * 0.01 * G


def servo_jaw_force(stall_torque_nm: float, lever_arm_m: float, *,
                    torque_limit: float = 0.30, efficiency: float = 0.7) -> float:
    if lever_arm_m <= 0.0:
        raise ValueError("lever_arm_m must be > 0")
    return torque_limit * stall_torque_nm * efficiency / lever_arm_m


def suction_force_n(gauge_pressure_pa: float, cup_diameter_m: float, efficiency: float = 1.0) -> float:
    area = math.pi * (cup_diameter_m / 2.0) ** 2
    return gauge_pressure_pa * area * efficiency


@dataclass(frozen=True)
class JawGripper:
    name: str
    stroke_m: float
    max_force_n: float
    pad_mu: float = 0.6
    pad_radius_m: float = 0.006
    finger_length_m: float = 0.05

    def fits(self, object_width_m: float, clearance_m: float = 0.010) -> bool:
        return object_width_m + clearance_m <= self.stroke_m

    def max_payload_kg(self, *, accel_m_s2: float = 1.0, safety: float = 2.0,
                       mu: float | None = None) -> float:
        return payload_from_force(self.max_force_n, self.pad_mu if mu is None else mu,
                                  n_contacts=2, accel_m_s2=accel_m_s2, safety=safety)


@dataclass(frozen=True)
class SuctionGripper:
    name: str
    cup_diameter_m: float
    n_cups: int = 1
    gauge_pressure_pa: float = 60_000.0
    efficiency: float = 0.7
    lip_mu: float = 0.5

    def normal_force_n(self) -> float:
        return self.n_cups * suction_force_n(self.gauge_pressure_pa, self.cup_diameter_m,
                                             self.efficiency)

    def shear_force_n(self) -> float:
        return self.lip_mu * self.normal_force_n()

    def max_payload_kg(self, *, accel_m_s2: float = 1.0, safety: float = 2.0,
                       vertical_pull: bool = True) -> float:
        f = self.normal_force_n() if vertical_pull else self.shear_force_n()
        return f / (safety * (G + accel_m_s2))


def evaluate(obj: TargetObject, jaw: JawGripper, suction: SuctionGripper) -> dict[str, str]:
    out: dict[str, str] = {}
    mu = MU_TABLE[obj.surface]

    if not jaw.fits(obj.width_m):
        out["parallel jaw"] = (f"no: needs {obj.width_m * 1000:.0f} mm + clearance, "
                               f"stroke is {jaw.stroke_m * 1000:.0f} mm")
    else:
        need = required_normal_force(obj.mass_kg, mu, accel_m_s2=1.0, safety=2.0)
        if need > jaw.max_force_n:
            out["parallel jaw"] = f"no: needs {need:.1f} N per pad, has {jaw.max_force_n:.1f} N"
        elif obj.fragile and jaw.pad_radius_m < 0.004:
            out["parallel jaw"] = f"risky: {need:.1f} N on a hard, small pad will mark a fragile object"
        else:
            out["parallel jaw"] = f"yes: {need:.1f} N of {jaw.max_force_n:.1f} N (mu {mu:.2f})"

    if obj.surface == "porous":
        out["suction"] = "no: porous surface, the cup cannot hold vacuum"
    elif not obj.flat_face:
        out["suction"] = "no: no flat patch for the cup to seal against"
    else:
        cap = suction.max_payload_kg()
        out["suction"] = (f"yes: {cap * 1000:.0f} g capacity vs {obj.mass_kg * 1000:.0f} g"
                          if cap >= obj.mass_kg else
                          f"no: {cap * 1000:.0f} g capacity, object is {obj.mass_kg * 1000:.0f} g")

    if not jaw.fits(obj.width_m):
        out["compliant jaw"] = "no: same stroke limit as the rigid jaw"
    elif obj.surface in ("irregular", "soft") or obj.fragile or not obj.flat_face:
        out["compliant jaw"] = "yes: conforms to the shape and spreads the load"
    else:
        out["compliant jaw"] = "works, but a rigid jaw holds position better"
    return out
