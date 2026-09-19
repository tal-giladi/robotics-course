"""Lesson 15.02 — gripper models: parallel jaw, compliant jaw, suction.

Three small physical models plus a selection table, so "which gripper?" becomes arithmetic
instead of opinion.

    py grippers.py                 the tables printed in the lesson
    py grippers.py --only suction

Units: SI (N, m, kg, Pa, N*m).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

from grasp_physics import G, max_payload_kg, required_normal_force, suction_force_n


# ============================================================================== parallel jaw
@dataclass(frozen=True)
class JawGripper:
    """A two-finger gripper described by the four numbers that decide what it can pick.

    stroke_m:        maximum opening between the pads (the widest object it can straddle)
    max_force_n:     normal force per pad at the commanded torque limit
    pad_mu:          friction coefficient of the pad against a typical object
    pad_radius_m:    effective radius of the contact patch (0 for a hard point contact)
    finger_length_m: pad centre to the last joint — the lever the object's weight acts on
    """

    name: str
    stroke_m: float
    max_force_n: float
    pad_mu: float = 0.6
    pad_radius_m: float = 0.006
    finger_length_m: float = 0.05

    def fits(self, object_width_m: float, clearance_m: float = 0.010) -> bool:
        """True if the object fits between the open pads with room to approach."""
        return object_width_m + clearance_m <= self.stroke_m

    def max_payload_kg(self, *, accel_m_s2: float = 1.0, safety: float = 2.0, mu: float | None = None) -> float:
        return max_payload_kg(self.max_force_n, self.pad_mu if mu is None else mu,
                              n_contacts=2, accel_m_s2=accel_m_s2, safety=safety)


def servo_jaw_force(stall_torque_nm: float, lever_arm_m: float, *,
                    torque_limit: float = 0.30, efficiency: float = 0.7) -> float:
    """Normal force per pad from a servo-driven jaw.

    ``torque_limit`` is the fraction of stall torque you actually command (14.11: never run a
    hobby bus servo at stall — it overheats and strips gears). ``efficiency`` covers the linkage
    and friction. For a jaw whose pad sits ``lever_arm_m`` from the jaw pivot:

        F = torque_limit * stall_torque * efficiency / lever_arm
    """
    if lever_arm_m <= 0.0:
        raise ValueError("lever_arm_m must be > 0")
    return torque_limit * stall_torque_nm * efficiency / lever_arm_m


def kgcm_to_nm(kg_cm: float) -> float:
    """Servo datasheets quote torque in kg*cm: 1 kg*cm = 0.01 m * 9.81 N = 0.0981 N*m."""
    return kg_cm * 0.01 * G


# ============================================================================== suction
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
        """Pull-off force along the cup axis."""
        return self.n_cups * suction_force_n(self.gauge_pressure_pa, self.cup_diameter_m, self.efficiency)

    def shear_force_n(self) -> float:
        """Force the cup resists *along* the surface: friction of the lip under the suction load."""
        return self.lip_mu * self.normal_force_n()

    def max_payload_kg(self, *, accel_m_s2: float = 1.0, safety: float = 2.0, vertical_pull: bool = True) -> float:
        f = self.normal_force_n() if vertical_pull else self.shear_force_n()
        return f / (safety * (G + accel_m_s2))


# ============================================================================== selection
@dataclass(frozen=True)
class TargetObject:
    name: str
    width_m: float              # the dimension the jaws must straddle
    mass_kg: float
    surface: str                # "smooth-rigid" | "textured" | "porous" | "soft" | "irregular"
    flat_face: bool = True      # is there a flat patch >= a cup diameter for suction?
    fragile: bool = False


MU_TABLE: dict[str, float] = {  # rubber pad against ..., measured the way 15.01-E4 measures it
    "smooth-rigid": 0.45,
    "textured": 0.75,
    "porous": 0.70,
    "soft": 0.90,
    "irregular": 0.55,
}


def evaluate(obj: TargetObject, jaw: JawGripper, suction: SuctionGripper) -> dict[str, str]:
    """Verdict per gripper type for one object. Strings, because the answer is a sentence."""
    out: dict[str, str] = {}
    mu = MU_TABLE[obj.surface]

    if not jaw.fits(obj.width_m):
        out["parallel jaw"] = f"no: needs {obj.width_m * 1000:.0f} mm + clearance, stroke is {jaw.stroke_m * 1000:.0f} mm"
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


# ============================================================================== catalogue
def so101_jaw() -> JawGripper:
    """The SO-101 follower's jaw, with the torque limit this course uses (14.11).

    Two numbers here are *placeholders you must replace with measurements of your own arm*
    (exercise 15.02-E1 measures the stroke, 15.02-E3 the force): the published stall torque of the
    Feetech STS3215 depends on the gear variant and the supply voltage, and the printed jaw's pad
    geometry changes with the pads you glue on. 1.0 N*m stall, a 45 mm pad-to-pivot lever and a
    45 mm stroke are conservative round numbers in the right ballpark.
    """
    f = servo_jaw_force(stall_torque_nm=1.0, lever_arm_m=0.045, torque_limit=0.30, efficiency=0.7)
    return JawGripper("SO-101 jaw", stroke_m=0.045, max_force_n=f, pad_mu=0.6,
                      pad_radius_m=0.006, finger_length_m=0.045)


def wide_jaw() -> JawGripper:
    """For comparison: an industrial-class electric parallel gripper (80 mm stroke, 30 N)."""
    return JawGripper("80 mm industrial jaw", stroke_m=0.080, max_force_n=30.0, pad_mu=0.6,
                      pad_radius_m=0.010, finger_length_m=0.08)


def small_suction() -> SuctionGripper:
    """A 20 mm cup on a small 12 V diaphragm pump (about 60 kPa of vacuum)."""
    return SuctionGripper("20 mm cup, 60 kPa", cup_diameter_m=0.020, n_cups=1,
                          gauge_pressure_pa=60_000.0, efficiency=0.7)


OBJECTS: list[TargetObject] = [
    TargetObject("wooden cube 30 mm", 0.030, 0.020, "textured", flat_face=True),
    TargetObject("marker pen", 0.017, 0.012, "smooth-rigid", flat_face=False),
    TargetObject("AA battery", 0.014, 0.024, "smooth-rigid", flat_face=True),
    TargetObject("cardboard box 30 mm", 0.030, 0.060, "porous", flat_face=True),
    TargetObject("golf ball", 0.043, 0.046, "textured", flat_face=False),
    TargetObject("egg", 0.045, 0.060, "smooth-rigid", flat_face=False, fragile=True),
    TargetObject("full 330 ml can", 0.066, 0.345, "smooth-rigid", flat_face=True),
    TargetObject("phone", 0.072, 0.190, "smooth-rigid", flat_face=True),
    TargetObject("sock (bundled)", 0.055, 0.050, "soft", flat_face=False),
]


# ============================================================================== demos
def demo_jaw() -> None:
    print("--- jaw force from servo torque (lever 45 mm, efficiency 0.7) ---")
    print(f"{'stall torque':>16}{'  ':2}{'30% limit N':>12}{'50% limit N':>13}{'100% (never) N':>16}")
    for kgcm in (10.0, 15.0, 20.0, 30.0):
        nm = kgcm_to_nm(kgcm)
        f30 = servo_jaw_force(nm, 0.045, torque_limit=0.30)
        f50 = servo_jaw_force(nm, 0.045, torque_limit=0.50)
        f100 = servo_jaw_force(nm, 0.045, torque_limit=1.00)
        print(f"{kgcm:>10.0f} kg*cm{'':2}{f30:>12.1f}{f50:>13.1f}{f100:>16.1f}")
    print()
    jaw = so101_jaw()
    print(f"{jaw.name}: stroke {jaw.stroke_m * 1000:.0f} mm, {jaw.max_force_n:.1f} N per pad")
    for mu in (0.3, 0.45, 0.6, 0.9):
        print(f"  mu = {mu:4.2f} -> max payload {jaw.max_payload_kg(mu=mu) * 1000:5.0f} g "
              f"(safety 2, 1 m/s^2)")
    print()


def demo_suction() -> None:
    print("--- suction: pull-off force and payload (efficiency 0.7, safety 2, 1 m/s^2) ---")
    print(f"{'cup mm':>8}{'kPa':>6}{'area mm2':>10}{'normal N':>10}{'shear N':>9}{'payload g':>11}")
    for d_mm in (10, 20, 30, 40):
        for kpa in (40, 60, 80):
            s = SuctionGripper("x", d_mm / 1000.0, 1, kpa * 1000.0, 0.7)
            area = math.pi * (d_mm / 2.0) ** 2
            print(f"{d_mm:>8}{kpa:>6}{area:>10.0f}{s.normal_force_n():>10.1f}"
                  f"{s.shear_force_n():>9.1f}{s.max_payload_kg() * 1000:>11.0f}")
    print()
    print("  the same 20 mm / 60 kPa cup, held sideways (shear, not pull-off):")
    s = small_suction()
    print(f"    vertical pull: {s.max_payload_kg() * 1000:.0f} g   "
          f"sideways: {s.max_payload_kg(vertical_pull=False) * 1000:.0f} g")
    print()


def demo_selection() -> None:
    jaw, suction = so101_jaw(), small_suction()
    print(f"--- which gripper? ({jaw.name}, {jaw.max_force_n:.1f} N; {suction.name}) ---")
    for obj in OBJECTS:
        v = evaluate(obj, jaw, suction)
        print(f"{obj.name:<22} ({obj.width_m * 1000:3.0f} mm, {obj.mass_kg * 1000:4.0f} g, {obj.surface})")
        for kind in ("parallel jaw", "suction", "compliant jaw"):
            print(f"    {kind:<15}{v[kind]}")
    print()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["jaw", "suction", "select"])
    args = ap.parse_args(argv)
    if args.only in (None, "jaw"):
        demo_jaw()
    if args.only in (None, "suction"):
        demo_suction()
    if args.only in (None, "select"):
        demo_selection()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
