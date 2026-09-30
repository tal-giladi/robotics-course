"""karmel as a war machine: the mass, centre of gravity, power and stability budget for the weapons.

Lesson 20.02 built the model for the course robot (two power domains, branch sizing, CG and
tipping). This file extends that model with the weapon platform from lesson 21.01:

* **packages** — Scout / Assault / Siege, the three build levels from
  ``21-war-machine/hardware/war-machine-bom.md`` §G;
* **mass** — the CG and both tipping limits for the base robot and each package;
* **power** — the energy cost of firing (it is almost nothing — the weapons tax the robot
  through mass and peak current, not energy);
* **delta** — the ten-row spec delta of lesson 21.01 Level 5, computed from the parts tables;
* **check** — every rule at once; exit 1 if any fails.

    py 21-war-machine/code/war_power_budget.py mass
    py 21-war-machine/code/war_power_budget.py power
    py 21-war-machine/code/war_power_budget.py packages
    py 21-war-machine/code/war_power_budget.py delta
    py 21-war-machine/code/war_power_budget.py sag --current 15
    py 21-war-machine/code/war_power_budget.py check

Every number is either (measured), (datasheet) or (estimate). Replace the estimates with your
own scale and INA226 readings — that is exercise 21.01-E4, and the point of the file.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field

G = 9.81  # m/s^2

# karmel's configured acceleration limit (labs/config/karmel.yaml: limits) and the pack.
ACCEL_LIMIT_M_S2 = 1.0
PACK_NOMINAL_V = 10.8
PACK_AH = 3.5
PACK_WH = PACK_NOMINAL_V * PACK_AH          # 37.8 Wh
USABLE_FRACTION = 0.80

# The course robot's session average power (power_budget_v2.py `runtime` profile): the
# always-on compute domain plus the drive duty. Used to scale the runtime tax of added mass:
# flat-floor drive power is proportional to the mass on the wheels.
BASE_AVERAGE_W = 17.0                        # (course, measured profile)
BASE_DRIVE_W = BASE_AVERAGE_W - 11.1         # 11.1 W compute (course `runtime` output)


# ==============================================================================================
#  Mass model
# ==============================================================================================
@dataclass(frozen=True)
class Part:
    """A mass at a position in ``base_link`` (x forward, y left, z up; origin on the wheel axis)."""

    name: str
    mass_kg: float
    x_m: float
    y_m: float
    z_m: float
    source: str = "estimate"  # measured | datasheet | estimate


def base_parts() -> list[Part]:
    """The course robot exactly as built in module 20 (mirrors karmel_v2_parts())."""
    return [
        Part("chassis plate + motors + wheels", 0.90, 0.000, 0.0, 0.030),
        Part("3S battery pack", 0.45, 0.060, 0.0, 0.035),
        Part("Raspberry Pi 5 + cooler + board", 0.18, 0.010, 0.0, 0.060),
        Part("RPLIDAR C1 on its riser", 0.12, 0.000, 0.0, 0.120),
        Part("camera + mount", 0.06, 0.100, 0.0, 0.100),
        Part("e-stop box + relay + harness", 0.15, -0.060, 0.0, 0.070),
        Part("arm base plate", 0.10, 0.030, 0.0, 0.050),
        Part("SO-101 arm, folded", 0.70, 0.040, 0.0, 0.140),
    ]


# The weapon platform, split so the packages compose. Positions are the rail layout of
# 21.01: rail x = +20…+110 mm, everything below deck 2, payloads low.
WEAPON_PARTS: dict[str, list[Part]] = {
    "scout": [
        Part("weapon rail 300 mm PETG", 0.25, 0.065, 0.0, 0.045),
        Part("rail hardware (M3 kit, standoffs)", 0.20, 0.050, 0.0, 0.045),
        Part("drop bin + 2x MG996R latches", 0.35, 0.085, 0.0, 0.055),
        Part("payload rack, 3 slots loaded ~0.15 kg each", 0.45, 0.075, 0.0, 0.060),
        Part("slingshot frame + rubber bands", 0.15, 0.070, 0.0, 0.050),
        Part("12 V boost tap + fuses + wiring", 0.10, 0.030, 0.0, 0.045),
    ],
    "assault": [
        Part("cannon, 120 N linear actuator + mount", 0.80, 0.090, 0.0, 0.055),
        Part("CYMA CO2 magazine + 12 g cartridge", 0.55, 0.060, 0.0, 0.070),
    ],
    "siege": [
        Part("AGF heavy-throw arm + counterweight", 0.90, 0.080, 0.0, 0.080),
        Part("IR beacon + 95 g CO2 + latching solenoids", 0.05, 0.050, 0.0, 0.050),
    ],
}

PACKAGES: dict[str, list[str]] = {
    "base": [],
    "scout": ["scout"],
    "assault": ["scout", "assault"],
    "siege": ["scout", "assault", "siege"],
}


def parts_for(package: str) -> list[Part]:
    if package not in PACKAGES:
        raise KeyError(f"unknown package {package!r}; expected one of {sorted(PACKAGES)}")
    parts = base_parts()
    for group in PACKAGES[package]:
        parts += WEAPON_PARTS[group]
    return parts


# The three ground contacts of karmel: two drive wheels on the axle, one ball caster 100 mm in
# FRONT of them (labs/config/karmel.yaml: chassis.caster_offset_x_m = 0.10).
CONTACTS: tuple[tuple[float, float], ...] = ((0.0, 0.100), (0.0, -0.100), (0.100, 0.0))
CASTER_X_M = 0.100


@dataclass(frozen=True)
class CentreOfGravity:
    mass_kg: float
    x_m: float
    y_m: float
    z_m: float


def centre_of_gravity(parts: list[Part]) -> CentreOfGravity:
    """The mass-weighted mean position: $x_{cg} = \\sum m_i x_i / \\sum m_i$."""
    m = sum(p.mass_kg for p in parts)
    return CentreOfGravity(
        m,
        sum(p.mass_kg * p.x_m for p in parts) / m,
        sum(p.mass_kg * p.y_m for p in parts) / m,
        sum(p.mass_kg * p.z_m for p in parts) / m,
    )


def cg_shift(base_mass: float, base_cgx: float, base_cgz: float,
             added: list[tuple[float, float, float]]) -> tuple[float, float, float]:
    """Where the CG moves when you add masses: (new_mass, new_x, new_z).

    ``added`` is a list of (mass_kg, x_m, z_m). Exercise 21.01-E2 implements this for you to
    write; the model just calls it.
    """
    m = base_mass + sum(a[0] for a in added)
    x = (base_mass * base_cgx + sum(a[0] * a[1] for a in added)) / m
    z = (base_mass * base_cgz + sum(a[0] * a[2] for a in added)) / m
    return m, x, z


def tipping_accel_m_s2(cg: CentreOfGravity) -> float:
    """Forward acceleration that lifts the caster and sits the robot back on its tail.

    The caster is 100 mm in FRONT of the axle, so the unsupported end is the rear: the robot
    pivots about the wheel axle (x = 0) when $m a z_{cg} > m g x_{cg}$, i.e.
    $a_{tip} = g\\,x_{cg}/z_{cg}$. A CG at or behind the axle gives $a_{tip} \\le 0$: already
    resting on its tail.
    """
    if cg.x_m <= 0.0:
        return 0.0
    return G * cg.x_m / cg.z_m


def tipping_decel_m_s2(cg: CentreOfGravity) -> float:
    """Braking deceleration that walks the CG past the caster and puts the robot on its nose."""
    if cg.x_m >= CASTER_X_M:
        return 0.0
    return G * (CASTER_X_M - cg.x_m) / cg.z_m


def stability_margin_m(cg: CentreOfGravity) -> float:
    """Shortest distance from the CG's ground projection to the support triangle's edge.

    Negative means the CG is outside the contacts — the robot rests on something that is not a
    wheel. (Same construction as power_budget_v2.py; the triangle is tiny, so the answer is
    basically the rear edge distance for a forward CG.)
    """
    poly = list(CONTACTS)
    best = float("inf")
    inside = True
    for i, (ax, ay) in enumerate(poly):
        bx, by = poly[(i + 1) % len(poly)]
        ex, ey = bx - ax, by - ay
        cross = ex * (cg.y_m - ay) - ey * (cg.x_m - ax)
        length = (ex * ex + ey * ey) ** 0.5
        inside &= cross <= 0.0 if _clockwise(poly) else cross >= 0.0
        best = min(best, abs(cross) / length)
    return best if inside else -best


def _clockwise(poly: list[tuple[float, float]]) -> bool:
    area = 0.0
    for i, (ax, ay) in enumerate(poly):
        bx, by = poly[(i + 1) % len(poly)]
        area += ax * by - bx * ay
    return area < 0.0


# ==============================================================================================
#  Power: the energy cost of firing (and why it is not the budget)
# ==============================================================================================
@dataclass(frozen=True)
class Shot:
    """One launch event: mechanical energy delivered, and the electrical peak it draws."""

    name: str
    energy_j: float
    peak_a: float
    pulse_s: float
    source: str = "estimate"

    @property
    def energy_wh(self) -> float:
        return self.energy_j / 3600.0


SHOTS = [
    Shot("cannon shot (120 N x 0.10 m)", 12.0, 5.5, 0.5),
    Shot("solenoid pin pulse (85 N x 0.015 m + I^2R)", 2.0, 1.5, 0.3),
    Shot("slingshot shot (2 bands, 120 mm stretch)", 4.0, 0.3, 0.2),
    Shot("CYMA trigger shot (servo; CO2 is chemical)", 0.5, 2.0, 0.2),
]


def firing_cost(n_shots: dict[str, int]) -> dict[str, float]:
    """Wh consumed by a firing plan, as a fraction of the pack."""
    total_j = sum(s.energy_j * n_shots.get(s.name, 0) for s in SHOTS)
    return {
        "energy_j": total_j,
        "energy_wh": total_j / 3600.0,
        "pack_fraction_pct": 100.0 * total_j / 3600.0 / PACK_WH,
    }


def runtime_min(package: str) -> float:
    """Session minutes for a package: compute power is fixed, drive power scales with mass.

    Flat-floor rolling drag is proportional to the mass on the wheels, so the drive part of the
    session average scales by (mass / base mass). Crude, but the right order — measure yours.
    """
    mass = centre_of_gravity(parts_for(package)).mass_kg
    scale = mass / centre_of_gravity(base_parts()).mass_kg
    avg_w = (BASE_AVERAGE_W - BASE_DRIVE_W) + BASE_DRIVE_W * scale
    return 60.0 * PACK_WH * USABLE_FRACTION / avg_w


# ==============================================================================================
#  The spec delta (21.01 Level 5)
# ==============================================================================================
def spec_delta(package: str = "assault") -> list[tuple[str, str, str, str]]:
    """The ten-row delta table, computed. Returns (row, base, package, why) tuples."""
    base = centre_of_gravity(base_parts())
    war = centre_of_gravity(parts_for(package))
    weapon_kg = war.mass_kg - base.mass_kg
    base_rt = runtime_min("base")
    war_rt = runtime_min(package)
    return [
        ("mass", f"{base.mass_kg:.2f} kg", f"{war.mass_kg:.2f} kg (estimate)",
         f"+{weapon_kg:.2f} kg weapon platform + loaded payloads"),
        ("centre of gravity", f"x {base.x_m * 1000:+.0f} mm, z {base.z_m * 1000:.0f} mm",
         f"x {war.x_m * 1000:+.0f} mm, z {war.z_m * 1000:.0f} mm (estimate)",
         "weapons low and forward on the rail"),
        ("tipping (rear / nose)", f"{tipping_accel_m_s2(base):.2f} / {tipping_decel_m_s2(base):.2f} m/s^2",
         f"{tipping_accel_m_s2(war):.2f} / {tipping_decel_m_s2(war):.2f} m/s^2",
         "forward+low mass: rearing limit up, braking limit down"),
        ("switched rail peak", "10.1 A, one 20 A relay",
         f"10.1 A on the 20 A relay + up to {BUS_PEAKS_A[package]:.1f} A on the 30 A relay",
         "weapons on their own relay behind the same e-stop"),
        ("runtime", f"{base_rt:.0f} min", f"{war_rt:.0f} min (estimate)",
         "flat-floor drive power scales with the added mass"),
        ("LiDAR plane", "chassis furniture only", "rail top must stay < z 0.10 m or > 0.14 m",
         "new furniture in the 360 deg scan"),
        ("camera cone", "arm parked out of frame", "weapons parked out of the 1.20 rad cone",
         "a cannon in every frame breaks the detector"),
        ("e-stop removes", "actuator energy", "actuator AND weapon energy (two relays)",
         "the weapon bus is a new switched domain"),
        ("software states", "driving / stopped", "+ SAFE/LOADED/ARMED/FIRED/JAMMED per weapon",
         "weapons are state machines; e-stop is the hard reset"),
        ("power hardware", "10 A fuse, 20 A relay", "+ 30 A relay, 5 A/2 A fuses, 12 V boost tap",
         "peak current budget and the V^2 solenoid problem"),
    ]


# ==============================================================================================
#  Voltage sag at launch current (21.08 Level 2)
# ==============================================================================================
# The pack is a Thevenin source: V_load = V_oc - I * (R_internal + R_branch).
# Samsung 35E: ~8 mΩ per cell at 1C (datasheet) -> ~24 mΩ in series for the 3S pack (estimate).
PACK_R_INT_OHM = 0.024
FUSE_R_OHM = 0.005            # blade fuse
RELAY_R_OHM = 0.020           # relay contacts (estimate; measure yours)
CONNECTOR_R_OHM = 0.010       # XT60 + solder joints
COPPER_R_PER_M = {18: 0.0071, 16: 0.0040, 14: 0.0025}   # Ω/m, (datasheet)


def wire_resistance_ohm(awg: int, length_m: float) -> float:
    """Round-trip branch wiring. ``length_m`` is the TOTAL wire length (out + return)."""
    if awg not in COPPER_R_PER_M:
        raise KeyError(f"awg {awg!r}; expected one of {sorted(COPPER_R_PER_M)}")
    return COPPER_R_PER_M[awg] * length_m


def branch_resistance_ohm(awg: int = 18, wire_length_m: float = 0.6) -> float:
    """Fuse + relay + connector + wiring: the whole path from pack to the weapon bus."""
    return (FUSE_R_OHM + RELAY_R_OHM + CONNECTOR_R_OHM
            + wire_resistance_ohm(awg, wire_length_m))


def voltage_at_load(v_oc: float, current_a: float, r_total_ohm: float) -> float:
    """Thevenin: what the load actually sees while the current flows."""
    return v_oc - current_a * r_total_ohm


def solenoid_force_fraction(v: float, rated_v: float = 12.0) -> float:
    """The V^2 law as a fraction of rated solenoid force (21.01 Level 2)."""
    return (v / rated_v) ** 2


# ==============================================================================================
#  check: every rule at once
# ==============================================================================================
def check(package: str = "assault") -> list[str]:
    problems: list[str] = []
    parts = parts_for(package)
    cg = centre_of_gravity(parts)

    if stability_margin_m(cg) <= 0.0:
        problems.append(f"CG outside the support polygon (margin {stability_margin_m(cg) * 1000:+.0f} mm)")
    if cg.x_m <= 0.0:
        problems.append("CG at or behind the wheel axle: the robot rests on its tail, stationary")

    a_rear = tipping_accel_m_s2(cg)
    a_brake = tipping_decel_m_s2(cg)
    if a_rear < 2 * ACCEL_LIMIT_M_S2:
        problems.append(f"rearing limit {a_rear:.2f} m/s^2 < 2x the {ACCEL_LIMIT_M_S2} m/s^2 drive limit")
    if a_brake < 2 * ACCEL_LIMIT_M_S2:
        problems.append(f"braking limit {a_brake:.2f} m/s^2 < 2x the {ACCEL_LIMIT_M_S2} m/s^2 brake limit")

    # Weapon bus peak (every branch of this package at once) vs the 30 A relay.
    if package in BUS_PEAKS_A:
        peak = BUS_PEAKS_A[package]
        if peak > 0.9 * 30.0:
            problems.append(f"weapon bus peak {peak:.1f} A exceeds the 30 A relay's 90 % margin")

    # The rail must stay out of the LiDAR scan plane (z = 0.12 +/- 0.02 m).
    rail_top = 0.080  # rail 20 mm + standoffs 8 mm + mount 52 mm -> top of a parked weapon
    if 0.10 <= rail_top <= 0.14:
        problems.append(f"rail top at z = {rail_top:.3f} m is inside the LiDAR scan plane")

    rt = runtime_min(package)
    if rt < 30.0:
        problems.append(f"runtime {rt:.0f} min < 30 min: the weapons ate the session")

    for p in parts:
        if p.source == "estimate" and p.mass_kg >= 0.2:
            problems.append(f"heavy part still (estimate): {p.name} ({p.mass_kg:.2f} kg) -> weigh it")
    return problems


# ==============================================================================================
#  CLI
# ==============================================================================================
def _print_mass() -> None:
    for pkg in ("base", "scout", "assault", "siege"):
        cg = centre_of_gravity(parts_for(pkg))
        print(f"{pkg}")
        print(f"  total mass {cg.mass_kg:.2f} kg   CG x {cg.x_m * 1000:+.0f} mm  z {cg.z_m * 1000:.0f} mm")
        print(f"  margin {stability_margin_m(cg) * 1000:+.0f} mm   "
              f"rears up at {tipping_accel_m_s2(cg):.2f} m/s^2   "
              f"noses over at {tipping_decel_m_s2(cg):.2f} m/s^2")
        print(f"  runtime {runtime_min(pkg):.0f} min")
        print()


def _print_power() -> None:
    for s in SHOTS:
        print(f"{s.name:<42} {s.energy_j:6.1f} J   {s.energy_wh * 1000:8.2f} mWh   peak {s.peak_a:4.1f} A")
    plan = firing_cost({"cannon shot (120 N x 0.10 m)": 100})
    print()
    print(f"100 cannon shots: {plan['energy_j']:.0f} J = {plan['energy_wh']:.2f} Wh "
          f"= {plan['pack_fraction_pct']:.1f} % of the {PACK_WH:.1f} Wh pack")
    print("verdict: firing costs ~1 % of the pack per 100 shots.")
    print("        the weapons tax the robot through MASS (wheels carry it) and PEAK CURRENT (wires carry it),")
    print("        not through energy.")


# Worst-case simultaneous draw on the 30 A weapon bus, per package — cumulative, because each
# package builds on the previous one (BOM §F branch values). Servo entries are stall currents;
# solenoid entries are the 12 V boost branch. The AGF arm (Siege) runs on its own battery, so
# it is not on this bus.
BUS_PEAKS_A: dict[str, float] = {
    "scout": 3.0 + 0.3 + 2.0,                              # 2x MG996R latch (stall) + slingshot trigger + stink-bomb valve (held)
    "assault": 3.0 + 0.3 + 2.0 + 5.5 + 2.0 + 0.5,          # + cannon actuator + CYMA servo (stall) + CO2 valve
    "siege": 3.0 + 0.3 + 2.0 + 5.5 + 2.0 + 0.5 + 1.5 + 1.0 # + latching solenoid + IR beacon burst
}


def _print_packages() -> None:
    base = centre_of_gravity(base_parts())
    for pkg in ("scout", "assault", "siege"):
        cg = centre_of_gravity(parts_for(pkg))
        added = cg.mass_kg - base.mass_kg
        print(f"{pkg:<8} added {added:5.2f} kg   total {cg.mass_kg:5.2f} kg   "
              f"peak bus ~ {BUS_PEAKS_A[pkg]:3.1f} A   runtime {runtime_min(pkg):.0f} min")


def _print_delta(package: str = "assault") -> None:
    print(f"spec delta: base (module 20) -> {package}")
    for row, old, new, why in spec_delta(package):
        print(f"  {row:<18} {old:<28} {new}")
        print(f"  {'':<18} why: {why}")


def _print_sag(current_a: float, awg: int, wire_length_m: float) -> None:
    r_branch = branch_resistance_ohm(awg, wire_length_m)
    r_total = PACK_R_INT_OHM + r_branch
    print(f"voltage sag at I = {current_a:.1f} A (pack -> weapon bus path)")
    print(f"  pack internal R        {PACK_R_INT_OHM * 1000:.0f} mohm   (3S 35E, estimate)")
    print(f"  branch R               {r_branch * 1000:.0f} mohm   (fuse {FUSE_R_OHM * 1000:.0f} + relay "
          f"{RELAY_R_OHM * 1000:.0f} + connector {CONNECTOR_R_OHM * 1000:.0f} + "
          f"{awg} AWG {wire_length_m:.1f} m = {wire_resistance_ohm(awg, wire_length_m) * 1000:.1f} mohm)")
    print(f"  total R                {r_total * 1000:.0f} mohm")
    for label, v_oc in (("full charge", 12.6), ("empty pack", 9.9)):
        v_load = voltage_at_load(v_oc, current_a, r_total)
        print(f"  at {label:<12} ({v_oc:.1f} V): sag {v_oc - v_load:5.2f} V   load sees {v_load:5.2f} V")
    v_empty_load = voltage_at_load(9.9, current_a, r_total)
    print()
    print(f"  a 12 V solenoid on the RAW rail at empty pack: {solenoid_force_fraction(v_empty_load) * 100:.0f}% of rated force (V^2)")
    print("  the same solenoid on the 12.0 V boost tap: 100% at every pack voltage -")
    print("  the boost buys back the V^2 loss; servos and the actuator just run a bit slower.")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("command", choices=("mass", "power", "packages", "delta", "sag", "check"))
    ap.add_argument("--package", default="assault", choices=sorted(PACKAGES),
                    help="which build level (delta/check only)")
    ap.add_argument("--current", type=float, default=15.0, help="launch current in A (sag)")
    ap.add_argument("--awg", type=int, default=18, choices=sorted(COPPER_R_PER_M), help="branch wire gauge (sag)")
    ap.add_argument("--wire-m", type=float, default=0.6, help="total branch wire length in m (sag)")
    args = ap.parse_args(argv)

    if args.command == "mass":
        _print_mass()
    elif args.command == "power":
        _print_power()
    elif args.command == "packages":
        _print_packages()
    elif args.command == "delta":
        _print_delta(args.package)
    elif args.command == "sag":
        _print_sag(args.current, args.awg, args.wire_m)
    elif args.command == "check":
        problems = check(args.package)
        if problems:
            print(f"check ({args.package}): {len(problems)} problem(s)")
            for p in problems:
                print(f"  PROBLEM  {p}")
            return 1
        print(f"check ({args.package}): all rules pass")
    return 0


if __name__ == "__main__":
    sys.exit(main())
