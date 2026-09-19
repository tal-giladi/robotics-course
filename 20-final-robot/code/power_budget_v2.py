"""karmel v2: the integrated robot's power, wiring and mass budget — as numbers you can check.

Lesson 02.02 budgeted a base with two motors and a Pi. The final robot adds an arm, an e-stop
relay, a second switched domain and (optionally) a Jetson, and it has to survive being carried,
braked and driven into a chair leg. This file holds the whole thing:

* **two power domains** — COMPUTE (always on, so the robot can log and shut down cleanly) and
  ACTUATOR (behind the e-stop relay: motors and arm servos);
* **branches** — every wire that carries real current, with gauge, length, voltage drop and the
  fuse that protects it;
* **runtime** from a duty-cycle profile instead of one optimistic average;
* **mass and centre of gravity**, because an arm bolted to the front of a 1.6 kg robot tips it.

    py 20-final-robot/code/power_budget_v2.py rails        # current per domain and rail
    py 20-final-robot/code/power_budget_v2.py branches     # wire gauge, drop, fuse, relay
    py 20-final-robot/code/power_budget_v2.py runtime      # session profile -> minutes
    py 20-final-robot/code/power_budget_v2.py mass         # CG, support polygon, tipping
    py 20-final-robot/code/power_budget_v2.py check        # every rule; exit 1 if any fails
    py 20-final-robot/code/power_budget_v2.py whatif arm-forward   # the arm mounted at the front
    py 20-final-robot/code/power_budget_v2.py whatif jetson        # Jetson Orin Nano added

Every number is either (datasheet), (measured) or (estimate). Replace the estimates with your own
INA226 measurements — that is exercise 20.02-E4, and the point of the file.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, replace

G = 9.81  # m/s^2

COMPUTE, ACTUATOR = "compute", "actuator"

# Copper resistance at 20 C, ohm per metre (PowerStream AWG table).
AWG_OHM_PER_M: dict[int, float] = {
    22: 0.05296, 20: 0.03331, 18: 0.02095, 16: 0.01318, 14: 0.008286, 12: 0.005211,
}
# Chassis wiring ampacity (conservative, bundled, 60 C insulation). Not the free-air rating.
AWG_AMPACITY_A: dict[int, float] = {22: 3.0, 20: 5.0, 18: 8.0, 16: 13.0, 14: 20.0, 12: 30.0}
BLADE_FUSE_A = (3.0, 5.0, 7.5, 10.0, 15.0, 20.0, 25.0, 30.0)


# ==============================================================================================
#  Loads
# ==============================================================================================
@dataclass(frozen=True)
class Load:
    """One consumer. ``peak_a`` is the worst case the wiring and the fuse must survive."""

    name: str
    domain: str          # COMPUTE | ACTUATOR
    rail_v: float        # the voltage it is fed at
    typical_a: float
    peak_a: float
    duty: float = 1.0    # fraction of a session it draws typical_a (used by runtime)
    note: str = ""
    tag: str = "sensor"  # drive | arm | compute | sensor — what kind of thing this is

    @property
    def is_actuator(self) -> bool:
        """Does this load MOVE something? Those must sit behind the e-stop relay."""
        return self.tag in ("drive", "arm")

    @property
    def typical_w(self) -> float:
        return self.rail_v * self.typical_a

    @property
    def peak_w(self) -> float:
        return self.rail_v * self.peak_a


def karmel_v2_loads() -> list[Load]:
    """The final robot: base + LiDAR + camera + IMU + 6-servo arm + e-stop relay."""
    return [
        # ---- COMPUTE domain: 5 V from the D42V55F5, always powered ---------------------------
        Load("Raspberry Pi 5 (8 GB), full ROS 2 stack", COMPUTE, 5.0, 1.30, 2.00, 1.0,
             "0.8 A idle - 2.0 A with Nav2 + camera (estimate)", "compute"),
        Load("Active Cooler fan", COMPUTE, 5.0, 0.05, 0.10, 1.0, "estimate", "compute"),
        Load("RPLIDAR C1 (spin motor + scanner)", COMPUTE, 5.0, 0.30, 0.40, 1.0,
             "Slamtec C1: 300 mA typical, 400 mA start (datasheet)", "sensor"),
        Load("USB webcam C920", COMPUTE, 5.0, 0.20, 0.35, 1.0, "estimate, MJPEG 640x480@15", "sensor"),
        Load("Pico 2 + 3.3 V sensors (via USB)", COMPUTE, 5.0, 0.07, 0.11, 1.0,
             "3.3 V rail of 02.02, converted at ~90 %", "compute"),
        Load("INA226 + e-stop relay coil", COMPUTE, 5.0, 0.08, 0.09, 1.0,
             "coil ~70 mA at 5 V (estimate), fed THROUGH the e-stop NC contact", "compute"),

        # ---- ACTUATOR domain: pack voltage, behind the e-stop relay ----------------------------
        Load("2x drive motors (Yahboom 520, 1:56)", ACTUATOR, 10.8, 1.00, 5.90, 0.35,
             "0.5 A each cruising (estimate); 2.95 A each = DRV8874 ITRIP (datasheet)", "drive"),
        Load("6x arm servos (SO-101, STS3215 class)", ACTUATOR, 10.8, 0.60, 4.20, 0.20,
             "0.1 A each holding, 0.7 A each stalled (estimate; measure yours)", "arm"),
    ]


# ==============================================================================================
#  Branches: the wires that carry it
# ==============================================================================================
@dataclass(frozen=True)
class Branch:
    """One current-carrying run. ``length_m`` is one way; drop counts the return conductor too."""

    name: str
    domain: str
    source_v: float
    continuous_a: float   # the largest current this branch can carry for more than a second or two
    peak_a: float         # worst-case transient (stall, inrush) — sizes the voltage drop
    awg: int
    length_m: float
    fuse_a: float | None = None
    max_drop_pct: float = 5.0

    @property
    def resistance_ohm(self) -> float:
        return 2.0 * self.length_m * AWG_OHM_PER_M[self.awg]   # out and back

    @property
    def drop_v(self) -> float:
        return self.peak_a * self.resistance_ohm

    @property
    def drop_pct(self) -> float:
        return 100.0 * self.drop_v / self.source_v

    @property
    def heat_w(self) -> float:
        return self.peak_a ** 2 * self.resistance_ohm


def fuse_for(continuous_a: float, awg: int) -> float:
    """Smallest standard blade fuse at or above 1.25x continuous that the wire can still protect.

    The rule has two ends: a fuse below 1.25x the continuous current nuisance-blows, and a fuse
    above the wire's ampacity protects nothing — the wire becomes the fuse.
    """
    target = 1.25 * continuous_a
    for value in BLADE_FUSE_A:
        if value >= target:
            if value > AWG_ampacity(awg):
                raise ValueError(f"{value:g} A fuse exceeds the {awg} AWG ampacity "
                                 f"({AWG_ampacity(awg):g} A) — use thicker wire")
            return value
    raise ValueError(f"no standard blade fuse >= {target:.1f} A")


def AWG_ampacity(awg: int) -> float:  # noqa: N802 - reads like the table it wraps
    return AWG_AMPACITY_A[awg]


def regulator_input_a(loads: list[Load], pack_v: float = 9.9, efficiency: float = 0.90) -> float:
    """A switching regulator is a constant-POWER load: the *lowest* pack voltage draws the most."""
    return sum(x.peak_w for x in loads if x.domain == COMPUTE) / (efficiency * pack_v)


def karmel_v2_branches(loads: list[Load] | None = None) -> list[Branch]:
    """The four branches that carry real current, sized from the load table.

    ``continuous_a`` is what the branch can carry for seconds (the drive branch's continuous case
    is "both motors pushing a wall at the DRV8874 current limit"); ``peak_a`` is the transient.
    """
    loads = loads or karmel_v2_loads()
    drive = sum(x.peak_a for x in loads if x.tag == "drive")
    arm = sum(x.peak_a for x in loads if x.tag == "arm")
    reg = regulator_input_a(loads)
    return [
        Branch("pack -> fuse -> main switch (everything)", ACTUATOR, 10.8,
               continuous_a=round(drive + reg, 2), peak_a=round(drive + arm + reg, 2),
               awg=16, length_m=0.15),
        Branch("main switch -> 5 V regulator (compute)", COMPUTE, 10.8,
               continuous_a=round(reg, 2), peak_a=round(reg * 1.3, 2), awg=18, length_m=0.20),
        Branch("e-stop relay -> motor drivers", ACTUATOR, 10.8,
               continuous_a=round(drive, 2), peak_a=round(drive * 1.5, 2), awg=18, length_m=0.25),
        Branch("e-stop relay -> arm servo bus", ACTUATOR, 10.8,
               continuous_a=round(arm, 2), peak_a=round(arm * 1.3, 2), awg=18, length_m=0.45),
    ]


# ==============================================================================================
#  Mass and centre of gravity
# ==============================================================================================
@dataclass(frozen=True)
class Part:
    """A mass at a position in ``base_link`` (x forward, y left, z up; origin on the wheel axis)."""

    name: str
    mass_kg: float
    x_m: float
    y_m: float
    z_m: float


def karmel_v2_parts() -> list[Part]:
    """Where the mass is. Weigh yours on a kitchen scale; these are the course build (estimate)."""
    return [
        Part("chassis plate + motors + wheels", 0.90, -0.010, 0.0, 0.030),
        Part("3S battery pack", 0.45, -0.060, 0.0, 0.035),
        Part("Raspberry Pi 5 + cooler + board", 0.18, -0.030, 0.0, 0.060),
        Part("RPLIDAR C1 on its riser", 0.12, 0.000, 0.0, 0.120),
        Part("camera + mount", 0.06, 0.100, 0.0, 0.100),
        Part("e-stop box + relay + harness", 0.15, -0.080, 0.0, 0.070),
        Part("arm base plate", 0.10, -0.040, 0.0, 0.050),
        Part("SO-101 arm, folded", 0.70, -0.030, 0.0, 0.140),
    ]


# The three ground contacts of karmel: two drive wheels on the axle, one ball caster behind.
CONTACTS: tuple[tuple[float, float], ...] = ((0.0, 0.100), (0.0, -0.100), (-0.100, 0.0))


@dataclass(frozen=True)
class CentreOfGravity:
    mass_kg: float
    x_m: float
    y_m: float
    z_m: float


def centre_of_gravity(parts: list[Part]) -> CentreOfGravity:
    """The mass-weighted mean position. $x_{cg} = \\sum m_i x_i / \\sum m_i$."""
    m = sum(p.mass_kg for p in parts)
    return CentreOfGravity(
        m,
        sum(p.mass_kg * p.x_m for p in parts) / m,
        sum(p.mass_kg * p.y_m for p in parts) / m,
        sum(p.mass_kg * p.z_m for p in parts) / m,
    )


def tipping_decel_m_s2(cg: CentreOfGravity) -> float:
    """Braking deceleration that lifts the caster and puts the robot on its nose.

    Inertia acts forward at the CG height, gravity acts down at the CG. The robot pivots about
    the wheel axle (x = 0) when $m a z_{cg} > m g |x_{cg}|$, so $a_{tip} = g |x_{cg}| / z_{cg}$.
    A CG at or in front of the axle means $a_{tip} \\le 0$: it tips at any braking at all.
    """
    if cg.x_m >= 0.0:
        return 0.0
    return G * abs(cg.x_m) / cg.z_m


def stability_margin_m(cg: CentreOfGravity) -> float:
    """Shortest distance from the CG's ground projection to the edge of the support triangle.

    Negative means the CG is outside the contacts — the robot is already resting on something
    that is not a wheel.
    """
    poly = list(CONTACTS)
    best = float("inf")
    inside = True
    for i, (ax, ay) in enumerate(poly):
        bx, by = poly[(i + 1) % len(poly)]
        ex, ey = bx - ax, by - ay
        # signed area of (a, b, cg) -> which side of the edge the CG is on
        cross = ex * (cg.y_m - ay) - ey * (cg.x_m - ax)
        length = (ex * ex + ey * ey) ** 0.5
        distance = abs(cross) / length
        inside &= cross <= 0.0 if _clockwise(poly) else cross >= 0.0
        best = min(best, distance)
    return best if inside else -best


def _clockwise(poly: list[tuple[float, float]]) -> bool:
    area = 0.0
    for i, (ax, ay) in enumerate(poly):
        bx, by = poly[(i + 1) % len(poly)]
        area += ax * by - bx * ay
    return area < 0.0


# ==============================================================================================
#  Runtime
# ==============================================================================================
def session_power_w(loads: list[Load]) -> float:
    """Average power at the pack over a session, using each load's duty and 90 % regulator efficiency."""
    total = 0.0
    for load in loads:
        watts = load.typical_w * load.duty
        total += watts / 0.90 if load.domain == COMPUTE else watts
    return total


def runtime_min(capacity_ah: float, nominal_v: float, avg_w: float, usable: float = 0.80) -> float:
    """Minutes from a pack, keeping ``1 - usable`` in reserve (you stop at the cutoff, not at empty)."""
    return 60.0 * capacity_ah * nominal_v * usable / avg_w


# ==============================================================================================
#  The rules
# ==============================================================================================
def check(loads: list[Load], branches: list[Branch], parts: list[Part]) -> list[str]:
    """Everything the integrated robot's power and mass must satisfy. Empty list = it is sound."""
    problems: list[str] = []

    rail_5v_peak = sum(x.peak_a for x in loads if x.domain == COMPUTE and x.rail_v == 5.0)
    if rail_5v_peak > 5.5:
        problems.append(f"5 V rail peaks at {rail_5v_peak:.2f} A, above the D42V55F5's 5.5 A")

    for load in loads:
        if load.is_actuator and load.domain != ACTUATOR:
            problems.append(f"{load.name}: an actuator on the always-on domain — the e-stop cannot cut it")

    for b in branches:
        if b.drop_pct > b.max_drop_pct:
            problems.append(f"{b.name}: {b.drop_v:.2f} V drop at {b.peak_a:.1f} A "
                            f"({b.drop_pct:.1f} %, limit {b.max_drop_pct:.0f} %) — thicker wire or a shorter run")
        if b.continuous_a > AWG_ampacity(b.awg):
            problems.append(f"{b.name}: {b.continuous_a:.1f} A continuous through {b.awg} AWG "
                            f"(ampacity {AWG_ampacity(b.awg):g} A)")
        try:
            fuse = b.fuse_a if b.fuse_a is not None else fuse_for(b.continuous_a, b.awg)
        except ValueError as exc:
            problems.append(f"{b.name}: {exc}")
            continue
        if fuse > AWG_ampacity(b.awg):
            problems.append(f"{b.name}: {fuse:g} A fuse on {b.awg} AWG — the wire is the fuse")

    cg = centre_of_gravity(parts)
    if cg.x_m >= 0.0:
        problems.append(f"centre of gravity is {cg.x_m * 1000:.0f} mm forward of the wheel axle: "
                        "nothing supports the robot in front — it rests on its nose")
    elif tipping_decel_m_s2(cg) < 2.0 * 1.0:   # 2x karmel's 1.0 m/s^2 deceleration limit
        problems.append(f"tips forward at {tipping_decel_m_s2(cg):.1f} m/s^2, less than 2x the "
                        f"configured 1.0 m/s^2 braking limit — move mass back or down")
    if stability_margin_m(cg) < 0.020:
        problems.append(f"stability margin {stability_margin_m(cg) * 1000:.0f} mm (want >= 20 mm)")
    return problems


# ==============================================================================================
#  Scenarios
# ==============================================================================================
def whatif(name: str) -> tuple[str, list[Load], list[Part]]:
    loads, parts = karmel_v2_loads(), karmel_v2_parts()
    if name == "arm-forward":
        parts = [replace(p, x_m=0.085, z_m=0.155) if "SO-101" in p.name else p for p in parts]
        return "The arm bolted to the FRONT deck instead of behind the wheel axle", loads, parts
    if name == "arm-extended":
        parts = [replace(p, x_m=0.160, z_m=0.180) if "SO-101" in p.name else p for p in parts]
        return "The arm reaching forward at full extension (payload in the gripper)", loads, parts
    if name == "jetson":
        loads = loads + [Load("Jetson Orin Nano Super (15 W mode)", COMPUTE, 5.0, 2.40, 3.00, 1.0,
                              "15 W module power / 5 V, plus board overhead (estimate)")]
        parts = parts + [Part("Jetson + carrier + fan", 0.30, -0.050, 0.0, 0.075)]
        return "Jetson Orin Nano added to the compute deck", loads, parts
    if name == "no-arm":
        parts = [p for p in parts if "SO-101" not in p.name and "arm base" not in p.name]
        loads = [x for x in loads if "servo" not in x.name]
        return "Base only: no arm fitted", loads, parts
    raise SystemExit(f"unknown scenario '{name}' (try: arm-forward, arm-extended, jetson, no-arm)")


# ==============================================================================================
#  Output
# ==============================================================================================
def print_rails(loads: list[Load]) -> None:
    for domain in (COMPUTE, ACTUATOR):
        rows = [x for x in loads if x.domain == domain]
        if not rows:
            continue
        label = "COMPUTE (always on: logs, shuts down cleanly)" if domain == COMPUTE \
            else "ACTUATOR (behind the e-stop relay)"
        print(f"\n{label}")
        print(f"  {'load':46} {'rail':>5} {'typ A':>7} {'peak A':>7} {'peak W':>7}  note")
        for x in rows:
            print(f"  {x.name:46} {x.rail_v:>4.1f}V {x.typical_a:>7.2f} {x.peak_a:>7.2f} "
                  f"{x.peak_w:>7.1f}  {x.note}")
        print(f"  {'TOTAL':46} {'':>5} {sum(x.typical_a for x in rows):>7.2f} "
              f"{sum(x.peak_a for x in rows):>7.2f} {sum(x.peak_w for x in rows):>7.1f}")


def print_branches(branches: list[Branch]) -> None:
    print(f"{'branch':44} {'AWG':>4} {'m':>5} {'cont A':>7} {'peak A':>7} {'drop V':>7} "
          f"{'drop %':>7} {'heat W':>7} {'fuse':>6}")
    for b in branches:
        try:
            fuse = f"{b.fuse_a if b.fuse_a is not None else fuse_for(b.continuous_a, b.awg):4.1f}A"
        except ValueError:
            fuse = "  none"
        print(f"{b.name:44} {b.awg:>4} {b.length_m:>5.2f} {b.continuous_a:>7.2f} {b.peak_a:>7.2f} "
              f"{b.drop_v:>7.3f} {b.drop_pct:>7.2f} {b.heat_w:>7.2f} {fuse:>6}")


def print_mass(parts: list[Part]) -> None:
    print(f"{'part':38} {'kg':>6} {'x mm':>7} {'z mm':>7}")
    for p in sorted(parts, key=lambda p: -p.mass_kg):
        print(f"{p.name:38} {p.mass_kg:>6.2f} {p.x_m * 1000:>7.0f} {p.z_m * 1000:>7.0f}")
    cg = centre_of_gravity(parts)
    print(f"\ntotal mass          {cg.mass_kg:.2f} kg")
    print(f"centre of gravity   x {cg.x_m * 1000:+.0f} mm   y {cg.y_m * 1000:+.0f} mm   z {cg.z_m * 1000:.0f} mm")
    print(f"stability margin    {stability_margin_m(cg) * 1000:.0f} mm to the nearest support edge")
    print(f"tips forward at     {tipping_decel_m_s2(cg):.2f} m/s^2 braking "
          f"(karmel's configured limit is 1.0 m/s^2)")


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["rails", "branches", "runtime", "mass", "check", "whatif"])
    ap.add_argument("scenario", nargs="?", default="arm-forward")
    ap.add_argument("--capacity-ah", type=float, default=3.35, help="35E minimum capacity (datasheet)")
    args = ap.parse_args(argv)

    loads, parts = karmel_v2_loads(), karmel_v2_parts()
    if args.command == "whatif":
        title, loads, parts = whatif(args.scenario)
        print(f"{title}\n")
    branches = karmel_v2_branches(loads)

    if args.command == "rails":
        print_rails(loads)
        return 0
    if args.command == "branches":
        print_branches(branches)
        return 0
    if args.command == "mass":
        print_mass(parts)
        return 0
    if args.command == "runtime":
        avg = session_power_w(loads)
        idle = session_power_w([replace(x, duty=0.0) if x.domain == ACTUATOR else x for x in loads])
        print(f"compute domain, always on          {idle:6.1f} W at the pack")
        print(f"mission average (driving + arm)    {avg:6.1f} W at the pack")
        print(f"runtime, 80 % of {args.capacity_ah:.2f} Ah at 10.8 V   "
              f"{runtime_min(args.capacity_ah, 10.8, avg):6.0f} min mission, "
              f"{runtime_min(args.capacity_ah, 10.8, idle):.0f} min idling")
        return 0

    print_rails(loads)
    print()
    print_branches(branches)
    print()
    print_mass(parts)
    problems = check(loads, branches, parts)
    print()
    if not problems:
        print("power, wiring and mass: no problems")
        return 0
    for p in problems:
        print(f"PROBLEM  {p}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
