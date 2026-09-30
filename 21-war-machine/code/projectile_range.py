"""Releasers II — launch mechanics: where the energy goes, how fast the payload leaves, how far.

The four launchers of the BOM (slingshot, linear-actuator cannon, throw arm, CO2 magazine)
are different energy-storage mechanisms with one common output question:

    stored energy -> efficiency -> muzzle velocity -> range -> recoil on the robot

    py 21-war-machine/code/projectile_range.py slingshot
    py 21-war-machine/code/projectile_range.py cannon
    py 21-war-machine/code/projectile_range.py arm
    py 21-war-machine/code/projectile_range.py coga
    py 21-war-machine/code/projectile_range.py recoil
    py 21-war-machine/code/projectile_range.py check

Range model: ideal ballistic range (no drag) times a per-projectile drag factor. The factor is
an (estimate) — fit it at your range with three measured ranges (21.04-E3). That is the whole
model; the point is that the *order* of the numbers is right and the inputs are all measurable.
"""

from __future__ import annotations

import argparse
import math
import sys

G = 9.81  # m/s^2


# ==============================================================================================
#  Energy -> velocity -> range
# ==============================================================================================
def elastic_energy_j(k_n_m: float, stretch_m: float) -> float:
    """Energy stored in stretched rubber: E = 1/2 k x^2. ``k`` is the TOTAL stiffness of the
    band set in parallel (2 heavy bands: ~120 N/m at this thickness, estimate — measure yours)."""
    return 0.5 * k_n_m * stretch_m ** 2


def muzzle_velocity_m_s(energy_j: float, mass_kg: float, efficiency: float = 1.0) -> float:
    """v = sqrt(2 E_eff / m): all of the (efficiency-weighted) energy becomes kinetic energy."""
    return math.sqrt(2.0 * energy_j * efficiency / mass_kg)


def ideal_range_m(muzzle_velocity_m_s: float, angle_deg: float = 45.0) -> float:
    """No-drag projectile range: R = v^2 sin(2 theta) / g. 45 degrees maximises it."""
    a = math.radians(angle_deg)
    return muzzle_velocity_m_s ** 2 * math.sin(2.0 * a) / G


def range_m(muzzle_velocity_m_s: float, drag_factor: float, angle_deg: float = 45.0) -> float:
    """Dragged range: ideal range x drag_factor. Factor is an empirical (estimate) per
    projectile size/density — a 6 mm BB is a feather compared with a 4 cm bouncy ball."""
    return ideal_range_m(muzzle_velocity_m_s, angle_deg) * drag_factor


def recoil_m_s(muzzle_velocity_m_s: float, projectile_kg: float, robot_kg: float) -> float:
    """Conservation of momentum: the robot gets v * m_projectile / m_robot of kick."""
    return muzzle_velocity_m_s * projectile_kg / robot_kg


# ==============================================================================================
#  The launchers, with the BOM's hardware (war-machine-bom.md §C/§D)
# ==============================================================================================
def slingshot(k_n_m: float = 120.0, stretch_m: float = 0.120, efficiency: float = 0.70):
    """2 heavy rubber bands, 120 mm stretch. Elastic store -> 70% to the projectile
    (band friction and snap losses, estimate)."""
    e_store = elastic_energy_j(k_n_m, stretch_m)
    return e_store, e_store * efficiency


def cannon(force_n: float = 120.0, stroke_m: float = 0.100, efficiency: float = 0.50):
    """120 N linear actuator, 100 mm stroke. Work W = F d -> 50% to the projectile
    (motor losses, friction, ball compliance, estimate)."""
    w = force_n * stroke_m
    return w, w * efficiency


def arm_throw(payload_kg: float, radius_m: float, arm_inertia_kg_m2: float,
              seconds_per_60deg: float):
    """Servo throw arm: E = 1/2 I omega^2, I = m r^2 + arm inertia, v_tip = omega r.
    Torque servos are strong at stall and slow at the tip — this is a lobber, not a rifle."""
    i_total = payload_kg * radius_m ** 2 + arm_inertia_kg_m2
    omega = math.radians(60.0) / seconds_per_60deg
    e = 0.5 * i_total * omega ** 2
    v = omega * radius_m
    return i_total, e, v


# Per-projectile drag factors on ideal range (estimate — fit at your range):
DRAG = {
    "steel ball 5 mm": 0.30,     # dense, small, but slow-ish: still heavy drag at 17 m/s
    "bouncy ball 4 cm": 0.45,    # light and fat: the cannon's soft round
    "airsoft BB 6 mm": 0.05,     # feather at 100 m/s; the factor compresses the huge ideal range
    "magnet 50x15 mm": 0.90,     # dense blob: drag barely matters below 3 m/s
    "paintball 0.25 g": 0.30,
}

# Robot masses by package (war_power_budget.py `mass` output).
ROBOT_KG = {"base": 2.66, "scout": 4.16, "assault": 5.51, "siege": 6.46}


def _print_slingshot() -> None:
    e_store, e_proj = slingshot()
    for name, m in (("steel ball 5 mm (0.004 kg)", 0.004), ("paintball (0.00025 kg)", 0.00025)):
        v = muzzle_velocity_m_s(e_proj, m)
        d = range_m(v, DRAG["steel ball 5 mm" if "steel" in name else "paintball 0.25 g"])
        print(f"slingshot  {name}")
        print(f"  stored {e_store:.2f} J  ->  {e_proj:.2f} J to projectile (70% eff)")
        print(f"  muzzle {v:6.1f} m/s   ideal range {ideal_range_m(v):7.1f} m   dragged {d:5.1f} m")
    print(f"  band snap-back hits the frame with the stored {e_store:.1f} J in ~50 ms -> ~{2 * e_store / 0.05:.0f} N impulse average: BOLT the frame, don't glue it")
    print()


def _print_cannon() -> None:
    w, e_proj = cannon()
    m = 0.05
    v = muzzle_velocity_m_s(e_proj, m)
    print(f"cannon (120 N x 100 mm actuator)")
    print(f"  work {w:.1f} J  ->  {e_proj:.1f} J to projectile (50% eff)")
    print(f"  bouncy ball 4 cm (0.05 kg): muzzle {v:.1f} m/s   ideal {ideal_range_m(v):.1f} m   dragged {range_m(v, DRAG['bouncy ball 4 cm']):.1f} m")
    for pkg, robot in ROBOT_KG.items():
        if pkg in ("scout", "assault", "siege"):
            print(f"  recoil kick on {pkg} ({robot:.2f} kg): {recoil_m_s(v, m, robot):.3f} m/s")
    print()


def _print_arm() -> None:
    print("throw arms (servo tip speed is the limit, not the torque)")
    for label, (m, r, i_arm, t60) in {
        "AGF 78 kg*cm, 0.15 kg magnet, r 0.30 m, 0.15 s/60deg": (0.15, 0.30, 0.003, 0.15),
        "MG996R-360, 0.05 kg magnet, r 0.25 m, 0.12 s/60deg": (0.05, 0.25, 0.002, 0.12),
    }.items():
        i, e, v = arm_throw(m, r, i_arm, t60)
        print(f"{label}")
        print(f"  I {i:.4f} kg*m^2  energy {e:.2f} J  tip {v:.2f} m/s  ideal range {ideal_range_m(v):.2f} m")
    print("  verdict: the heavy throw is a ~2 m/s LOBBER. Torque stalls the load,")
    print("  but range is set by tip speed = slew rate x radius. The BOM's '~4 J'")
    print("  entry is the energy the arm CAN put in at full speed; at real slew")
    print("  rates with a 0.15 kg tip it is ~0.4 J (this is the model, 21.04 Level 4).")
    print()


def _print_coga() -> None:
    # CYMA CO2 magazine: the energy is chemical (the 12 g cartridge), not electrical.
    for m, v, label in ((0.00025, 100.0, "BB 0.25 g at 100 m/s (estimate; chronograph yours)"),
                        (0.00020, 120.0, "BB 0.20 g at 120 m/s (estimate)")):
        e = 0.5 * m * v ** 2
        print(f"coga  {label}")
        print(f"  projectile energy {e:.2f} J   ideal range {ideal_range_m(v):.0f} m   dragged ~{range_m(v, DRAG['airsoft BB 6 mm']):.0f} m")
    print("  a 12 g paintball cartridge holds ~8-15 shots in a magazine of this class")
    print("  (estimate; the CYMA magazine is ~25 rounds — count yours).")
    print()


def _print_recoil() -> None:
    print("recoil budget (momentum conservation, worst launcher per package)")
    rows = [
        ("slingshot", 0.004, 17.4, "scout"),
        ("cannon", 0.05, 15.5, "assault"),
        ("AGF arm", 0.15, 2.1, "siege"),
    ]
    for name, m, v, pkg in rows:
        robot = ROBOT_KG[pkg]
        print(f"  {name:<10} on {pkg:<8} kick {recoil_m_s(v, m, robot):.3f} m/s   "
              f"(bleed-off time ~0.5 s -> {recoil_m_s(v, m, robot) / 0.5:.3f} m/s^2 effective)")
    print("  all far under the 1.0 m/s^2 drive limit — recoil is a stability nuisance,")
    print("  not a tipping event (21.01 Troubleshooting).")
    print()


def _check() -> list[str]:
    problems = []
    _, e_proj = slingshot()
    v = muzzle_velocity_m_s(e_proj, 0.004)
    if range_m(v, DRAG["steel ball 5 mm"]) > 30.0:
        problems.append(f"slingshot dragged range {range_m(v, DRAG['steel ball 5 mm']):.0f} m > 30 m: re-check k or stretch")
    _, e_c = cannon()
    vc = muzzle_velocity_m_s(e_c, 0.05)
    if range_m(vc, DRAG["bouncy ball 4 cm"]) > 25.0:
        problems.append(f"cannon dragged range {range_m(vc, DRAG['bouncy ball 4 cm']):.0f} m > 25 m: shorten the range")
    if recoil_m_s(vc, 0.05, ROBOT_KG["assault"]) > 0.5:
        problems.append("cannon recoil kick > 0.5 m/s: gusset the rail")
    i, e, v = arm_throw(0.15, 0.30, 0.003, 0.15)
    if e > 2.0:
        problems.append(f"AGF arm energy {e:.1f} J > 2 J: the tip is hitting something it shouldn't")
    return problems


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("command", choices=("slingshot", "cannon", "arm", "coga", "recoil", "check"))
    args = ap.parse_args(argv)
    if args.command == "check":
        problems = _check()
        if problems:
            print(f"check: {len(problems)} problem(s)")
            for p in problems:
                print(f"  PROBLEM  {p}")
            return 1
        print("check: all rules pass")
        return 0
    {
        "slingshot": _print_slingshot,
        "cannon": _print_cannon,
        "arm": _print_arm,
        "coga": _print_coga,
        "recoil": _print_recoil,
    }[args.command]()
    return 0


if __name__ == "__main__":
    sys.exit(main())
