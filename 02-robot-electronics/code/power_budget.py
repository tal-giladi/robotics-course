"""02.02 — karmel's power budget: current per rail, battery runtime and static voltage sag.

Numbers marked (datasheet) come from the manufacturer; (estimate) are assumptions you should
replace with your own measurements (INA219, multimeter). Edit the tables, rerun, re-decide.

Run:  python 02-robot-electronics/code/power_budget.py
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass


@dataclass(frozen=True)
class Load:
    name: str
    typical_a: float
    worst_a: float
    note: str


# --- 3.3 V rail: the Pico 2's on-board regulator (keep < 300 mA, Pico 2 datasheet) ---------------
RAIL_3V3 = [
    Load("RP2350 + flash (Pico 2 itself)", 0.025, 0.040, "estimate; measure at VSYS"),
    Load("2x Hall encoders (Yahboom 520)", 0.010, 0.020, "estimate, 5-10 mA each"),
    Load("VL53L1X carrier", 0.020, 0.040, "Pololu 3415: 20 mA typ, 40 mA peak (datasheet)"),
    Load("INA219", 0.0007, 0.001, "TI: 0.7 mA typ, 1 mA max (datasheet)"),
    Load("US-100", 0.002, 0.005, "Adafruit 4019: 2 mA; transmit burst higher (estimate)"),
]

# --- 5 V rail: the D42V55F5 regulator --------------------------------------------------------------
RAIL_5V = [
    Load("Raspberry Pi 5 board", 0.80, 2.00, "0.8 A = official 'typical bare-board active'; 2.0 A heavy CPU (estimate)"),
    Load("Pico 2 + its 3.3 V loads (via USB)", 0.07, 0.11, "3.3 V rail above, converted at ~90 %"),
    Load("Active Cooler fan", 0.05, 0.10, "estimate"),
    Load("USB allowance (Stage 2-3: camera, LiDAR)", 0.00, 1.60, "Pi 5 USB limit 1.6 A with a 5 A supply (official)"),
]

# --- Battery rail (9.9-12.6 V): motors through the DRV8874s ----------------------------------------
MOTOR_STALL_A_12V = 4.0            # Yahboom 520 1:56, stall 4 A at 12 V (datasheet)
MOTOR_R_OHM = 12.0 / MOTOR_STALL_A_12V
DRV8874_ITRIP_A = 3.3 / (2490 * 450e-6)   # VREF = SLEEP = 3.3 V, R_IPROPI 2.49 k, A_IPROPI 450 uA/A


def sum_rail(loads: list[Load]) -> tuple[float, float]:
    return sum(x.typical_a for x in loads), sum(x.worst_a for x in loads)


def regulator_input_a(p_out_w: float, v_in: float, efficiency: float = 0.90) -> float:
    """A switching regulator is (roughly) a constant-POWER load: lower input voltage -> MORE current."""
    return p_out_w / (efficiency * v_in)


def motor_currents(v_batt: float, n_motors: int = 2) -> dict[str, float]:
    stall_each = v_batt / MOTOR_R_OHM
    limited_each = min(stall_each, DRV8874_ITRIP_A)
    return {
        "cruise_each": 0.5,                                   # estimate: flat floor, 0.3 m/s
        "stall_each_unlimited": stall_each,
        "stall_each_drv8874": limited_each,
        "reversal_peak_each_unlimited": 2 * v_batt / MOTOR_R_OHM,   # (V + back-EMF) / R, back-EMF ~ V
        "worst_total_drv8874": n_motors * limited_each,
        "worst_total_unlimited": n_motors * 2 * v_batt / MOTOR_R_OHM,
    }


@dataclass(frozen=True)
class PathElement:
    name: str
    ohms: float
    note: str


# Resistance between the cells' chemistry and the robot's loads. Measure yours (lesson exercise).
KARMEL_PATH = [
    PathElement("3x Samsung 35E cells (DC)", 3 * 0.045, "35 mOhm AC at 1 kHz (datasheet); DC higher, 45 mOhm (estimate)"),
    PathElement("3x holder spring contacts", 3 * 0.015, "estimate; worn springs are worse"),
    PathElement("BMS discharge MOSFETs", 0.010, "estimate"),
    PathElement("10 A 5x20 mm fuse", 0.008, "estimate"),
    PathElement("rocker switch", 0.005, "estimate"),
    PathElement("XT60 pair", 0.0004, "Amass: ~0.4 mOhm (distributor listing)"),
    PathElement("18 AWG, 0.6 m out and back", 0.6 * 0.02094, "20.94 mOhm/m (PowerStream table)"),
]
BREADBOARD_EXTRA = [
    PathElement("4 Dupont contacts", 4 * 0.03, "estimate; 2.54 mm contacts, worse when loose"),
    PathElement("breadboard power rail + clips", 0.04, "estimate"),
    PathElement("24 AWG jumpers, 0.6 m out and back", 0.6 * 0.0842, "84.2 mOhm/m (PowerStream table)"),
]


def path_resistance(elements: list[PathElement]) -> float:
    return sum(e.ohms for e in elements)


def sag(v_rest: float, amps: float, r_path: float) -> float:
    return v_rest - amps * r_path


def runtime_h(capacity_ah: float, v_nominal: float, avg_power_w: float, usable_fraction: float = 0.9) -> float:
    return capacity_ah * v_nominal * usable_fraction / avg_power_w


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--capacity-ah", type=float, default=3.35, help="35E minimum capacity (datasheet)")
    ap.add_argument("--driving-fraction", type=float, default=0.3, help="fraction of the session spent driving")
    args = ap.parse_args(argv)

    t33, w33 = sum_rail(RAIL_3V3)
    t5, w5 = sum_rail(RAIL_5V)
    print("3.3 V rail (Pico regulator, keep < 0.300 A)")
    for x in RAIL_3V3:
        print(f"  {x.name:42} {x.typical_a:6.3f} A {x.worst_a:6.3f} A   {x.note}")
    print(f"  {'TOTAL':42} {t33:6.3f} A {w33:6.3f} A")
    print("5 V rail (D42V55F5, 5.5 A class)")
    for x in RAIL_5V:
        print(f"  {x.name:42} {x.typical_a:6.3f} A {x.worst_a:6.3f} A   {x.note}")
    print(f"  {'TOTAL':42} {t5:6.3f} A {w5:6.3f} A  -> {5 * t5:.1f} W typical, {5 * w5:.1f} W worst")

    results: dict = {"rail_3v3": (t33, w33), "rail_5v": (t5, w5)}
    for v in (12.6, 10.8, 9.9):
        m = motor_currents(v)
        reg_typ = regulator_input_a(5 * t5, v)
        reg_worst = regulator_input_a(5 * w5, v)
        worst_batt = m["worst_total_drv8874"] + reg_worst
        results[f"battery_worst_{v}"] = worst_batt
        results[f"motors_{v}"] = m
        print(f"Battery rail at {v:4.1f} V: regulator draws {reg_typ:.2f} A typ / {reg_worst:.2f} A worst; "
              f"stall {m['stall_each_unlimited']:.2f} A/motor unlimited, {m['stall_each_drv8874']:.2f} A with DRV8874 limit; "
              f"reversal peak {m['reversal_peak_each_unlimited']:.1f} A/motor unlimited; "
              f"WORST TOTAL {worst_batt:.2f} A (DRV8874) vs {m['worst_total_unlimited'] + reg_worst:.1f} A (no limit)")

    r_karmel = path_resistance(KARMEL_PATH)
    r_bb = r_karmel + path_resistance(BREADBOARD_EXTRA)
    print(f"\nSource path resistance: karmel harness {r_karmel * 1000:.0f} mOhm, breadboard prototype {r_bb * 1000:.0f} mOhm")
    for label, r in (("harness", r_karmel), ("breadboard", r_bb)):
        for amps in (1.5, 6.3, 18.3):
            print(f"  {label:10} {amps:5.1f} A: 12.4 V rest -> {sag(12.4, amps, r):5.2f} V;"
                  f"  10.4 V rest -> {sag(10.4, amps, r):5.2f} V;  heat in path {amps ** 2 * r:5.1f} W")
    results["r_path_karmel"] = r_karmel
    results["r_path_breadboard"] = r_bb

    pi_avg_w = 5 * 1.2 / 0.9                       # Pi + Pico averaged over a session, at the battery
    motors_driving_w = 2 * 0.5 * 10.8               # both motors cruising
    avg_w = pi_avg_w + args.driving_fraction * motors_driving_w
    worst_w = 5 * w5 / 0.9 + 2 * 0.8 * 10.8
    rt_typ = runtime_h(args.capacity_ah, 10.8, avg_w)
    rt_worst = runtime_h(args.capacity_ah, 10.8, worst_w)
    print(f"\nRuntime: average {avg_w:.1f} W -> {rt_typ:.1f} h; continuous hard driving + full 5 V rail "
          f"{worst_w:.1f} W -> {rt_worst:.1f} h")
    results.update(avg_w=avg_w, worst_w=worst_w, runtime_typ_h=rt_typ, runtime_worst_h=rt_worst)
    return results


if __name__ == "__main__":
    main()
