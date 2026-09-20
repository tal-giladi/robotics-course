"""02.07 — size karmel's wiring harness: gauge, voltage drop, heat, connector and fuse coordination.

Wire resistance is computed from physics (solid copper, 20 C) and compared with the PowerStream
table; the "chassis wiring" ampacity column is a rule of thumb for short wires in free air.
Connector ratings: JST XH 3 A (AWG 22), JST PH 2 A (AWG 24) (JST product pages); Amass XT30 15 A,
XT60 30 A (distributor listings).

Run:  python 02-robot-electronics/code/harness.py
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

RHO_CU = 1.72e-8                         # ohm*m at 20 C
CHASSIS_AMPS = {16: 22.0, 18: 16.0, 20: 11.0, 22: 7.0, 24: 3.5, 26: 2.2, 28: 1.4}   # PowerStream
TABLE_OHM_PER_KM = {16: 13.17, 18: 20.94, 20: 33.29, 22: 52.94, 24: 84.20, 26: 133.86, 28: 212.87}


def awg_diameter_mm(awg: int) -> float:
    return 0.127 * 92 ** ((36 - awg) / 39)


def ohm_per_m(awg: int) -> float:
    d = awg_diameter_mm(awg) / 1000
    return RHO_CU / (math.pi * d ** 2 / 4)


@dataclass(frozen=True)
class Connector:
    name: str
    rated_a: float
    note: str


CONNECTORS = [
    Connector("XT60", 30.0, "battery, main power"),
    Connector("XT30", 15.0, "driver and regulator branches"),
    Connector("JST XH 2.5 mm", 3.0, "3 A with AWG 22 crimps"),
    Connector("JST PH 2.0 mm", 2.0, "2 A with AWG 24 crimps; the motor encoder cable"),
    Connector("Dupont 2.54 mm", 1.0, "course rule: signals only, budget <= 1 A (estimate, contacts vary widely)"),
]


@dataclass(frozen=True)
class Run:
    name: str
    amps_worst: float
    length_m: float          # one way; the current returns through a second conductor
    v_rail: float
    max_drop_fraction: float


KARMEL_RUNS = [
    Run("battery -> fuse -> switch -> power board", 8.0, 0.30, 10.8, 0.03),
    Run("power board -> each DRV8874 VIN", 3.0, 0.15, 10.8, 0.02),
    Run("DRV8874 OUT -> motor", 3.0, 0.25, 10.8, 0.03),
    Run("5 V regulator -> Pi 5 (USB-C pigtail)", 3.8, 0.15, 5.0, 0.02),
    Run("Pico 3V3 -> sensors", 0.1, 0.30, 3.3, 0.02),
]


def drop_v(run: Run, awg: int) -> float:
    return run.amps_worst * ohm_per_m(awg) * 2 * run.length_m


def choose_gauge(run: Run, fuse_a: float | None = None) -> int:
    """Thinnest gauge that meets the drop budget, the ampacity rule and (if the run is protected
    only by an upstream fuse) the fuse rating."""
    for awg in (28, 26, 24, 22, 20, 18, 16):
        if drop_v(run, awg) > run.max_drop_fraction * run.v_rail:
            continue
        if CHASSIS_AMPS[awg] < run.amps_worst * 1.25:
            continue
        if fuse_a is not None and CHASSIS_AMPS[awg] < fuse_a:
            continue
        return awg
    raise ValueError(f"no gauge fits {run.name}")


def pick_connector(amps: float, derate: float = 0.8) -> Connector:
    for c in sorted(CONNECTORS, key=lambda c: c.rated_a):
        if amps <= derate * c.rated_a:
            return c
    raise ValueError("no connector")


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--fuse", type=float, default=10.0)
    args = ap.parse_args(argv)
    out: dict = {}

    print("AWG   d [mm]  computed mOhm/m  table mOhm/m  chassis A")
    for awg in (16, 18, 20, 22, 24, 26, 28):
        print(f"{awg:3d}  {awg_diameter_mm(awg):6.3f}  {ohm_per_m(awg) * 1000:14.2f}  {TABLE_OHM_PER_KM[awg]:12.2f}  {CHASSIS_AMPS[awg]:9.1f}")

    print(f"\nHarness runs (worst-case current, both conductors, {args.fuse:.0f} A main fuse)")
    for run in KARMEL_RUNS:
        fused_by_main = run.v_rail > 6 and run.amps_worst > 0.5
        awg = choose_gauge(run, args.fuse if fused_by_main else None)
        conn = pick_connector(run.amps_worst)
        d = drop_v(run, awg)
        heat = run.amps_worst * d
        out[run.name] = (awg, conn.name, d)
        print(f"  {run.name:42} {run.amps_worst:4.1f} A {run.length_m:4.2f} m -> AWG {awg}, drop {d * 1000:5.0f} mV "
              f"({100 * d / run.v_rail:4.1f} %), {heat:4.2f} W, connector {conn.name}")
        for alt in (22, 24):
            if alt > awg:
                print(f"      with AWG {alt}: drop {drop_v(run, alt) * 1000:5.0f} mV, chassis rating {CHASSIS_AMPS[alt]} A")

    print("\nFuse coordination: a 10 A fuse only protects wire that can carry 10 A until the fuse opens.")
    for awg in (18, 20, 22, 24):
        ok = CHASSIS_AMPS[awg] >= args.fuse
        print(f"  AWG {awg}: chassis {CHASSIS_AMPS[awg]:4.1f} A -> {'protected' if ok else 'NOT protected: thicker wire or a local fuse'}")
    return out


if __name__ == "__main__":
    main()
