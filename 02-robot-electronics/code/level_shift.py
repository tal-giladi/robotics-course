"""02.05 — logic levels and pin protection, with worst-case tolerances.

* HC-SR04 echo divider: worst-case high/low levels with resistor and supply tolerance, and the
  timing error the divider adds.
* Series resistors: fault current into a pin, and what that current does to the 3.3 V rail.
* Bidirectional MOSFET shifter for I2C: rise time on each side.

Run:  python 02-robot-electronics/code/level_shift.py
"""
from __future__ import annotations

import argparse
import itertools
from dataclasses import dataclass

PICO_VIH = 2.0
PICO_VIL = 0.8
PICO_FT_ABS_MAX_POWERED = 5.5
PICO_FT_ABS_MAX_UNPOWERED = 3.63
PICO_STD_ABS_MAX = 3.3 + 0.5          # GP26-GP29 (ADC-capable pins are standard, not FT)


@dataclass(frozen=True)
class DividerCorner:
    v_source: float
    r_top: float
    r_bottom: float

    @property
    def v_out(self) -> float:
        return self.v_source * self.r_bottom / (self.r_top + self.r_bottom)


def divider_corners(r_top: float, r_bottom: float, tol: float, v_min: float, v_max: float) -> tuple[float, float]:
    """Lowest possible 'high' and highest possible 'high' at the Pico pin."""
    outs = [DividerCorner(v, r_top * (1 + a), r_bottom * (1 + b)).v_out
            for v, a, b in itertools.product((v_min, v_max), (-tol, tol), (-tol, tol))]
    return min(outs), max(outs)


def divider_timing_error_mm(r_top: float, r_bottom: float, c_pin_pf: float, speed_m_s: float = 343.0) -> float:
    """The divider and the pin capacitance form an RC low-pass. Both edges of the echo pulse are
    delayed by roughly the same time, so the PULSE WIDTH barely changes; the worst case is an
    asymmetric threshold: rising edge crosses 2.0 V, falling edge crosses 0.8 V. Estimate that
    width error and convert it to distance (round trip, so / 2)."""
    import math
    r_th = r_top * r_bottom / (r_top + r_bottom)
    tau = r_th * c_pin_pf * 1e-12
    v_high = 5.0 * r_bottom / (r_top + r_bottom)
    t_rise = -tau * math.log(1 - PICO_VIH / v_high)
    t_fall = -tau * math.log(PICO_VIL / v_high)
    width_error_s = t_fall - t_rise
    return width_error_s * speed_m_s / 2 * 1000


def fault_current_ma(v_fault: float, r_series: float, v_clamp: float) -> float:
    return max(0.0, (v_fault - v_clamp) / r_series) * 1000


def mosfet_shifter_rise_ns(r_pullup: float, c_side_pf: float) -> float:
    """When every device releases the line, that side rises through its own pull-up (30->70 %)."""
    return 0.8473 * r_pullup * c_side_pf * 1e-12 * 1e9


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--tol", type=float, default=0.05, help="resistor tolerance (0.05 = 5 %%)")
    args = ap.parse_args(argv)
    out: dict = {}

    print("1) HC-SR04 echo (4.75-5.25 V USB supply) into a Pico GPIO")
    print(f"   {'divider':>14} {'lowest high':>12} {'highest high':>13} {'VIH margin':>11} {'vs unpowered 3.63':>18} {'load mA':>8} {'width err':>10}")
    for r_top, r_bottom in ((1_000, 2_000), (1_000, 1_500), (2_200, 3_300), (10_000, 20_000), (1_000, 1_000)):
        lo, hi = divider_corners(r_top, r_bottom, args.tol, 4.75, 5.25)
        load = 5.25 / (r_top + r_bottom) * 1000
        err = divider_timing_error_mm(r_top, r_bottom, c_pin_pf=15.0)
        ok = lo - PICO_VIH > 0.3 and hi <= PICO_FT_ABS_MAX_UNPOWERED
        out[(r_top, r_bottom)] = (lo, hi, ok)
        print(f"   {r_top:>6}/{r_bottom:<6} {lo:11.2f}V {hi:12.2f}V {lo - PICO_VIH:10.2f}V "
              f"{'OK' if hi <= PICO_FT_ABS_MAX_UNPOWERED else 'EXCEEDS':>18} {load:8.2f} {err:8.4f}mm  {'<- good' if ok else ''}")

    print("\n2) Series resistor + clamp: current forced into a pin by a wiring fault")
    cases = [
        ("12 V onto an ADC pin (GP26) via 1 k: internal diode to 3V3", 12.0, 1_000, 3.3 + 0.5),
        ("12 V onto an ADC pin via 10 k", 12.0, 10_000, 3.3 + 0.5),
        ("12 V onto GP26 via 100 k/22 k divider top resistor", 12.0, 100_000, 3.3 + 0.5),
        ("5 V onto GP26 via 1 k", 5.0, 1_000, 3.3 + 0.5),
        ("12 V onto an FT pin via 10 k + external BAT54S to 3V3", 12.0, 10_000, 3.3 + 0.35),
    ]
    faults = {}
    for name, v, r, clamp in cases:
        ma = fault_current_ma(v, r, clamp)
        faults[name] = ma
        verdict = "back-powers the 3.3 V rail if the Pico draws less than this" if ma > 1 else "small"
        print(f"   {name:62} {ma:6.2f} mA  ({verdict})")
    out["faults"] = faults
    short_ma = fault_current_ma(3.3, 330, 0.0)
    print(f"   output HIGH shorted to GND through 330 ohm: {short_ma:.1f} mA (without it: limited only by the pad driver)")

    print("\n3) MOSFET bidirectional shifter (BSS138 style), 10 k pull-ups each side")
    for c in (30, 60, 120):
        print(f"   {c:4d} pF per side: rise {mosfet_shifter_rise_ns(10_000, c):5.0f} ns "
              f"({'OK' if mosfet_shifter_rise_ns(10_000, c) <= 300 else 'too slow'} for 400 kHz); "
              f"with 4.7 k: {mosfet_shifter_rise_ns(4_700, c):4.0f} ns")
    return out


if __name__ == "__main__":
    main()
