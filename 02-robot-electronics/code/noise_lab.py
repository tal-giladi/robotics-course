"""02.06 — noise arithmetic for karmel, and the analysis of noise_probe.py logs.

1. Ground bounce: V = L di/dt in a shared ground wire at every PWM edge.
2. Bulk and ceramic capacitors at the driver: who supplies the edge current.
3. Motor-terminal capacitors: noise shunting vs. the power they cost at 20 kHz.
4. Magnetic coupling from a motor wire into a sensor wire loop; twisted vs. untwisted.
5. RC filters on encoder lines: spike rejection vs. the fastest real edge.
6. analyse(): per-condition statistics of a noise_probe.py CSV log.

Run:  python 02-robot-electronics/code/noise_lab.py [--log noise.csv]
"""
from __future__ import annotations

import argparse
import math
import statistics
from dataclasses import dataclass

MU0 = 4e-7 * math.pi
NH_PER_MM_WIRE = 1.0          # rule of thumb for an isolated wire (estimate)


def ground_bounce_v(length_mm: float, delta_i: float, t_edge_s: float) -> float:
    return NH_PER_MM_WIRE * length_mm * 1e-9 * delta_i / t_edge_s


def cap_droop_v(i_pulse: float, t_s: float, c_f: float, esr_ohm: float) -> float:
    """Voltage dip while a capacitor alone supplies a current pulse: charge loss + ESR drop."""
    return i_pulse * t_s / c_f + i_pulse * esr_ohm


def motor_cap_power_w(c_f: float, v: float, f_pwm: float) -> float:
    """A capacitor across the motor is charged and discharged every PWM period by the bridge:
    each full cycle dissipates C V^2 in the switches and wiring."""
    return c_f * v ** 2 * f_pwm


def reactance_ohm(c_f: float | None = None, l_h: float | None = None, f: float = 1e6) -> float:
    if c_f is not None:
        return 1 / (2 * math.pi * f * c_f)
    assert l_h is not None
    return 2 * math.pi * f * l_h


def mutual_inductance_h(parallel_len_m: float, distance_m: float, loop_width_m: float) -> float:
    """Long straight wire next to a rectangular loop (both in one plane)."""
    return MU0 * parallel_len_m / (2 * math.pi) * math.log((distance_m + loop_width_m) / distance_m)


def rc_spike_residual(v_spike: float, spike_s: float, r: float, c: float) -> float:
    return v_spike * (1 - math.exp(-spike_s / (r * c)))


def encoder_quadrature_spacing_s(wheel_rad_s: float = 21.5, cpr_motor: int = 11, gear: float = 56.0) -> float:
    """Time between successive A/B edges at a wheel speed (4 edges per encoder cycle)."""
    edges_per_s = wheel_rad_s / (2 * math.pi) * cpr_motor * gear * 4
    return 1 / edges_per_s


@dataclass(frozen=True)
class Stats:
    n: int
    mean: float
    stdev: float
    p2p: float
    outliers: int


def stats(values: list[float]) -> Stats:
    med = statistics.median(values)
    mad = statistics.median(abs(v - med) for v in values) or 1e-9
    outliers = sum(1 for v in values if abs(v - med) > 5 * 1.4826 * mad)
    return Stats(len(values), statistics.fmean(values), statistics.pstdev(values), max(values) - min(values), outliers)


def analyse(csv_text: str) -> dict[tuple[str, str], Stats]:
    """Lines 'condition,signal,value'. Non-numeric values (e.g. 'err') count as errors, reported as
    a separate signal '<signal>_errors' with value = count."""
    data: dict[tuple[str, str], list[float]] = {}
    errors: dict[tuple[str, str], int] = {}
    for line in csv_text.strip().splitlines():
        if not line or line.startswith("#") or line.count(",") != 2:
            continue
        cond, sig, val = line.split(",")
        try:
            data.setdefault((cond, sig), []).append(float(val))
        except ValueError:
            errors[(cond, sig + "_errors")] = errors.get((cond, sig + "_errors"), 0) + 1
    result = {k: stats(v) for k, v in data.items() if len(v) >= 2}
    for k, count in errors.items():
        result[k] = Stats(count, float(count), 0.0, 0.0, 0)
    return result


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--log", help="CSV captured from noise_probe.py")
    args = ap.parse_args(argv)
    out: dict = {}

    print("1) Ground bounce at one PWM edge (edge 150 ns)")
    for length in (20, 100, 300):
        for amps in (0.5, 2.95):
            v = ground_bounce_v(length, amps, 150e-9)
            out[("bounce", length, amps)] = v
            print(f"   {length:3d} mm shared ground wire, {amps:4.2f} A step: {v:5.2f} V spike")

    print("\n2) A capacitor alone supplying a 2.95 A load step: dip after 150 ns (edge), 25 us (half PWM period), 1 ms")
    for name, c, esr in (("100 nF ceramic, ESR 0.01 ohm", 100e-9, 0.01),
                         ("10 uF ceramic, ESR 0.005 ohm", 10e-6, 0.005),
                         ("470 uF electrolytic, ESR 0.10 ohm (estimate)", 470e-6, 0.10)):
        dips = [cap_droop_v(2.95, t, c, esr) for t in (150e-9, 25e-6, 1e-3)]
        out[("droop", name)] = dips
        cells = [f"{d:8.3f} V" if d < 12.6 else "   empty  " for d in dips]
        print(f"   {name:46}: {'  '.join(cells)}")
    print("   -> ceramics at the pins for the edges, the electrolytic for the PWM period, the battery for the rest")

    print("\n3) Capacitor across the motor terminals (12 V, 20 kHz)")
    for c in (10e-9, 47e-9, 100e-9, 220e-9):
        p = motor_cap_power_w(c, 12.0, 20_000)
        out[("motor_cap_w", c)] = p
        print(f"   {c * 1e9:5.0f} nF: extra switching loss {p * 1000:6.1f} mW; impedance at 1 MHz "
              f"{reactance_ohm(c_f=c):6.2f} ohm (motor winding {reactance_ohm(l_h=2e-3):.0f} ohm)")

    print("\n4) Coupling from a motor supply wire (2 A step in 150 ns) into a signal pair 20 cm alongside")
    for label, dist, width in (("untwisted pair, 5 mm apart, 5 mm from motor wire", 5e-3, 5e-3),
                               ("same, 30 mm from motor wire", 30e-3, 5e-3),
                               ("tight twisted pair (residual loop ~0.5 mm), 5 mm away", 5e-3, 0.5e-3)):
        m = mutual_inductance_h(0.20, dist, width)
        v = m * 2 / 150e-9
        out[("coupling", label)] = v
        print(f"   {label:52}: M = {m * 1e9:5.2f} nH, induced {v:5.3f} V")

    spacing = encoder_quadrature_spacing_s()
    print(f"\n5) Encoder: at 205 RPM the A/B edges are {spacing * 1e6:.0f} us apart")
    for r, c in ((1_000, 1e-9), (1_000, 10e-9), (10_000, 10e-9)):
        tau = r * c
        resid = rc_spike_residual(3.3, 20e-9, r, c)
        out[("rc", r, c)] = (tau, resid)
        print(f"   {r:6d} ohm + {c * 1e9:4.0f} nF: tau {tau * 1e6:6.1f} us ({100 * tau / spacing:5.1f} % of edge spacing), "
              f"a 3.3 V, 20 ns spike leaves {resid * 1000:6.1f} mV")

    if args.log:
        with open(args.log, encoding="utf-8") as f:
            res = analyse(f.read())
        print("\n6) Log analysis")
        for (cond, sig), s in sorted(res.items()):
            print(f"   {cond:14} {sig:18} n={s.n:4d} mean={s.mean:10.3f} std={s.stdev:8.3f} p2p={s.p2p:9.3f} outliers={s.outliers}")
        out["log"] = res
    return out


if __name__ == "__main__":
    main()
