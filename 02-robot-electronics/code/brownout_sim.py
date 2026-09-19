"""02.02 — why the Pi reboots when the motors start: a time-domain model of karmel's power path.

Model (average-value, no PWM ripple):

    V_oc ──R_path──┬── node V_bus ──┬── C_bulk
                   │                ├── 2 motors via H-bridge: duty d, current limit I_trip (optional)
                   │                └── 5 V regulator: constant power while V_bus > 5 V + dropout,
                   │                    pass-through (V_bus - dropout) below it
    5 V at the Pi  = V_reg_out - I_5V * R_5V_wiring

    motor:  L di/dt = d*V_bus - (R_m + R_drv) i - k w      J dw/dt = k i - k I0 sign(w)

The Pi 5 reports under-voltage below 4.63 V (+/-5 %), so the script flags anything below 4.63 V
and marks < 4.40 V (the bottom of that band) as a likely brownout.

Run:  python 02-robot-electronics/code/brownout_sim.py [--plot brownout.png]
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, replace

# --- Yahboom 520 1:56 motor, derived as in FE.10 (stall 4 A at 12 V, 205 RPM, I0 ~ 0.1 A) --------
R_M = 3.0                     # ohm
K_M = 0.00973                 # V*s/rad = N*m/A
I0 = 0.10                     # A, friction current (estimate)
L_M = 2.0e-3                  # H (estimate; measure it or fit the current rise)
TAU_MECH = 0.08               # s, karmel.yaml motor_time_constant_s (includes wheel + robot inertia)
J_M = TAU_MECH * K_M ** 2 / R_M   # kg*m^2 seen at the motor shaft, so that J R / k^2 = tau


@dataclass(frozen=True)
class Scenario:
    name: str
    v_rest: float = 12.3            # pack open-circuit voltage
    r_path: float = 0.216           # ohm, from power_budget.py
    c_bulk: float = 470e-6          # F at the drivers
    c_min: float = 20e-6            # F already on the regulator and driver boards (estimate)
    r_driver: float = 0.2           # ohm, bridge HS + LS
    i_trip: float | None = 2.95     # A per motor, DRV8874 with VREF = 3.3 V; None = no limit
    ramp_s: float = 0.0             # duty ramp time 0 -> 1 (0 = step)
    reg_dropout: float = 1.0        # V (estimate)
    reg_efficiency: float = 0.90
    i_5v: float = 2.5               # A drawn at 5 V by Pi + USB
    r_5v_wiring: float = 0.04       # ohm, regulator -> Pi (both conductors + connector)
    v_reg_set: float = 5.10


def duty_profile(t: float, s: Scenario) -> float:
    """0 until 0.05 s, full forward, then full REVERSE at 0.30 s (the worst normal event)."""
    def ramp(t0: float, d0: float, d1: float) -> float:
        if s.ramp_s <= 0:
            return d1
        frac = min(1.0, (t - t0) / s.ramp_s)
        return d0 + (d1 - d0) * frac
    if t < 0.05:
        return 0.0
    if t < 0.30:
        return ramp(0.05, 0.0, 1.0)
    return ramp(0.30, 1.0, -1.0)


def simulate(s: Scenario, t_end: float = 0.60, dt: float = 10e-6) -> dict:
    v_bus = s.v_rest
    i = 0.0
    w = 0.0
    c = s.c_bulk + s.c_min
    trace: list[tuple[float, float, float, float]] = []
    min_bus = v_bus
    min_pi = 99.0
    t_below_463 = 0.0
    peak_i = 0.0
    steps = int(t_end / dt)
    for n in range(steps):
        t = n * dt
        d = duty_profile(t, s)
        # --- motor (both identical, so the bus sees 2x) --------------------------------------
        v_app = d * v_bus
        di = (v_app - (R_M + s.r_driver) * i - K_M * w) / L_M
        i_new = i + di * dt
        if s.i_trip is not None and abs(i_new) > s.i_trip and abs(i_new) > abs(i):
            i_new = math.copysign(s.i_trip, i_new)      # the driver chops: current held at I_trip
        i = i_new
        friction = K_M * I0 * (1 if w > 0 else -1 if w < 0 else 0)
        w += (K_M * i - friction) / J_M * dt
        i_bridge = 2 * d * i                              # supply current of both bridges (average)
        # --- regulator -------------------------------------------------------------------------
        if v_bus - s.reg_dropout >= s.v_reg_set:
            v_reg = s.v_reg_set
            i_reg = s.v_reg_set * s.i_5v / (s.reg_efficiency * v_bus)
        else:
            v_reg = max(0.0, v_bus - s.reg_dropout)
            i_reg = s.i_5v
        v_pi = v_reg - s.i_5v * s.r_5v_wiring
        # --- bus node, backward Euler (stable for any dt) --------------------------------------
        i_load = i_bridge + i_reg
        v_bus = (v_bus + dt / c * (s.v_rest / s.r_path - i_load)) / (1 + dt / (s.r_path * c))
        min_bus = min(min_bus, v_bus)
        min_pi = min(min_pi, v_pi)
        peak_i = max(peak_i, abs(i))
        if v_pi < 4.63:
            t_below_463 += dt
        if n % 100 == 0:
            trace.append((t, v_bus, v_pi, 2 * i))
    return {"min_bus_v": min_bus, "min_pi_v": min_pi, "ms_below_4v63": 1000 * t_below_463,
            "peak_motor_a": peak_i, "brownout": min_pi < 4.40, "trace": trace}


def scenarios() -> list[Scenario]:
    karmel = Scenario("karmel as designed (harness, 470 uF, DRV8874 limit)")
    proto = Scenario("breadboard prototype, low pack, no current limit, no bulk cap, weak 5 V wiring",
                     v_rest=10.4, r_path=0.426, c_bulk=0.0, r_driver=0.5, i_trip=None,
                     reg_dropout=1.5, r_5v_wiring=0.15)
    return [
        karmel,
        replace(karmel, name="karmel, low pack (10.4 V)", v_rest=10.4),
        proto,
        replace(proto, name="prototype + 0.3 s duty ramp", ramp_s=0.3),
        replace(proto, name="prototype + 470 uF bulk cap only", c_bulk=470e-6),
        replace(proto, name="prototype + DRV8874 current limit only", i_trip=2.95, r_driver=0.2),
        replace(proto, name="prototype + harness wiring (216 mOhm) only", r_path=0.216, r_5v_wiring=0.04),
    ]


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--plot", help="save a PNG of bus and Pi voltage for the first three scenarios")
    args = ap.parse_args(argv)
    out: dict = {}
    print(f"{'scenario':84} {'min bus':>8} {'min Pi 5V':>10} {'ms<4.63':>8} {'peak A':>7}  result")
    for s in scenarios():
        r = simulate(s)
        out[s.name] = r
        verdict = "BROWNOUT/REBOOT" if r["brownout"] else ("under-voltage warning" if r["ms_below_4v63"] > 0 else "OK")
        print(f"{s.name:84} {r['min_bus_v']:7.2f}V {r['min_pi_v']:9.2f}V {r['ms_below_4v63']:8.1f} {r['peak_motor_a']:7.2f}  {verdict}")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        for name, r in list(out.items())[:3]:
            t = [p[0] for p in r["trace"]]
            axes[0].plot(t, [p[1] for p in r["trace"]], label=name[:40])
            axes[1].plot(t, [p[2] for p in r["trace"]], label=name[:40])
            axes[2].plot(t, [p[3] for p in r["trace"]], label=name[:40])
        axes[1].axhline(4.63, color="red", ls="--", lw=0.8)
        axes[0].set_ylabel("battery bus [V]")
        axes[1].set_ylabel("Pi 5 V [V]")
        axes[2].set_ylabel("motor current, both [A]")
        axes[2].set_xlabel("time [s]  (start at 0.05 s, full reversal at 0.30 s)")
        axes[0].legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=120)
        print(f"saved {args.plot}")
    return out


if __name__ == "__main__":
    main()
