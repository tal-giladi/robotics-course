"""08.07 — PID mathematically: forms, sampling, model-based gains and why delay destabilises.

    python 08-control/code/pid_math.py                      # uses YOUR labs/exercises/08.07/student.py
    python 08-control/code/pid_math.py --solution --plot pid_math.png
    python 08-control/code/pid_math.py --port /dev/ttyACM0  # real robot, WHEELS IN THE AIR

Part 1  Positional vs velocity form of the same PI, on a normal step and on the windup scenario
        of 08.05 (25 rad/s for 2 s, then 6 rad/s), with three anti-windup variants.
Part 2  The same continuous gains run at 100, 50 and 25 Hz, and the "forgot dt" bug.
Part 3  Gains from the motor model (lambda tuning and pole placement): what the linear model
        predicts vs what the (deadband-compensated) wheel does.
Part 4  One design, more and more delay: predicted phase margin vs measured overshoot.

The loops run on the Pi side (control_lab.run_loop), like 08.04-08.06.
"""

from __future__ import annotations

import argparse
import math

from control_lab import (
    Target, add_target_args, is_sim, let_wheels_stop, load_exercise, oscillation_std, plt_headless, run_loop,
)
from pid_lab import MotorModel, PositionalPID, VelocityFormPID, lambda_phase_margin_deg, simulate_pi_loop

SETPOINT, T_STEP = 8.0, 0.2
MEASUREMENT_LAG_S = 0.015  # the firmware speed estimate (08.02, 08.03)


def run(base, controller, profile, seconds: float, rate_hz: float = 50.0, delay_steps: int = 0,
        model: MotorModel | None = None):
    """Run ``controller`` on the left wheel. With ``model``, the output goes through deadband compensation."""
    controller.reset()

    def step(t, dt, state, extra):
        setpoint = profile(t)
        u = controller.update(setpoint, state.left_rad_s, dt)
        extra.update(setpoint=setpoint, u=u, integral=getattr(controller, "integral", math.nan))
        duty = model.compensate_deadband(u) if model else u
        return duty, 0.0

    log = run_loop(base, step, seconds, rate_hz=rate_hz, delay_steps=delay_steps)
    let_wheels_stop(base)
    if is_sim(base):
        base.sim.reset((0.0, 0.0, 0.0))
    return log


def step_profile(t: float) -> float:
    return SETPOINT if t >= T_STEP - 1e-9 else 0.0


def windup_profile(t: float) -> float:
    return 0.0 if t < T_STEP else (25.0 if t < 2.2 else 6.0)


def recovery_time(log, t_change: float = 2.2, target: float = 6.0, band: float = 0.5) -> float:
    t, w = log["t"], log["left_rad_s"]
    after = t >= t_change
    outside = [tt for tt, ww in zip(t[after], w[after]) if abs(ww - target) > band]
    return (outside[-1] + 0.02 - t_change) if outside else 0.0


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    ap.add_argument("--solution", action="store_true", help="use the reference solution of 08.07")
    args = ap.parse_args(argv)
    ex = load_exercise("08.07", solution=args.solution)
    model = MotorModel.from_config()
    results: dict = {}
    plots: dict = {}

    with Target(args) as base:
        battery = base.read().battery_v if is_sim(base) else None
        K = model.top_speed(battery)  # plant gain after deadband compensation, rad/s per unit u

        # --- Part 1 ---------------------------------------------------------------------------
        print("Part 1 - positional vs velocity form, PI kp 0.1 ki 2.0 at 50 Hz (no deadband compensation)")
        a = run(base, PositionalPID(0.1, 2.0), step_profile, 1.5)
        b = run(base, VelocityFormPID(0.1, 2.0), step_profile, 1.5)
        diff = float(max(abs(a["u"] - b["u"])))
        print(f"  0 -> 8 rad/s step: largest output difference between the forms = {diff:.4f} duty")
        results["forms_max_diff"] = diff
        print(f"  {'windup scenario':30s} {'integral at 2.2 s':>17s} {'speed at 2.1 s':>14s} {'recovery (+-0.5)':>16s}")
        variants = [("positional, no anti-windup", PositionalPID(0.1, 2.0, anti_windup="none")),
                    ("positional, fill (firmware)", PositionalPID(0.1, 2.0)),
                    ("positional, back-calc 50 ms", PositionalPID(0.1, 2.0, anti_windup="back_calculation")),
                    ("velocity form", VelocityFormPID(0.1, 2.0))]
        for name, controller in variants:
            log = run(base, controller, windup_profile, 3.5)
            t = log["t"]
            k21, k22 = int((abs(t - 2.1)).argmin()), int((abs(t - 2.2)).argmin())
            integral = log["integral"][k22]
            r = {"speed_2_1": float(log["left_rad_s"][k21]), "recovery": recovery_time(log),
                 "integral": float(integral)}
            results[f"windup: {name}"] = r
            plots.setdefault("windup", {})[name] = log
            shown = "   (none)" if math.isnan(integral) else f"{integral:17.2f}"
            print(f"  {name:30s} {shown:>17s} {r['speed_2_1']:14.2f} {r['recovery']:14.2f} s")

        # --- Part 2 ---------------------------------------------------------------------------
        kp, ki = ex.pi_gains_lambda(K, model.tau_s, 0.05)
        print(f"\nPart 2 - lambda design tau_cl = 50 ms (kp {kp:.3f}, ki {ki:.2f}), deadband compensated")
        print(f"  {'loop':26s} {'rise':>7s} {'overshoot':>9s} {'settling 3%':>11s} {'ss error':>8s}")
        for name, rate, ki_used in (("100 Hz", 100.0, ki), ("50 Hz", 50.0, ki), ("25 Hz", 25.0, ki),
                                    ("100 Hz, ki*e without dt*", 100.0, ki * 2.0)):
            log = run(base, PositionalPID(kp, ki_used), step_profile, 1.5, rate_hz=rate, model=model)
            m = ex.step_metrics(log["t"], log["left_rad_s"], T_STEP, 0.0, SETPOINT, settle_band=0.03)
            results[f"rate: {name}"] = m
            plots.setdefault("rate", {})[name] = log
            print(f"  {name:26s} {m.rise_time * 1000:5.0f}ms {m.overshoot_percent:8.1f}% "
                  f"{m.settling_time:10.2f}s {m.steady_state_error:8.2f}")
        print("  * the bug: integral += ki_per_step * e, with ki_per_step tuned at 50 Hz; at 100 Hz it integrates twice as fast")

        # --- Part 3 ---------------------------------------------------------------------------
        print(f"\nPart 3 - gains from the model K = {K:.2f} rad/s per u, tau = {model.tau_s * 1000:.0f} ms, 50 Hz")
        print(f"  {'design':24s} {'kp':>6s} {'ki':>6s} | {'rise / overshoot: ideal model':>29s} | "
              f"{'model + 50 Hz + lag':>19s} | {'wheel':>15s}")
        designs = [(f"lambda {tc * 1000:.0f} ms", ex.pi_gains_lambda(K, model.tau_s, tc)) for tc in (0.2, 0.1, 0.05, 0.03)]
        designs.append(("poles zeta 0.7 wn 25", ex.pi_gains_pole_placement(K, model.tau_s, 0.7, 25.0)))
        for name, (kp_d, ki_d) in designs:
            ideal = ex.step_metrics(*simulate_pi_loop(K, model.tau_s, kp_d, ki_d, loop_dt=0.0005, seconds=1.5), 0.0, 0.0, 8.0)
            sampled = ex.step_metrics(*simulate_pi_loop(K, model.tau_s, kp_d, ki_d, measurement_lag_s=MEASUREMENT_LAG_S,
                                                        seconds=1.5), 0.0, 0.0, 8.0)
            log = run(base, PositionalPID(kp_d, ki_d), step_profile, 1.7, model=model)
            wheel = ex.step_metrics(log["t"], log["left_rad_s"], T_STEP, 0.0, SETPOINT)
            results[f"design: {name}"] = {"kp": kp_d, "ki": ki_d, "ideal": ideal, "sampled": sampled, "wheel": wheel}
            plots.setdefault("design", {})[name] = log
            print(f"  {name:24s} {kp_d:6.3f} {ki_d:6.2f} | {ideal.rise_time * 1000:13.0f} ms {ideal.overshoot_percent:9.1f}% | "
                  f"{sampled.rise_time * 1000:6.0f} ms {sampled.overshoot_percent:6.1f}% | "
                  f"{wheel.rise_time * 1000:4.0f} ms {wheel.overshoot_percent:5.1f}%")

        # --- Part 4 ---------------------------------------------------------------------------
        tau_cl = 0.05
        kp, ki = ex.pi_gains_lambda(K, model.tau_s, tau_cl)
        print(f"\nPart 4 - lambda 50 ms at 50 Hz with extra delay: theta = 10 ms (half period) + 15 ms (estimate) + extra")
        print(f"  {'extra delay':>11s} {'theta':>6s} {'phase margin':>12s} {'overshoot':>9s} {'settling 3%':>11s} {'wobble':>6s}")
        for n in range(4):
            theta = 0.010 + MEASUREMENT_LAG_S + 0.02 * n
            pm = lambda_phase_margin_deg(tau_cl, theta)
            log = run(base, PositionalPID(kp, ki), step_profile, 2.5, delay_steps=n, model=model)
            m = ex.step_metrics(log["t"], log["left_rad_s"], T_STEP, 0.0, SETPOINT, settle_band=0.03)
            wobble = oscillation_std(log["t"], log["left_rad_s"])
            results[f"delay: {n}"] = {"theta": theta, "pm": pm, "metrics": m, "wobble": wobble}
            plots.setdefault("delay", {})[f"+{20 * n} ms"] = log
            print(f"  {20 * n:8d} ms {theta * 1000:4.0f}ms {pm:10.0f} deg {m.overshoot_percent:8.0f}% "
                  f"{m.settling_time:10.2f}s {wobble:6.2f}")

    if args.plot:
        plt = plt_headless()
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        ax = axes[0]
        for name, log in plots["windup"].items():
            ax.plot(log["t"], log["left_rad_s"], lw=1.2, label=name)
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="1. windup: 25 rad/s, then 6 rad/s at 2.2 s")
        ax.legend(fontsize=7)
        ax = axes[1]
        for name, log in plots["design"].items():
            ax.plot(log["t"], log["left_rad_s"], lw=1.2, label=name)
        ax.axhline(SETPOINT, color="k", ls="--", lw=0.8)
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="3. model-based PI designs (50 Hz)", xlim=(0, 1.2))
        ax.legend(fontsize=7, loc="lower right")
        ax = axes[2]
        for name, log in plots["delay"].items():
            ax.plot(log["t"], log["left_rad_s"], lw=1.2, label=f"extra delay {name}")
        ax.axhline(SETPOINT, color="k", ls="--", lw=0.8)
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="4. lambda 50 ms + delay", xlim=(0, 1.5))
        ax.legend(fontsize=7, loc="lower right")
        for a_ in axes:
            a_.grid(True)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
