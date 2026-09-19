"""08.08 — Tuning PID systematically: from a logged step response to gains you can defend.

    python 08-control/code/pid_tuning.py                      # uses YOUR labs/exercises/08.07/student.py
    python 08-control/code/pid_tuning.py --solution --plot tuning.png
    python 08-control/code/pid_tuning.py --port /dev/ttyACM0  # real robot, WHEELS IN THE AIR

Part 1  The measurement everything else is built on: one open-loop duty step, fitted to
        K / (tau s + 1) * exp(-theta s).
Part 2  Gains from that model: lambda tuning at three speeds, and Ziegler-Nichols' 1942
        reaction-curve rules. Measured on the wheel, scored with IAE and ITAE.
Part 3  Ziegler-Nichols the closed-loop way: raise kp until it oscillates, read Ku and Tu.
Part 4  The same Ku and Tu without ever losing control: a relay experiment (auto-tune).
Part 5  What a computer does with the same simulator: a coordinate search on ITAE.

Everything runs the Pi-side loop at 50 Hz with the motor deadband compensated, so the plant the
theory talks about ("speed = K * u") is the plant the controller sees.
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from control_lab import Target, add_target_args, is_sim, let_wheels_stop, load_exercise, plt_headless, run_loop
from pid_lab import (
    MotorModel, PositionalPID, fit_first_order_plus_delay, iae, itae, relay_ultimate_gain,
    ziegler_nichols_open_loop, ziegler_nichols_ultimate,
)

SETPOINT, T_STEP, SECONDS = 8.0, 0.2, 1.6
RATE_HZ = 50.0


# --- the runs ---------------------------------------------------------------------------------------
def open_loop_step(base, model, duty_u: float = 0.45, seconds: float = 1.2):
    """A step of ``duty_u`` (after deadband compensation) on the left wheel: the tuning measurement."""
    def step(t, dt, state, extra):
        u = duty_u if t >= T_STEP - 1e-9 else 0.0
        extra["u"] = u
        return model.compensate_deadband(u), 0.0

    log = run_loop(base, step, seconds, rate_hz=RATE_HZ)
    let_wheels_stop(base)
    return log


def closed_loop(base, model, kp: float, ki: float, kd: float = 0.0, seconds: float = SECONDS,
                setpoint: float = SETPOINT):
    controller = PositionalPID(kp, ki, kd)
    controller.reset()

    def step(t, dt, state, extra):
        target = setpoint if t >= T_STEP - 1e-9 else 0.0
        u = controller.update(target, state.left_rad_s, dt)
        extra.update(setpoint=target, u=u)
        return model.compensate_deadband(u), 0.0

    log = run_loop(base, step, seconds, rate_hz=RATE_HZ)
    let_wheels_stop(base)
    return log


def relay_run(base, model, bias_u: float, amplitude: float = 0.08, seconds: float = 3.0,
              setpoint: float = SETPOINT):
    """Bang-bang around the setpoint: the auto-tuner. The loop can never run away — |u| <= bias + d."""
    def step(t, dt, state, extra):
        if t < T_STEP:
            u = 0.0
        else:
            u = bias_u + (amplitude if state.left_rad_s < setpoint else -amplitude)
        extra.update(setpoint=setpoint, u=u)
        return model.compensate_deadband(u), 0.0

    log = run_loop(base, step, seconds, rate_hz=RATE_HZ)
    let_wheels_stop(base)
    return log


# --- measuring an oscillation -------------------------------------------------------------------------
def oscillation(t: np.ndarray, y: np.ndarray, after: float) -> tuple[float, float]:
    """(half amplitude, period) of the steady oscillation after time ``after``; period = nan if none."""
    mask = t >= after
    tt, yy = t[mask], y[mask]
    if len(yy) < 8:
        return 0.0, math.nan
    centred = yy - np.mean(yy)
    amplitude = 0.5 * (np.max(yy) - np.min(yy))
    crossings = [tt[k] + (tt[k + 1] - tt[k]) * (-centred[k]) / (centred[k + 1] - centred[k])
                 for k in range(len(centred) - 1)
                 if centred[k] < 0 <= centred[k + 1]]  # rising zero crossings
    if len(crossings) < 2:
        return float(amplitude), math.nan
    return float(amplitude), float(np.mean(np.diff(crossings)))


def score(ex, log, setpoint: float = SETPOINT) -> dict[str, float]:
    t, y = log["t"], log["left_rad_s"]
    m = ex.step_metrics(t, y, T_STEP, 0.0, setpoint, settle_band=0.03)
    return {"rise_ms": m.rise_time * 1000.0, "overshoot": m.overshoot_percent, "settling": m.settling_time,
            "ss_error": m.steady_state_error, "iae": iae(t, y, setpoint), "itae": itae(t, y, setpoint, T_STEP)}


def print_score(name: str, kp: float, ki: float, s: dict[str, float]) -> None:
    print(f"  {name:26s} {kp:6.3f} {ki:6.2f} {s['rise_ms']:7.0f}ms {s['overshoot']:8.1f}% "
          f"{s['settling']:9.2f}s {s['iae']:7.2f} {s['itae']:7.3f}")


SCORE_HEADER = (f"  {'gains from':26s} {'kp':>6s} {'ki':>6s} {'rise':>9s} {'overshoot':>9s} "
                f"{'settling':>10s} {'IAE':>7s} {'ITAE':>7s}")


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    ap.add_argument("--solution", action="store_true", help="use the reference solution of 08.07 (metrics)")
    args = ap.parse_args(argv)

    ex = load_exercise("08.07", solution=args.solution)
    model = MotorModel.from_config()
    results: dict = {}
    plots: dict = {}

    with Target(args) as base:
        # --- Part 1 ------------------------------------------------------------------------
        duty_u = 0.45
        log = open_loop_step(base, model, duty_u)
        K, tau, theta = fit_first_order_plus_delay(log["t"], log["left_rad_s"], duty_u, T_STEP)
        results["fit"] = {"K": K, "tau": tau, "theta": theta}
        plots["open loop step"] = log
        print("Part 1 - one open-loop step, fitted to K/(tau s + 1) * exp(-theta s)")
        print(f"  step: u = {duty_u} after deadband compensation -> duty "
              f"{model.compensate_deadband(duty_u):.3f}, final speed "
              f"{np.mean(log['left_rad_s'][log['t'] > 1.0]):.2f} rad/s")
        print(f"  K = {K:.2f} rad/s per unit u   tau = {tau * 1000:.0f} ms   theta = {theta * 1000:.0f} ms")
        print(f"  karmel.yaml says the motor's own tau is {model.tau_s * 1000:.0f} ms; the fit sees the "
              "estimator's filter and the loop period on top of it")
        print(f"  controllability ratio theta/tau = {theta / tau:.2f} "
              f"({'easy' if theta / tau < 0.2 else 'delay-dominated'})")

        # --- Part 2 ------------------------------------------------------------------------
        print("\nPart 2 - gains computed from that model, measured on the wheel")
        print(SCORE_HEADER)
        designs: list[tuple[str, float, float]] = []
        for lam in (2.0, 1.0, 0.5):
            kp, ki = ex.pi_gains_lambda(K, tau, lam * tau)
            designs.append((f"lambda = {lam:g} x tau", kp, ki))
        zn_pi = ziegler_nichols_open_loop(K, tau, theta, "PI")
        designs.append(("Ziegler-Nichols PI", zn_pi[0], zn_pi[1]))
        for name, kp, ki in designs:
            log = closed_loop(base, model, kp, ki)
            s = score(ex, log)
            results[f"design: {name}"] = {"kp": kp, "ki": ki, **s}
            plots[name] = log
            print_score(name, kp, ki, s)

        # --- Part 3 ------------------------------------------------------------------------
        print("\nPart 3 - the closed-loop experiment: raise kp (P only) until it oscillates")
        print(f"  {'kp':>6s} {'oscillation':>12s} {'period':>9s} {'settled':>9s}")
        ku = tu = math.nan
        for kp in (0.2, 0.4, 0.6, 0.8, 1.2):
            log = closed_loop(base, model, kp, 0.0, seconds=3.0)
            amplitude, period = oscillation(log["t"], log["left_rad_s"], after=1.5)
            settled = float(np.mean(log["left_rad_s"][log["t"] > 2.5]))
            results[f"ku scan: {kp}"] = {"amplitude": amplitude, "period": period}
            print(f"  {kp:6.2f} {amplitude:9.2f} rad/s {period * 1000:7.0f}ms {settled:8.2f}")
            if math.isnan(ku) and amplitude > 0.8 and period == period:
                ku, tu = kp, period
                plots["at the ultimate gain"] = log
        results["ultimate"] = {"ku": ku, "tu": tu}
        print(f"  -> Ku ~ {ku:.2f}, Tu ~ {tu * 1000:.0f} ms (the first gain with a sustained wobble)")
        print(SCORE_HEADER)
        for kind in ("PI", "PID", "no_overshoot"):
            kp, ki, kd = ziegler_nichols_ultimate(ku, tu, kind)
            log = closed_loop(base, model, kp, ki, kd)
            s = score(ex, log)
            results[f"ZN {kind}"] = {"kp": kp, "ki": ki, "kd": kd, **s}
            plots[f"ZN {kind}"] = log
            print_score(f"ZN closed loop, {kind}", kp, ki, s)

        # --- Part 4 ------------------------------------------------------------------------
        bias_u = SETPOINT / model.top_speed(base.read().battery_v if is_sim(base) else None)
        amplitude_u = 0.2  # big enough that the limit cycle is far above the encoder's 0.13 rad/s step
        log = relay_run(base, model, bias_u, amplitude_u)
        a, period = oscillation(log["t"], log["left_rad_s"], after=1.0)
        ku_relay, tu_relay = relay_ultimate_gain(amplitude_u, a, period)
        results["relay"] = {"a": a, "period": period, "ku": ku_relay, "tu": tu_relay}
        plots["relay"] = log
        print(f"\nPart 4 - relay auto-tune: u = {bias_u:.2f} +- {amplitude_u}")
        print(f"  the wheel oscillates by +-{a:.2f} rad/s with a period of {period * 1000:.0f} ms")
        print(f"  Ku = 4d/(pi a) = {ku_relay:.2f} (Part 3 measured {ku:.2f}), Tu = {tu_relay * 1000:.0f} ms "
              f"(Part 3: {tu * 1000:.0f} ms)")

        # --- Part 5 ------------------------------------------------------------------------
        print("\nPart 5 - let the computer search: coordinate descent on ITAE (overshoot > 10 % rejected)")

        def cost(kp: float, ki: float) -> float:
            s = score(ex, closed_loop(base, model, kp, ki))
            penalty = 10.0 * max(0.0, s["overshoot"] - 10.0)
            return s["itae"] + penalty + (100.0 if not math.isfinite(s["settling"]) else 0.0)

        grid = [(kp, ki) for kp in (0.02, 0.04, 0.08, 0.16, 0.32) for ki in (0.3, 0.6, 1.2, 2.4, 4.8)]
        scored = sorted(((cost(kp, ki), kp, ki) for kp, ki in grid))
        evaluations = len(grid)
        best_cost, kp_best, ki_best = scored[0]
        print(f"  coarse grid, best three of {len(grid)}: " +
              ", ".join(f"kp {kp:.3f} ki {ki:.2f} (ITAE cost {c:.3f})" for c, kp, ki in scored[:3]))
        for step_factor in (1.4, 1.15):  # refine around the winner
            improved = True
            while improved:
                improved = False
                for kp_f, ki_f in ((step_factor, 1.0), (1 / step_factor, 1.0),
                                   (1.0, step_factor), (1.0, 1 / step_factor)):
                    c = cost(kp_best * kp_f, ki_best * ki_f)
                    evaluations += 1
                    if c < best_cost - 1e-6:
                        kp_best, ki_best, best_cost, improved = kp_best * kp_f, ki_best * ki_f, c, True
        log = closed_loop(base, model, kp_best, ki_best)
        s = score(ex, log)
        results["search"] = {"kp": kp_best, "ki": ki_best, "evaluations": evaluations, **s}
        plots["search"] = log
        print(f"  {evaluations} simulated step responses = about "
              f"{evaluations * SECONDS:.0f} s of simulated robot time, a few seconds of wall clock")
        print(SCORE_HEADER)
        print_score("the search", kp_best, ki_best, s)

    if args.plot and plots:
        plt = plt_headless()
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        ax = axes[0]
        log = plots["open loop step"]
        ax.plot(log["t"], log["left_rad_s"], lw=1.3, label="measured")
        fitted = np.where(log["t"] < T_STEP + theta, 0.0,
                          K * duty_u * (1 - np.exp(-(log["t"] - T_STEP - theta) / tau)))
        ax.plot(log["t"], fitted, "k--", lw=1.0, label=f"fit: K {K:.1f}, tau {tau * 1000:.0f} ms, "
                                                       f"theta {theta * 1000:.0f} ms")
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="1. the measurement")
        ax.grid(True)
        ax.legend(fontsize=8, loc="lower right")
        ax = axes[1]
        for name in [d[0] for d in designs]:
            ax.plot(plots[name]["t"], plots[name]["left_rad_s"], lw=1.2, label=name)
        ax.axhline(SETPOINT, color="k", ls="--", lw=0.8)
        ax.set(xlabel="time [s]", title="2. gains from the model", ylim=(0, 12))
        ax.grid(True)
        ax.legend(fontsize=8, loc="lower right")
        ax = axes[2]
        for name in ("ZN PI", "ZN no_overshoot", "search"):
            if name in plots:
                ax.plot(plots[name]["t"], plots[name]["left_rad_s"], lw=1.2, label=name)
        ax.axhline(SETPOINT, color="k", ls="--", lw=0.8)
        ax.set(xlabel="time [s]", title="3./5. Ziegler-Nichols vs a search", ylim=(0, 12))
        ax.grid(True)
        ax.legend(fontsize=8, loc="lower right")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
