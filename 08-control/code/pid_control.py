"""08.06 — Derivative control: derivative kick, damping a laggy loop, and what noise does to D.

    python 08-control/code/pid_control.py                 # uses YOUR labs/exercises/08.06/student.py
    python 08-control/code/pid_control.py --solution --plot pid_control.png
    python 08-control/code/pid_control.py --port /dev/ttyACM0      # real robot, WHEELS IN THE AIR

Part 1  Kick: setpoint 4 -> 8 rad/s at t = 1 s. D computed on the error vs. on the measurement.
Part 2  Damping: PI(0.1, 2.0) with one extra 20 ms of delay overshoots and rings. Add kd = 0.002
        with derivative filters of 0, 10 and 40 ms.
Part 3  Noise: the same loops when the speed measurement carries extra noise (sigma 0.3 rad/s,
        e.g. a vibrating chassis). How much does the duty chatter?
"""

from __future__ import annotations

import argparse

import numpy as np

from control_lab import (
    Target, add_target_args, is_sim, let_wheels_stop, load_exercise, overshoot_percent, plt_headless, run_loop,
    settling_time, steady_value,
)

KP, KI, KD = 0.1, 2.0, 0.002


class DerivativeOnError:
    """The textbook form: D acts on d(error)/dt. Only for the kick demo — don't use it."""

    def __init__(self, kp: float, ki: float, kd: float) -> None:
        self.kp, self.ki, self.kd = kp, ki, kd
        self.reset()

    def reset(self) -> None:
        self.integral, self.d_term, self.output, self._last_error = 0.0, 0.0, 0.0, None

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        error = setpoint - measured
        self.d_term = 0.0 if self._last_error is None else self.kd * (error - self._last_error) / dt
        self._last_error = error
        self.integral = min(max(self.integral + self.ki * error * dt, -1.0), 1.0)
        self.output = min(max(self.kp * error + self.integral + self.d_term, -1.0), 1.0)
        return self.output


def run(base, controller, profile, seconds: float, delay_steps: int = 0, noise: float = 0.0, seed: int = 0):
    controller.reset()
    rng = np.random.default_rng(seed)

    def step(t, dt, state, extra):
        setpoint = profile(t)
        measured = state.left_rad_s + (rng.normal(0.0, noise) if noise else 0.0)
        duty = controller.update(setpoint, measured, dt)
        extra.update(setpoint=setpoint, measured=measured, d_term=controller.d_term)
        return duty, 0.0

    log = run_loop(base, step, seconds, delay_steps=delay_steps)
    let_wheels_stop(base)
    if is_sim(base):
        base.sim.reset((0.0, 0.0, 0.0))
    return log


def chatter(log, from_s: float) -> float:
    """RMS change of the duty between consecutive loop periods: how much the motor is being shaken."""
    cmd = log["left_cmd"][log["t"] >= from_s]
    return float(np.sqrt(np.mean(np.diff(cmd) ** 2)))


def main(argv: list[str] | None = None) -> dict[str, dict[str, float]]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    ap.add_argument("--solution", action="store_true", help="use the reference solution")
    args = ap.parse_args(argv)
    ex = load_exercise("08.06", solution=args.solution)
    results: dict[str, dict[str, float]] = {}
    kick_logs, damp_logs, noise_logs = {}, {}, {}
    variants = [("PI (kd = 0)", 0.0, 0.0), ("PID, no filter", KD, 0.0), ("PID, 10 ms filter", KD, 0.01),
                ("PID, 40 ms filter", KD, 0.04)]

    def step_4_to_8(t: float) -> float:
        return 4.0 if t < 1.0 else 8.0

    def step_0_to_8(t: float) -> float:
        return 0.0 if t < 0.2 else 8.0

    with Target(args) as base:
        print("Part 1 - setpoint 4 -> 8 rad/s at t = 1.0 s, kp 0.1, ki 2.0, kd 0.005")
        print(f"  {'derivative on':16s} {'D term at the step':>18s} {'duty before':>11s} {'duty at the step':>16s}")
        for name, controller in (("the error", DerivativeOnError(KP, KI, 0.005)),
                                 ("the measurement", ex.PIDController(KP, KI, 0.005))):
            log = run(base, controller, step_4_to_8, 1.6)
            k = int(np.argmax(log["setpoint"] > 4.0))  # first loop period with the new setpoint
            r = {"d_term_at_step": float(log["d_term"][k]), "duty_before": float(log["left_cmd"][k - 1]),
                 "duty_at_step": float(log["left_cmd"][k])}
            results[f"kick: {name}"] = r
            kick_logs[name] = log
            print(f"  {name:16s} {r['d_term_at_step']:18.3f} {r['duty_before']:11.3f} {r['duty_at_step']:16.3f}")

        print("\nPart 2 - step 0 -> 8 rad/s with 20 ms extra delay: PI vs PID (kp 0.1, ki 2.0)")
        print(f"  {'controller':20s} {'overshoot':>9s} {'settling (+-0.3)':>16s} {'duty chatter':>12s}")
        for name, kd, tau in variants:
            log = run(base, ex.PIDController(KP, KI, kd, tau), step_0_to_8, 3.0, delay_steps=1)
            t, w = log["t"], log["left_rad_s"]
            r = {"overshoot": overshoot_percent(w, 0.0, 8.0), "settling": settling_time(t, w, 8.0, 0.3, 0.2),
                 "final": steady_value(t, w, 1.0), "chatter": chatter(log, 2.0)}
            results[f"damping: {name}"] = r
            damp_logs[name] = log
            print(f"  {name:20s} {r['overshoot']:8.0f}% {r['settling']:13.2f} s {r['chatter']:12.4f}")

        print("\nPart 3 - same, holding 8 rad/s with measurement noise sigma = 0.3 rad/s")
        print(f"  {'controller':20s} {'speed std':>9s} {'duty chatter':>12s}")
        for name, kd, tau in variants:
            log = run(base, ex.PIDController(KP, KI, kd, tau), step_0_to_8, 3.0, delay_steps=1, noise=0.3,
                      seed=args.seed)
            t, w = log["t"], log["left_rad_s"]
            late = t >= 1.5
            r = {"speed_std": float(np.std(w[late])), "chatter": chatter(log, 1.5)}
            results[f"noise: {name}"] = r
            noise_logs[name] = log
            print(f"  {name:20s} {r['speed_std']:9.3f} {r['chatter']:12.4f}")

    if args.plot:
        plt = plt_headless()
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        ax = axes[0]
        for name, log in kick_logs.items():
            ax.plot(log["t"], log["left_cmd"], lw=1.4, label=f"duty, D on {name}")
        ax.set(xlabel="time [s]", ylabel="duty", title="1. derivative kick (setpoint 4 -> 8 at 1 s)", xlim=(0.8, 1.4))
        ax.legend(fontsize=8)
        ax = axes[1]
        for name, log in damp_logs.items():
            ax.plot(log["t"], log["left_rad_s"], lw=1.2, label=name)
        ax.axhline(8.0, color="k", ls="--", lw=1)
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="2. damping (20 ms extra delay)", xlim=(0, 1.5))
        ax.legend(fontsize=8, loc="lower right")
        ax = axes[2]
        for name in ("PI (kd = 0)", "PID, no filter", "PID, 40 ms filter"):
            log = noise_logs[name]
            window = (log["t"] >= 2.0) & (log["t"] <= 2.5)
            ax.step(log["t"][window], log["left_cmd"][window], where="post", lw=1.2, label=name)
        ax.set(xlabel="time [s]", ylabel="duty sent", title="3. duty with noisy measurement")
        ax.legend(fontsize=8)
        for a in axes:
            a.grid(True)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
