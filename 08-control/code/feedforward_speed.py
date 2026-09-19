"""08.09 — Feedforward + PI: drive at exactly 0.5 m/s.

    python 08-control/code/feedforward_speed.py                      # uses labs/exercises/08.09/student.py
    python 08-control/code/feedforward_speed.py --solution --plot ff.png
    python 08-control/code/feedforward_speed.py --port /dev/ttyACM0  # real robot, WHEELS IN THE AIR

Part 1  PI only vs feedforward only vs feedforward + PI on a 0 -> 0.5 m/s step, with the firmware's
        gentle gains (kp 0.02, ki 0.3): how fast, how exact, how much work is left for feedback.
Part 2  What each term of the model buys, with the feedback switched off: no deadband, no battery
        compensation, full model.
Part 3  The same three controllers on a tired battery (15 % state of charge).
Part 4  A setpoint that keeps moving (0.5 -> 0.2 -> 0.5 m/s ramps): feedforward tracks, feedback chases.
Part 5  On the floor (simulator only): true ground speed, which is NOT the same as wheel speed.

One controller per wheel. The simulator's left motor matches the model exactly and the right one is
6 % weaker, so the two wheels show both halves of the story: feedforward is worth everything when
the model is right, and the integral is what saves you when it isn't.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import replace

import numpy as np

from control_lab import (
    Target, add_target_args, is_sim, let_wheels_stop, load_exercise, plt_headless, run_loop, settling_time,
    steady_value,
)
from pid_lab import MotorModel, iae
from robotlab.config import load_config

SPEED_M_S = 0.5
T_STEP = 0.3
SECONDS = 2.0
KP, KI = 0.02, 0.3  # labs/firmware/pico/config.py: gentle gains, because the feedforward does the work


class NoFeedforward:
    """Duck-typed stand-in for the model, so "PI only" runs through exactly the same code path."""

    def duty(self, setpoint_rad_s: float, battery_v: float | None = None) -> float:
        return 0.0


def run(base, ex, ff_model, kp: float, ki: float, profile, seconds: float = SECONDS,
        reset_pose: bool = True):
    """Drive both wheels along ``profile(t)`` (rad/s) with feedforward + PI, one controller each."""
    controllers = {"left": ex.FeedforwardPI(ff_model, kp, ki), "right": ex.FeedforwardPI(ff_model, kp, ki)}

    def step(t, dt, state, extra):
        setpoint = profile(t)
        left = controllers["left"].update(setpoint, state.left_rad_s, dt, state.battery_v)
        right = controllers["right"].update(setpoint, state.right_rad_s, dt, state.battery_v)
        extra.update(setpoint=setpoint, ff=controllers["left"].feedforward,
                     left_feedback=left - controllers["left"].feedforward,
                     right_feedback=right - controllers["right"].feedforward)
        if is_sim(base):
            extra.update(x=base.sim.pose.x, y=base.sim.pose.y)
        return left, right

    log = run_loop(base, step, seconds)
    let_wheels_stop(base)
    if is_sim(base) and reset_pose:
        base.sim.reset((0.0, 0.0, 0.0))
    return log


def summary(log, wheel_setpoint: float, radius_m: float) -> dict[str, float]:
    t = log["t"]
    out = {"ff_duty": float(max(log["ff"]))}
    for side in ("left", "right"):
        w = log[f"{side}_rad_s"]
        settled = steady_value(t, w, last_s=0.5)
        out[f"{side}_m_s"] = settled * radius_m
        out[f"{side}_error_pct"] = (settled - wheel_setpoint) / wheel_setpoint * 100.0
        out[f"{side}_settling_s"] = settling_time(t, w, wheel_setpoint, 0.02 * wheel_setpoint, t_start=T_STEP)
        out[f"{side}_peak_feedback"] = float(max(abs(log[f"{side}_feedback"])))
        out[f"{side}_iae"] = iae(t[t >= T_STEP], w[t >= T_STEP], wheel_setpoint)
    return out


HEADER = (f"  {'controller':28s} {'left m/s':>8s} {'err':>7s} {'right m/s':>9s} {'err':>7s} "
          f"{'settling':>9s} {'R |u_fb|':>9s} {'R IAE':>7s}")


def print_row(name: str, r: dict[str, float]) -> None:
    print(f"  {name:28s} {r['left_m_s']:8.3f} {r['left_error_pct']:+6.1f}% {r['right_m_s']:9.3f} "
          f"{r['right_error_pct']:+6.1f}% {r['right_settling_s']:8.2f}s {r['right_peak_feedback']:9.3f} "
          f"{r['right_iae']:7.2f}")


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    ap.add_argument("--solution", action="store_true", help="use the reference solution of 08.09")
    ap.add_argument("--speed", type=float, default=SPEED_M_S, help="target ground speed [m/s]")
    args = ap.parse_args(argv)

    ex = load_exercise("08.09", solution=args.solution)
    cfg = load_config()
    model = MotorModel.from_config(cfg)
    radius = cfg.drive.wheel_radius_m
    target = ex.wheel_speed_for(args.speed, radius)
    results: dict = {}
    plots: dict = {}

    full = ex.FeedforwardModel(model.max_wheel_speed_rad_s, model.deadband, model.nominal_v)
    no_deadband = replace(full, deadband=0.0)
    no_battery = replace(full, use_battery=False)

    def step_profile(t: float) -> float:
        return target if t >= T_STEP - 1e-9 else 0.0

    def tracking_profile(t: float) -> float:
        """Ramp up, hold, ramp down to 0.4x, hold, ramp back: the shape a motion profile produces."""
        low = 0.4 * target
        knots = [(0.0, 0.0), (0.3, 0.0), (0.8, target), (1.3, target), (1.8, low), (2.3, low),
                 (2.8, target), (3.5, target)]
        for (t0, v0), (t1, v1) in zip(knots, knots[1:]):
            if t < t1:
                return v0 + (v1 - v0) * (t - t0) / (t1 - t0)
        return target

    print(f"{args.speed:.2f} m/s with {radius * 1000:.0f} mm wheels = {target:.3f} rad/s per wheel "
          f"({target / (2 * math.pi) * 60:.0f} rpm), {target / model.max_wheel_speed_rad_s:.0%} of top speed")
    print(f"feedforward duty at the nominal {model.nominal_v:.1f} V: {full.duty(target):.3f}")

    with Target(args) as base:
        variants = [("PI only (kp 0.02, ki 0.3)", NoFeedforward(), KP, KI),
                    ("feedforward only", full, 0.0, 0.0),
                    ("feedforward + PI", full, KP, KI)]

        print(f"\nPart 1 - step 0 -> {args.speed} m/s, fresh battery")
        print(HEADER)
        for name, ff_model, kp, ki in variants:
            log = run(base, ex, ff_model, kp, ki, step_profile)
            r = summary(log, target, radius)
            results[f"fresh: {name}"] = r
            plots.setdefault("fresh", {})[name] = log
            print_row(name, r)

        print("\nPart 2 - feedforward ONLY, one model term at a time (nothing hides the error)")
        print(f"  {'feedforward model':28s} {'duty':>6s} {'left m/s':>9s} {'err':>8s} {'right m/s':>10s} {'err':>8s}")
        for name, ff_model in (("no deadband term", no_deadband), ("no battery term", no_battery),
                               ("full model", full)):
            log = run(base, ex, ff_model, 0.0, 0.0, step_profile)
            r = summary(log, target, radius)
            results[f"model: {name}"] = r
            print(f"  {name:28s} {r['ff_duty']:6.3f} {r['left_m_s']:9.3f} {r['left_error_pct']:+7.1f}% "
                  f"{r['right_m_s']:10.3f} {r['right_error_pct']:+7.1f}%")

        if is_sim(base):
            # --- Part 3: a tired battery ------------------------------------------------------
            base.sim.params = replace(base.sim.params, battery_initial_soc=0.15)
            base.sim.reset((0.0, 0.0, 0.0))
            battery = base.read().battery_v
            print(f"\nPart 3 - battery at 15 % state of charge ({battery:.2f} V under load)")
            print(HEADER)
            for name, ff_model, kp, ki in variants + [("feedforward only, nominal V", no_battery, 0.0, 0.0)]:
                log = run(base, ex, ff_model, kp, ki, step_profile)
                r = summary(log, target, radius)
                results[f"tired: {name}"] = r
                plots.setdefault("tired", {})[name] = log
                print_row(name, r)
            base.sim.params = replace(base.sim.params, battery_initial_soc=1.0)
            base.sim.reset((0.0, 0.0, 0.0))

        # --- Part 4: a setpoint that keeps moving ---------------------------------------------
        print(f"\nPart 4 - tracking a moving setpoint (ramp to {args.speed}, down to "
              f"{0.4 * args.speed:.1f}, back up), measured from t = 0.8 s")
        print(f"  {'controller':28s} {'RMS error':>12s} {'worst error':>13s}")
        for name, ff_model, kp, ki in variants:
            log = run(base, ex, ff_model, kp, ki, tracking_profile, seconds=3.5)
            after = log["t"] >= 0.8
            error = (log["setpoint"] - log["right_rad_s"])[after]
            r = {"rms_rad_s": float(np.sqrt(np.mean(error**2))), "worst": float(np.max(np.abs(error)))}
            results[f"tracking: {name}"] = r
            plots.setdefault("tracking", {})[name] = log
            print(f"  {name:28s} {r['rms_rad_s']:8.3f} rad/s {r['worst']:9.3f} rad/s")

        # --- Part 5: on the floor -------------------------------------------------------------
        if is_sim(base):
            seconds = SECONDS + 1.0
            print(f"\nPart 5 - on the floor: true path length / time over the last {seconds - 1.0:.0f} s")
            print(f"  {'controller':28s} {'ground speed':>13s} {'err':>7s} {'heading drift':>14s}")
            for name, ff_model, kp, ki in variants:
                base.sim.reset((0.0, 0.0, 0.0))
                log = run(base, ex, ff_model, kp, ki, step_profile, seconds=seconds, reset_pose=False)
                heading = math.degrees(base.sim.pose.theta)
                t, x, y = log["t"], log["x"], log["y"]
                window = t >= 1.0  # after the wheels are up to speed
                path = float(np.sum(np.hypot(np.diff(x[window]), np.diff(y[window]))))
                true_speed = path / (t[window][-1] - t[window][0])
                results[f"floor: {name}"] = {"true_speed": true_speed, "path_m": path,
                                             "heading_deg": heading}
                print(f"  {name:28s} {true_speed:11.3f} m/s "
                      f"{(true_speed - args.speed) / args.speed * 100:+6.1f}% {heading:+11.1f} deg")

    if args.plot and plots:
        plt = plt_headless()
        panels = [("fresh", f"right wheel, 0 -> {args.speed} m/s"), ("tired", "right wheel, battery at 15 %"),
                  ("tracking", "right wheel, moving setpoint")]
        panels = [p for p in panels if p[0] in plots]
        fig, axes = plt.subplots(1, len(panels), figsize=(5.2 * len(panels), 4), sharey=True)
        axes = np.atleast_1d(axes)
        for ax, (key, title) in zip(axes, panels):
            for name, log in plots[key].items():
                ax.plot(log["t"], log["right_rad_s"] * radius, lw=1.3, label=name)
            first = next(iter(plots[key].values()))
            ax.plot(first["t"], first["setpoint"] * radius, color="k", ls="--", lw=0.8, label="setpoint")
            ax.set(xlabel="time [s]", title=title)
            ax.grid(True)
            ax.legend(fontsize=8, loc="lower right")
        axes[0].set_ylabel("wheel rim speed [m/s]")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
