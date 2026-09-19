"""08.11 — Rotate exactly 90°: a trapezoidal profile, IMU yaw feedback and wrap-safe angles.

    python 08-control/code/rotate_90.py                       # uses labs/exercises/08.11/student.py
    python 08-control/code/rotate_90.py --solution --plot rotate.png
    python 08-control/code/rotate_90.py --port /dev/ttyACM0   # real robot, ON THE FLOOR, space around it

Part 1  No profile: a step target of 90 deg with P only, three gains. Watch it overshoot or crawl.
Part 2  Trapezoidal profile + feedforward + PI: the same turn, done properly.
Part 3  Where the heading comes from: gyro (bias removed) vs the encoder difference.
Part 4  Repeatability: 10 turns of 90 deg from 10 different seeds — mean, spread, worst.
Part 5  Wrap-around: start at +170 deg and turn +90 deg. The answer is -100 deg, not +260 deg.

The inner loop is the Pico's wheel-speed PID (velocity mode, 100 Hz); this script only decides a
yaw rate and converts it to two wheel-speed setpoints.
"""

from __future__ import annotations

import argparse
import math

import numpy as np

from control_lab import Target, add_target_args, is_sim, let_wheels_stop, load_exercise, plt_headless, run_loop
from pid_lab import PositionalPID, wheel_speeds_for
from robotlab.config import load_config
from straight_line import EncoderHeading, GyroHeading, measure_gyro_bias

V_MAX, A_MAX = 1.5, 3.0  # rad/s, rad/s^2 for the profile
MAX_YAW = 2.5  # karmel.yaml drive.max_angular_speed_rad_s
KP, KI = 4.0, 1.0
SETTLE_S = 0.8
DELAY_STEPS = 1  # one 20 ms period of extra lag: the outer loop lives on the Pi, not on the Pico


def has_gyro(base) -> bool:
    """SimBase has one; a real robot only if an IMU is wired to the Pi (lesson 07.05)."""
    return hasattr(base, "gyro_z")


def make_heading(base, cfg, source: str, bias: float = 0.0):
    if source == "gyro":
        return GyroHeading(base.gyro_z, bias)
    return EncoderHeading(cfg.drive.meters_per_tick, cfg.drive.wheel_separation_m)


def turn(base, cfg, ex, degrees: float, controller_factory, source: str | None = None, bias: float = 0.0,
         seconds: float | None = None, start_heading: float = 0.0):
    """Run one turn; returns the log. ``controller_factory(start_heading)`` builds the controller."""
    if source is None:  # without an IMU the encoders are all there is (and Part 3 shows what that costs)
        source = "gyro" if has_gyro(base) else "encoder"
    estimator = make_heading(base, cfg, source, bias)
    estimator.theta = start_heading
    controller = controller_factory(start_heading)
    radius, separation = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    duration = seconds if seconds is not None else controller.profile.duration + SETTLE_S

    def step(t, dt, state, extra):
        heading = estimator.update(state, dt)
        yaw = controller.update(t, heading, dt)
        left, right = wheel_speeds_for(0.0, yaw, radius, separation)
        extra.update(heading=heading, yaw_cmd=yaw, wheel_cmd=right,
                     target=getattr(controller, "target", lambda _t: math.nan)(t))
        if is_sim(base):
            extra["true_heading"] = base.sim.pose.theta
        return left, right

    log = run_loop(base, step, duration, command="velocity", delay_steps=DELAY_STEPS)
    let_wheels_stop(base)
    return log


class StepController:
    """Part 1's straw man: no profile at all, just P on the wrapped heading error."""

    def __init__(self, ex, target_rad: float, kp: float, start: float = 0.0) -> None:
        self.ex = ex
        self.final = ex.heading_error(start + target_rad, 0.0)  # wrapped absolute target
        self.pid = PositionalPID(kp, 0.0, output_min=-MAX_YAW, output_max=MAX_YAW)
        self.pid.reset()
        self.profile = type("_P", (), {"duration": 0.0})()

    def target(self, t: float) -> float:
        return self.final

    def update(self, t: float, measured: float, dt: float) -> float:
        # PositionalPID subtracts setpoint - measured, which knows nothing about wrapping.
        # Feed it the already-wrapped error as the setpoint and 0 as the measurement.
        return self.pid.update(self.ex.heading_error(self.final, measured), 0.0, dt)


def final_error_deg(log, degrees: float, start_deg: float = 0.0, simulation: bool = True) -> float:
    key = "true_heading" if simulation else "heading"
    end = math.degrees(log[key][-1])
    wanted = start_deg + degrees
    return (end - wanted + 180.0) % 360.0 - 180.0


def overshoot_deg(log, degrees: float, simulation: bool = True) -> float:
    key = "true_heading" if simulation else "heading"
    unwrapped = np.degrees(np.unwrap(log[key]))
    peak = np.max(unwrapped) if degrees > 0 else np.min(unwrapped)
    return float(max(0.0, (peak - degrees) * (1 if degrees > 0 else -1)))


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    ap.add_argument("--solution", action="store_true", help="use the reference solution of 08.11")
    ap.add_argument("--degrees", type=float, default=90.0, help="how far to turn")
    ap.add_argument("--trials", type=int, default=10, help="Part 4: how many repeats")
    args = ap.parse_args(argv)

    ex = load_exercise("08.11", solution=args.solution)
    cfg = load_config()
    radians = math.radians(args.degrees)
    profile = ex.TrapezoidalProfile(radians, V_MAX, A_MAX)
    results: dict = {}
    plots: dict = {}

    print(f"turn {args.degrees:.0f} deg = {radians:.4f} rad, v_max {V_MAX} rad/s, a_max {A_MAX} rad/s^2")
    print(f"profile: ramp {profile.t_accel:.2f} s, cruise {profile.t_cruise:.2f} s, "
          f"total {profile.duration:.2f} s, peak {profile.peak_v:.2f} rad/s "
          f"(wheels +-{profile.peak_v * cfg.drive.wheel_separation_m / (2 * cfg.drive.wheel_radius_m):.2f} rad/s)")

    def profiled(start: float):
        return ex.TurnController(ex.TrapezoidalProfile(radians, V_MAX, A_MAX), start, KP, KI, MAX_YAW)

    with Target(args, on_the_floor="a clear circle at least 1 m across") as base:
        simulation = is_sim(base)
        bias = measure_gyro_bias(base) if has_gyro(base) else 0.0
        if has_gyro(base):
            print(f"gyro bias while standing still: {bias:+.5f} rad/s ({math.degrees(bias):+.2f} deg/s)")

        def reset() -> None:
            if simulation:
                base.sim.reset((0.0, 0.0, 0.0))

        # --- Part 1 -------------------------------------------------------------------------
        print(f"\nPart 1 - step target, P only (no profile), {SETTLE_S + profile.duration:.1f} s per run")
        print(f"  {'controller':24s} {'final error':>12s} {'overshoot':>10s} {'biggest wheel jump':>19s} {'wobble':>8s}")
        for kp in (2.0, 6.0, 15.0):
            reset()
            name = f"P only, kp = {kp:g}"
            log = turn(base, cfg, ex, radians, lambda s, kp=kp: StepController(ex, radians, kp, s),
                       bias=bias, seconds=profile.duration + SETTLE_S)
            r = {"error": final_error_deg(log, args.degrees, simulation=simulation),
                 "overshoot": overshoot_deg(log, args.degrees, simulation),
                 "peak_yaw": float(np.max(np.abs(log["yaw_cmd"]))),
                 "wheel_jump": float(np.max(np.abs(np.diff(np.concatenate(([0.0], log["wheel_cmd"])))))),
                 "wobble": float(np.std(np.degrees(log["true_heading" if simulation else "heading"])[-25:]))}
            results[name] = r
            plots[name] = log
            print(f"  {name:24s} {r['error']:+10.2f} deg {r['overshoot']:8.2f} deg "
                  f"{r['wheel_jump']:14.2f} rad/s {r['wobble']:7.2f} deg")

        # --- Part 2 -------------------------------------------------------------------------
        print("\nPart 2 - trapezoidal profile + feedforward + PI (kp 4, ki 1)")
        reset()
        log = turn(base, cfg, ex, radians, profiled, bias=bias)
        r = {"error": final_error_deg(log, args.degrees, simulation=simulation),
             "overshoot": overshoot_deg(log, args.degrees, simulation),
             "peak_yaw": float(np.max(np.abs(log["yaw_cmd"]))),
             "wheel_jump": float(np.max(np.abs(np.diff(np.concatenate(([0.0], log["wheel_cmd"])))))),
             "duration": float(log["t"][-1])}
        results["profile + PI"] = r
        plots["profile + PI"] = log
        print(f"  final error {r['error']:+.2f} deg, overshoot {r['overshoot']:.2f} deg, "
              f"peak yaw {r['peak_yaw']:.2f} rad/s, biggest wheel jump {r['wheel_jump']:.2f} rad/s, "
              f"done in {profile.duration:.2f} s")

        # --- Part 3 -------------------------------------------------------------------------
        if has_gyro(base):
            print("\nPart 3 - where the heading comes from")
            print(f"  {'source':28s} {'believed':>10s} {'true':>10s} {'error':>10s}")
            for name, source, b in (("gyro, bias removed", "gyro", bias), ("gyro, raw", "gyro", 0.0),
                                    ("encoder difference", "encoder", 0.0)):
                reset()
                log = turn(base, cfg, ex, radians, profiled, source=source, bias=b)
                believed = math.degrees(log["heading"][-1])
                true = math.degrees(log["true_heading"][-1]) if simulation else math.nan
                results[f"source: {name}"] = {"believed": believed, "true": true, "error": true - args.degrees}
                plots[f"source: {name}"] = log
                print(f"  {name:28s} {believed:8.2f} deg {true:8.2f} deg {true - args.degrees:+8.2f} deg")

        # --- Part 4 -------------------------------------------------------------------------
        print(f"\nPart 4 - repeatability: {args.trials} turns of {args.degrees:.0f} deg")
        errors = []
        for trial in range(args.trials):
            if simulation:
                base.sim.reset((0.0, 0.0, 0.0))
                base.sim.rng = np.random.default_rng(1000 + trial)
            log = turn(base, cfg, ex, radians, profiled, bias=bias)
            errors.append(final_error_deg(log, args.degrees, simulation=simulation))
        errors_arr = np.array(errors)
        results["repeatability"] = {"mean": float(errors_arr.mean()), "std": float(errors_arr.std()),
                                    "worst": float(np.max(np.abs(errors_arr))), "errors": errors}
        print("  errors [deg]: " + " ".join(f"{e:+.2f}" for e in errors))
        print(f"  mean {errors_arr.mean():+.2f}  std {errors_arr.std():.2f}  "
              f"worst |error| {np.max(np.abs(errors_arr)):.2f}  -> "
              f"{'PASS' if np.max(np.abs(errors_arr)) <= 2.0 else 'FAIL'} the +-2 deg criterion")

        # --- Part 5 -------------------------------------------------------------------------
        if simulation:
            start_deg = 170.0
            print(f"\nPart 5 - wrap-around: start at {start_deg:.0f} deg, turn {args.degrees:+.0f} deg")
            base.sim.reset((0.0, 0.0, math.radians(start_deg)))
            log = turn(base, cfg, ex, radians, profiled, bias=bias, start_heading=math.radians(start_deg))
            end = math.degrees(log["true_heading"][-1])
            wanted = (start_deg + args.degrees + 180.0) % 360.0 - 180.0
            swept = float(np.degrees(np.unwrap(log["true_heading"]))[-1] - start_deg)
            results["wrap"] = {"end_deg": end, "wanted_deg": wanted, "swept_deg": swept}
            print(f"  ended at {end:+.2f} deg (wanted {wanted:+.2f}), total sweep {swept:+.2f} deg — "
                  f"{'short way' if abs(swept) < 180 else 'THE LONG WAY ROUND'}")

    if args.plot and plots:
        plt = plt_headless()
        key = "true_heading" if "true_heading" in plots[next(iter(plots))].rows[0] else "heading"
        fig, axes = plt.subplots(1, 2, figsize=(12, 4))
        ax = axes[0]
        for name in [n for n in plots if n.startswith("P only")] + ["profile + PI"]:
            ax.plot(plots[name]["t"], np.degrees(plots[name][key]), lw=1.3, label=name)
        ax.axhline(args.degrees, color="k", ls="--", lw=0.8)
        ax.axhspan(args.degrees - 2, args.degrees + 2, color="green", alpha=0.12, label="+-2 deg")
        ax.set(xlabel="time [s]", ylabel="true heading [deg]", title="step target vs motion profile")
        ax.grid(True)
        ax.legend(fontsize=8, loc="lower right")
        ax = axes[1]
        log = plots["profile + PI"]
        ax.plot(log["t"], np.degrees(log["target"]), "k--", lw=1.0, label="profile position")
        ax.plot(log["t"], np.degrees(log[key]), lw=1.3, label="true heading")
        ax2 = ax.twinx()
        ax2.plot(log["t"], log["yaw_cmd"], color="tab:red", lw=1.0, label="yaw command")
        ax2.set_ylabel("yaw rate command [rad/s]", color="tab:red")
        ax.set(xlabel="time [s]", ylabel="heading [deg]", title="the profile and what it commands")
        ax.grid(True)
        ax.legend(fontsize=8, loc="lower right")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
