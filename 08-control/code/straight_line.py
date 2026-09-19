"""08.10 — Hold a straight line: an outer heading loop around the inner wheel-speed loops.

    python 08-control/code/straight_line.py --plot straight.png
    python 08-control/code/straight_line.py --metres 3 --speed 0.4
    python 08-control/code/straight_line.py --port /dev/ttyACM0   # real robot, ON THE FLOOR, 4 m clear

Cascade:  outer heading loop (here, 50 Hz)  ->  yaw-rate correction  ->  wheel-speed setpoints
          ->  inner wheel-speed PID (the Pico's, 100 Hz)  ->  duty.

Five ways to drive 3 m "straight", all at the same forward speed:

1. open loop        both wheels get the same speed setpoint; only the inner loops close
2. encoder heading  heading from the tick difference, outer PI drives it to zero
3. encoder heading, wheel radii calibrated (what lesson 09.05 measures for you)
4. gyro heading     integrate the yaw rate, no bias removal
5. gyro heading, bias removed by 1 s of standing still (this is what 08.11 uses)

Simulation prints the true lateral offset from ground truth; on the robot you measure it with a
tape measure (and the script prints each estimator's final heading so you can compare).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np

from control_lab import Target, add_target_args, is_sim, let_wheels_stop, plt_headless, run_loop
from pid_lab import PositionalPID, wheel_speeds_for
from robotlab.config import load_config

MAX_YAW_CORRECTION = 0.6  # rad/s: a heading loop may never spin the robot
KP_HEADING, KI_HEADING = 3.0, 1.0  # yaw rate per radian of heading error


# --- heading estimators ----------------------------------------------------------------------------
@dataclass
class EncoderHeading:
    """theta = (right distance - left distance) / wheel separation, from the encoder ticks.

    ``radius_ratio`` is right radius / left radius. Leave it at 1.0 unless you calibrated
    (09.05): the whole point of run 3 is to show what that one number is worth.
    """

    meters_per_tick: float
    wheel_separation_m: float
    radius_ratio: float = 1.0
    theta: float = 0.0
    _last: tuple[int, int] | None = field(default=None, repr=False)

    def update(self, state, dt: float) -> float:
        ticks = (state.left_ticks, state.right_ticks)
        if self._last is not None:
            left = (ticks[0] - self._last[0]) * self.meters_per_tick
            right = (ticks[1] - self._last[1]) * self.meters_per_tick * self.radius_ratio
            self.theta += (right - left) / self.wheel_separation_m
        self._last = ticks
        return self.theta


@dataclass
class GyroHeading:
    """theta = integral of the measured yaw rate, minus a constant bias."""

    read_gyro: object  # callable () -> rad/s
    bias_rad_s: float = 0.0
    theta: float = 0.0
    rate: float = 0.0

    def update(self, state, dt: float) -> float:
        self.rate = float(self.read_gyro()) - self.bias_rad_s  # type: ignore[operator]
        self.theta += self.rate * dt
        return self.theta


def measure_gyro_bias(base, seconds: float = 1.0, rate_hz: float = 50.0) -> float:
    """Average the yaw rate while the robot stands still. Do not touch it during this second."""
    samples = []

    def step(t, dt, state, extra):
        samples.append(base.gyro_z())
        return 0.0, 0.0

    run_loop(base, step, seconds, rate_hz=rate_hz, command="velocity")
    return float(np.mean(samples)) if samples else 0.0


# --- the run ---------------------------------------------------------------------------------------
def drive_straight(base, cfg, estimator, wheel_setpoint: float, seconds: float,
                   kp: float = KP_HEADING, ki: float = KI_HEADING, ramp_s: float = 0.4):
    """Drive forward for ``seconds``, correcting heading with ``estimator`` (None = open loop)."""
    controller = PositionalPID(kp, ki, output_min=-MAX_YAW_CORRECTION, output_max=MAX_YAW_CORRECTION)
    controller.reset()
    radius, separation = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m

    def step(t, dt, state, extra):
        speed = wheel_setpoint * min(t / ramp_s, 1.0)  # ramp up: a step would slip the wheels
        heading = estimator.update(state, dt) if estimator is not None else 0.0
        correction = controller.update(0.0, heading, dt) if estimator is not None else 0.0
        left, right = wheel_speeds_for(speed, correction, radius, separation)
        extra.update(heading=heading, correction=correction, speed=speed)
        if is_sim(base):
            pose = base.sim.pose
            extra.update(x=pose.x, y=pose.y, true_heading=pose.theta)
        return left, right

    log = run_loop(base, step, seconds, command="velocity")
    let_wheels_stop(base)
    return log


def summarise(log, is_simulation: bool) -> dict[str, float]:
    out = {"estimated_heading_deg": math.degrees(log["heading"][-1]),
           "max_correction": float(np.max(np.abs(log["correction"])))}
    if is_simulation:
        x, y = log["x"], log["y"]
        out.update(travelled_m=float(np.sum(np.hypot(np.diff(x), np.diff(y)))),
                   offset_cm=float(y[-1]) * 100.0,
                   max_offset_cm=float(np.max(np.abs(y))) * 100.0,
                   true_heading_deg=math.degrees(log["true_heading"][-1]))
        out["heading_error_deg"] = out["estimated_heading_deg"] - out["true_heading_deg"]
    return out


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    ap.add_argument("--metres", type=float, default=3.0, help="how far to drive")
    ap.add_argument("--speed", type=float, default=0.4, help="forward speed [m/s]")
    ap.add_argument("--kp", type=float, default=KP_HEADING, help="heading loop kp [rad/s per rad]")
    ap.add_argument("--ki", type=float, default=KI_HEADING, help="heading loop ki")
    ap.add_argument("--no-gyro", action="store_true", help="skip the runs that need an IMU")
    args = ap.parse_args(argv)

    cfg = load_config()
    d = cfg.drive
    wheel_setpoint = args.speed / d.wheel_radius_m
    seconds = args.metres / args.speed + 0.2  # the speed ramp costs about half of its own length
    results: dict = {}
    plots: dict = {}

    print(f"{args.metres:.1f} m at {args.speed:.2f} m/s = {wheel_setpoint:.2f} rad/s per wheel, "
          f"{seconds:.1f} s per run")
    print(f"outer heading loop: kp {args.kp} ki {args.ki} rad/s per rad, correction clamped to "
          f"+-{MAX_YAW_CORRECTION} rad/s")

    with Target(args, on_the_floor=f"a clear lane about {args.metres + 1:.0f} m long and 1 m wide") as base:
        simulation = is_sim(base)
        have_gyro = hasattr(base, "gyro_z") and not args.no_gyro

        def encoders(ratio: float = 1.0) -> EncoderHeading:
            return EncoderHeading(d.meters_per_tick, d.wheel_separation_m, radius_ratio=ratio)

        runs: list[tuple[str, object]] = [
            ("open loop (inner loops only)", None),
            ("encoder heading", encoders()),
        ]
        if simulation:
            # what 09.05 would measure on this simulated robot: true right radius / true left radius.
            # The tick-to-metre factor uses the NOMINAL radius, so the right wheel's estimated
            # distance must be scaled UP by exactly that ratio to become comparable with the left's.
            p = base.sim.params
            true_ratio = p.wheel_radius_scale_right / p.wheel_radius_scale_left
            runs.append((f"encoder heading, calibrated ({true_ratio:.4f})", encoders(true_ratio)))
        if have_gyro:
            runs.append(("gyro heading, raw", GyroHeading(base.gyro_z)))

        header = f"  {'run':38s} {'est. heading':>12s}"
        if simulation:
            header += f" {'true heading':>12s} {'offset':>9s} {'max offset':>11s} {'travelled':>10s}"
        print()
        print(header)
        for name, estimator in runs:
            if simulation:
                base.sim.reset((0.0, 0.0, 0.0))
            log = drive_straight(base, cfg, estimator, wheel_setpoint, seconds, args.kp, args.ki)
            r = summarise(log, simulation)
            results[name] = r
            plots[name] = log
            line = f"  {name:38s} {r['estimated_heading_deg']:+10.1f}  "
            if simulation:
                line += (f"{r['true_heading_deg']:+10.1f}   {r['offset_cm']:+7.1f}cm "
                         f"{r['max_offset_cm']:9.1f}cm {r['travelled_m']:9.3f}m")
            print(line)

        if have_gyro:
            bias = measure_gyro_bias(base)
            print(f"\n  gyro bias measured while standing still: {bias:+.5f} rad/s "
                  f"({math.degrees(bias):+.2f} deg/s -> {math.degrees(bias) * seconds:+.1f} deg over one run)")
            if simulation:
                base.sim.reset((0.0, 0.0, 0.0))
            log = drive_straight(base, cfg, GyroHeading(base.gyro_z, bias), wheel_setpoint, seconds,
                                 args.kp, args.ki)
            r = summarise(log, simulation)
            name = "gyro heading, bias removed"
            results[name] = r
            plots[name] = log
            line = f"  {name:38s} {r['estimated_heading_deg']:+10.1f}  "
            if simulation:
                line += (f"{r['true_heading_deg']:+10.1f}   {r['offset_cm']:+7.1f}cm "
                         f"{r['max_offset_cm']:9.1f}cm {r['travelled_m']:9.3f}m")
            print(line)

    if args.plot and plots:
        plt = plt_headless()
        has_path = "x" in plots[next(iter(plots))].rows[0]
        fig, axes = plt.subplots(1, 2 if has_path else 1, figsize=(12 if has_path else 6, 4))
        axes = np.atleast_1d(axes)
        if has_path:
            ax = axes[0]
            for name, log in plots.items():
                ax.plot(log["x"], log["y"] * 100.0, lw=1.3, label=name)
            ax.axhline(0.0, color="k", ls=":", lw=0.8)
            ax.set(xlabel="x [m]", ylabel="lateral offset y [cm]", title="true path (ground truth)")
            ax.grid(True)
            ax.legend(fontsize=7, loc="best")
        ax = axes[-1]
        for name, log in plots.items():
            ax.plot(log["t"], np.degrees(log["heading"]), lw=1.3, label=name)
        ax.set(xlabel="time [s]", ylabel="heading the controller believes in [deg]",
               title="what each estimator reports")
        ax.grid(True)
        ax.legend(fontsize=7, loc="best")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
