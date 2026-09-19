"""01.11 — Measure the duty→speed curve of YOUR motors, and the mismatch between them.

    python 01-first-robot/code/duty_calibration.py --fake
    python 01-first-robot/code/duty_calibration.py --fake --realistic --plot duty.png
    python 01-first-robot/code/duty_calibration.py --port /dev/ttyACM0   # WHEELS IN THE AIR

For each duty in --duties it drives both wheels at that duty, waits --settle seconds for the
speed to settle, then counts encoder ticks for --measure seconds. Encoder ticks are the
measurement: ticks / time / ticks-per-revolution gives the wheel speed, and the wheel radius
turns it into metres per second at the rim.

It then fits a straight line to the points that actually moved:

    v = gain * duty + offset          ->  deadband duty = -offset / gain

and reports each wheel's gain. The **ratio** of the two gains is why an open-loop robot curves:
a 6 % weaker right motor turns the robot to the right at roughly (1 - ratio) * v / track rad/s.

SAFETY: on the real robot, run this with the wheels off the ground (a box under the chassis)
and a hand near the power switch. Ctrl-C stops the motors.
"""

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

from first_robot_lab import add_target_args, open_target, plt_headless

DEFAULT_DUTIES = (0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.60, 0.80, 1.00)


@dataclass(frozen=True)
class Sample:
    duty: float
    left_m_s: float
    right_m_s: float


def fit_line(xs: list[float], ys: list[float]) -> tuple[float, float]:
    """Least-squares fit ``y = gain * x + offset`` (two points are enough; more is better)."""
    n = len(xs)
    if n < 2:
        raise ValueError("need at least two moving samples to fit a line")
    mean_x, mean_y = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mean_x) ** 2 for x in xs)
    sxy = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    if sxx == 0:
        raise ValueError("all samples have the same duty")
    gain = sxy / sxx
    return gain, mean_y - gain * mean_x


def measure(target, duty: float, settle_s: float, measure_s: float) -> Sample:
    """Drive both wheels at ``duty`` and measure how fast the rims actually move."""
    base, params = target.base, target.params
    deadline = time.monotonic() + settle_s
    while time.monotonic() < deadline:
        base.set_wheel_duty(duty, duty)  # SAFETY: the wheels turn here
        time.sleep(0.02)
    start = base.wait_for_telemetry(timeout_s=1.0)
    deadline = time.monotonic() + measure_s
    while time.monotonic() < deadline:
        base.set_wheel_duty(duty, duty)
        time.sleep(0.02)
    end = base.wait_for_telemetry(timeout_s=1.0, newer_than=start.t)
    base.stop()
    dt = end.t - start.t
    left = (end.left_ticks - start.left_ticks) * params.meters_per_tick / dt
    right = (end.right_ticks - start.right_ticks) * params.meters_per_tick / dt
    return Sample(duty, left, right)


def analyse(samples: list[Sample], wheel_separation_m: float) -> dict[str, float]:
    moving = [s for s in samples if max(abs(s.left_m_s), abs(s.right_m_s)) > 0.01]
    duties = [s.duty for s in moving]
    left_gain, left_offset = fit_line(duties, [s.left_m_s for s in moving])
    right_gain, right_offset = fit_line(duties, [s.right_m_s for s in moving])
    ratio = right_gain / left_gain
    fastest = max(moving, key=lambda s: s.duty)
    speed = 0.5 * (abs(fastest.left_m_s) + abs(fastest.right_m_s))
    return {
        "left_gain_m_s": left_gain,
        "right_gain_m_s": right_gain,
        "left_deadband": -left_offset / left_gain,
        "right_deadband": -right_offset / right_gain,
        "gain_ratio": ratio,
        # v_left - v_right over the track width: the heading rate an open-loop "straight" drive gets.
        "yaw_rate_at_full_rad_s": (1.0 - ratio) * speed / wheel_separation_m,
    }


def main(argv: list[str] | None = None) -> dict[str, float]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duties", type=float, nargs="+", default=list(DEFAULT_DUTIES),
                    help="duty values to measure, 0..1")
    ap.add_argument("--settle", type=float, default=0.6, help="seconds to let the speed settle")
    ap.add_argument("--measure", type=float, default=1.5, help="seconds of tick counting per duty")
    ap.add_argument("--plot", help="write a PNG of the curve here")
    add_target_args(ap)
    args = ap.parse_args(argv)
    if any(not 0.0 <= d <= 1.0 for d in args.duties):
        ap.error("--duties must be between 0 and 1")

    samples: list[Sample] = []
    with open_target(args) as target:
        print(f"{'duty':>5} {'left m/s':>9} {'right m/s':>10} {'right/left':>11}")
        for duty in sorted(args.duties):
            sample = measure(target, duty, args.settle, args.measure)
            samples.append(sample)
            ratio = sample.right_m_s / sample.left_m_s if abs(sample.left_m_s) > 1e-6 else float("nan")
            print(f"{duty:5.2f} {sample.left_m_s:9.3f} {sample.right_m_s:10.3f} {ratio:11.3f}")
            time.sleep(0.4)  # let everything come to rest before the next step
        result = analyse(samples, target.params.wheel_separation_m)

    print(f"\nleft  : v = {result['left_gain_m_s']:.3f} * duty, deadband at duty "
          f"{result['left_deadband']:.3f}")
    print(f"right : v = {result['right_gain_m_s']:.3f} * duty, deadband at duty "
          f"{result['right_deadband']:.3f}")
    print(f"right/left gain ratio {result['gain_ratio']:.3f} -> driving 'straight' open loop at "
          f"full duty turns at {math.degrees(result['yaw_rate_at_full_rad_s']):+.1f} deg/s")

    if args.plot:
        plt = plt_headless()
        fig, ax = plt.subplots(figsize=(7, 4.5))
        duties = [s.duty for s in samples]
        ax.plot(duties, [s.left_m_s for s in samples], "o-", label="left wheel")
        ax.plot(duties, [s.right_m_s for s in samples], "s-", label="right wheel")
        line = [result["left_gain_m_s"] * (d - result["left_deadband"]) for d in duties]
        ax.plot(duties, line, "k--", lw=1, label="fit (left)")
        ax.axvline(result["left_deadband"], color="grey", ls=":", label="deadband")
        ax.set_xlabel("commanded duty")
        ax.set_ylabel("rim speed (m/s)")
        ax.set_title("Duty to speed — measured")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print(f"wrote {args.plot}")
    return result


if __name__ == "__main__":
    main()
