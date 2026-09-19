"""Drive forward, backward and rotate in place for fixed durations — no feedback.

Lesson 01.11 (open-loop driving). Also the starting point of lesson 08.01: run the same
sequence twice and measure where the robot ends up; open loop is not repeatable.

    python labs/robot/drive_open_loop.py --fake
    python labs/robot/drive_open_loop.py --steps forward:2 rotate-left:1.2 backward:2 --duty 0.35

SAFETY: the robot moves as soon as the first step starts. Clear at least 1.5 m in front of
it, keep a hand near the battery switch, and try it with the wheels off the ground first.
Ctrl-C stops the robot.
"""

from __future__ import annotations

import argparse
import time

from robot_common import add_connection_args, connect

# (left sign, right sign) for each action: rotating left = left wheel back, right wheel forward.
ACTIONS = {
    "forward": (+1, +1),
    "backward": (-1, -1),
    "rotate-left": (-1, +1),
    "rotate-right": (+1, -1),
    "pause": (0, 0),
}


def parse_step(text: str) -> tuple[str, float]:
    try:
        action, seconds = text.split(":")
        duration = float(seconds)
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected ACTION:SECONDS, got {text!r}") from None
    if action not in ACTIONS:
        raise argparse.ArgumentTypeError(f"unknown action {action!r}; choose from {', '.join(ACTIONS)}")
    if not 0 < duration <= 30:
        raise argparse.ArgumentTypeError("duration must be between 0 and 30 s")
    return action, duration


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--steps", nargs="+", type=parse_step,
                        default=[("forward", 2.0), ("pause", 0.5), ("rotate-left", 1.0), ("pause", 0.5), ("backward", 2.0)],
                        metavar="ACTION:SECONDS", help=f"sequence of steps; actions: {', '.join(ACTIONS)}")
    parser.add_argument("--duty", type=float, default=0.4, help="duty for straight driving, 0..1 (default 0.4)")
    parser.add_argument("--turn-duty", type=float, default=0.35, help="duty for rotating, 0..1 (default 0.35)")
    add_connection_args(parser)
    args = parser.parse_args(argv)
    for name in ("duty", "turn_duty"):
        if not 0.0 <= getattr(args, name) <= 1.0:
            parser.error(f"--{name.replace('_', '-')} must be between 0 and 1")

    with connect(args) as conn:
        base, params = conn.base, conn.params
        base.reset_encoders()
        for action, duration in args.steps:
            left_sign, right_sign = ACTIONS[action]
            duty = args.duty if action in ("forward", "backward") else args.turn_duty
            start = base.read()
            print(f"{action:>12} for {duration:.1f} s at duty {duty if left_sign else 0:.2f}")
            end_time = time.monotonic() + duration
            while time.monotonic() < end_time:
                # SAFETY: the wheels turn here. We re-send every 50 ms (the firmware watchdog
                # stops the motors if commands stop for 300 ms).
                base.set_wheel_duty(left_sign * duty, right_sign * duty)
                time.sleep(0.05)
            base.stop()
            time.sleep(0.3)  # let the robot come to rest before measuring
            end = base.read()
            d_left = (end.left_ticks - start.left_ticks) * params.meters_per_tick
            d_right = (end.right_ticks - start.right_ticks) * params.meters_per_tick
            turned = (d_right - d_left) / params.wheel_separation_m
            print(f"{'':>12} encoders: left {d_left:+.3f} m, right {d_right:+.3f} m, "
                  f"turned {turned:+.2f} rad")
        if conn.true_pose() is not None:
            x, y, theta = conn.true_pose()
            print(f"simulator ground truth: x={x:.3f} m  y={y:.3f} m  theta={theta:+.2f} rad")


if __name__ == "__main__":
    main()
