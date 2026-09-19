"""Drive a square using the wheel encoders to measure distances and angles.

Lesson 01.12 (ticks to distance, turning by encoders, drive a square).

    python labs/robot/drive_square.py --fake
    python labs/robot/drive_square.py --side 0.5 --speed 0.15 --direction right

The math:
    distance per tick = wheel circumference / ticks per revolution = 2*pi*0.045 / 2464 = 0.115 mm
    straight line:      both wheels travel the side length
    turn in place by a: each wheel travels a * (track / 2) along an arc, in opposite directions

The wheel speeds are closed loop on the Pico (V command); this script only decides WHEN to stop,
slowing down near the goal so the robot doesn't coast past it.

SAFETY: needs a clear area of (side + 0.5 m) squared. Ctrl-C stops the robot.
"""

from __future__ import annotations

import argparse
import math
import time

from robot_common import Connection, RobotParams, add_connection_args, body_to_wheels, connect
from robotlab.hal import BaseState


def wheel_travel_m(start: BaseState, now: BaseState, params: RobotParams) -> tuple[float, float]:
    return ((now.left_ticks - start.left_ticks) * params.meters_per_tick,
            (now.right_ticks - start.right_ticks) * params.meters_per_tick)


def move(conn: Connection, distance_m: float, angle_rad: float, speed_m_s: float, timeout_s: float = 30.0) -> BaseState:
    """Drive straight (angle 0) or turn in place (distance 0) until the encoders say we're there."""
    base, params = conn.base, conn.params
    base.reset_encoders()
    start = base.wait_for_telemetry(timeout_s=1.0, newer_than=base.read().t)
    # Target travel of each wheel (positive = forward), and the direction each wheel turns.
    half_track = params.wheel_separation_m / 2.0
    target = abs(distance_m) if angle_rad == 0 else abs(angle_rad) * half_track
    state = start
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        left_m, right_m = wheel_travel_m(start, state, params)
        travelled = (abs(left_m) + abs(right_m)) / 2.0
        remaining = target - travelled
        if remaining <= 0.002:
            break
        # Full speed far away, proportionally slower in the last 10 cm, never below 3 cm/s.
        speed = max(0.03, min(speed_m_s, speed_m_s * remaining / 0.10))
        if angle_rad == 0:
            left, right = body_to_wheels(math.copysign(speed, distance_m), 0.0, params)
        else:
            left, right = body_to_wheels(0.0, math.copysign(speed / half_track, angle_rad), params)
        base.set_wheel_velocity(left, right)  # SAFETY: the wheels turn here
        state = base.wait_for_telemetry(timeout_s=1.0, newer_than=state.t)
    else:
        base.stop()
        raise TimeoutError("move did not finish in time - is a wheel blocked?")
    base.stop()
    time.sleep(0.25)
    return base.read()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--side", type=float, default=0.5, help="side length in metres (default 0.5)")
    parser.add_argument("--speed", type=float, default=0.15, help="driving speed in m/s (default 0.15)")
    parser.add_argument("--direction", choices=("left", "right"), default="left", help="turn direction")
    parser.add_argument("--laps", type=int, default=1, help="how many squares (default 1)")
    add_connection_args(parser)
    args = parser.parse_args(argv)
    if not 0.05 <= args.side <= 3.0:
        parser.error("--side must be between 0.05 and 3 m")

    with connect(args) as conn:
        speed = min(args.speed, conn.params.max_linear_speed_m_s)
        turn = math.pi / 2 if args.direction == "left" else -math.pi / 2
        for lap in range(args.laps):
            for corner in range(4):
                move(conn, args.side, 0.0, speed)
                end = move(conn, 0.0, turn, speed)
                print(f"lap {lap + 1}, side {corner + 1} done (last wheel ticks {end.left_ticks:+d} / {end.right_ticks:+d})")
        truth = conn.true_pose()
        if truth is not None:
            print(f"simulator: ended at x={truth[0]:.3f} m y={truth[1]:.3f} m theta={truth[2]:+.3f} rad "
                  "(started at the room's start pose - a perfect square returns there)")


if __name__ == "__main__":
    main()
