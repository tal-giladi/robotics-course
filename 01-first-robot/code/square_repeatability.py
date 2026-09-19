"""01.12 — How repeatable is your square? Run it N times and do the statistics.

    python 01-first-robot/code/square_repeatability.py --fake --runs 5
    python 01-first-robot/code/square_repeatability.py --fake --realistic --runs 5 --seed 7
    python 01-first-robot/code/square_repeatability.py --port /dev/ttyACM0 --runs 5 --measured

One run is an anecdote. Five runs are a measurement: a systematic error (the robot always ends
up 12 cm short and 6 degrees clockwise) points at a wrong constant — wheel radius, track width,
ticks per revolution — and is *fixable* by calibration (09.05). A random spread (sometimes left,
sometimes right) is slip and surface, and can only be reduced, not calibrated away.

Each run drives the square with ``labs/robot/drive_square.py``'s own ``move()``, so this script
measures exactly the code you run on the robot. It reports, per run:

* **odometry**  — where the robot *thinks* it ended up, integrated from its own ticks;
* **truth**     — in simulation, the simulator's ground truth; on the real robot, the numbers
                  you measure with a tape and type in (``--measured``).

SAFETY: the robot drives a square of (side + 0.5) m. Clear the area, keep pets and cables out,
and keep a hand near the switch. Ctrl-C stops the motors.
"""

from __future__ import annotations

import argparse
import math
import statistics
from dataclasses import dataclass

from first_robot_lab import add_target_args, open_target
from robotlab.geometry import angle_diff
from robotlab.sim.robot import arc_update
from robotlab.geometry import SE2

import drive_square  # labs/robot/drive_square.py — the script the lesson runs


@dataclass(frozen=True)
class RunResult:
    run: int
    odom: tuple[float, float, float]           # where the robot thinks it is, relative to the start
    truth: tuple[float, float, float] | None   # ground truth (simulator) or tape measurements


def closure_error(pose: tuple[float, float, float]) -> tuple[float, float]:
    """Distance from the start point, and heading error, for a pose relative to the start."""
    return math.hypot(pose[0], pose[1]), angle_diff(pose[2], 0.0)


def drive_one_square(target, side_m: float, speed_m_s: float, clockwise: bool) -> tuple[float, float, float]:
    """Drive the square, integrating the robot's own ticks; return its odometric end pose."""
    turn = -math.pi / 2 if clockwise else math.pi / 2
    pose = SE2(0.0, 0.0, 0.0)
    mpt = target.params.meters_per_tick
    track = target.params.wheel_separation_m
    for _ in range(4):
        for distance, angle in ((side_m, 0.0), (0.0, turn)):
            # move() zeroes the encoders first, so its final tick counts ARE this segment's travel.
            end = drive_square.move(target, distance, angle, speed_m_s)  # SAFETY: the wheels turn
            pose = arc_update(pose, end.left_ticks * mpt, end.right_ticks * mpt, track)
    return pose.x, pose.y, pose.theta


def ask_measurement(run: int) -> tuple[float, float, float]:
    """On the real robot: type in what you measured with a tape and a protractor."""
    print(f"\nRun {run}: measure the robot's marker against the start mark.")
    forward = float(input("  along the first side, + = past the mark (m): "))
    left = float(input("  sideways, + = to the left of the mark (m): "))
    heading = float(input("  heading error, + = counter-clockwise (degrees): "))
    return forward, left, math.radians(heading)


def summarise(name: str, values: list[float], unit: str) -> str:
    spread = statistics.stdev(values) if len(values) > 1 else 0.0
    return (f"{name:<22} mean {statistics.mean(values):+8.3f} {unit}   "
            f"sd {spread:6.3f} {unit}   min {min(values):+7.3f}   max {max(values):+7.3f}")


def main(argv: list[str] | None = None) -> list[RunResult]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=5, help="how many squares (default 5)")
    ap.add_argument("--side", type=float, default=1.0, help="side length in metres (default 1.0)")
    ap.add_argument("--speed", type=float, default=0.2, help="driving speed in m/s (default 0.2)")
    ap.add_argument("--clockwise", action="store_true", help="turn right at every corner")
    ap.add_argument("--measured", action="store_true",
                    help="real robot: ask for the tape measurement after each run")
    add_target_args(ap)
    args = ap.parse_args(argv)
    if not 1 <= args.runs <= 50:
        ap.error("--runs must be between 1 and 50")

    results: list[RunResult] = []
    with open_target(args) as target:
        speed = min(args.speed, target.params.max_linear_speed_m_s)
        for run in range(1, args.runs + 1):
            start_truth = target.true_pose()
            odom = drive_one_square(target, args.side, speed, args.clockwise)
            truth = None
            if target.simulated and start_truth is not None:
                end = target.true_pose()
                # Express the end pose in the frame the run started in.
                relative = SE2(*start_truth).inverse() @ SE2(*end)
                truth = (relative.x, relative.y, relative.theta)
            elif args.measured:
                truth = ask_measurement(run)
            results.append(RunResult(run, odom, truth))
            distance, heading = closure_error(truth if truth is not None else odom)
            label = "truth" if truth is not None else "odometry"
            print(f"run {run}: {label} ended {distance:.3f} m from the start, "
                  f"heading off by {math.degrees(heading):+.1f} deg")

    print()
    for label, poses in (("odometry", [r.odom for r in results]),
                         ("truth", [r.truth for r in results if r.truth is not None])):
        if not poses:
            continue
        print(f"--- {label} over {len(poses)} runs ---")
        print(summarise("forward offset", [p[0] for p in poses], "m"))
        print(summarise("sideways offset", [p[1] for p in poses], "m"))
        print(summarise("heading error", [math.degrees(angle_diff(p[2], 0.0)) for p in poses], "deg"))
        print(summarise("distance from start", [closure_error(p)[0] for p in poses], "m"))
    return results


if __name__ == "__main__":
    main()
