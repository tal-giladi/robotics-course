#!/usr/bin/env python3
"""06.02 — A guided tour of the course simulator (`labs/python/robotlab/sim`).

    python 06-simulation/code/mini_sim_tour.py                # all five parts
    python 06-simulation/code/mini_sim_tour.py --part 3

Every part prints numbers you can check against the source of ``DiffDriveSim.step``:

1. One step, stage by stage. duty -> steady-state speed -> motor lag -> wheel travel -> arc ->
   encoder ticks. The same seven lines the lesson walks through.
2. The motor curve: duty in, steady-state wheel speed out, with the deadband and saturation.
3. What each imperfection knob costs. A 2 m square is driven with the ideal robot, with the
   realistic preset, and with each knob switched on alone.
4. Encoders lie about slip. With slip_std > 0 the ticks still say "we went 2 m" while the true
   pose says otherwise — the reason odometry needs a second sensor (module 09/10).
5. What the simulator does NOT model. A frontal collision: the real robot would bounce, tip or
   push; DiffDriveSim simply stalls the wheels and keeps the encoders honest.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

_LABS_PYTHON = Path(__file__).resolve().parent.parent.parent / "labs" / "python"
if str(_LABS_PYTHON) not in sys.path:  # robotlab without `pip install -e labs/python`
    sys.path.insert(0, str(_LABS_PYTHON))

from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World  # noqa: E402
from robotlab.sim.components import deadband_speed, first_order_alpha  # noqa: E402

DT = 0.02  # 50 Hz, the rate base_node commands the real Pico at


# --------------------------------------------------------------------------------- part 1
def part_one_step() -> None:
    p = DiffDriveParams.ideal()
    sim = DiffDriveSim(World(), p, SensorParams.ideal(), seed=0)
    duty = (0.6, 0.6)
    sim.set_duty(*duty)

    target = deadband_speed(np.array(duty), p.duty_deadband, p.max_wheel_speed_rad_s)
    alpha = first_order_alpha(DT, p.motor_time_constant_s)
    w_prev = sim.wheel_rad_s.copy()
    w_next = w_prev + (target - w_prev) * alpha
    turn = 0.5 * (w_prev + w_next) * DT
    travel = turn * np.array([p.true_wheel_radius_left_m, p.true_wheel_radius_right_m])

    print("Part 1 — one 20 ms step of DiffDriveSim.step, by hand")
    print(f"  duty                         {duty}")
    print(f"  deadband {p.duty_deadband}, max {p.max_wheel_speed_rad_s} rad/s")
    print(f"  -> steady-state target       {target[0]:.4f} rad/s "
          f"= sign*(|0.6| - {p.duty_deadband}) / (1 - {p.duty_deadband}) * {p.max_wheel_speed_rad_s}")
    print(f"  motor lag alpha              {alpha:.4f}  = 1 - exp(-{DT}/{p.motor_time_constant_s})")
    print(f"  -> wheel speed after 1 step  {w_next[0]:.4f} rad/s")
    print(f"  -> wheel rotation this step  {turn[0]:.6f} rad (average of start and end speed)")
    print(f"  -> ground travel             {travel[0] * 1000:.3f} mm  (x true wheel radius "
          f"{p.true_wheel_radius_left_m} m)")

    sim.step(DT)
    print(f"  sim.pose after the step      x={sim.pose.x * 1000:.3f} mm  y={sim.pose.y * 1000:.3f} mm  "
          f"theta={sim.pose.theta:.6f} rad")
    print(f"  sim.ticks                    {sim.ticks}   "
          f"(1 tick = {2 * math.pi / p.ticks_per_wheel_rev * 1000:.4f} mrad "
          f"= {2 * math.pi / p.ticks_per_wheel_rev * p.wheel_radius_m * 1000:.4f} mm of travel)")
    print(f"  sim.wheel_velocity_estimate  {sim.wheel_velocity_estimate[0]:.4f} rad/s "
          f"(encoder-derived and low-passed — NOT the true {sim.wheel_rad_s[0]:.4f})")


# --------------------------------------------------------------------------------- part 2
def part_motor_curve() -> None:
    p = DiffDriveParams.ideal()
    print("Part 2 — duty in, steady-state wheel speed out (the motor's deadband and saturation)")
    print(f"{'duty':>6} {'rad/s':>8} {'m/s at the rim':>15}")
    for duty in (0.0, 0.05, 0.11, 0.12, 0.13, 0.2, 0.5, 0.8, 1.0, 1.5):
        speed = float(deadband_speed(np.array([duty]), p.duty_deadband, p.max_wheel_speed_rad_s)[0])
        print(f"{duty:6.2f} {speed:8.3f} {speed * p.wheel_radius_m:15.3f}")


# --------------------------------------------------------------------------------- part 3 and 4
def drive_square(params: DiffDriveParams, side: float = 2.0, speed: float = 0.25,
                 seed: int = 0) -> DiffDriveSim:
    """Four sides and four 90-degree left turns, commanded in WHEEL VELOCITY (closed loop)."""
    sim = DiffDriveSim(World(), params, SensorParams.ideal(), seed=seed)
    wheel_speed = speed / params.wheel_radius_m  # what the robot BELIEVES gives `speed`
    turn_rate = 1.0  # rad/s of body rotation
    wheel_turn = turn_rate * params.wheel_separation_m / 2.0 / params.wheel_radius_m
    for _ in range(4):
        sim.set_velocity(wheel_speed, wheel_speed)
        sim.advance(side / speed, DT)
        sim.set_velocity(-wheel_turn, wheel_turn)
        sim.advance((math.pi / 2.0) / turn_rate, DT)
    sim.set_velocity(0.0, 0.0)
    sim.advance(0.5, DT)
    return sim


def part_imperfections() -> None:
    ideal = DiffDriveParams.ideal()
    knobs = {
        "ideal": {},
        "motor_gain_right 0.94": {"motor_gain_right": 0.94},
        "wheel_radius_scale_right 1.008": {"wheel_radius_scale_right": 1.008},
        "wheel_separation_scale 1.04": {"wheel_separation_scale": 1.04},
        "slip_std 0.02": {"slip_std": 0.02},
        "realistic() (all of them)": None,
    }
    print("Part 3 — a 2 m square, driven open-loop in wheel-velocity mode, one knob at a time")
    print(f"{'imperfection':32s} {'end x (m)':>10} {'end y (m)':>10} {'end yaw (deg)':>14} {'error (m)':>10}")
    for label, override in knobs.items():
        params = DiffDriveParams.realistic() if override is None else replace(ideal, **override)
        sim = drive_square(params, seed=7)
        error = math.hypot(sim.pose.x, sim.pose.y)
        print(f"{label:32s} {sim.pose.x:10.3f} {sim.pose.y:10.3f} "
              f"{math.degrees(sim.pose.theta):14.2f} {error:10.3f}")
    print("  (a perfect robot ends where it started: 0.000, 0.000, 0 deg)")


def part_encoders_lie() -> None:
    print("\nPart 4 — the encoders cannot see slip")
    ideal = DiffDriveParams.ideal()
    for label, params in (("no slip", ideal), ("slip_std 0.05", replace(ideal, slip_std=0.05))):
        sim = DiffDriveSim(World(), params, SensorParams.ideal(), seed=3)
        sim.set_velocity(5.0, 5.0)
        sim.advance(9.0, DT)
        left_ticks, right_ticks = sim.ticks
        meters_per_tick = 2 * math.pi * params.wheel_radius_m / params.ticks_per_wheel_rev
        odometry_says = 0.5 * (left_ticks + right_ticks) * meters_per_tick
        print(f"  {label:14s} encoders say {odometry_says:6.3f} m   truth {sim.pose.x:6.3f} m   "
              f"error {odometry_says - sim.pose.x:+.3f} m")


# --------------------------------------------------------------------------------- part 5
def part_limits() -> None:
    print("\nPart 5 — what the simulator does not model: contact")
    world = World.rectangle_room(4.0, 3.0)
    sim = DiffDriveSim(world, DiffDriveParams.ideal(), SensorParams.ideal(), pose=(2.0, 1.5, 0.0), seed=0)
    sim.set_velocity(6.0, 6.0)
    for _ in range(round(6.0 / DT)):
        sim.step(DT)
        if sim.collided:
            break
    print(f"  drove into the east wall at x = {sim.pose.x:.3f} m "
          f"(the wall is at 4.000, the collision radius is {sim.params.robot_radius_m:.3f} m)")
    print(f"  collided = {sim.collided}; true wheel speed is now {sim.wheel_rad_s[0]:.3f} rad/s")
    print(f"  front range sensor reads {sim.front_range():.3f} m")
    ticks_before = sim.ticks
    sim.advance(1.0, DT)
    print(f"  after another second of full throttle the encoders added "
          f"{sim.ticks[0] - ticks_before[0]} ticks: a stalled wheel, not a spinning one")
    print("  A real robot would bounce, tip, or push the obstacle, and the wheels would keep spinning.")


PARTS = {1: part_one_step, 2: part_motor_curve, 3: part_imperfections, 4: part_encoders_lie,
         5: part_limits}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--part", type=int, choices=sorted(PARTS), help="run only this part")
    args = ap.parse_args()
    for number in ([args.part] if args.part else sorted(PARTS)):
        PARTS[number]()
        print()


if __name__ == "__main__":
    main()
