"""llm_in_the_loop.py - what happens when a slow decision-maker drives the wheels directly (19.01).

karmel drives at 0.3 m/s towards a wall 3 m ahead. The rule is simple: "stop when the front
distance sensor reads less than 0.40 m". Three drivers apply that rule:

  controller   decides every 20 ms (50 Hz) from a fresh sensor reading - a normal control loop
  llm          decides from the reading at request time, and the answer arrives LATENCY seconds
               later (a slow LLM round-trip); its last command is held until the next answer
  llm+watchdog the same, but the firmware watchdog (0.3 s) stops the motors when commands stop

    py 19-llm-robot-agents/code/llm_in_the_loop.py
    py 19-llm-robot-agents/code/llm_in_the_loop.py --latency 0.8 --speed 0.5
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "labs" / "python"))

from robotlab.config import load_config  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World  # noqa: E402

WALL_X = 4.0
STOP_BELOW_M = 0.40


@dataclass
class Outcome:
    driver: str
    collided: bool
    final_gap_m: float  # from the front of the chassis circle to the wall
    overshoot_m: float  # how far past the 0.40 m stop line the sensor reading went
    reached_stop_line_s: float  # nan: never got there in the simulated time
    commands_sent: int


def drive(driver: str, latency_s: float, speed: float, sim_s: float = 20.0) -> Outcome:
    cfg = load_config()
    watchdog = 0.3 if driver == "llm+watchdog" else None
    sim = DiffDriveSim(World.rectangle_room(WALL_X, 3.0), DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg),
                       pose=(1.0, 1.5, 0.0), seed=0)
    base = SimBase(sim, dt=0.01, watchdog_s=watchdog)  # 100 Hz, like the firmware loop
    wheel = speed / cfg.drive.wheel_radius_m
    period = 0.02 if driver == "controller" else latency_s
    pending: tuple[float, float] | None = None  # (arrival time, wheel command) of the LLM answer in flight
    command = wheel if driver == "controller" else 0.0  # the LLM drivers wait for their first answer
    sent, reached, collided, min_reading = 0, float("nan"), False, float("inf")
    reading = base.read().range_m
    next_decision = 0.0
    while sim.t < sim_s:
        if pending is not None and sim.t >= pending[0] - 1e-9:  # an LLM answer arrives
            command, pending = pending[1], None
            base.set_wheel_velocity(command, command)
            sent += 1
        elif driver == "llm":
            base.set_wheel_velocity(command, command)  # no watchdog: the last command keeps the wheels turning
        if sim.t >= next_decision - 1e-9:
            decision = 0.0 if (reading is not None and reading < STOP_BELOW_M) else wheel
            if driver == "controller":
                command = decision
                base.set_wheel_velocity(command, command)
                sent += 1
            elif pending is None:
                pending = (sim.t + latency_s, decision)  # the answer to "what should I do now?" arrives later
            next_decision = sim.t + period
        state = base.read()
        reading = state.range_m
        collided = collided or sim.collided
        if reading is not None:
            min_reading = min(min_reading, reading)
            if reading < STOP_BELOW_M and reached != reached:  # first time (nan != nan)
                reached = sim.t
    gap = WALL_X - sim.pose.x - sim.params.robot_radius_m
    return Outcome(driver, collided, round(gap, 3), round(max(0.0, STOP_BELOW_M - min_reading), 3), round(reached, 2), sent)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--latency", type=float, default=2.0, help="LLM round-trip time, s")
    ap.add_argument("--speed", type=float, default=0.3, help="driving speed, m/s")
    args = ap.parse_args()
    print(f"speed {args.speed} m/s, stop rule: range < {STOP_BELOW_M} m, LLM latency {args.latency} s\n")
    print(f"{'driver':<14}{'collided':>9}{'final gap m':>13}{'overshoot m':>13}{'at stop line s':>16}{'commands':>10}")
    for driver in ("controller", "llm", "llm+watchdog"):
        o = drive(driver, args.latency, args.speed)
        print(f"{o.driver:<14}{str(o.collided):>9}{o.final_gap_m:>13.3f}{o.overshoot_m:>13.3f}"
              f"{o.reached_stop_line_s:>16}{o.commands_sent:>10}")


if __name__ == "__main__":
    main()
