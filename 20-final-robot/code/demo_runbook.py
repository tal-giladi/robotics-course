"""The demo, planned like a mission: a timeline, a battery budget, and a fallback for every segment.

A demo is the one time the robot runs in front of people who did not build it, in a room it has
never mapped, with someone's toddler in the test area. Everything that can go wrong has already
gone wrong during the course; the only new thing is that you cannot debug it live. So you plan it
the way you plan a deployment: segments with durations, an abort for each one, a battery budget
with reserve, and a rehearsal that is identical to the performance.

    py 20-final-robot/code/demo_runbook.py plan       # the timeline and the battery budget
    py 20-final-robot/code/demo_runbook.py runbook    # what the human does, second by second
    py 20-final-robot/code/demo_runbook.py risks      # what can fail, and the fallback for each
    py 20-final-robot/code/demo_runbook.py check      # the rules; exit 1 if the plan is not ready

The power numbers come from ``power_budget_v2.py``, so a change to the robot's hardware changes
the demo plan automatically — which is the point of keeping both as code.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from power_budget_v2 import (  # noqa: E402
    karmel_v2_loads, runtime_min, session_power_w,
)

BATTERY_AH = 3.35          # Samsung 35E minimum capacity (datasheet)
BATTERY_V = 10.8
RESERVE_FRACTION = 0.30    # you start the demo with a full pack and never plan below this


@dataclass(frozen=True)
class Segment:
    """One part of the demo: what it shows, how long it takes, and what you do when it fails."""

    name: str
    shows: str                 # the capability a viewer actually sees
    minutes: float
    driving_fraction: float    # how much of the segment the motors are running
    arm_fraction: float        # how much of it the arm is moving
    failure: str               # the most likely way this segment fails
    fallback: str              # what you do, live, when it does
    abort: str                 # how you stop it safely, right now
    lesson: str = ""


def demo() -> list[Segment]:
    """karmel's ten-minute demo: the whole course, in the order a viewer can follow."""
    return [
        Segment("0. Power-on and bringup", "the robot knows whether it is healthy", 1.0, 0.0, 0.0,
                "a unit never reaches ready (usually the LiDAR or the arm bus)",
                "show the diagnostics tree failing — a robot that refuses to start is the feature",
                "leave the e-stop pressed until the state is READY", "20.03"),
        Segment("1. E-stop test", "the safety case, demonstrated not claimed", 0.5, 0.2, 0.0,
                "the relay chatters or the robot keeps creeping",
                "stop the demo. This is the one failure you do not work around",
                "the button is already pressed", "20.04"),
        Segment("2. Teleop and the watchdog", "the lowest layer: commands, limits, watchdog", 1.0, 0.8, 0.0,
                "the laptop's Wi-Fi drops",
                "that IS the watchdog demo — narrate it: the robot stopped by itself in 300 ms",
                "release the dead-man key", "01.15"),
        Segment("3. Mapping a corner of the room", "SLAM building a map live in RViz", 1.5, 0.9, 0.0,
                "a glass wall or a mirror makes the map fold",
                "switch to the pre-recorded map and say why glass defeats a 2D LiDAR",
                "cancel the teleop; the base stops", "11.07"),
        Segment("4. Autonomous navigation to a goal", "Nav2: plan, follow, avoid, recover", 1.5, 0.9, 0.0,
                "somebody stands in the path and the robot stops and replans",
                "let it. Then step in deliberately and show the collision monitor firing",
                "Nav2 cancel, then the e-stop if it does not stop", "12.08"),
        Segment("5. Detect and approach an object", "vision closing the loop with motion", 1.0, 0.6, 0.1,
                "the lighting is different and the detector misses",
                "hold up the training object; if it still misses, show the detector's confidence live",
                "cancel the skill", "15.07"),
        Segment("6. Pick and place", "the arm doing the thing the whole course was for", 1.5, 0.1, 0.9,
                "the grasp slips or the object is 2 cm off",
                "one retry, then place it by hand and continue — do not debug live",
                "torque off the arm", "15.08"),
        Segment("7. 'Bring me the bottle' in one sentence", "language to a validated plan to motion", 1.5, 0.7, 0.5,
                "the model is slow, or asks for a place the geofence refuses",
                "show the refusal. A robot that says no to an illegal request is the demo",
                "cancel the mission; the executive stops the base and parks the arm", "19.08"),
    ]


# ==============================================================================================
#  The budget
# ==============================================================================================
def segment_energy_wh(segment: Segment) -> float:
    """Energy this segment takes from the pack, from the real load table."""
    scaled = []
    for load in karmel_v2_loads():
        duty = load.duty
        if load.tag == "drive":
            duty = segment.driving_fraction
        elif load.tag == "arm":
            duty = max(0.05, segment.arm_fraction)   # servos hold position even when standing still
        scaled.append(replace(load, duty=duty))
    return session_power_w(scaled) * segment.minutes / 60.0


def budget(segments: list[Segment]) -> dict[str, float]:
    energy = sum(segment_energy_wh(s) for s in segments)
    pack_wh = BATTERY_AH * BATTERY_V
    return {
        "minutes": sum(s.minutes for s in segments),
        "energy_wh": energy,
        "pack_wh": pack_wh,
        "used_fraction": energy / pack_wh,
        "idle_runtime_min": runtime_min(BATTERY_AH, BATTERY_V, session_power_w(
            [replace(x, duty=0.0) if x.is_actuator else x for x in karmel_v2_loads()])),
    }


def check(segments: list[Segment]) -> list[str]:
    """The rules a demo plan has to pass before you invite anyone."""
    problems: list[str] = []
    b = budget(segments)
    if b["used_fraction"] > 1.0 - RESERVE_FRACTION:
        problems.append(f"the demo uses {b['used_fraction'] * 100:.0f} % of the pack; "
                        f"the plan reserves {RESERVE_FRACTION * 100:.0f} % for the reruns and the "
                        "drive back to the dock")
    if b["minutes"] > 12.0:
        problems.append(f"{b['minutes']:.0f} minutes is longer than an audience stays attentive; "
                        "cut a segment")
    for s in segments:
        if not s.fallback:
            problems.append(f"{s.name}: no fallback — you will improvise in front of people")
        if not s.abort:
            problems.append(f"{s.name}: no abort — you cannot stop it safely on demand")
    if not any("e-stop" in s.name.lower() for s in segments):
        problems.append("the demo never tests the e-stop in front of the audience")
    return problems


# ==============================================================================================
#  Output
# ==============================================================================================
def print_plan(segments: list[Segment]) -> None:
    b = budget(segments)
    print(f"{'segment':38} {'min':>5} {'Wh':>6}  shows")
    for s in segments:
        print(f"{s.name:38} {s.minutes:>5.1f} {segment_energy_wh(s):>6.2f}  {s.shows}")
    print(f"{'TOTAL':38} {b['minutes']:>5.1f} {b['energy_wh']:>6.2f}")
    print(f"\npack {b['pack_wh']:.1f} Wh nominal; the demo uses {b['used_fraction'] * 100:.0f} % of it")
    print(f"with the actuators idle the robot survives {b['idle_runtime_min']:.0f} min of standing "
          "around before the demo starts — which is where demo batteries actually die")


def print_runbook(segments: list[Segment]) -> None:
    print("RUNBOOK — print this. One person drives, one person holds the e-stop.\n")
    print("Before anyone arrives:")
    print("  - charge the pack to full and measure it (12.4-12.6 V); keep the charged spare in the bag")
    print("  - run `py 20-final-robot/code/safety_case.py checklist` and do every line")
    print("  - run `py 20-final-robot/code/scenario_suite.py run` — a red suite means no demo")
    print("  - map the actual room, save it, and drive the whole route once, at demo speed")
    print("  - clear the area: no cables, no pets, stairs blocked, the audience behind a line")
    print("  - put the laptop where you can see the terminal and the robot at the same time\n")
    elapsed = 0.0
    for s in segments:
        print(f"t+{elapsed:4.1f} min  {s.name}")
        print(f"             say: \"{s.shows}\"")
        print(f"             if it fails ({s.failure}): {s.fallback}")
        print(f"             abort: {s.abort}")
        elapsed += s.minutes
    print(f"\nt+{elapsed:4.1f} min  stop, e-stop pressed, answer questions with the robot OFF.")


def print_risks(segments: list[Segment]) -> None:
    print(f"{'segment':38} {'lesson':8} failure -> fallback")
    for s in segments:
        print(f"{s.name:38} {s.lesson:8} {s.failure}")
        print(f"{'':47} -> {s.fallback}")


def main(argv: list[str] | None = None) -> int:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=["plan", "runbook", "risks", "check"])
    args = ap.parse_args(argv)

    segments = demo()
    if args.command == "plan":
        print_plan(segments)
        return 0
    if args.command == "runbook":
        print_runbook(segments)
        return 0
    if args.command == "risks":
        print_risks(segments)
        return 0

    problems = check(segments)
    if not problems:
        print(f"{len(segments)} segments, {budget(segments)['minutes']:.0f} minutes: the plan is ready")
        return 0
    for p in problems:
        print(f"PROBLEM  {p}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
