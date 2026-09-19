"""Regenerate ``run_telemetry.csv`` — the recorded run the 03.07 tests read.

    python labs/exercises/03.07/make_run.py

The file is committed, so students (and CI) need no simulator run to do the exercise. It is
written with exactly the columns of ``labs/robot/log_telemetry.py``, by driving the course
simulator through a script that contains, on purpose, everything a real log contains:

  phase 1  0.00-0.60 s   nobody commands the robot -> the watchdog trips at 0.30 s
  phase 2  0.60-4.00 s   drive forward at 8 rad/s, approaching a wall 3 m ahead
  phase 3  ~2.0 s        10 consecutive samples are DROPPED (a lost-telemetry gap)
  phase 4  4.00-6.00 s   turn in place at +-4 rad/s
  phase 5  6.00-7.00 s   commands stop -> the watchdog trips again
  start                  the wall ahead is beyond the ToF sensor's 4 m range, so ``range_m`` is
                         empty (None) until the robot has driven close enough

Deterministic: fixed seed, lockstep simulation, no wall-clock time anywhere.
"""

from __future__ import annotations

import csv
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "python"))

from robotlab.config import load_config  # noqa: E402
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World  # noqa: E402

COLUMNS = ["host_time_s", "t_s", "left_ticks", "right_ticks", "left_rad_s", "right_rad_s",
           "battery_v", "range_m", "flags", "left_cmd", "right_cmd"]

DT = 0.02
DROP_FROM, DROP_COUNT = 100, 10  # sample index 100 is t = 2.02 s


def command_for(t: float) -> tuple[float, float] | None:
    """The wheel-velocity command at simulated time ``t`` (None = send nothing)."""
    if t < 0.60:
        return None                 # phase 1: the watchdog must trip
    if t < 4.00:
        return (8.0, 8.0)           # phase 2: forward
    if t < 6.00:
        return (4.0, -4.0)          # phase 4: turn in place
    return None                     # phase 5: the watchdog trips again


def generate() -> list[list[object]]:
    cfg = load_config()
    sim = DiffDriveSim(World.rectangle_room(6.0, 3.0), DiffDriveParams.ideal(cfg),
                       SensorParams.ideal(cfg), pose=(1.0, 1.5, 0.0), seed=7)
    base = SimBase(sim, dt=DT, watchdog_s=cfg.serial.watchdog_ms / 1000.0)
    rows: list[list[object]] = []
    for index in range(350):        # 7.0 s at 50 Hz
        command = command_for(index * DT)
        if command is not None:
            base.set_wheel_velocity(*command)
        state = base.read()
        if DROP_FROM <= index < DROP_FROM + DROP_COUNT:
            continue                # phase 3: these samples never reach the host
        rows.append([
            f"{state.t:.4f}",       # host_time_s: identical to robot time in a lockstep sim
            f"{state.t:.3f}",
            state.left_ticks, state.right_ticks,
            f"{state.left_rad_s:.3f}", f"{state.right_rad_s:.3f}",
            "" if state.battery_v is None else f"{state.battery_v:.3f}",
            "" if state.range_m is None else f"{state.range_m:.3f}",
            state.flags,
            "" if command is None else command[0],
            "" if command is None else command[1],
        ])
    base.close()
    return rows


def main() -> None:
    rows = generate()
    out = HERE / "run_telemetry.csv"
    with out.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        writer.writerows(rows)
    print(f"wrote {len(rows)} samples to {out}")


if __name__ == "__main__":
    main()
