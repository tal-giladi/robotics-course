"""Record the telemetry stream to a CSV file, optionally while applying a constant command.

Lessons 03.07 (logging and telemetry) and 08.02 (measuring a motor's step response: log while
applying a duty step, then plot it with plot_telemetry.py).

    python labs/robot/log_telemetry.py --fake --duration 5 --output run.csv
    python labs/robot/log_telemetry.py --duration 3 --duty 0.5 0.5 --output step.csv
    python labs/robot/plot_telemetry.py step.csv

Every telemetry sample (50 Hz) becomes one row. Columns:
    host_time_s   seconds since logging started, Pi clock
    t_s           robot clock (Pico), seconds
    left_ticks, right_ticks, left_rad_s, right_rad_s, battery_v, range_m, flags
    left_cmd, right_cmd   the duty or rad/s we were commanding ("" when none)

SAFETY: with --duty or --velocity the robot moves for --duration seconds. Put it on a stand
(wheels off the ground) for step-response measurements.
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

from robot_common import add_connection_args, connect

COLUMNS = ["host_time_s", "t_s", "left_ticks", "right_ticks", "left_rad_s", "right_rad_s",
           "battery_v", "range_m", "flags", "left_cmd", "right_cmd"]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, default=Path("telemetry.csv"), help="CSV file (default telemetry.csv)")
    parser.add_argument("--duration", type=float, default=10.0, help="seconds to record (default 10)")
    parser.add_argument("--pre-roll", type=float, default=0.5, help="seconds recorded before the command starts")
    command = parser.add_mutually_exclusive_group()
    command.add_argument("--duty", type=float, nargs=2, metavar=("LEFT", "RIGHT"), help="open-loop duty -1..1")
    command.add_argument("--velocity", type=float, nargs=2, metavar=("LEFT", "RIGHT"), help="wheel speeds, rad/s")
    add_connection_args(parser)
    args = parser.parse_args(argv)

    with connect(args) as conn, args.output.open("w", newline="", encoding="utf-8") as f:
        base = conn.base
        writer = csv.writer(f)
        writer.writerow(COLUMNS)
        start = time.monotonic()
        state = base.read()
        rows = 0
        while (now := time.monotonic()) - start < args.duration:
            active = now - start >= args.pre_roll
            cmd: tuple[float, float] | None = None
            if active and args.duty:
                cmd = (args.duty[0], args.duty[1])
                base.set_wheel_duty(*cmd)  # SAFETY: the wheels turn here
            elif active and args.velocity:
                cmd = (args.velocity[0], args.velocity[1])
                base.set_wheel_velocity(*cmd)  # SAFETY: the wheels turn here
            state = base.wait_for_telemetry(timeout_s=1.0, newer_than=state.t)
            writer.writerow([
                f"{time.monotonic() - start:.4f}", f"{state.t:.3f}", state.left_ticks, state.right_ticks,
                f"{state.left_rad_s:.3f}", f"{state.right_rad_s:.3f}",
                "" if state.battery_v is None else f"{state.battery_v:.3f}",
                "" if state.range_m is None else f"{state.range_m:.3f}",
                state.flags,
                "" if cmd is None else cmd[0], "" if cmd is None else cmd[1],
            ])
            rows += 1
        base.stop()
    print(f"wrote {rows} samples to {args.output}")


if __name__ == "__main__":
    main()
