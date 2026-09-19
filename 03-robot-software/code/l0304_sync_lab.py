"""Timer-driven vs telemetry-driven loops: which samples does your loop actually see?

Lesson 03.04. The Pico sends telemetry at 50 Hz on ITS clock. A loop on the Pi that runs on ITS
OWN timer at 50 Hz is two clocks beating against each other: sometimes it reads the same sample
twice (duplicate), sometimes a sample is overwritten before it is read (missed). A loop that
waits for each new sample (`wait_for_telemetry(newer_than=...)`) runs exactly once per sample.

    python 03-robot-software/code/l0304_sync_lab.py --fake                      # both modes, 5 s each
    python 03-robot-software/code/l0304_sync_lab.py --fake --rate 45 --mode timer
    python 03-robot-software/code/l0304_sync_lab.py --port /dev/karmel          # the real Pico (read only)

Read-only: this script never commands the motors.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "labs" / "python"))

from l0304_loop_lab import GridRate  # noqa: E402
from robotlab.serial_base import SerialBase  # noqa: E402

TELEMETRY_PERIOD_S = 0.02  # karmel.yaml serial.telemetry_hz = 50


@dataclass(frozen=True)
class SyncStats:
    mode: str
    iterations: int
    duplicates: int  # iterations that saw the same robot timestamp as the previous one
    missed: int  # telemetry samples that were never seen (gaps in robot time)

    def line(self) -> str:
        return (f"{self.mode:9s} iterations={self.iterations:4d}  duplicates={self.duplicates:3d}  "
                f"missed={self.missed:3d}")


def analyze(mode: str, robot_times: list[float], sample_period_s: float = TELEMETRY_PERIOD_S) -> SyncStats:
    """Count duplicate and missed samples from the robot timestamps each iteration saw."""
    duplicates = missed = 0
    for previous, current in zip(robot_times, robot_times[1:]):
        if current == previous:
            duplicates += 1
        elif current - previous > 1.5 * sample_period_s:  # a gap of 40 ms = 1 sample never seen
            missed += round((current - previous) / sample_period_s) - 1
    return SyncStats(mode, len(robot_times), duplicates, missed)


def run_timer(base: SerialBase, rate_hz: float, seconds: float) -> list[float]:
    rate = GridRate(rate_hz)
    seen = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        seen.append(base.read().t)  # the newest sample, whenever that was
        time.sleep(rate.delay())
    return seen


def run_telemetry(base: SerialBase, seconds: float) -> list[float]:
    state = base.read()
    seen = [state.t]
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        state = base.wait_for_telemetry(timeout_s=1.0, newer_than=state.t)  # blocks until a NEW sample
        seen.append(state.t)
    return seen


def main(argv: list[str] | None = None) -> list[SyncStats]:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="socket://localhost:5760")
    parser.add_argument("--fake", action="store_true", help="start a fake Pico in this process")
    parser.add_argument("--mode", choices=("timer", "telemetry", "both"), default="both")
    parser.add_argument("--rate", type=float, default=50.0, help="timer mode: loop rate in Hz")
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args(argv)

    server = None
    port = args.port
    if args.fake:
        from robotlab.fake_pico import FakePico, FakePicoServer

        server = FakePicoServer(FakePico(), port=0).start()
        port = server.url
    results = []
    try:
        with SerialBase(port) as base:
            if args.mode in ("timer", "both"):
                results.append(analyze(f"timer@{args.rate:g}", run_timer(base, args.rate, args.seconds)))
                print(results[-1].line(), flush=True)
            if args.mode in ("telemetry", "both"):
                results.append(analyze("telemetry", run_telemetry(base, args.seconds)))
                print(results[-1].line(), flush=True)
    finally:
        if server is not None:
            server.stop()
    return results


if __name__ == "__main__":
    main()
