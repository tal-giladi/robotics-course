"""01.15 — Measure the link you are driving over: round trip, telemetry rate, command→motion.

    python 01-first-robot/code/link_latency.py --fake
    python 01-first-robot/code/link_latency.py --port socket://karmel.local:5760   # over Wi-Fi
    python 01-first-robot/code/link_latency.py --port /dev/ttyACM0 --move          # WHEELS IN THE AIR

Three numbers decide whether teleop feels alive and how far the robot travels after you let go:

1. **round trip** — send ``H``, wait for the ``I`` reply. USB is sub-millisecond; add Wi-Fi and
   it becomes single-digit milliseconds, with a long tail when the link is busy or far away.
2. **telemetry interval** — how often the robot tells you what it is doing (50 Hz = 20 ms by
   default). The *jitter* matters more than the mean: a 200 ms gap means 200 ms of blindness.
3. **command to motion** (``--move``) — from ``set_wheel_velocity`` to the first telemetry that
   shows the wheels turning. That is link latency plus the motor's own time constant, and it is
   the number to put in ``--reaction-time`` in ``labs/robot/stop_before_obstacle.py``.

SAFETY: ``--move`` spins the wheels. Do it with the robot on a stand (wheels in the air).
"""

from __future__ import annotations

import argparse
import statistics
import time

from first_robot_lab import add_target_args, open_target

PERCENTILES = (50, 90, 99)


def percentile(values: list[float], pct: float) -> float:
    """Nearest-rank percentile — no numpy needed, and exact for small samples."""
    ordered = sorted(values)
    rank = max(1, min(len(ordered), round(pct / 100.0 * len(ordered))))
    return ordered[rank - 1]


def report(name: str, values_s: list[float]) -> dict[str, float]:
    ms = [v * 1000.0 for v in values_s]
    stats = {"mean_ms": statistics.mean(ms), "max_ms": max(ms)}
    stats.update({f"p{p}_ms": percentile(ms, p) for p in PERCENTILES})
    tail = "  ".join(f"p{p} {stats[f'p{p}_ms']:6.2f}" for p in PERCENTILES)
    print(f"{name:<22} n={len(ms):4d}  mean {stats['mean_ms']:6.2f} ms  {tail}  max {stats['max_ms']:7.2f} ms")
    return stats


def measure_round_trip(target, count: int) -> list[float]:
    """Time ``H`` -> ``I``: the smallest possible request/response on this link."""
    samples = []
    for _ in range(count):
        start = time.monotonic()
        target.base.hello(timeout_s=2.0)
        samples.append(time.monotonic() - start)
        time.sleep(0.01)
    return samples


def measure_telemetry_intervals(target, seconds: float) -> list[float]:
    """Gaps between consecutive telemetry samples, measured by the host clock."""
    samples = []
    state = target.base.wait_for_telemetry(timeout_s=2.0)
    previous_wall = time.monotonic()
    deadline = previous_wall + seconds
    while time.monotonic() < deadline:
        state = target.base.wait_for_telemetry(timeout_s=2.0, newer_than=state.t)
        now = time.monotonic()
        samples.append(now - previous_wall)
        previous_wall = now
    return samples


def measure_command_to_motion(target, count: int, speed_rad_s: float = 6.0) -> list[float]:
    """From set_wheel_velocity() to the first telemetry showing the wheels actually turning."""
    samples = []
    for _ in range(count):
        target.base.stop()
        time.sleep(0.5)  # make sure the wheels are at rest before each trial
        state = target.base.wait_for_telemetry(timeout_s=2.0)
        start = time.monotonic()
        while time.monotonic() - start < 2.0:
            target.base.set_wheel_velocity(speed_rad_s, speed_rad_s)  # SAFETY: the wheels turn here
            state = target.base.wait_for_telemetry(timeout_s=2.0, newer_than=state.t)
            if abs(state.left_rad_s) > 0.5 * speed_rad_s:
                samples.append(time.monotonic() - start)
                break
        target.base.stop()
        time.sleep(0.3)
    return samples


def main(argv: list[str] | None = None) -> dict[str, dict[str, float]]:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pings", type=int, default=50, help="round-trip samples (default 50)")
    ap.add_argument("--listen", type=float, default=3.0, help="seconds of telemetry to time")
    ap.add_argument("--move", action="store_true", help="also measure command->motion (wheels in the air!)")
    ap.add_argument("--moves", type=int, default=5, help="command->motion trials (default 5)")
    add_target_args(ap)
    args = ap.parse_args(argv)

    results: dict[str, dict[str, float]] = {}
    with open_target(args) as target:
        firmware = target.base.firmware
        print(f"connected to {target.base.port} — firmware {firmware.firmware_version}, "
              f"protocol v{firmware.protocol_version}\n")
        results["round_trip"] = report("round trip (H->I)", measure_round_trip(target, args.pings))
        intervals = measure_telemetry_intervals(target, args.listen)
        results["telemetry_interval"] = report("telemetry interval", intervals)
        print(f"{'telemetry rate':<22} {len(intervals) / sum(intervals):.1f} Hz")
        if args.move:
            results["command_to_motion"] = report("command -> motion", measure_command_to_motion(target, args.moves))
        print(f"\nlink counters: {target.base.stats}")
    return results


if __name__ == "__main__":
    main()
