"""FC.07 - measure jitter on the machine you are sitting at, then budget a real-time loop.

Run it on your PC or the Pi:  py fc07_jitter_lab.py       (or python3 fc07_jitter_lab.py)
Standard library only.

Four parts:
  1. a 100 Hz loop with a naive `sleep(period)` and with a deadline-based sleep - measured
  2. the same loop while the machine is busy - what a *tail* looks like
  3. the latency chain of one karmel control step, as a budget with numbers
  4. a watchdog timeline: when does "no command for 300 ms" actually fire
"""

from __future__ import annotations

import math
import statistics
import time
from dataclasses import dataclass


# --------------------------------------------------------------------------- 1 & 2: measurement
def run_loop(rate_hz: float, seconds: float, *, deadline_based: bool, work_us: int = 0) -> list[float]:
    """Run a fixed-rate loop and return the measured period of each iteration, in milliseconds."""
    period = 1.0 / rate_hz
    stamps: list[float] = []
    start = time.perf_counter()
    next_deadline = start + period
    while time.perf_counter() - start < seconds:
        if work_us:
            spin_until = time.perf_counter() + work_us / 1e6
            while time.perf_counter() < spin_until:
                pass
        stamps.append(time.perf_counter())
        if deadline_based:
            sleep_for = next_deadline - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            # Keep the grid even after an overrun: never `next = now + period`.
            missed = math.floor((time.perf_counter() - next_deadline) / period)
            next_deadline += period * (1 + max(missed, 0))
        else:
            time.sleep(period)
    return [1000.0 * (b - a) for a, b in zip(stamps, stamps[1:])]


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round(fraction * (len(ordered) - 1)))))
    return ordered[index]


def report(label: str, periods: list[float], nominal_ms: float) -> None:
    errors = [abs(p - nominal_ms) for p in periods]
    print(
        f"  {label:<34} n={len(periods):5d}  mean {statistics.fmean(periods):6.3f} ms"
        f"  p50 err {percentile(errors, 0.50):5.3f}  p99 err {percentile(errors, 0.99):6.3f}"
        f"  worst err {max(errors):7.3f} ms"
    )


def drift(periods: list[float], nominal_ms: float) -> float:
    """How far the loop has slipped from the ideal grid after all these iterations, in ms."""
    return sum(periods) - nominal_ms * len(periods)


# --------------------------------------------------------------------------- 3: latency budget
@dataclass(frozen=True)
class Stage:
    name: str
    typical_us: float
    worst_us: float
    where: str


KARMEL_VELOCITY_LOOP = [
    Stage("encoder edge -> PIO register", 0.1, 0.2, "Pico hardware"),
    Stage("control tick wakes up (asyncio)", 200.0, 3000.0, "Pico, MicroPython"),
    Stage("read counts, estimate speed", 300.0, 600.0, "Pico, MicroPython"),
    Stage("PID + feedforward", 250.0, 500.0, "Pico, MicroPython"),
    Stage("write PWM duty registers", 20.0, 40.0, "Pico hardware"),
    Stage("motor current reaches the new value", 500.0, 800.0, "electrical, L/R"),
]

KARMEL_NAV_CHAIN = [
    Stage("telemetry line leaves the Pico", 200.0, 400.0, "Pico"),
    Stage("USB CDC transfer to the Pi", 1000.0, 5000.0, "USB, 1 ms frames"),
    Stage("Python read + parse", 300.0, 3000.0, "Pi, CPython + GC"),
    Stage("odometry + Nav2 controller step", 10000.0, 40000.0, "Pi, 20 Hz controller"),
    Stage("command line back to the Pico", 1200.0, 5000.0, "USB"),
]


def print_budget(title: str, stages: list[Stage], deadline_ms: float) -> None:
    print(f"\n{title}   (deadline {deadline_ms:.1f} ms)")
    print(f"  {'stage':<36} {'typical':>10} {'worst':>10}   where")
    for stage in stages:
        print(
            f"  {stage.name:<36} {stage.typical_us:7.1f} us {stage.worst_us:7.1f} us"
            f"   {stage.where}"
        )
    typical = sum(s.typical_us for s in stages) / 1000.0
    worst = sum(s.worst_us for s in stages) / 1000.0
    print(f"  {'TOTAL':<36} {typical:7.3f} ms {worst:7.3f} ms")
    print(
        f"  utilisation: typical {100 * typical / deadline_ms:5.1f} %, "
        f"worst case {100 * worst / deadline_ms:5.1f} %  -> "
        f"{'MEETS the deadline' if worst <= deadline_ms else 'MISSES the deadline in the worst case'}"
    )


# --------------------------------------------------------------------------- 4: watchdog timeline
def watchdog_timeline(command_times_ms: list[float], timeout_ms: float, horizon_ms: float) -> list[str]:
    """Replay the CommandWatchdog rule from labs/firmware/pico/watchdog.py at 100 Hz."""
    events: list[str] = []
    last_feed = 0.0
    tripped = True                      # boot state: stopped until the first command
    events.append("t=   0.0 ms  boot: watchdog tripped, motors stopped")
    pending = list(command_times_ms)
    tick = 0.0
    while tick <= horizon_ms:
        while pending and pending[0] <= tick:
            last_feed = pending.pop(0)
            if tripped:
                events.append(f"t={last_feed:7.1f} ms  command arrived: watchdog fed, motors released")
            tripped = False
        if not tripped and tick - last_feed >= timeout_ms:
            tripped = True
            events.append(
                f"t={tick:7.1f} ms  no command for {tick - last_feed:.1f} ms: BRAKE "
                f"(last command was at {last_feed:.1f} ms)"
            )
        tick += 10.0                    # the control step, config.CONTROL_HZ = 100
    return events


def main() -> None:
    nominal_ms = 10.0                   # 100 Hz, karmel's control rate

    print("1. A 100 Hz loop on this machine, 3 seconds each way")
    naive = run_loop(100.0, 3.0, deadline_based=False)
    exact = run_loop(100.0, 3.0, deadline_based=True)
    report("sleep(period) after the work", naive, nominal_ms)
    report("sleep until the next deadline", exact, nominal_ms)
    print(f"  accumulated drift: naive {drift(naive, nominal_ms):+8.1f} ms, "
          f"deadline-based {drift(exact, nominal_ms):+8.1f} ms")

    print("\n2. The same deadline-based loop with 4 ms of work in each 10 ms step")
    busy = run_loop(100.0, 3.0, deadline_based=True, work_us=4000)
    report("deadline-based, 40 % utilisation", busy, nominal_ms)
    print("  The mean barely moves; look at the p99 and the worst case instead. That is jitter:")
    print("  a robot is hurt by the tail, not by the average.")

    print_budget("3a. karmel wheel-velocity loop, on the Pico", KARMEL_VELOCITY_LOOP, 10.0)
    print_budget("3b. sensor -> decision -> actuation, through the Pi", KARMEL_NAV_CHAIN, 50.0)

    print("\n4. Command watchdog, timeout 300 ms, checked every 10 ms control step")
    for event in watchdog_timeline([50.0, 120.0, 190.0, 260.0, 900.0], 300.0, 1200.0):
        print("  " + event)
    print("  Note the two costs: the robot keeps its last command for up to 300 ms + one")
    print("  control step, and it does not restart until a command arrives.")


if __name__ == "__main__":
    main()
