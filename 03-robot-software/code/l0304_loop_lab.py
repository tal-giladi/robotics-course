"""How regular is a Python control loop, really? Five scenarios, one table.

Lesson 03.04 (loops, timing and concurrency). Standard library + numpy, runs anywhere; run it on
the Raspberry Pi for the numbers that matter.

    python 03-robot-software/code/l0304_loop_lab.py                       # all scenarios, 50 Hz, 3 s each
    python 03-robot-software/code/l0304_loop_lab.py --rate 100 --seconds 5 --only naive drift_free gil_thread
    python 03-robot-software/code/l0304_loop_lab.py --fifo 50              # Linux: SCHED_FIFO priority 50 (needs CAP_SYS_NICE)

Scenarios (each loop body "works" for --work-ms of CPU, like a controller update):
    naive          work(); time.sleep(period)                          -> drifts
    drift_free     deadlines on a grid t0 + k*period                    -> no drift, same jitter
    gil_thread     drift_free + a pure-Python CPU-bound thread           -> GIL contention
    process        drift_free + the same CPU burner in another process  -> no GIL contention
    asyncio        drift_free inside an asyncio event loop
    asyncio_block  the same + another task that makes one 40 ms blocking call per second

Columns: ticks run, ticks lost (a perfect loop would have run that many more in the same time:
drift or skipped overruns), mean period, standard deviation, p99 and worst |period - nominal|.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import multiprocessing as mp
import os
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np


# --- a minimal drift-free rate (the exercise 03.04 builds the full version) ---------------------------
class GridRate:
    """Sleep until t0 + k*period; resynchronize to the grid after an overrun."""

    def __init__(self, rate_hz: float, clock: Callable[[], float] = time.perf_counter) -> None:
        self.period, self.clock = 1.0 / rate_hz, clock
        self.next = clock() + self.period

    def delay(self) -> float:
        """Seconds to wait until the next deadline (0 if late); advances the deadline."""
        now = self.clock()
        delay = self.next - now
        if delay < 0:  # late: run now; the next deadline is the first grid point after now
            self.next += max(1, math.ceil(-delay / self.period)) * self.period
            return 0.0
        self.next += self.period
        return delay


def burn(seconds: float) -> None:
    """Pure-Python CPU work (holds the GIL), like a naive controller or planner step."""
    end = time.perf_counter() + seconds
    x = 0
    while time.perf_counter() < end:
        x += 1


def cpu_hog(stop: object) -> None:
    """Run forever (or until ``stop`` is set) doing pure-Python arithmetic."""
    n = 0
    while not stop.is_set():  # type: ignore[attr-defined]
        for _ in range(10_000):
            n = (n * 31 + 7) % 1_000_003


@dataclass(frozen=True)
class Result:
    name: str
    samples: int
    mean_ms: float
    std_ms: float
    p99_err_ms: float
    max_err_ms: float
    lost_ticks: int  # ticks a perfect loop would have run in the same time, minus the ticks we ran

    def row(self) -> str:
        return (f"{self.name:13s} {self.samples:6d} {self.lost_ticks:5d} {self.mean_ms:9.3f} {self.std_ms:8.3f} "
                f"{self.p99_err_ms:9.3f} {self.max_err_ms:9.3f}")


HEADER = (f"{'scenario':13s} {'ticks':>6s} {'lost':>5s} {'mean ms':>9s} {'std ms':>8s} "
          f"{'p99 err':>9s} {'max err':>9s}")


def summarize(name: str, wakes: list[float], period: float) -> Result:
    t = np.asarray(wakes)
    periods = np.diff(t)
    errors = np.abs(periods - period)
    expected = round((t[-1] - t[0]) / period)
    return Result(name, len(periods), periods.mean() * 1e3, periods.std() * 1e3,
                  float(np.percentile(errors, 99)) * 1e3, errors.max() * 1e3, expected - len(periods))


# --- scenarios --------------------------------------------------------------------------------------------
def run_naive(rate: float, seconds: float, work_s: float) -> list[float]:
    period, wakes = 1.0 / rate, []
    end = time.perf_counter() + seconds
    while (now := time.perf_counter()) < end:
        wakes.append(now)
        burn(work_s)
        time.sleep(period)  # "sleep one period" — ignores the work time and every oversleep
    return wakes


def run_drift_free(rate: float, seconds: float, work_s: float) -> list[float]:
    rate_ = GridRate(rate)
    wakes = []
    end = time.perf_counter() + seconds
    while (now := time.perf_counter()) < end:
        wakes.append(now)
        burn(work_s)
        time.sleep(rate_.delay())
    return wakes


def run_with_thread_hog(rate: float, seconds: float, work_s: float) -> list[float]:
    stop = threading.Event()
    hog = threading.Thread(target=cpu_hog, args=(stop,), daemon=True)
    hog.start()
    try:
        return run_drift_free(rate, seconds, work_s)
    finally:
        stop.set()
        hog.join()


def run_with_process_hog(rate: float, seconds: float, work_s: float) -> list[float]:
    ctx = mp.get_context("spawn")  # the same on Windows, macOS and Linux
    stop = ctx.Event()
    hog = ctx.Process(target=cpu_hog, args=(stop,), daemon=True)
    hog.start()
    time.sleep(0.5)  # let the child start before measuring
    try:
        return run_drift_free(rate, seconds, work_s)
    finally:
        stop.set()
        hog.join()


def run_asyncio_clean(rate: float, seconds: float, work_s: float) -> list[float]:
    return run_asyncio(rate, seconds, work_s, blocking_s=0.0)


def run_asyncio(rate: float, seconds: float, work_s: float, blocking_s: float = 0.040) -> list[float]:
    async def control(wakes: list[float]) -> None:
        loop = asyncio.get_running_loop()
        rate_ = GridRate(rate, clock=loop.time)
        end = loop.time() + seconds
        while loop.time() < end:
            wakes.append(time.perf_counter())
            burn(work_s)
            await asyncio.sleep(rate_.delay())

    async def blocker(seconds_total: float) -> None:
        loop = asyncio.get_running_loop()
        end = loop.time() + seconds_total
        while loop.time() < end:
            await asyncio.sleep(1.0)
            time.sleep(blocking_s)  # BUG on purpose: a blocking call (e.g. a synchronous HTTP/serial read)

    async def main() -> list[float]:
        wakes: list[float] = []
        await asyncio.gather(control(wakes), blocker(seconds))
        return wakes

    return asyncio.run(main())


SCENARIOS: dict[str, Callable[[float, float, float], list[float]]] = {
    "naive": run_naive,
    "drift_free": run_drift_free,
    "gil_thread": run_with_thread_hog,
    "process": run_with_process_hog,
    "asyncio": run_asyncio_clean,
    "asyncio_block": run_asyncio,
}


def set_fifo(priority: int) -> str:
    """Linux only: real-time FIFO scheduling for this process (needs root or CAP_SYS_NICE)."""
    if not hasattr(os, "sched_setscheduler"):
        return "SCHED_FIFO not available on this OS"
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
    except PermissionError:
        return "SCHED_FIFO refused: run with CAP_SYS_NICE (sudo, or docker --cap-add SYS_NICE)"
    return f"SCHED_FIFO priority {priority}"


def main(argv: list[str] | None = None) -> list[Result]:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--rate", type=float, default=50.0, help="loop rate in Hz (default 50)")
    parser.add_argument("--seconds", type=float, default=3.0, help="duration of each scenario")
    parser.add_argument("--work-ms", type=float, default=3.0, help="CPU time of the loop body")
    parser.add_argument("--only", nargs="+", choices=list(SCENARIOS), help="run only these scenarios")
    parser.add_argument("--fifo", type=int, metavar="PRIORITY", help="Linux: SCHED_FIFO with this priority (1-99)")
    args = parser.parse_args(argv)

    policy = set_fifo(args.fifo) if args.fifo else "default scheduler"
    print(f"{sys.platform}, Python {sys.version.split()[0]}, {os.cpu_count()} CPUs, {policy}, "
          f"{args.rate:g} Hz, body {args.work_ms:g} ms, {args.seconds:g} s per scenario")
    print(HEADER)
    results = []
    for name in args.only or SCENARIOS:
        result = summarize(name, SCENARIOS[name](args.rate, args.seconds, args.work_ms / 1e3), 1.0 / args.rate)
        print(result.row(), flush=True)
        results.append(result)
    return results


if __name__ == "__main__":
    main()
