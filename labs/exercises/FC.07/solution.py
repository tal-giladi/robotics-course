"""FC.07 — reference solution: deadline monitor, schedulability test and watchdog."""

from __future__ import annotations

import statistics
from dataclasses import dataclass


@dataclass(frozen=True)
class LoopStats:
    count: int
    mean_ms: float
    min_ms: float
    max_ms: float
    p99_ms: float
    worst_jitter_ms: float
    misses: int
    miss_fraction: float


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise ValueError("percentile of an empty list")
    ordered = sorted(values)
    index = int(round(fraction * (len(ordered) - 1)))
    index = min(len(ordered) - 1, max(0, index))
    return ordered[index]


def analyse_loop(timestamps_s: list[float], nominal_hz: float, tolerance: float = 0.10) -> LoopStats:
    if nominal_hz <= 0:
        raise ValueError("nominal_hz must be positive")
    if len(timestamps_s) < 2:
        return LoopStats(0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0)
    nominal_ms = 1000.0 / nominal_hz
    periods = [1000.0 * (b - a) for a, b in zip(timestamps_s, timestamps_s[1:])]
    limit = nominal_ms * (1.0 + tolerance)
    misses = sum(1 for p in periods if p > limit)
    return LoopStats(
        count=len(periods),
        mean_ms=statistics.fmean(periods),
        min_ms=min(periods),
        max_ms=max(periods),
        p99_ms=percentile(periods, 0.99),
        worst_jitter_ms=max(abs(p - nominal_ms) for p in periods),
        misses=misses,
        miss_fraction=misses / len(periods),
    )


@dataclass(frozen=True)
class Task:
    name: str
    period_ms: float
    wcet_ms: float


def total_utilisation(tasks: list[Task]) -> float:
    return sum(task.wcet_ms / task.period_ms for task in tasks)


def rate_monotonic_bound(n: int) -> float:
    if n <= 0:
        raise ValueError("n must be positive")
    return n * (2.0 ** (1.0 / n) - 1.0)


def is_schedulable(tasks: list[Task]) -> tuple[bool, str]:
    utilisation = total_utilisation(tasks)
    bound = rate_monotonic_bound(len(tasks)) if tasks else 1.0
    if utilisation > 1.0:
        return False, f"overloaded: U = {utilisation:.3f} > 1.0"
    if utilisation <= bound:
        return True, f"guaranteed: U = {utilisation:.3f} <= bound {bound:.3f}"
    return True, (f"possible: U = {utilisation:.3f} is above the bound {bound:.3f} "
                  "- needs response-time analysis")


TICKS_PERIOD = 1 << 30
TICKS_HALF = TICKS_PERIOD // 2


def ticks_diff(new_ms: int, old_ms: int) -> int:
    return ((new_ms - old_ms + TICKS_HALF) % TICKS_PERIOD) - TICKS_HALF


class CommandWatchdog:
    """See student.py for the contract; this mirrors labs/firmware/pico/watchdog.py."""

    def __init__(self, timeout_ms: int, now_ms: int) -> None:
        self.timeout_ms = timeout_ms
        self._last_feed_ms = now_ms
        self.tripped = True

    def feed(self, now_ms: int) -> None:
        self._last_feed_ms = now_ms
        self.tripped = False

    def elapsed_ms(self, now_ms: int) -> int:
        return ticks_diff(now_ms, self._last_feed_ms)

    def check(self, now_ms: int) -> bool:
        if self.tripped:
            return False
        if self.elapsed_ms(now_ms) >= self.timeout_ms:
            self.tripped = True
            return True
        return False


def stopping_distance_m(speed_m_s: float, timeout_ms: float, control_period_ms: float,
                        decel_m_s2: float) -> float:
    if decel_m_s2 <= 0:
        raise ValueError("decel_m_s2 must be positive")
    coasting_s = (timeout_ms + control_period_ms) / 1000.0
    return speed_m_s * coasting_s + speed_m_s ** 2 / (2.0 * decel_m_s2)
