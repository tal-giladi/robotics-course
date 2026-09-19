"""Reference solution for 03.04 — A drift-free fixed-rate scheduler.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np


class FixedRateScheduler:
    """Wake up on the grid ``t0 + k * period`` of an injectable monotonic clock."""

    def __init__(
        self,
        rate_hz: float,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if not rate_hz > 0:
            raise ValueError(f"rate_hz must be positive, got {rate_hz!r}")
        self.period = 1.0 / rate_hz
        self._clock = clock
        self._sleep = sleep
        self.t0 = clock()
        self._k = 1  # index of the next deadline
        self.ticks = 0
        self.skipped = 0

    @property
    def next_deadline(self) -> float:
        return self.t0 + self._k * self.period

    def wait(self) -> int:
        deadline = self.next_deadline
        now = self._clock()
        missed = 0
        if now < deadline:
            self._sleep(deadline - now)
        else:
            # Late. If whole periods have passed, skip those deadlines instead of bursting.
            missed = math.floor((now - deadline) / self.period)
        self._k += missed + 1
        self.ticks += 1
        self.skipped += missed
        return missed


@dataclass(frozen=True)
class PeriodStats:
    count: int  # number of periods (len(timestamps) - 1)
    mean_s: float
    std_s: float
    min_s: float
    max_s: float
    worst_error_s: float  # max |period - nominal|
    p99_error_s: float  # 99th percentile of |period - nominal| (numpy.percentile, linear)


def summarize_periods(timestamps: Sequence[float], nominal_period_s: float) -> PeriodStats:
    t = np.asarray(timestamps, dtype=float)
    if t.size < 2:
        raise ValueError("need at least two timestamps")
    periods = np.diff(t)
    errors = np.abs(periods - nominal_period_s)
    return PeriodStats(
        count=int(periods.size),
        mean_s=float(periods.mean()),
        std_s=float(periods.std()),
        min_s=float(periods.min()),
        max_s=float(periods.max()),
        worst_error_s=float(errors.max()),
        p99_error_s=float(np.percentile(errors, 99)),
    )
