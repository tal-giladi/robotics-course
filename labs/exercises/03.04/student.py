"""03.04 — A drift-free fixed-rate scheduler, tested with an injectable clock.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 03.04``.
Standard library + numpy.

The loop you are building::

    sched = FixedRateScheduler(50.0)          # real clock by default
    while running:
        state = base.read()
        base.set_wheel_velocity(*controller.update(state))
        missed = sched.wait()                 # sleep until the next 20 ms grid point
        if missed:
            log.warning("control loop overran by %d periods", missed)

Tests pass ``clock=`` and ``sleep=`` fakes, so the scheduler must never call ``time.monotonic``
or ``time.sleep`` directly — only the injected callables.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np  # noqa: F401  (useful for summarize_periods)


class FixedRateScheduler:
    """Wake up on the grid ``t0 + k * period`` (k = 1, 2, 3, …) of a monotonic clock.

    * ``t0`` is ``clock()`` at construction; ``period = 1 / rate_hz``; ``rate_hz <= 0`` -> ValueError.
    * ``wait()``:
        - on time (``now < next deadline``): ``sleep(deadline - now)``, return 0;
        - a little late (less than one full period past the deadline): don't sleep, return 0;
        - very late (``n >= 1`` whole periods past the deadline, ``n = floor((now - deadline) / period)``):
          don't sleep, skip those ``n`` deadlines instead of running a burst of ticks, return ``n``.
      In every case the next deadline stays on the grid — never ``now + period``.
    * Counters: ``ticks`` (calls to wait) and ``skipped`` (sum of the returned values).
    """

    def __init__(
        self,
        rate_hz: float,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        # TODO(student): validate rate_hz; store period, clock, sleep; t0 = clock(); counters = 0;
        #   and whatever you need to know which grid point comes next.
        raise NotImplementedError("FixedRateScheduler.__init__")

    def wait(self) -> int:
        # TODO(student): implement the three cases in the class docstring.
        raise NotImplementedError("FixedRateScheduler.wait")


@dataclass(frozen=True)
class PeriodStats:
    count: int  # number of periods (len(timestamps) - 1)
    mean_s: float
    std_s: float  # population standard deviation (numpy's default, ddof=0)
    min_s: float
    max_s: float
    worst_error_s: float  # max |period - nominal|
    p99_error_s: float  # 99th percentile of |period - nominal|, numpy.percentile default method


def summarize_periods(timestamps: Sequence[float], nominal_period_s: float) -> PeriodStats:
    """Loop wake-up timestamps -> period statistics. Fewer than 2 timestamps -> ValueError."""
    # TODO(student): periods = differences of consecutive timestamps; errors = |period - nominal|.
    raise NotImplementedError("summarize_periods")
