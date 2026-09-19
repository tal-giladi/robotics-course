"""Checker for 03.04 — A drift-free fixed-rate scheduler, tested with an injectable clock.

Run: ``python course.py check 03.04`` (or ``--solution`` to see the reference pass).

Almost every test uses FakeClock: time only moves when the scheduler sleeps or the "loop body"
does work, so 10 simulated seconds take microseconds and the results are exact.
"""

from __future__ import annotations

import math
import time

import numpy as np
import pytest


class FakeClock:
    """A monotonic clock you control. ``sleep`` can oversleep, like a real OS scheduler."""

    def __init__(self, start: float = 1000.0, oversleep_s: float = 0.0) -> None:
        self.now = start
        self.oversleep_s = oversleep_s
        self.sleeps: list[float] = []

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        assert seconds >= 0.0, f"sleep() called with a negative duration {seconds}"
        self.sleeps.append(seconds)
        self.now += seconds + self.oversleep_s

    def work(self, seconds: float) -> None:
        self.now += seconds


def make(impl, rate_hz: float, clock: FakeClock):
    return impl.FixedRateScheduler(rate_hz, clock=clock, sleep=clock.sleep)


# --- basics ---------------------------------------------------------------------------------------
def test_period_and_start_come_from_the_injected_clock(impl):
    clock = FakeClock(start=1000.0)
    sched = make(impl, 50.0, clock)
    assert sched.period == pytest.approx(0.02)
    sched.wait()
    assert clock.now == pytest.approx(1000.02), "first tick = one period after construction"
    assert clock.sleeps == pytest.approx([0.02])


@pytest.mark.parametrize("rate", [0.0, -5.0])
def test_rejects_non_positive_rates(impl, rate):
    with pytest.raises(ValueError):
        impl.FixedRateScheduler(rate, clock=FakeClock(), sleep=FakeClock().sleep)


def test_perfect_sleep_1000_ticks_is_exactly_10_seconds(impl):
    clock = FakeClock()
    sched = make(impl, 100.0, clock)
    for _ in range(1000):
        assert sched.wait() == 0
    assert clock.now - 1000.0 == pytest.approx(10.0, abs=1e-9)
    assert sched.ticks == 1000 and sched.skipped == 0


# --- drift-free: errors don't accumulate ------------------------------------------------------------
def test_oversleep_does_not_accumulate(impl):
    # The OS wakes us 1 ms late EVERY time. A naive `sleep(period)` loop would end at 11.0 s.
    clock = FakeClock(oversleep_s=0.001)
    sched = make(impl, 100.0, clock)
    for _ in range(1000):
        sched.wait()
    assert clock.now - 1000.0 == pytest.approx(10.001, abs=1e-6), (
        "each wake-up is 1 ms late, but the next deadline must stay on the grid t0 + k*period"
    )


def test_loop_body_time_is_absorbed(impl):
    clock = FakeClock()
    sched = make(impl, 100.0, clock)
    wakes = []
    for _ in range(500):
        clock.work(0.003)  # the control computation takes 3 ms
        sched.wait()
        wakes.append(clock.now)
    periods = np.diff(wakes)
    assert periods == pytest.approx(0.01, abs=1e-9), "sleep(period - elapsed), not sleep(period)"
    assert clock.sleeps[-1] == pytest.approx(0.007)


def test_slightly_late_tick_runs_immediately_and_stays_on_the_grid(impl):
    clock = FakeClock()
    sched = make(impl, 100.0, clock)
    sched.wait()  # t = 0.010
    clock.work(0.012)  # t = 0.022: the deadline 0.020 was missed by 2 ms (less than a period)
    n_sleeps = len(clock.sleeps)
    assert sched.wait() == 0
    assert len(clock.sleeps) == n_sleeps, "late: don't sleep at all"
    assert clock.now == pytest.approx(1000.022)
    sched.wait()
    assert clock.now == pytest.approx(1000.030), "next deadline is still t0 + 3 * period"


# --- overruns -------------------------------------------------------------------------------------
def test_long_overrun_skips_ticks_instead_of_bursting(impl):
    clock = FakeClock()
    sched = make(impl, 100.0, clock)
    sched.wait()  # t = 0.010
    clock.work(0.025)  # t = 0.035: deadlines 0.020 and 0.030 both passed
    assert sched.wait() == 1, "0.020 is this tick (late); 0.030 is skipped: 1 missed"
    sleeps_before = len(clock.sleeps)
    sched.wait()
    assert len(clock.sleeps) == sleeps_before + 1, "must sleep again, not burst through old deadlines"
    assert clock.now == pytest.approx(1000.040)
    assert sched.skipped == 1 and sched.ticks == 3


def test_phase_is_preserved_after_a_debugger_pause(impl):
    clock = FakeClock(start=0.0)
    sched = make(impl, 20.0, clock)  # 50 ms
    for _ in range(3):
        sched.wait()
    clock.work(1.234)  # breakpoint
    missed = sched.wait()
    assert missed == 23  # now = 1.384: 24 deadlines (0.20 ... 1.35) passed; one runs now, 23 skipped
    sched.wait()
    assert clock.now == pytest.approx(1.40)
    assert (clock.now / 0.05) == pytest.approx(round(clock.now / 0.05)), "wake-ups stay on the 50 ms grid"


def test_never_sleeps_negative(impl):
    clock = FakeClock(oversleep_s=0.0004)
    sched = make(impl, 250.0, clock)
    rng = np.random.default_rng(0)
    for _ in range(2000):
        clock.work(float(rng.exponential(0.002)))
        sched.wait()  # FakeClock.sleep asserts the duration is >= 0
    assert sched.ticks + sched.skipped == pytest.approx((clock.now - 1000.0) / 0.004, abs=1.0)


# --- statistics ----------------------------------------------------------------------------------------
def test_summarize_periods_by_hand(impl):
    # periods: 0.010, 0.012, 0.008, 0.010 -> mean 0.010; errors 0, 2, 2, 0 ms
    stats = impl.summarize_periods([0.0, 0.010, 0.022, 0.030, 0.040], nominal_period_s=0.010)
    assert stats.count == 4
    assert stats.mean_s == pytest.approx(0.010)
    assert stats.std_s == pytest.approx(math.sqrt((0 + 4e-6 + 4e-6 + 0) / 4))  # population std
    assert (stats.min_s, stats.max_s) == pytest.approx((0.008, 0.012))
    assert stats.worst_error_s == pytest.approx(0.002)
    assert stats.p99_error_s == pytest.approx(np.percentile([0, 0.002, 0.002, 0], 99))


def test_summarize_periods_needs_two_samples(impl):
    with pytest.raises(ValueError):
        impl.summarize_periods([1.0], 0.01)


# --- the real clock ------------------------------------------------------------------------------------
def test_real_clock_smoke(impl):
    sched = impl.FixedRateScheduler(50.0)
    start = time.monotonic()
    wakes = []
    for _ in range(10):
        sched.wait()
        wakes.append(time.monotonic())
    elapsed = time.monotonic() - start
    assert 0.17 < elapsed < 0.40, f"10 ticks at 50 Hz should take ~0.2 s, took {elapsed:.3f} s"
    stats = impl.summarize_periods(wakes, 0.02)
    assert stats.mean_s == pytest.approx(0.02, abs=0.01)
