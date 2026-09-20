"""Checker for FC.07 — is this loop real-time?

Run: ``python course.py check FC.07`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import pytest


def grid(n: int, period_s: float, start: float = 1000.0) -> list[float]:
    return [start + i * period_s for i in range(n)]


# --- percentile -------------------------------------------------------------------------------------
def test_percentile_picks_by_rank(impl):
    values = [float(i) for i in range(1, 101)]      # 1..100
    assert impl.percentile(values, 0.0) == 1.0
    assert impl.percentile(values, 1.0) == 100.0
    assert impl.percentile(values, 0.5) == pytest.approx(50.0, abs=1.0)
    assert impl.percentile(values, 0.99) == pytest.approx(99.0, abs=1.0)


def test_percentile_of_an_empty_list_raises(impl):
    with pytest.raises(ValueError):
        impl.percentile([], 0.5)


# --- analyse_loop -----------------------------------------------------------------------------------
def test_a_perfect_loop_has_no_jitter(impl):
    stats = impl.analyse_loop(grid(101, 0.01), 100.0)
    assert stats.count == 100
    assert stats.mean_ms == pytest.approx(10.0)
    assert stats.worst_jitter_ms == pytest.approx(0.0, abs=1e-9)
    assert stats.misses == 0
    assert stats.miss_fraction == 0.0


def test_one_late_step_is_one_miss(impl):
    timestamps = grid(11, 0.01)
    timestamps[5] += 0.005                 # step 5 arrives 5 ms late, step 6 is 5 ms early
    stats = impl.analyse_loop(timestamps, 100.0)
    assert stats.misses == 1, "only the long period is a miss; the short one is catching up"
    assert stats.max_ms == pytest.approx(15.0)
    assert stats.min_ms == pytest.approx(5.0)
    assert stats.worst_jitter_ms == pytest.approx(5.0)
    assert stats.miss_fraction == pytest.approx(0.1)


def test_tolerance_is_applied_strictly(impl):
    timestamps = [0.0, 0.011]              # exactly 10 % late with tolerance 0.10 -> not a miss
    assert impl.analyse_loop(timestamps, 100.0, tolerance=0.10).misses == 0
    timestamps = [0.0, 0.0111]
    assert impl.analyse_loop(timestamps, 100.0, tolerance=0.10).misses == 1


def test_too_few_timestamps(impl):
    empty = impl.analyse_loop([], 100.0)
    assert empty.count == 0 and empty.misses == 0 and empty.miss_fraction == 0.0
    assert impl.analyse_loop([1.0], 100.0).count == 0


def test_bad_rate_raises(impl):
    with pytest.raises(ValueError):
        impl.analyse_loop(grid(5, 0.01), 0.0)


# --- schedulability ---------------------------------------------------------------------------------
def karmel_tasks(impl):
    """The Pico's periodic work (labs/firmware/pico/main.py), with plausible worst-case times."""
    return [
        impl.Task("control", period_ms=10.0, wcet_ms=1.2),      # 100 Hz PID
        impl.Task("telemetry", period_ms=20.0, wcet_ms=1.0),    # 50 Hz
        impl.Task("sensors", period_ms=100.0, wcet_ms=30.0),    # 10 Hz, ultrasonic blocks 30 ms
    ]


def test_utilisation(impl):
    assert impl.total_utilisation(karmel_tasks(impl)) == pytest.approx(0.12 + 0.05 + 0.30)
    assert impl.total_utilisation([]) == 0.0


def test_rate_monotonic_bound_values(impl):
    assert impl.rate_monotonic_bound(1) == pytest.approx(1.0)
    assert impl.rate_monotonic_bound(2) == pytest.approx(0.8284, abs=1e-4)
    assert impl.rate_monotonic_bound(3) == pytest.approx(0.7798, abs=1e-4)
    assert impl.rate_monotonic_bound(1000) == pytest.approx(math.log(2), abs=1e-3)
    with pytest.raises(ValueError):
        impl.rate_monotonic_bound(0)


def test_karmel_is_guaranteed(impl):
    ok, reason = impl.is_schedulable(karmel_tasks(impl))
    assert ok
    assert reason == "guaranteed: U = 0.470 <= bound 0.780"


def test_between_the_bound_and_one(impl):
    tasks = [impl.Task("a", 10.0, 3.0), impl.Task("b", 20.0, 6.0), impl.Task("c", 50.0, 15.0)]
    ok, reason = impl.is_schedulable(tasks)
    assert ok
    assert reason == ("possible: U = 0.900 is above the bound 0.780 "
                      "- needs response-time analysis")


def test_overloaded(impl):
    tasks = [impl.Task("a", 10.0, 6.0), impl.Task("b", 20.0, 6.0), impl.Task("c", 50.0, 14.0)]
    ok, reason = impl.is_schedulable(tasks)
    assert not ok
    assert reason == "overloaded: U = 1.180 > 1.0"


def test_empty_task_list(impl):
    ok, reason = impl.is_schedulable([])
    assert ok
    assert reason == "guaranteed: U = 0.000 <= bound 1.000"


# --- watchdog ---------------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("new", "old", "expected"),
    [(100, 50, 50), (50, 100, -50), (200, 1073741000, 1024), (0, 0, 0)],
)
def test_ticks_diff(impl, new, old, expected):
    assert impl.ticks_diff(new, old) == expected


def test_the_watchdog_starts_tripped(impl):
    wd = impl.CommandWatchdog(timeout_ms=300, now_ms=0)
    assert wd.tripped is True
    assert wd.check(0) is False, "already tripped: do not report it again"
    assert wd.check(5000) is False


def test_it_trips_once_at_the_timeout(impl):
    wd = impl.CommandWatchdog(timeout_ms=300, now_ms=0)
    wd.feed(100)
    fired = [t for t in range(100, 1000, 10) if wd.check(t)]
    assert fired == [400], "exactly one report, at 300 ms after the feed"
    assert wd.tripped is True


def test_feeding_releases_it_again(impl):
    wd = impl.CommandWatchdog(timeout_ms=300, now_ms=0)
    wd.feed(0)
    assert wd.check(299) is False
    assert wd.check(300) is True
    wd.feed(310)
    assert wd.tripped is False
    assert wd.check(400) is False
    assert wd.check(610) is True


def test_elapsed_survives_the_wrap(impl):
    wd = impl.CommandWatchdog(timeout_ms=300, now_ms=0)
    wd.feed(1073741000)
    assert wd.elapsed_ms(200) == 1024
    assert wd.check(1073741200) is False    # 200 ms elapsed
    assert wd.check(200) is True            # 1024 ms elapsed, across the wrap


# --- stopping distance ------------------------------------------------------------------------------
def test_stopping_distance_karmel(impl):
    # 0.5 m/s, 300 ms watchdog, 10 ms control step, braking at 2 m/s^2
    distance = impl.stopping_distance_m(0.5, 300.0, 10.0, 2.0)
    assert distance == pytest.approx(0.5 * 0.31 + 0.25 / 4.0)
    assert distance == pytest.approx(0.2175, abs=1e-4)


def test_halving_the_timeout_shortens_the_coast(impl):
    slow = impl.stopping_distance_m(0.5, 300.0, 10.0, 2.0)
    fast = impl.stopping_distance_m(0.5, 150.0, 10.0, 2.0)
    assert slow - fast == pytest.approx(0.5 * 0.150)


def test_bad_deceleration_raises(impl):
    with pytest.raises(ValueError):
        impl.stopping_distance_m(0.5, 300.0, 10.0, 0.0)
