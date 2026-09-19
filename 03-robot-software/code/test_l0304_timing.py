"""Tests for the module-03.04 scripts: py -m pytest 03-robot-software/code/test_l0304_timing.py

Timing *quality* depends on the machine, so these tests check the arithmetic exactly (fake clock,
synthetic timestamps) and only smoke-test the real-time scenarios.
"""

from __future__ import annotations

import pytest

import l0304_loop_lab as lab


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def test_grid_rate_stays_on_the_grid_and_skips_overruns():
    clock = Clock()
    rate = lab.GridRate(100.0, clock=clock)
    assert rate.delay() == pytest.approx(0.010)
    clock.now = 0.010 + 0.004  # woke 4 ms late
    assert rate.delay() == pytest.approx(0.006)  # next deadline is 0.020, not 0.024
    clock.now = 0.055  # a 35 ms overrun: deadlines 0.020, 0.030, 0.040, 0.050 passed
    assert rate.delay() == 0.0
    clock.now = 0.055
    assert rate.delay() == pytest.approx(0.005)  # back on the grid at 0.060


def test_summarize_counts_lost_ticks():
    period = 0.02
    perfect = [i * period for i in range(51)]
    result = lab.summarize("perfect", perfect, period)
    assert (result.samples, result.lost_ticks) == (50, 0)
    assert result.mean_ms == pytest.approx(20.0) and result.max_err_ms == pytest.approx(0.0, abs=1e-9)
    naive = [i * 0.025 for i in range(41)]  # a loop that really runs at 25 ms: 1.0 s, 40 ticks
    result = lab.summarize("naive", naive, period)
    assert result.lost_ticks == 10  # a 50 Hz loop would have run 50
    assert result.p99_err_ms == pytest.approx(5.0)


@pytest.mark.parametrize("name", list(lab.SCENARIOS))
def test_scenarios_run(name):
    wakes = lab.SCENARIOS[name](50.0, 0.3, 0.001)
    assert len(wakes) >= 3
    assert all(b > a for a, b in zip(wakes, wakes[1:]))


def test_main_prints_a_table(capsys):
    results = lab.main(["--rate", "50", "--seconds", "0.3", "--only", "drift_free", "naive"])
    out = capsys.readouterr().out
    assert "scenario" in out and "drift_free" in out and len(results) == 2


# --- timer-driven vs telemetry-driven loops ------------------------------------------------------------
def test_analyze_counts_duplicates_and_gaps():
    import l0304_sync_lab as sync

    seen = [0.00, 0.02, 0.02, 0.04, 0.10, 0.12, 0.12, 0.12]  # 2 duplicates, then a gap of 60 ms
    stats = sync.analyze("x", seen)
    assert (stats.iterations, stats.duplicates, stats.missed) == (8, 3, 2)


def test_telemetry_driven_loop_never_sees_a_sample_twice():
    import l0304_sync_lab as sync

    timer, telemetry = sync.main(["--fake", "--seconds", "1.0"])
    assert telemetry.duplicates == 0
    assert telemetry.iterations >= 20 and timer.iterations >= 20
