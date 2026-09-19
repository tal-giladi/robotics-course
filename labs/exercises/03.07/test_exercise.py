"""Checker for 03.07 — parse and reduce a telemetry log.

Run: ``python course.py check 03.07`` (or ``--solution`` to see the reference pass).

Small hand-computed cases first, then the committed recording ``run_telemetry.csv`` — 7 s of
simulated karmel that contains, on purpose, two watchdog episodes, a 0.22 s telemetry gap that
swallowed exactly 10 samples, and a stretch where the front sensor had no reading at all.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from robotlab.config import load_config
from robotlab.hal import FLAG_LOW_BATTERY, FLAG_VELOCITY_MODE, FLAG_WATCHDOG

HERE = Path(__file__).resolve().parent
RECORDING = HERE / "run_telemetry.csv"
CFG = load_config()
MPT = CFG.drive.meters_per_tick  # 2*pi*0.045 / 2464 = 0.1147 mm

HEADER = ("host_time_s,t_s,left_ticks,right_ticks,left_rad_s,right_rad_s,"
          "battery_v,range_m,flags,left_cmd,right_cmd\n")


def write_csv(tmp_path: Path, body: str, header: str = HEADER) -> Path:
    path = tmp_path / "telemetry.csv"
    path.write_text(header + body, encoding="utf-8")
    return path


def make(impl, t: float, left: int = 0, right: int = 0, flags: int = 0,
         battery: float | None = 12.0, range_m: float | None = None,
         left_rad_s: float = 0.0, right_rad_s: float = 0.0):
    """A sample built directly, for tests that are about summarize/flag_episodes only."""
    return impl.TelemetrySample(t, t, left, right, left_rad_s, right_rad_s,
                                battery, range_m, flags, None, None)


# --- read_telemetry_csv -----------------------------------------------------------------------
def test_reads_rows_into_samples(tmp_path: Path, impl) -> None:
    path = write_csv(tmp_path, "0.0200,0.020,12,-13,1.500,-1.500,12.240,0.850,8,8.0,-8.0\n")
    (sample,) = impl.read_telemetry_csv(path)
    assert (sample.host_time_s, sample.t_s) == (0.02, 0.02)
    assert (sample.left_ticks, sample.right_ticks) == (12, -13)
    assert isinstance(sample.left_ticks, int), "ticks are counts, not floats"
    assert (sample.left_rad_s, sample.right_rad_s) == (1.5, -1.5)
    assert (sample.battery_v, sample.range_m, sample.flags) == (12.24, 0.85, 8)
    assert (sample.left_cmd, sample.right_cmd) == (8.0, -8.0)


def test_empty_cells_become_none_not_zero(tmp_path: Path, impl) -> None:
    path = write_csv(tmp_path, "0.0200,0.020,0,0,0.000,0.000,,,1,,\n")
    (sample,) = impl.read_telemetry_csv(path)
    assert sample.battery_v is None, "an empty cell means 'no reading', not 0.0"
    assert sample.range_m is None and sample.left_cmd is None and sample.right_cmd is None
    assert sample.flags == 1


def test_a_file_without_the_telemetry_columns_is_rejected(tmp_path: Path, impl) -> None:
    path = write_csv(tmp_path, "1,2\n", header="time,value\n")
    with pytest.raises(ValueError) as excinfo:
        impl.read_telemetry_csv(path)
    assert "t_s" in str(excinfo.value), "say which column is missing"


def test_a_truncated_last_row_is_skipped_but_the_run_survives(tmp_path: Path, impl) -> None:
    """A robot that loses power mid-write leaves half a line. Keep the other 20,000 samples."""
    body = ("0.0200,0.020,0,0,0.000,0.000,12.240,,0,,\n"
            "0.0400,0.040,63,63,8.000,8.000,12.239,,8,8.0,8.0\n"
            "0.0600,0.060,126,12")
    samples = impl.read_telemetry_csv(write_csv(tmp_path, body))
    assert len(samples) == 2
    with pytest.raises(ValueError):
        impl.read_telemetry_csv(write_csv(tmp_path, body), strict=True)


# --- summarize --------------------------------------------------------------------------------
def test_summarize_needs_at_least_two_samples(impl) -> None:
    with pytest.raises(ValueError):
        impl.summarize([], MPT)
    with pytest.raises(ValueError):
        impl.summarize([make(impl, 0.0)], MPT)


def test_summarize_hand_computed(impl) -> None:
    # 5 samples, 20 ms apart, both wheels +100 ticks per step -> 400 ticks of travel.
    samples = [make(impl, 0.02 * (i + 1), left=100 * i, right=100 * i, battery=12.0 - 0.1 * i,
                    range_m=None if i < 2 else 1.0, left_rad_s=2.0, right_rad_s=-3.0)
               for i in range(5)]
    s = impl.summarize(samples, 0.001)  # 1 mm per tick keeps the arithmetic obvious
    assert s.samples == 5
    assert s.duration_s == pytest.approx(0.08)      # 0.10 - 0.02
    assert s.rate_hz == pytest.approx(50.0)         # 4 intervals / 0.08 s
    assert s.median_interval_s == pytest.approx(0.02)
    assert (s.gaps, s.lost_samples) == (0, 0)
    assert s.max_gap_s == pytest.approx(0.02)
    assert s.distance_m == pytest.approx(0.400)     # 4 steps x 100 ticks x 1 mm
    assert s.max_wheel_speed_rad_s == pytest.approx(3.0)
    assert s.battery_start_v == pytest.approx(12.0) and s.battery_min_v == pytest.approx(11.6)
    assert s.range_valid_fraction == pytest.approx(0.6)  # 3 of 5


def test_distance_is_path_length_so_reversing_adds(impl) -> None:
    ticks = [0, 1000, 2000, 1000, 0]  # out and back: 2 m out, 2 m back at 1 mm/tick
    samples = [make(impl, 0.02 * (i + 1), left=n, right=n) for i, n in enumerate(ticks)]
    assert impl.summarize(samples, 0.001).distance_m == pytest.approx(4.0)


def test_a_turn_in_place_covers_no_distance(impl) -> None:
    samples = [make(impl, 0.02 * (i + 1), left=100 * i, right=-100 * i) for i in range(5)]
    assert impl.summarize(samples, 0.001).distance_m == pytest.approx(0.0, abs=1e-12)


def test_gaps_are_counted_and_converted_to_lost_samples(impl) -> None:
    # Intervals: 0.02 0.02 0.10 0.02 0.02 0.03 0.02 -> median 0.02, threshold 1.5 x 0.02 = 0.03.
    # The 0.10 s hole is a gap and swallowed 4 samples. The 0.03 s hiccup sits EXACTLY on the
    # threshold, and the rule is "longer than", so it is normal jitter, not a gap.
    times = [0.02, 0.04, 0.06, 0.16, 0.18, 0.20, 0.23, 0.25]
    samples = [make(impl, t) for t in times]
    s = impl.summarize(samples, 0.001)
    assert s.median_interval_s == pytest.approx(0.02)
    assert s.gaps == 1, "only the 0.10 s interval is longer than 1.5 x the median"
    assert s.max_gap_s == pytest.approx(0.10)
    assert s.lost_samples == 4, "round(0.10 / 0.02) - 1 = 4"


def test_battery_is_none_when_nothing_was_measured(impl) -> None:
    samples = [make(impl, 0.02 * (i + 1), battery=None) for i in range(3)]
    s = impl.summarize(samples, 0.001)
    assert s.battery_start_v is None and s.battery_min_v is None
    assert s.range_valid_fraction == 0.0


# --- flag_episodes ------------------------------------------------------------------------------
def test_no_flag_no_episodes(impl) -> None:
    assert impl.flag_episodes([], FLAG_WATCHDOG) == []
    assert impl.flag_episodes([make(impl, 0.02, flags=FLAG_VELOCITY_MODE)], FLAG_WATCHDOG) == []


def test_two_episodes_with_times_and_counts(impl) -> None:
    flags = [0, 1, 1, 1, 0, 0, 1, 0]
    samples = [make(impl, 0.02 * (i + 1), flags=f) for i, f in enumerate(flags)]
    a, b = impl.flag_episodes(samples, FLAG_WATCHDOG)
    assert (a.start_t, a.end_t, a.samples) == pytest.approx((0.04, 0.08, 3))
    assert a.duration_s == pytest.approx(0.04)
    assert (b.start_t, b.end_t, b.samples) == pytest.approx((0.14, 0.14, 1))
    assert b.duration_s == pytest.approx(0.0), "a one-sample episode has zero duration"


def test_an_episode_that_runs_to_the_end_of_the_log_is_reported(impl) -> None:
    samples = [make(impl, 0.02 * (i + 1), flags=f) for i, f in enumerate([0, 1, 1])]
    (episode,) = impl.flag_episodes(samples, FLAG_WATCHDOG)
    assert (episode.start_t, episode.end_t, episode.samples) == pytest.approx((0.04, 0.06, 2))


def test_identical_samples_do_not_confuse_the_counting(impl) -> None:
    """Two rows can be byte-identical (a stalled robot). Count by position, not by value."""
    samples = [make(impl, 0.0, flags=1), make(impl, 0.0, flags=1), make(impl, 0.0, flags=1)]
    (episode,) = impl.flag_episodes(samples, FLAG_WATCHDOG)
    assert episode.samples == 3


def test_flags_are_a_bitfield_not_a_number(impl) -> None:
    samples = [make(impl, 0.02, flags=FLAG_WATCHDOG | FLAG_LOW_BATTERY)]
    assert len(impl.flag_episodes(samples, FLAG_WATCHDOG)) == 1
    assert len(impl.flag_episodes(samples, FLAG_LOW_BATTERY)) == 1
    assert impl.flag_episodes(samples, FLAG_VELOCITY_MODE) == []


# --- the committed recording --------------------------------------------------------------------
def test_the_recording_parses(impl) -> None:
    samples = impl.read_telemetry_csv(RECORDING)
    assert len(samples) == 340
    assert samples[0].t_s == pytest.approx(0.02) and samples[-1].t_s == pytest.approx(7.0)
    assert samples[0].range_m is None, "the wall starts beyond the 4 m ToF range"


def test_the_recording_summarizes(impl) -> None:
    s = impl.summarize(impl.read_telemetry_csv(RECORDING), MPT)
    assert s.samples == 340
    assert s.duration_s == pytest.approx(6.98)
    assert s.median_interval_s == pytest.approx(0.02)
    assert s.rate_hz == pytest.approx(48.57, abs=0.05), "339 intervals in 6.98 s: below 50 Hz"
    assert (s.gaps, s.lost_samples) == (1, 10), "one 0.22 s hole swallowed exactly 10 samples"
    assert s.max_gap_s == pytest.approx(0.22)
    assert s.distance_m == pytest.approx(1.29, abs=0.02), "3.4 s at 8 rad/s x 0.045 m = 1.22 m + transients"
    assert s.battery_start_v == pytest.approx(12.24)
    assert s.battery_min_v is not None and s.battery_min_v <= s.battery_start_v
    assert s.range_valid_fraction == pytest.approx(0.594, abs=0.01)


def test_the_recording_has_two_watchdog_episodes(impl) -> None:
    samples = impl.read_telemetry_csv(RECORDING)
    first, second = impl.flag_episodes(samples, FLAG_WATCHDOG)
    assert first.start_t == pytest.approx(0.32), "300 ms after boot with nobody commanding"
    assert first.end_t == pytest.approx(0.60)
    assert second.start_t == pytest.approx(6.30), "300 ms after the last command at 6.00 s"
    assert second.samples == 36
    assert sum(e.samples for e in impl.flag_episodes(samples, FLAG_WATCHDOG)) == 51


def test_velocity_mode_covers_exactly_the_driving_part(impl) -> None:
    samples = impl.read_telemetry_csv(RECORDING)
    (episode,) = impl.flag_episodes(samples, FLAG_VELOCITY_MODE)
    assert episode.start_t == pytest.approx(0.62) and episode.end_t == pytest.approx(6.28)
    assert episode.duration_s == pytest.approx(5.66, abs=0.01)
