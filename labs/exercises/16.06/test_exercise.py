"""Checker for 16.06 — evaluating an ML component inside a robot.

Run: ``python course.py check 16.06`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import numpy as np
import pytest


def overconfident_scores(n: int, seed: int, overconfidence: float = 2.0):
    """A detector whose true chance of being right is p but which reports a sharpened score."""
    rng = np.random.default_rng(seed)
    p = np.clip(rng.beta(2.0, 1.2, n), 1e-4, 1 - 1e-4)
    correct = (rng.random(n) < p).astype(float)
    scores = 1 / (1 + np.exp(-np.log(p / (1 - p)) * overconfidence))
    return scores, correct


# --- reliability and ECE ------------------------------------------------------------------------
def test_reliability_bins_count_and_average_correctly(impl):
    scores = np.array([0.05, 0.15, 0.16, 0.95, 0.99, 1.0])
    correct = np.array([0.0, 1.0, 0.0, 1.0, 1.0, 1.0])
    table = impl.reliability_table(scores, correct, bins=10)
    assert [b.n for b in table] == [1, 2, 3], "empty bins are skipped; the last bin includes 1.0"
    assert table[1].confidence == pytest.approx(0.155)
    assert table[1].accuracy == pytest.approx(0.5)
    assert table[2].accuracy == pytest.approx(1.0)


def test_perfectly_calibrated_scores_have_zero_ece(impl):
    """Half the detections reported at 0.50 are right, and 95 % of those reported at 0.95."""
    scores = np.concatenate([np.full(100, 0.50), np.full(100, 0.95)])
    correct = np.concatenate([np.repeat([1.0, 0.0], 50), np.repeat([1.0, 0.0], [95, 5])])
    assert impl.expected_calibration_error(scores, correct) == pytest.approx(0.0, abs=0.01)


def test_overconfident_scores_have_a_large_ece(impl):
    scores = np.full(200, 0.95)
    correct = np.repeat([1.0, 0.0], [120, 80])            # only 60 % right, reported 95 %
    assert impl.expected_calibration_error(scores, correct) == pytest.approx(0.35, abs=0.01)


def test_ece_of_a_real_detector_is_a_few_percent(impl):
    scores, correct = overconfident_scores(4000, seed=1)
    ece = impl.expected_calibration_error(scores, correct)
    assert 0.05 < ece < 0.20, "an uncalibrated detector typically lands here"


# --- temperature scaling ------------------------------------------------------------------------
def test_temperature_greater_than_one_fixes_overconfidence(impl):
    val_s, val_c = overconfident_scores(4000, seed=1)
    test_s, test_c = overconfident_scores(4000, seed=2)
    t = impl.fit_temperature(val_s, val_c)
    assert t > 1.5, "over-confident scores need softening, so T > 1"
    before = impl.expected_calibration_error(test_s, test_c)
    after = impl.expected_calibration_error(impl.apply_temperature(test_s, t), test_c)
    assert after < before / 3, f"temperature scaling should cut ECE a lot ({before:.3f} -> {after:.3f})"


def test_temperature_one_changes_nothing(impl):
    scores = np.linspace(0.01, 0.99, 50)
    assert impl.apply_temperature(scores, 1.0) == pytest.approx(scores, abs=1e-6)


def test_temperature_scaling_preserves_the_ranking(impl):
    """This is why calibration cannot change AP."""
    scores = np.array([0.1, 0.3, 0.45, 0.8, 0.99])
    for t in (0.5, 1.7, 3.0):
        out = impl.apply_temperature(scores, t)
        assert np.all(np.diff(out) > 0), "a monotone transform must not reorder detections"


def test_the_threshold_that_means_ninety_percent(impl):
    """The lesson's punchline: 'act above 0.9' is not a 90 % guarantee."""
    val_s, val_c = overconfident_scores(4000, seed=1)
    test_s, test_c = overconfident_scores(4000, seed=2)
    t = impl.fit_temperature(val_s, val_c)
    keep = test_s >= 0.9
    assert 0.80 < test_c[keep].mean() < 0.92, "the raw 0.9 threshold promises more than it delivers"
    assert impl.apply_temperature(np.array([0.9]), t)[0] < 0.85, \
        "the same detections read lower once the score means a probability"


# --- Wilson interval ----------------------------------------------------------------------------
def test_wilson_interval_on_eighteen_of_twenty(impl):
    lo, hi = impl.wilson_interval(18, 20)
    assert (lo, hi) == pytest.approx((0.699, 0.972), abs=0.002), \
        "18/20 is not '90 % success' — the interval reaches down to 70 %"


def test_more_trials_narrow_the_interval(impl):
    narrow = impl.wilson_interval(89, 100)
    wide = impl.wilson_interval(9, 10)
    assert (wide[1] - wide[0]) > 3 * (narrow[1] - narrow[0])
    assert narrow == pytest.approx((0.814, 0.937), abs=0.002)


def test_zero_failures_still_allows_a_real_failure_rate(impl):
    """The rule of three: 0/50 clean runs is consistent with ~6 % failures."""
    lo, hi = impl.wilson_interval(50, 50)
    assert lo == pytest.approx(0.929, abs=0.002)
    assert 1 - lo == pytest.approx(3 / 50, abs=0.02)


def test_wilson_handles_the_degenerate_cases(impl):
    assert impl.wilson_interval(0, 0) == (0.0, 1.0)
    lo, hi = impl.wilson_interval(0, 20)
    assert lo == pytest.approx(0.0, abs=1e-9) and 0.10 < hi < 0.20


# --- PSI ----------------------------------------------------------------------------------------
def test_psi_of_a_sample_against_itself_is_about_zero(impl):
    rng = np.random.default_rng(0)
    ref = rng.beta(6, 2, 5000)
    assert impl.psi(ref, ref) == pytest.approx(0.0, abs=1e-6)


def test_psi_stays_small_for_another_quiet_day(impl):
    rng = np.random.default_rng(0)
    ref, today = rng.beta(6, 2, 5000), rng.beta(6, 2, 1500)
    assert impl.psi(ref, today) < 0.10, "a normal day must not alarm"


def test_psi_is_large_when_the_distribution_moves(impl):
    rng = np.random.default_rng(0)
    ref, dim_room = rng.beta(6, 2, 5000), rng.beta(4, 3, 1500)
    assert impl.psi(ref, dim_room) > 0.25, "a shifted day must alarm"


def test_psi_is_finite_when_a_bin_is_empty(impl):
    rng = np.random.default_rng(0)
    ref = rng.uniform(0, 1, 5000)
    assert math.isfinite(impl.psi(ref, rng.uniform(0.9, 1.0, 500)))


# --- episode summary ----------------------------------------------------------------------------
def episodes(impl):
    """20 kitchen episodes (18 successes) and 20 hallway ones (12 successes)."""
    out = [impl.Episode("success", 10.0 + i, "kitchen") for i in range(18)]
    out += [impl.Episode("wrong_place", 45.0, "kitchen"), impl.Episode("timeout", 45.0, "kitchen")]
    out += [impl.Episode("success", 20.0 + i, "hallway") for i in range(12)]
    out += [impl.Episode("wrong_place", 45.0, "hallway") for _ in range(6)]
    out += [impl.Episode("safety_stop", 12.0, "hallway") for _ in range(2)]
    return out


def test_summary_reports_each_condition_separately(impl):
    s = impl.summarise_episodes(episodes(impl))
    assert set(s) == {"kitchen", "hallway"}
    assert s["kitchen"]["n"] == 20 and s["kitchen"]["successes"] == 18
    assert s["kitchen"]["rate"] == pytest.approx(0.90)
    assert s["hallway"]["rate"] == pytest.approx(0.60)


def test_summary_carries_the_confidence_interval_and_outcome_counts(impl):
    s = impl.summarise_episodes(episodes(impl))
    assert s["kitchen"]["ci"] == pytest.approx((0.699, 0.972), abs=0.002)
    assert s["hallway"]["outcomes"] == {"success": 12, "wrong_place": 6, "safety_stop": 2}
    lo_k, _ = s["kitchen"]["ci"]
    _, hi_h = s["hallway"]["ci"]
    assert lo_k < hi_h, "20 episodes per room cannot separate 90 % from 60 % with certainty"


def test_median_time_uses_only_the_successful_episodes(impl):
    s = impl.summarise_episodes(episodes(impl))
    assert s["kitchen"]["median_time"] == pytest.approx(18.5)     # median of 10..27, not of the 45 s failures
    assert s["hallway"]["median_time"] == pytest.approx(25.5)


def test_a_condition_with_no_successes_reports_nan_not_a_crash(impl):
    s = impl.summarise_episodes([impl.Episode("timeout", 45.0, "dark")])
    assert math.isnan(s["dark"]["median_time"])
    assert s["dark"]["rate"] == 0.0
