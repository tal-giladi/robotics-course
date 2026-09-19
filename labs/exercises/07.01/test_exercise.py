"""Checker for 07.01 — Sensor fundamentals: characterize a distance sensor from a static test.

Run: ``python course.py check 07.01`` (``--solution`` runs the reference).
The ``impl`` fixture (labs/exercises/conftest.py) is student.py or solution.py.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
DATA = HERE / "range_samples.csv"
# the hidden truth behind range_samples.csv (see make_samples.py)
GAIN, OFFSET_M = 1.025, 0.015
DISTANCES = (0.2, 0.5, 1.0, 1.5, 2.0, 3.0)


def sigma_m(true_m: float) -> float:
    return 0.003 + 0.002 * true_m


# --- load_samples -----------------------------------------------------------------------------------
def test_load_samples_shape(impl):
    samples = impl.load_samples(DATA)
    assert sorted(samples) == pytest.approx(list(DISTANCES))
    assert all(len(v) == 150 for v in samples.values()), "150 readings per distance, dropouts included"
    assert any(r is None for v in samples.values() for r in v), "empty measured_m cells must become None"
    assert impl.load_samples(DATA, sensor="us100") == {}


def test_load_samples_filters_sensor(impl, tmp_path):
    path = tmp_path / "two.csv"
    path.write_text("true_m,sensor,measured_m\n0.5,us100,0.51\n0.5,tof,\n0.5,tof,0.49\n1.0,us100,1.02\n",
                    encoding="utf-8")
    assert impl.load_samples(path, "tof") == {0.5: [None, 0.49]}
    assert impl.load_samples(path) == {0.5: [0.51, None, 0.49], 1.0: [1.02]}


# --- is_outlier -------------------------------------------------------------------------------------
def test_is_outlier_hand_computed(impl):
    # median 1.02; deviations 0.01, 0, 0.01, 0.01, 0, 1.98 -> MAD = 0.01 -> robust sigma 0.014826
    # 1.98 / 0.014826 = 133 > 3.5 ; 0.01 / 0.014826 = 0.67
    assert impl.is_outlier([1.01, 1.02, 1.03, 1.01, 1.02, 3.0]) == [False, False, False, False, False, True]


def test_is_outlier_threshold_k(impl):
    # median 0, MAD 1 -> robust sigma 1.4826; 5 / 1.4826 = 3.37 (inlier at k=3.5, outlier at k=3)
    values = [-1.0, 0.0, 1.0, -1.0, 1.0, 0.0, 5.0]
    assert not impl.is_outlier(values)[-1]
    assert impl.is_outlier(values, k=3.0)[-1]


def test_is_outlier_edge_cases(impl):
    assert impl.is_outlier([]) == []
    assert impl.is_outlier([0.5, 0.5, 0.5, 0.6]) == [False] * 4, "MAD = 0: flag nothing"


# --- summarize --------------------------------------------------------------------------------------
def test_summarize_hand_computed(impl):
    s = impl.summarize([1.01, 1.02, 1.03, 1.01, 1.02, 3.0, None, None], 1.0)
    assert (s.n_total, s.n_valid, s.n_outliers) == (8, 6, 1)
    assert s.mean_m == pytest.approx(1.018)
    assert s.bias_m == pytest.approx(0.018)
    # deviations from 1.018: -0.008, 0.002, 0.012, -0.008, 0.002 -> sum sq 0.00028 / 4
    assert s.std_m == pytest.approx(math.sqrt(0.00028 / 4))
    assert s.dropout_rate == pytest.approx(0.25)
    assert s.outlier_rate == pytest.approx(1 / 6)


def test_summarize_all_dropouts(impl):
    s = impl.summarize([None, None], 2.0)
    assert (s.n_total, s.n_valid) == (2, 0)
    assert math.isnan(s.mean_m) and math.isnan(s.bias_m)


@pytest.mark.parametrize("true_m", DISTANCES)
def test_summarize_provided_data(impl, true_m):
    s = impl.summarize(impl.load_samples(DATA)[true_m], true_m)
    expected_bias = (GAIN - 1.0) * true_m + OFFSET_M
    assert s.bias_m == pytest.approx(expected_bias, abs=0.0015), f"bias at {true_m} m should be ~{expected_bias * 1000:.1f} mm"
    assert s.std_m == pytest.approx(sigma_m(true_m), rel=0.25), "sigma of the inliers, outliers excluded"
    assert 0.0 <= s.dropout_rate <= 0.06
    assert 0.0 < s.outlier_rate <= 0.08, "about 3 % of the readings are outliers"


# --- calibration line -------------------------------------------------------------------------------
def test_fit_line_exact(impl):
    gain, offset = impl.fit_calibration_line([0.1, 1.0, 2.0], [0.113, 1.04, 2.07])
    assert (gain, offset) == pytest.approx((1.03, 0.01))
    with pytest.raises(ValueError):
        impl.fit_calibration_line([1.0, 1.0], [1.01, 1.02])


def test_correct_reading(impl):
    assert impl.correct_reading(1.04, 1.03, 0.01) == pytest.approx(1.0)


def test_end_to_end_calibration_removes_bias(impl):
    samples = impl.load_samples(DATA)
    stats = [impl.summarize(samples[d], d) for d in sorted(samples)]
    gain, offset = impl.fit_calibration_line([s.true_m for s in stats], [s.mean_m for s in stats])
    assert gain == pytest.approx(GAIN, abs=0.004)
    assert offset == pytest.approx(OFFSET_M, abs=0.003)
    for s in stats:
        corrected = impl.correct_reading(s.mean_m, gain, offset)
        assert abs(corrected - s.true_m) < 0.003, f"after calibration the mean at {s.true_m} m should be within 3 mm"
