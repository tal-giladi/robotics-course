"""07.01 — Sensor fundamentals: characterize a distance sensor from a static test.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 07.01``.
You may use the standard library and numpy.

The data: ``range_samples.csv`` next to this file — a static test of a ToF-like sensor at six
tape-measured distances, 150 readings each. Columns ``true_m,sensor,measured_m``; an EMPTY
``measured_m`` is a dropout (the sensor gave no reading).

Vocabulary:
    dropout   no reading at all
    outlier   a reading far from the others (robust test below), caused by a different mechanism
    inlier    a valid reading that is not an outlier; mean, sigma and bias use inliers only
    bias      mean of inliers - true distance
    sigma     sample standard deviation of the inliers (divide by n - 1)
"""

from __future__ import annotations

import csv  # noqa: F401  (you will need these)
import math  # noqa: F401
from dataclasses import dataclass
from pathlib import Path

import numpy as np  # noqa: F401

MAD_TO_SIGMA = 1.4826  # for Gaussian noise, sigma = 1.4826 * MAD


@dataclass(frozen=True)
class RangeStats:
    """Summary of the readings at one true distance (meters). Already complete."""

    true_m: float
    n_total: int  # all readings, dropouts included
    n_valid: int  # readings that are not dropouts
    n_outliers: int  # valid readings flagged by is_outlier
    mean_m: float  # mean of inliers
    std_m: float  # sample standard deviation of inliers (ddof=1); 0.0 if only one inlier
    bias_m: float  # mean_m - true_m

    @property
    def dropout_rate(self) -> float:
        return (self.n_total - self.n_valid) / self.n_total if self.n_total else 0.0

    @property
    def outlier_rate(self) -> float:
        return self.n_outliers / self.n_valid if self.n_valid else 0.0


def load_samples(path: str | Path, sensor: str | None = None) -> dict[float, list[float | None]]:
    """Read a bench CSV into ``{true_m: [reading or None, ...]}``, keeping file order.

    Only rows whose ``sensor`` column equals ``sensor`` are kept (all rows if ``sensor`` is None).
    An empty ``measured_m`` becomes ``None``.
    """
    # TODO(student): csv.DictReader; float(row["true_m"]) as the key.
    raise NotImplementedError("load_samples")


def is_outlier(values: list[float], k: float = 3.5) -> list[bool]:
    """Flag outliers with the modified z-score: |x - median| / (1.4826 * MAD) > k.

    MAD = median(|x - median(x)|). If the MAD is 0 (more than half the values identical), flag
    nothing. An empty list gives an empty list.
    Why not "more than 3 sigma from the mean"? Because the outliers inflate the mean and sigma
    you would be testing them against (see lesson 07.01, Level 3).
    """
    # TODO(student): median, absolute deviations, MAD, the ratio, compare with k.
    raise NotImplementedError("is_outlier")


def summarize(samples: list[float | None], true_m: float, k: float = 3.5) -> RangeStats:
    """Characterize the readings of a target at ``true_m``.

    1. valid = readings that are not None (and not NaN)
    2. if there are no valid readings: counts as usual, mean/std/bias = math.nan
    3. flags = is_outlier(valid, k); inliers = valid readings not flagged
    4. mean, sample std (ddof=1) and bias from the inliers
    """
    # TODO(student)
    raise NotImplementedError("summarize")


def fit_calibration_line(true_m: list[float], mean_measured_m: list[float]) -> tuple[float, float]:
    """Least-squares line ``measured = gain * true + offset``; return ``(gain, offset_m)``.

    Raise ValueError if fewer than two DIFFERENT true distances are given (a line needs two points).
    Hint: np.polyfit(x, y, 1) or np.linalg.lstsq (lesson FM.19).
    """
    # TODO(student)
    raise NotImplementedError("fit_calibration_line")


def correct_reading(measured_m: float, gain: float, offset_m: float) -> float:
    """Invert the calibration line: the best estimate of the true distance for a raw reading."""
    # TODO(student)
    raise NotImplementedError("correct_reading")
