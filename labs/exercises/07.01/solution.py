"""Reference solution for 07.01 — Sensor fundamentals: characterize a distance sensor.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAD_TO_SIGMA = 1.4826


@dataclass(frozen=True)
class RangeStats:
    true_m: float
    n_total: int
    n_valid: int
    n_outliers: int
    mean_m: float
    std_m: float
    bias_m: float

    @property
    def dropout_rate(self) -> float:
        return (self.n_total - self.n_valid) / self.n_total if self.n_total else 0.0

    @property
    def outlier_rate(self) -> float:
        return self.n_outliers / self.n_valid if self.n_valid else 0.0


def load_samples(path: str | Path, sensor: str | None = None) -> dict[float, list[float | None]]:
    samples: dict[float, list[float | None]] = {}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if sensor is not None and row["sensor"] != sensor:
                continue
            text = row["measured_m"].strip()
            samples.setdefault(float(row["true_m"]), []).append(float(text) if text else None)
    return samples


def is_outlier(values: list[float], k: float = 3.5) -> list[bool]:
    if not values:
        return []
    x = np.asarray(values, dtype=float)
    med = np.median(x)
    dev = np.abs(x - med)
    mad = float(np.median(dev))
    if mad == 0.0:
        return [False] * len(values)
    return [bool(v) for v in dev / (MAD_TO_SIGMA * mad) > k]


def summarize(samples: list[float | None], true_m: float, k: float = 3.5) -> RangeStats:
    valid = [s for s in samples if s is not None and math.isfinite(s)]
    if not valid:
        return RangeStats(true_m, len(samples), 0, 0, math.nan, math.nan, math.nan)
    flags = is_outlier(valid, k)
    inliers = np.array([v for v, bad in zip(valid, flags) if not bad])
    mean = float(inliers.mean())
    std = float(inliers.std(ddof=1)) if inliers.size > 1 else 0.0
    return RangeStats(true_m, len(samples), len(valid), sum(flags), mean, std, mean - true_m)


def fit_calibration_line(true_m: list[float], mean_measured_m: list[float]) -> tuple[float, float]:
    t = np.asarray(true_m, dtype=float)
    m = np.asarray(mean_measured_m, dtype=float)
    if np.unique(t).size < 2:
        raise ValueError("need at least two different true distances")
    a = np.column_stack([t, np.ones_like(t)])
    (gain, offset), *_ = np.linalg.lstsq(a, m, rcond=None)
    return float(gain), float(offset)


def correct_reading(measured_m: float, gain: float, offset_m: float) -> float:
    return (measured_m - offset_m) / gain
