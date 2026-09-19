"""Regenerate ``range_samples.csv`` for exercise 07.01 (you don't need to run this).

A made-up ToF-like distance sensor, static test at six tape-measured distances, 150 readings each:

    measured = 1.025 * true + 0.015 m + noise,   noise sigma = 3 mm + 2 mm per meter
    3 % outliers (uniform 0.05-4 m: multipath, a foot walking past), 2 % dropouts (empty cell)

    python labs/exercises/07.01/make_samples.py
"""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

GAIN = 1.025
OFFSET_M = 0.015
DISTANCES_M = (0.2, 0.5, 1.0, 1.5, 2.0, 3.0)
SAMPLES = 150
OUTLIER_PROB = 0.03
DROPOUT_PROB = 0.02


def sigma_m(true_m: float) -> float:
    return 0.003 + 0.002 * true_m


def main() -> None:
    rng = np.random.default_rng(701)
    path = Path(__file__).with_name("range_samples.csv")
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["true_m", "sensor", "measured_m"])
        for d in DISTANCES_M:
            for _ in range(SAMPLES):
                u = rng.random()
                if u < DROPOUT_PROB:
                    w.writerow([f"{d:.3f}", "tof", ""])
                    continue
                if u < DROPOUT_PROB + OUTLIER_PROB:
                    value = rng.uniform(0.05, 4.0)
                else:
                    value = GAIN * d + OFFSET_M + rng.normal(0.0, sigma_m(d))
                w.writerow([f"{d:.3f}", "tof", f"{value:.4f}"])
    print(f"wrote {path}")


if __name__ == "__main__":
    main()
