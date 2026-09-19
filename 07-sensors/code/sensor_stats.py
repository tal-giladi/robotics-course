"""07.01 — Characterize a distance sensor: bias, noise, outliers, dropouts and a calibration line.

    py 07-sensors/code/range_bench_sim.py --sensor tof --out tof_sim.csv     # no hardware: simulated bench
    py 07-sensors/code/sensor_stats.py tof_sim.csv --plot tof_sim.png         # analyse any bench CSV

    # real robot: log on the machine the Pico is plugged into, one distance at a time
    cd labs/firmware/pico
    mpremote mount . run ../../../07-sensors/code/pico/range_logger.py | python ../../../07-sensors/code/sensor_stats.py tag --true-m 0.50 --out bench.csv

CSV format, one row per reading:  ``true_m,sensor,measured_m``  (``measured_m`` empty = no reading).

Vocabulary (lesson 07.01):
    bias      mean(measured) - true             systematic error; calibration removes it
    sigma     sample standard deviation         random error; averaging N samples shrinks it by sqrt(N)
    outlier   far from the median (MAD test)    a different failure mechanism, not "big noise"
    dropout   no reading at all                 the sensor saw nothing it trusted
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import sys
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

MAD_TO_SIGMA = 1.4826  # for Gaussian data, sigma = 1.4826 * MAD
COLORS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300", "#4a3aa7", "#e34948")  # fixed order


@dataclass(frozen=True)
class Characterization:
    """Everything a static test at one known distance tells you (meters)."""

    true_m: float
    n_total: int
    n_valid: int  # readings that were not dropouts
    n_outliers: int
    mean_m: float  # of inliers
    median_m: float  # of valid readings
    std_m: float  # sample standard deviation of inliers (ddof=1)
    raw_std_m: float  # of all valid readings, outliers included: shows what outliers do to sigma
    bias_m: float  # mean_m - true_m
    rmse_m: float  # of inliers vs truth: sqrt(bias^2 + sigma^2) (approximately)
    sem_m: float  # standard error of the mean: std / sqrt(n_inliers)
    min_m: float
    max_m: float

    @property
    def dropout_rate(self) -> float:
        return 1.0 - self.n_valid / self.n_total if self.n_total else 0.0

    @property
    def outlier_rate(self) -> float:
        return self.n_outliers / self.n_valid if self.n_valid else 0.0


def as_array(samples: Iterable[float | None]) -> NDArray[np.float64]:
    """Readings as floats; ``None`` (no reading) becomes NaN."""
    return np.array([math.nan if s is None else float(s) for s in samples], dtype=float)


def mad_outliers(values: Sequence[float] | NDArray[np.float64], k: float = 3.5) -> NDArray[np.bool_]:
    """True where a value is an outlier by the modified z-score |x - median| / (1.4826 MAD) > k.

    The median and the MAD (median absolute deviation) ignore the outliers they are hunting, unlike
    mean and sigma, which the outliers themselves inflate. If more than half the readings are
    identical (MAD = 0, common with a 1 mm-resolution sensor), fall back to the mean absolute
    deviation; if that is also 0 there is no spread and nothing is an outlier.
    """
    x = np.asarray(values, dtype=float)
    if x.size == 0:
        return np.zeros(0, dtype=bool)
    med = np.median(x)
    dev = np.abs(x - med)
    scale = MAD_TO_SIGMA * np.median(dev)
    if scale == 0.0:
        scale = 1.2533 * float(np.mean(dev))  # mean absolute deviation -> sigma, for Gaussian data
    if scale == 0.0:
        return np.zeros(x.size, dtype=bool)
    return dev / scale > k


def characterize(samples: Iterable[float | None], true_m: float, k: float = 3.5) -> Characterization:
    """Summarize repeated readings of a target at a known distance ``true_m``."""
    x = as_array(samples)
    valid = x[np.isfinite(x)]
    if valid.size == 0:
        nan = math.nan
        return Characterization(true_m, x.size, 0, 0, nan, nan, nan, nan, nan, nan, nan, nan, nan)
    out = mad_outliers(valid, k)
    inliers = valid[~out]
    n_in = inliers.size
    mean = float(np.mean(inliers))
    std = float(np.std(inliers, ddof=1)) if n_in > 1 else 0.0
    raw_std = float(np.std(valid, ddof=1)) if valid.size > 1 else 0.0
    return Characterization(
        true_m=true_m,
        n_total=int(x.size),
        n_valid=int(valid.size),
        n_outliers=int(out.sum()),
        mean_m=mean,
        median_m=float(np.median(valid)),
        std_m=std,
        raw_std_m=raw_std,
        bias_m=mean - true_m,
        rmse_m=float(np.sqrt(np.mean((inliers - true_m) ** 2))),
        sem_m=std / math.sqrt(n_in) if n_in else math.nan,
        min_m=float(valid.min()),
        max_m=float(valid.max()),
    )


@dataclass(frozen=True)
class CalibrationLine:
    """measured = gain * true + offset, fitted by least squares over several distances."""

    gain: float
    offset_m: float
    residual_std_m: float  # how far the per-distance means sit from the line

    def correct(self, measured_m: float | NDArray[np.float64]) -> float | NDArray[np.float64]:
        """Invert the line: the best estimate of the true distance for a raw reading."""
        return (measured_m - self.offset_m) / self.gain


def fit_calibration(true_m: Sequence[float], measured_m: Sequence[float]) -> CalibrationLine:
    """Least-squares line through (true, measured) pairs (FM.19). Needs at least two distances."""
    t = np.asarray(true_m, dtype=float)
    m = np.asarray(measured_m, dtype=float)
    ok = np.isfinite(t) & np.isfinite(m)
    t, m = t[ok], m[ok]
    if np.unique(t).size < 2:
        raise ValueError("a calibration line needs readings at two or more different distances")
    gain, offset = np.polyfit(t, m, 1)
    residuals = m - (gain * t + offset)
    dof = max(t.size - 2, 1)
    return CalibrationLine(float(gain), float(offset), float(np.sqrt(np.sum(residuals**2) / dof)))


# --- CSV ---------------------------------------------------------------------------------------------
Groups = dict[tuple[str, float], NDArray[np.float64]]


def read_samples(path: str | Path) -> Groups:
    """``{(sensor, true_m): readings}`` from a bench CSV; missing readings are NaN."""
    rows: dict[tuple[str, float], list[float | None]] = {}
    with open(path, newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            key = (row["sensor"].strip(), round(float(row["true_m"]), 4))
            text = (row.get("measured_m") or "").strip()
            rows.setdefault(key, []).append(float(text) if text else None)
    return {key: as_array(values) for key, values in sorted(rows.items())}


def write_samples(path: str | Path, rows: Iterable[tuple[float, str, float | None]], append: bool = False) -> int:
    """Write ``(true_m, sensor, measured_m or None)`` rows; returns how many were written."""
    path = Path(path)
    new_file = not append or not path.exists() or path.stat().st_size == 0
    count = 0
    with open(path, "a" if append else "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if new_file:
            writer.writerow(["true_m", "sensor", "measured_m"])
        for true_m, sensor, measured in rows:
            writer.writerow([f"{true_m:.4f}", sensor, "" if measured is None else f"{measured:.4f}"])
            count += 1
    return count


LOGGER_LINE = re.compile(r"^\s*(us100|vl53l1x)\s*,\s*(-?\d+)\s*$")


def parse_logger_lines(lines: Iterable[str], true_m: float) -> list[tuple[float, str, float | None]]:
    """Turn ``pico/range_logger.py`` output (``sensor,mm`` with -1 = no reading) into CSV rows.

    Anything else mpremote prints (banners, blank lines) is ignored.
    """
    rows: list[tuple[float, str, float | None]] = []
    for line in lines:
        m = LOGGER_LINE.match(line)
        if m:
            mm = int(m.group(2))
            rows.append((true_m, m.group(1), None if mm < 0 else mm / 1000.0))
    return rows


# --- report and plot -------------------------------------------------------------------------------
def report(groups: Groups) -> str:
    """A text table per sensor plus the calibration line through the per-distance means."""
    lines = []
    for sensor in sorted({s for s, _ in groups}):
        lines.append(f"\n{sensor}")
        lines.append(
            "  true [m]    n  dropout  outlier   mean [m]  bias [mm]  sigma [mm]  sigma+outl [mm]  sem [mm]"
        )
        trues, means = [], []
        for (s, true_m), values in groups.items():
            if s != sensor:
                continue
            c = characterize(values, true_m)
            lines.append(
                f"  {true_m:8.3f} {c.n_total:4d}  {c.dropout_rate:6.1%}  {c.outlier_rate:6.1%}  {c.mean_m:9.4f}"
                f"  {c.bias_m * 1000:9.1f}  {c.std_m * 1000:10.1f}  {c.raw_std_m * 1000:15.1f}  {c.sem_m * 1000:8.2f}"
            )
            if math.isfinite(c.mean_m):
                trues.append(true_m)
                means.append(c.mean_m)
        if len(set(trues)) >= 2:
            line = fit_calibration(trues, means)
            lines.append(
                f"  calibration: measured = {line.gain:.4f} * true {line.offset_m * 1000:+.1f} mm"
                f"   (residual {line.residual_std_m * 1000:.1f} mm)"
            )
    return "\n".join(lines)


def plot(groups: Groups, path: str | Path) -> None:
    """Left: error histograms per distance. Right: mean error ± sigma vs distance, per sensor."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    sensors = sorted({s for s, _ in groups})
    fig, axes = plt.subplots(len(sensors), 2, figsize=(11, 3.6 * len(sensors)), squeeze=False)
    for row, sensor in enumerate(sensors):
        keys = [k for k in groups if k[0] == sensor]
        ax_hist, ax_err = axes[row]
        errors_all = [groups[k][np.isfinite(groups[k])] - k[1] for k in keys]
        inlier_errors = [e[~mad_outliers(e)] for e in errors_all if e.size]
        finite = np.concatenate(inlier_errors) * 1000 if inlier_errors else np.zeros(1)
        bins = np.linspace(finite.min() - 1, finite.max() + 1, 50)
        off_scale = 0
        for i, (key, err) in enumerate(zip(keys, errors_all)):
            mm = err * 1000
            inside = (mm >= bins[0]) & (mm <= bins[-1])
            off_scale += int((~inside).sum())
            ax_hist.hist(mm[inside], bins=bins, histtype="step", linewidth=2, label=f"{key[1]:.2f} m",
                         color=COLORS[i % len(COLORS)])
        ax_hist.set_title(f"{sensor}: error histogram (measured − true)")
        ax_hist.set_xlabel(f"error [mm]  ({off_scale} outliers off the scale, not drawn)")
        ax_hist.set_ylabel("readings")
        ax_hist.legend(frameon=False, fontsize=8)

        stats = [characterize(groups[k], k[1]) for k in keys]
        trues = np.array([c.true_m for c in stats])
        ax_err.errorbar(trues, [c.bias_m * 1000 for c in stats], yerr=[c.std_m * 1000 for c in stats],
                        fmt="o", color=COLORS[0], capsize=4, markersize=6, linewidth=2, label="bias ± σ (inliers)")
        ax_err.axhline(0.0, color="#52514e", linewidth=1)
        ax_err.set_title(f"{sensor}: bias and noise vs distance")
        ax_err.set_xlabel("true distance [m]")
        ax_err.set_ylabel("error [mm]")
        ax_err.legend(frameon=False, fontsize=8)
        for ax in (ax_hist, ax_err):
            ax.grid(True, color="#e4e3df", linewidth=0.8)
            ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


def main(argv: Sequence[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv[:1] == ["tag"]:
        ap = argparse.ArgumentParser(prog="sensor_stats.py tag", description="stdin logger lines -> bench CSV")
        ap.add_argument("--true-m", type=float, required=True, help="tape-measured distance for these readings")
        ap.add_argument("--out", required=True, help="CSV to append to")
        args = ap.parse_args(argv[1:])
        rows = parse_logger_lines(sys.stdin, args.true_m)
        n = write_samples(args.out, rows, append=True)
        print(f"appended {n} readings at {args.true_m:.3f} m to {args.out}", file=sys.stderr)
        return 0 if n else 1
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("csv", help="bench CSV: true_m,sensor,measured_m")
    ap.add_argument("--plot", help="save histograms and bias plot to this PNG")
    args = ap.parse_args(argv)
    groups = read_samples(args.csv)
    print(report(groups))
    if args.plot:
        plot(groups, args.plot)
        print(f"\nplot saved to {args.plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
