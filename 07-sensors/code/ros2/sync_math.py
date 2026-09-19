"""Timestamp arithmetic for sensor fusion, without ROS: pairing, jitter, age, rate.

Pure Python + numpy, runs anywhere (`py -m pytest 07-sensors/code/ros2`).

`pair_nearest` is a *simplified* stand-in for `message_filters.ApproximateTimeSynchronizer`: it
pairs each message of the slower stream with the nearest message of the faster one and drops pairs
further apart than `slop`. The real policy is smarter (it looks at whole sets and guarantees each
message is published in exactly one output set), but the failure modes you need to understand --
what "slop" buys you and what it costs -- are identical. `sync_demo.py` in this folder runs the
real one under ROS 2.

Lessons: 07.10 (sensor data done right), 07.11 (troubleshooting).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

__all__ = ["pair_nearest", "RateReport", "rate_report", "stamp_age_stats"]


def pair_nearest(a: list[float] | np.ndarray, b: list[float] | np.ndarray,
                 slop: float) -> list[tuple[int, int, float]]:
    """Pair every stamp in `a` with the nearest stamp in `b` no further away than `slop` seconds.

    Returns a list of `(index_in_a, index_in_b, dt)` with `dt = a[i] - b[j]`, each `b` index used
    at most once (the closest `a` wins). Both inputs must be sorted ascending.
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if a.size == 0 or b.size == 0:
        return []
    idx = np.searchsorted(b, a)
    candidates: list[tuple[float, int, int]] = []
    for i, ai in enumerate(a):
        for j in (idx[i] - 1, idx[i]):
            if 0 <= j < b.size:
                dt = ai - b[j]
                if abs(dt) <= slop:
                    candidates.append((abs(dt), i, j))
    candidates.sort()
    used_a: set[int] = set()
    used_b: set[int] = set()
    out: list[tuple[int, int, float]] = []
    for _, i, j in candidates:
        if i in used_a or j in used_b:
            continue
        used_a.add(i)
        used_b.add(j)
        out.append((i, j, float(a[i] - b[j])))
    out.sort()
    return out


@dataclass(frozen=True)
class RateReport:
    """What `ros2 topic hz` tells you, plus the part it does not: the worst gap."""

    n: int
    mean_hz: float
    jitter_ms: float
    max_gap_ms: float
    dropped_estimate: int

    def __str__(self) -> str:  # pragma: no cover - formatting only
        return (f"n={self.n} mean={self.mean_hz:.2f} Hz jitter={self.jitter_ms:.2f} ms "
                f"max_gap={self.max_gap_ms:.2f} ms dropped~{self.dropped_estimate}")


def rate_report(stamps: list[float] | np.ndarray, expected_hz: float | None = None) -> RateReport:
    """Mean rate, jitter (std of the inter-arrival time) and the worst gap, from a list of stamps.

    `dropped_estimate` counts, for each gap, how many extra periods it spans -- a gap of 3 periods
    means 2 messages went missing. A mean rate that looks right with a `dropped_estimate` above
    zero is the classic "the average hides the stall" case.
    """
    s = np.asarray(stamps, dtype=float)
    if s.size < 2:
        return RateReport(int(s.size), 0.0, 0.0, 0.0, 0)
    d = np.diff(s)
    mean_dt = float(np.mean(d))
    period = 1.0 / expected_hz if expected_hz else mean_dt
    dropped = int(np.sum(np.maximum(0, np.round(d / period).astype(int) - 1))) if period > 0 else 0
    return RateReport(
        n=int(s.size),
        mean_hz=1.0 / mean_dt if mean_dt > 0 else float("inf"),
        jitter_ms=float(np.std(d) * 1e3),
        max_gap_ms=float(np.max(d) * 1e3),
        dropped_estimate=dropped,
    )


def stamp_age_stats(stamps: list[float] | np.ndarray,
                    received: list[float] | np.ndarray) -> dict[str, float]:
    """Age = when you got it minus when it says it was measured, in milliseconds.

    This is the number that matters for control: a 40 ms old obstacle is 4 cm of stopping distance
    at 1 m/s. A *negative* age means the stamp is in the future -- two different clocks, or a node
    that has not been told `use_sim_time`.
    """
    age = (np.asarray(received, dtype=float) - np.asarray(stamps, dtype=float)) * 1e3
    return {
        "mean_ms": float(np.mean(age)),
        "median_ms": float(np.median(age)),
        "p95_ms": float(np.percentile(age, 95)),
        "max_ms": float(np.max(age)),
        "min_ms": float(np.min(age)),
    }
