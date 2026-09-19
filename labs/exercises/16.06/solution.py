"""Reference solution for 16.06 — evaluating an ML component inside a robot.

Same public names and signatures as ``student.py``.

Six small tools, each answering one question you cannot answer with mAP:

* how sure is the robot allowed to be?        reliability_table, expected_calibration_error
* how do we make a score mean what it says?   fit_temperature, apply_temperature
* how good is the robot, really?              wilson_interval, summarise_episodes
* is today still like the training data?      psi
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

# chi-square-free rules of thumb from the lesson.
PSI_STABLE = 0.10
PSI_SHIFTED = 0.25


# --- given ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Bin:
    """One row of a reliability table."""

    lo: float           # bin lower edge
    hi: float           # bin upper edge
    n: int              # how many detections fell in it
    confidence: float   # mean reported score in the bin
    accuracy: float     # fraction of those that were actually correct


@dataclass(frozen=True)
class Episode:
    """One closed-loop trial."""

    outcome: str        # "success", "wrong_place", "timeout", "safety_stop", ...
    seconds: float
    condition: str = "default"


def logit(p: NDArray[np.float64] | float) -> NDArray[np.float64]:
    """log(p / (1 - p)), clipped so 0 and 1 do not produce infinities."""
    p = np.clip(np.asarray(p, dtype=float), 1e-6, 1 - 1e-6)
    return np.log(p / (1 - p))


def sigmoid(x: NDArray[np.float64] | float) -> NDArray[np.float64]:
    return 1.0 / (1.0 + np.exp(-np.asarray(x, dtype=float)))


# --- implement -----------------------------------------------------------------------------------
def reliability_table(scores: NDArray[np.float64], correct: NDArray[np.float64],
                      bins: int = 10) -> list[Bin]:
    """Group detections into equal-width score bins and compare confidence with accuracy."""
    scores = np.asarray(scores, dtype=float)
    correct = np.asarray(correct, dtype=float)
    edges = np.linspace(0.0, 1.0, bins + 1)
    out: list[Bin] = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        inside = (scores >= lo) & (scores < hi if hi < 1.0 else scores <= hi)
        if not inside.any():
            continue
        out.append(Bin(float(lo), float(hi), int(inside.sum()),
                       float(scores[inside].mean()), float(correct[inside].mean())))
    return out


def expected_calibration_error(scores: NDArray[np.float64], correct: NDArray[np.float64],
                               bins: int = 10) -> float:
    """ECE = sum over bins of (n_b / N) * |accuracy_b - confidence_b|."""
    table = reliability_table(scores, correct, bins)
    total = sum(b.n for b in table)
    if total == 0:
        return 0.0
    return float(sum(b.n / total * abs(b.accuracy - b.confidence) for b in table))


def fit_temperature(scores: NDArray[np.float64], correct: NDArray[np.float64],
                    grid: NDArray[np.float64] | None = None) -> float:
    """Temperature scaling: the single T minimising negative log-likelihood on THIS set."""
    grid = np.linspace(0.25, 5.0, 96) if grid is None else np.asarray(grid, dtype=float)
    z = logit(scores)
    y = np.asarray(correct, dtype=float)

    def nll(t: float) -> float:
        p = sigmoid(z / t)
        return float(-np.mean(y * np.log(p + 1e-9) + (1 - y) * np.log(1 - p + 1e-9)))

    return float(min(grid, key=nll))


def apply_temperature(scores: NDArray[np.float64], temperature: float) -> NDArray[np.float64]:
    """Rescale scores with a fitted temperature. Monotone, so it cannot change AP."""
    return sigmoid(logit(scores) / float(temperature))


def wilson_interval(successes: int, trials: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval for a binomial proportion (95 % by default)."""
    if trials <= 0:
        return 0.0, 1.0
    p = successes / trials
    denom = 1.0 + z * z / trials
    centre = (p + z * z / (2 * trials)) / denom
    half = z * math.sqrt(p * (1 - p) / trials + z * z / (4 * trials * trials)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def psi(reference: NDArray[np.float64], current: NDArray[np.float64], bins: int = 10) -> float:
    """Population stability index between a reference and a current sample."""
    reference = np.asarray(reference, dtype=float)
    current = np.asarray(current, dtype=float)
    inner = np.quantile(reference, np.linspace(0.0, 1.0, bins + 1)[1:-1])
    r = np.bincount(np.searchsorted(inner, reference), minlength=bins) / len(reference) + 1e-3
    c = np.bincount(np.searchsorted(inner, current), minlength=bins) / len(current) + 1e-3
    return float(np.sum((c - r) * np.log(c / r)))


def summarise_episodes(episodes: list[Episode], success: str = "success"
                       ) -> dict[str, dict[str, float | int | tuple[float, float]]]:
    """Per-condition summary of a closed-loop trial."""
    out: dict[str, dict[str, float | int | tuple[float, float]]] = {}
    for condition in dict.fromkeys(e.condition for e in episodes):
        group = [e for e in episodes if e.condition == condition]
        wins = [e for e in group if e.outcome == success]
        outcomes: dict[str, int] = {}
        for e in group:
            outcomes[e.outcome] = outcomes.get(e.outcome, 0) + 1
        out[condition] = {
            "n": len(group),
            "successes": len(wins),
            "rate": len(wins) / len(group),
            "ci": wilson_interval(len(wins), len(group)),
            "median_time": float(np.median([e.seconds for e in wins])) if wins else float("nan"),
            "outcomes": outcomes,
        }
    return out
