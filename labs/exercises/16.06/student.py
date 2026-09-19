"""16.06 — evaluating an ML component inside a robot.

Fill in every ``TODO(student)``. Check your work with ``python course.py check 16.06``.
Only the standard library and numpy are needed.

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
    """Group detections into equal-width score bins and compare confidence with accuracy.

    Bins are [0, 0.1), [0.1, 0.2), ... [0.9, 1.0] — the last one includes its upper edge.
    Skip empty bins entirely (do not emit a Bin with n = 0). ``correct`` is 1.0 / 0.0 per detection.
    """
    # TODO(student): implement.
    raise NotImplementedError("reliability_table")


def expected_calibration_error(scores: NDArray[np.float64], correct: NDArray[np.float64],
                               bins: int = 10) -> float:
    """ECE = sum over bins of (n_b / N) * |accuracy_b - confidence_b|.

    0 means the scores are perfectly calibrated; a typical uncalibrated detector is 0.05-0.15.
    """
    # TODO(student): implement.
    raise NotImplementedError("expected_calibration_error")


def fit_temperature(scores: NDArray[np.float64], correct: NDArray[np.float64],
                    grid: NDArray[np.float64] | None = None) -> float:
    """Temperature scaling: the single T minimising negative log-likelihood on THIS set.

    calibrated = sigmoid(logit(score) / T). Search ``grid`` (default: 96 values from 0.25 to 5.0)
    and return the best T. Fit this on validation data, never on the set you report.
    """
    # TODO(student): implement.
    raise NotImplementedError("fit_temperature")


def apply_temperature(scores: NDArray[np.float64], temperature: float) -> NDArray[np.float64]:
    """Rescale scores with a fitted temperature. Monotone, so it cannot change AP."""
    # TODO(student): implement.
    raise NotImplementedError("apply_temperature")


def wilson_interval(successes: int, trials: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval for a binomial proportion (95 % by default).

        centre = (p + z^2 / 2n) / (1 + z^2 / n)
        half   = z * sqrt(p(1-p)/n + z^2/(4n^2)) / (1 + z^2 / n)

    Returns (low, high), clipped to [0, 1]. With trials = 0, return (0.0, 1.0): no information.
    """
    # TODO(student): implement.
    raise NotImplementedError("wilson_interval")


def psi(reference: NDArray[np.float64], current: NDArray[np.float64], bins: int = 10) -> float:
    """Population stability index between a reference and a current sample.

    Bin edges are the REFERENCE's quantiles (so the reference is uniform across bins by
    construction). PSI = sum over bins of (c_b - r_b) * ln(c_b / r_b) with r_b, c_b the fractions
    in each bin. Add 1e-3 to both fractions so an empty bin does not produce infinity.
    """
    # TODO(student): implement.
    raise NotImplementedError("psi")


def summarise_episodes(episodes: list[Episode], success: str = "success"
                       ) -> dict[str, dict[str, float | int | tuple[float, float]]]:
    """Per-condition summary of a closed-loop trial.

    For each distinct ``condition`` return a dict with:
        "n"              number of episodes
        "successes"      number of episodes whose outcome == ``success``
        "rate"           successes / n
        "ci"             wilson_interval(successes, n)
        "median_time"    median ``seconds`` of the SUCCESSFUL episodes (nan if there are none)
        "outcomes"       {outcome: count} for every outcome seen in that condition

    Report per condition, never only the mean: a 90 % average of 100 % and 60 % is two robots.
    """
    # TODO(student): implement.
    raise NotImplementedError("summarise_episodes")
