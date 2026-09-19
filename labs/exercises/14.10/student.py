"""14.10 — The gate, the statistics and the correction.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 14.10``.
Only the standard library and numpy are needed.

Three jobs, in the order a real command meets them:

1. **The gate.** A workspace box that refuses a target you never want to visit, *before* anything
   is solved or executed — the only check in the pipeline that knows about the world.
2. **The statistics.** Accuracy and repeatability, computed separately, because they have
   different causes and different fixes.
3. **The correction.** Fit the systematic part of the error out, and validate the fit on points it
   was not fitted to.

Units: metres throughout.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- TODO(student) -------------------------------------------------------------------------------
@dataclass(frozen=True)
class WorkspaceBox:
    """The region of space you are willing to send the gripper into.

    Each bound is (low, high) in metres, in the arm's base frame. Reject a box whose low bound is
    not below its high bound with ``ValueError`` — a box that cannot contain anything is a
    configuration bug that would otherwise look like "the arm refuses everything".
    """

    x: tuple[float, float] = (0.10, 0.34)
    y: tuple[float, float] = (-0.22, 0.22)
    z: tuple[float, float] = (0.02, 0.32)

    def __post_init__(self) -> None:
        raise NotImplementedError  # TODO(student)

    def violations(self, point: ArrayLike) -> list[str]:
        """One human-readable line per axis that is outside, naming the axis and both numbers.

        Empty list = the point is inside. A refusal that does not say WHICH face it hit sends the
        reader back to the code, so name the axis, the value and the bound.
        """
        raise NotImplementedError  # TODO(student)

    def contains(self, point: ArrayLike) -> bool:
        raise NotImplementedError  # TODO(student)

    def clearance(self, point: ArrayLike) -> float:
        """Distance to the nearest face: positive inside, negative outside.

        Negative values are the depth of the worst violation. Useful for "how close am I to the
        edge of what I allow?" without a second implementation of the bounds.
        """
        raise NotImplementedError  # TODO(student)


def accuracy_and_repeatability(measured: ArrayLike, commanded: ArrayLike) -> tuple[float, float]:
    """(accuracy, repeatability) [m] for repeated runs at ONE commanded target.

        accuracy      = |mean(measured) - commanded|      how far the average lands from the truth
        repeatability = max |measured_i - mean(measured)| how much the runs scatter about it

    ``measured`` is (N, 3). A single run has a repeatability of 0.0 — which is exactly why one run
    tells you nothing. Raise ``ValueError`` on an empty set of measurements.
    """
    raise NotImplementedError  # TODO(student)


def fit_offset(commanded: ArrayLike, measured: ArrayLike) -> Array:
    """The constant (3,) offset that best corrects measured -> commanded, by least squares.

    For a constant offset the least-squares answer is just the mean of (commanded - measured).
    """
    raise NotImplementedError  # TODO(student)


def fit_affine(commanded: ArrayLike, measured: ArrayLike) -> tuple[Array, Array]:
    """The (A, b) of ``commanded ~= A @ measured + b``, by least squares.

    12 parameters, so you need at least 4 points that are not coplanar — raise ``ValueError``
    with fewer than 4. Returns A (3, 3) and b (3,).

    Hint: solve it as one linear system by appending a column of ones to ``measured``.
    """
    raise NotImplementedError  # TODO(student)


def apply_affine(A: ArrayLike, b: ArrayLike, points: ArrayLike) -> Array:
    """Apply a fitted correction to one point (3,) or many (N, 3), keeping the input's shape."""
    raise NotImplementedError  # TODO(student)


def residuals(commanded: ArrayLike, corrected: ArrayLike) -> Array:
    """Per-point error magnitudes |corrected_i - commanded_i| [m], shape (N,)."""
    raise NotImplementedError  # TODO(student)


def holdout_validate(commanded: ArrayLike, measured: ArrayLike, train_index: Sequence[int],
                     test_index: Sequence[int]) -> tuple[float, float]:
    """Fit an affine correction on the training points, score it on the test points.

    Returns (mean residual on train, mean residual on test), both in metres. A test error much
    worse than the train error means the fit is describing noise, not the arm — which is the only
    way to find that out, and the reason this function exists at all.

    Raise ``ValueError`` if the two index sets overlap: validating on a point you fitted to is
    not validation.
    """
    raise NotImplementedError  # TODO(student)
