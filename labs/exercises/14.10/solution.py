"""14.10 — reference solution. Read it after you have tried ``student.py`` yourself."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


@dataclass(frozen=True)
class WorkspaceBox:
    x: tuple[float, float] = (0.10, 0.34)
    y: tuple[float, float] = (-0.22, 0.22)
    z: tuple[float, float] = (0.02, 0.32)

    def __post_init__(self) -> None:
        for name in "xyz":
            low, high = getattr(self, name)
            if not low < high:
                raise ValueError(f"{name} bounds must satisfy low < high, got ({low}, {high})")

    def _bounds(self) -> tuple[tuple[float, float], ...]:
        return (self.x, self.y, self.z)

    def violations(self, point: ArrayLike) -> list[str]:
        p = np.asarray(point, dtype=float).reshape(-1)
        out: list[str] = []
        for name, value, (low, high) in zip("xyz", p, self._bounds()):
            if value < low:
                out.append(f"{name} = {value:.3f} m is below the box minimum {low:.3f} m")
            elif value > high:
                out.append(f"{name} = {value:.3f} m is above the box maximum {high:.3f} m")
        return out

    def contains(self, point: ArrayLike) -> bool:
        return not self.violations(point)

    def clearance(self, point: ArrayLike) -> float:
        p = np.asarray(point, dtype=float).reshape(-1)
        gaps = []
        for value, (low, high) in zip(p, self._bounds()):
            gaps.append(min(value - low, high - value))
        return float(min(gaps))


def accuracy_and_repeatability(measured: ArrayLike, commanded: ArrayLike) -> tuple[float, float]:
    points = np.atleast_2d(np.asarray(measured, dtype=float))
    if points.size == 0:
        raise ValueError("need at least one measurement")
    target = np.asarray(commanded, dtype=float).reshape(-1)
    mean = points.mean(axis=0)
    accuracy = float(np.linalg.norm(mean - target))
    repeatability = float(np.linalg.norm(points - mean, axis=1).max())
    return accuracy, repeatability


def fit_offset(commanded: ArrayLike, measured: ArrayLike) -> Array:
    commanded = np.atleast_2d(np.asarray(commanded, dtype=float))
    measured = np.atleast_2d(np.asarray(measured, dtype=float))
    if commanded.shape != measured.shape:
        raise ValueError(f"shape mismatch: {commanded.shape} vs {measured.shape}")
    return (commanded - measured).mean(axis=0)


def fit_affine(commanded: ArrayLike, measured: ArrayLike) -> tuple[Array, Array]:
    commanded = np.atleast_2d(np.asarray(commanded, dtype=float))
    measured = np.atleast_2d(np.asarray(measured, dtype=float))
    if commanded.shape != measured.shape:
        raise ValueError(f"shape mismatch: {commanded.shape} vs {measured.shape}")
    if len(measured) < 4:
        raise ValueError(f"an affine fit has 12 parameters; need at least 4 points, got {len(measured)}")
    design = np.hstack([measured, np.ones((len(measured), 1))])       # (N, 4)
    solution, *_ = np.linalg.lstsq(design, commanded, rcond=None)     # (4, 3)
    return solution[:3].T.copy(), solution[3].copy()


def apply_affine(A: ArrayLike, b: ArrayLike, points: ArrayLike) -> Array:
    A = np.asarray(A, dtype=float)
    b = np.asarray(b, dtype=float).reshape(-1)
    p = np.asarray(points, dtype=float)
    if p.ndim == 1:
        return A @ p + b
    return p @ A.T + b


def residuals(commanded: ArrayLike, corrected: ArrayLike) -> Array:
    commanded = np.atleast_2d(np.asarray(commanded, dtype=float))
    corrected = np.atleast_2d(np.asarray(corrected, dtype=float))
    return np.linalg.norm(corrected - commanded, axis=1)


def holdout_validate(commanded: ArrayLike, measured: ArrayLike, train_index: Sequence[int],
                     test_index: Sequence[int]) -> tuple[float, float]:
    train = list(train_index)
    test = list(test_index)
    if set(train) & set(test):
        raise ValueError("train and test indices overlap: that is not validation")
    commanded = np.atleast_2d(np.asarray(commanded, dtype=float))
    measured = np.atleast_2d(np.asarray(measured, dtype=float))
    A, b = fit_affine(commanded[train], measured[train])
    train_error = float(residuals(commanded[train], apply_affine(A, b, measured[train])).mean())
    test_error = float(residuals(commanded[test], apply_affine(A, b, measured[test])).mean())
    return train_error, test_error
