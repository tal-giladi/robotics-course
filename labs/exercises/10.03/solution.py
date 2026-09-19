"""Reference solution for 10.03 — The Bayes filter: predict and update on a 1D grid.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

Belief = NDArray[np.floating]


def normalize(belief: Belief) -> Belief:
    """Scale a non-negative array so it sums to 1 (a new array; the input is not modified)."""
    b = np.asarray(belief, dtype=float)
    total = b.sum()
    if not np.isfinite(total) or total <= 0.0:
        raise ValueError(f"cannot normalize a belief that sums to {total}")
    return b / total


def predict(belief: Belief, move: int, kernel: Sequence[float], wrap: bool = False) -> Belief:
    """Motion update (prediction): shift the belief by ``move`` cells and blur it with ``kernel``.

    ``kernel[i]`` is the probability that the robot really moved ``move + i - len(kernel) // 2``
    cells. Law of total probability: bel_bar[j] = sum_i P(j | i, move) * bel[i].
    """
    b = np.asarray(belief, dtype=float)
    k = np.asarray(kernel, dtype=float)
    n, half = b.size, k.size // 2
    out = np.zeros(n)
    for offset_index, p_offset in enumerate(k):
        shift = move + offset_index - half
        if wrap:
            out += p_offset * np.roll(b, shift)
        else:
            targets = np.clip(np.arange(n) + shift, 0, n - 1)  # the corridor ends are walls
            np.add.at(out, targets, p_offset * b)
    return out


def door_likelihood(doors: NDArray[np.bool_], z_door: bool, p_hit: float, p_false: float) -> Belief:
    """P(z | robot in cell i) for every cell: the measurement model of the door detector."""
    doors = np.asarray(doors, dtype=bool)
    p_says_door = np.where(doors, p_hit, p_false)
    return p_says_door if z_door else 1.0 - p_says_door


def update(belief: Belief, likelihood: Belief) -> Belief:
    """Measurement update (correction): posterior = normalize(likelihood * prior)."""
    return normalize(np.asarray(likelihood, dtype=float) * np.asarray(belief, dtype=float))


def run_filter(
    prior: Belief,
    doors: NDArray[np.bool_],
    moves: Sequence[int],
    readings: Sequence[bool | None],
    kernel: Sequence[float],
    p_hit: float,
    p_false: float,
    wrap: bool = False,
) -> list[Belief]:
    """Predict with each move, then update with the matching reading (``None`` = no reading)."""
    if len(moves) != len(readings):
        raise ValueError("moves and readings must have the same length")
    belief = normalize(prior)
    history = []
    for move, z in zip(moves, readings):
        belief = predict(belief, move, kernel, wrap)
        if z is not None:
            belief = update(belief, door_likelihood(doors, z, p_hit, p_false))
        history.append(belief)
    return history
