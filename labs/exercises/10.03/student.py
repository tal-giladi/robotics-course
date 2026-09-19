"""10.03 — The Bayes filter: predict and update on a 1D grid (a histogram filter).

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 10.03``.
Only numpy is needed.

The world: a corridor cut into ``n`` cells, numbered 0 .. n-1 from left to right. The robot's
belief is a numpy array ``belief`` of length ``n`` with ``belief[i] = P(robot is in cell i)``;
it is non-negative and sums to 1. Some cells have a door on the left wall; the robot has a door
detector that is right most of the time.
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

Belief = NDArray[np.floating]


def normalize(belief: Belief) -> Belief:
    """Return a NEW array that is ``belief`` scaled to sum to 1.

    Raise ``ValueError`` if the sum is zero, negative or not finite: that means every hypothesis
    was ruled out, which is a bug (a likelihood of exactly 0 somewhere) — never divide by zero.
    """
    # TODO(student): convert to a float array, check the sum, divide.
    raise NotImplementedError("normalize")


def predict(belief: Belief, move: int, kernel: Sequence[float], wrap: bool = False) -> Belief:
    """Motion update: where can the robot be after it tried to move ``move`` cells?

    The motion is noisy. ``kernel`` has an odd length and ``kernel[i]`` is the probability that the
    robot really moved ``move + i - len(kernel) // 2`` cells. Example: ``move=2``,
    ``kernel=[0.1, 0.8, 0.1]`` -> it moved 1 cell (10%), 2 cells (80%) or 3 cells (10%).

    Law of total probability: for every cell i and every possible real move, carry
    ``kernel[..] * belief[i]`` of probability to the cell the robot lands in.

    * ``wrap=True``: a circular corridor; leaving at one end re-enters at the other
      (``np.roll`` does exactly this).
    * ``wrap=False``: the ends are walls; a robot that would go past cell n-1 stays in cell n-1
      (and past cell 0 stays in cell 0). Probability is never lost: the result still sums to 1.

    Do not modify ``belief`` in place.
    """
    # TODO(student): loop over the kernel entries; for each real shift, move the whole belief.
    #   wrap=True : out += p * np.roll(belief, shift)
    #   wrap=False: target cells = clip(arange(n) + shift, 0, n - 1); np.add.at(out, targets, p * belief)
    raise NotImplementedError("predict")


def door_likelihood(doors: NDArray[np.bool_], z_door: bool, p_hit: float, p_false: float) -> Belief:
    """Measurement model: ``P(z | robot in cell i)`` for every cell i.

    ``doors[i]`` is True if cell i has a door. The detector says "door" with probability ``p_hit``
    in a door cell and with probability ``p_false`` in a wall cell. ``z_door`` is what it said.
    The result does NOT need to sum to 1 (it is a likelihood over cells, not a distribution).
    """
    # TODO(student): np.where(doors, ..., ...), and remember P(says "no door") = 1 - P(says "door").
    raise NotImplementedError("door_likelihood")


def update(belief: Belief, likelihood: Belief) -> Belief:
    """Measurement update (Bayes' rule): posterior = normalize(likelihood * prior)."""
    # TODO(student): one line, using normalize().
    raise NotImplementedError("update")


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
    """The Bayes filter loop. For each step k: predict with ``moves[k]``, then update with
    ``readings[k]`` (``None`` means the sensor gave no reading: skip the update).

    ``prior`` may be unnormalized (e.g. ``np.ones(n)`` for "I have no idea"): normalize it first.
    Return the list of beliefs after each step (same length as ``moves``). Raise ``ValueError``
    if ``moves`` and ``readings`` have different lengths.
    """
    # TODO(student): the whole filter is ~8 lines built from the functions above.
    raise NotImplementedError("run_filter")
