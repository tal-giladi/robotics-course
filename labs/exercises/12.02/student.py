"""12.02 — A* on an occupancy grid.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 12.02``.
Only the standard library (``heapq``, ``math``) and numpy are needed.

Conventions:
* ``blocked`` is a 2D numpy bool array indexed ``blocked[row, col]``; True = the robot may not be there
  (obstacles already grown by the robot radius — the configuration space).
* A cell is a tuple ``(row, col)``. Costs are in cells: a straight step costs 1, a diagonal step sqrt(2).
* Moves are 8-connected, but a diagonal step is NOT allowed when either of the two cells it squeezes
  between is blocked ("no corner cutting").
"""

from __future__ import annotations

import heapq  # noqa: F401  (you will want it)
import math
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

Cell = tuple[int, int]  # (row, col)
Heuristic = Callable[[Cell, Cell], float]
SQRT2 = math.sqrt(2.0)


@dataclass
class PlanResult:
    path: list[Cell] | None  # start ... goal (both included), or None if the goal is unreachable
    cost: float  # length of the path in cells; math.inf when there is no path
    expanded: int  # number of cells taken off the open list and closed


def octile_distance(a: Cell, b: Cell) -> float:
    """Length of the shortest 8-connected path between two cells on an EMPTY grid.

    Example: from (0, 0) to (3, 5) take 3 diagonal steps and 2 straight ones: 3 * sqrt(2) + 2 = 6.243.
    """
    # TODO(student): max(|dr|, |dc|) straight-equivalent steps, min(|dr|, |dc|) of them diagonal.
    raise NotImplementedError("octile_distance")


def neighbors(blocked: NDArray[np.bool_], cell: Cell) -> list[tuple[Cell, float]]:
    """Free 8-connected neighbours of ``cell`` as ``[(neighbour_cell, step_length), ...]``.

    Skip cells outside the grid, blocked cells, and diagonal steps that cut a blocked corner
    (moving from (r, c) to (r+dr, c+dc) needs (r, c+dc) AND (r+dr, c) free). Order does not matter.
    """
    # TODO(student)
    raise NotImplementedError("neighbors")


def astar(blocked: NDArray[np.bool_], start: Cell, goal: Cell, heuristic: Heuristic = octile_distance) -> PlanResult:
    """A* from ``start`` to ``goal``.

    * Blocked start or goal -> ``PlanResult(None, math.inf, 0)``. ``start == goal`` -> path ``[start]``, cost 0.
    * Open list: a heap keyed by f = g + h. Break ties on f toward the SMALLER h (put h second in the
      tuple, then a counter so cells are never compared). On an open floor this is what keeps A* from
      expanding every cell with the same f.
    * A cell may be pushed several times; when you pop one that is already closed, skip it.
    * Count a cell in ``expanded`` when you close it (including the goal).
    * No path -> ``PlanResult(None, math.inf, expanded)``.
    """
    # TODO(student):
    #   1. g = {start: 0.0}, parent = {}, closed = set(), heap = [(h(start), h(start), 0, start)]
    #   2. pop; skip closed; close; if goal: rebuild the path through `parent` and return
    #   3. for each neighbour: new_g = g[cell] + step; if better, record g, parent, push (new_g + h, h, counter, nxt)
    raise NotImplementedError("astar")


def dijkstra(blocked: NDArray[np.bool_], start: Cell, goal: Cell) -> PlanResult:
    """Dijkstra = A* with a heuristic that is always zero."""
    # TODO(student): one line.
    raise NotImplementedError("dijkstra")
