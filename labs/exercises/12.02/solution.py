"""Reference solution for 12.02 — A* on an occupancy grid.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import heapq
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
    """Length of the shortest 8-connected path between two cells on an EMPTY grid."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc)


def neighbors(blocked: NDArray[np.bool_], cell: Cell) -> list[tuple[Cell, float]]:
    """Free 8-connected neighbours of ``cell`` with the step length (1 or sqrt 2), no corner cutting."""
    r, c = cell
    rows, cols = blocked.shape
    out: list[tuple[Cell, float]] = []
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or blocked[nr, nc]:
                continue
            if dr != 0 and dc != 0 and (blocked[r, nc] or blocked[nr, c]):
                continue
            out.append(((nr, nc), SQRT2 if dr != 0 and dc != 0 else 1.0))
    return out


def astar(blocked: NDArray[np.bool_], start: Cell, goal: Cell, heuristic: Heuristic = octile_distance) -> PlanResult:
    """A* from ``start`` to ``goal``; ties on f are broken toward the smaller h."""
    if blocked[start] or blocked[goal]:
        return PlanResult(None, math.inf, 0)
    g: dict[Cell, float] = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    closed: set[Cell] = set()
    h0 = heuristic(start, goal)
    counter = 0
    open_heap: list[tuple[float, float, int, Cell]] = [(h0, h0, counter, start)]
    while open_heap:
        _f, _h, _n, cell = heapq.heappop(open_heap)
        if cell in closed:
            continue
        closed.add(cell)
        if cell == goal:
            path = [goal]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return PlanResult(path[::-1], g[goal], len(closed))
        for nxt, step in neighbors(blocked, cell):
            if nxt in closed:
                continue
            new_g = g[cell] + step
            if new_g < g.get(nxt, math.inf):
                g[nxt] = new_g
                parent[nxt] = cell
                h = heuristic(nxt, goal)
                counter += 1
                heapq.heappush(open_heap, (new_g + h, h, counter, nxt))
    return PlanResult(None, math.inf, len(closed))


def dijkstra(blocked: NDArray[np.bool_], start: Cell, goal: Cell) -> PlanResult:
    """Dijkstra = A* with a heuristic that is always zero."""
    return astar(blocked, start, goal, lambda a, b: 0.0)
