"""Reference solution for 12.04 — The inflation layer, exactly like Nav2.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np
from numpy.typing import NDArray

FREE_SPACE = 0
MAX_NON_OBSTACLE = 252
INSCRIBED_INFLATED_OBSTACLE = 253
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255


def footprint_radii(footprint: Sequence[tuple[float, float]]) -> tuple[float, float]:
    """(inscribed, circumscribed) radius of a closed polygon around base_link, as Nav2 computes them."""
    lo, hi = math.inf, 0.0
    n = len(footprint)
    for i in range(n):
        ax, ay = footprint[i]
        bx, by = footprint[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0 else max(0.0, min(1.0, -(ax * dx + ay * dy) / length2))
        edge = math.hypot(ax + t * dx, ay + t * dy)
        vertex = math.hypot(ax, ay)
        lo, hi = min(lo, vertex, edge), max(hi, vertex, edge)
    return lo, hi


def inflation_cost(distance_m: float, inscribed_radius_m: float, cost_scaling_factor: float) -> int:
    """Nav2 InflationLayer::computeCost."""
    if distance_m == 0.0:
        return LETHAL_OBSTACLE
    if distance_m <= inscribed_radius_m:
        return INSCRIBED_INFLATED_OBSTACLE
    return int(MAX_NON_OBSTACLE * math.exp(-cost_scaling_factor * (distance_m - inscribed_radius_m)))


def distance_for_cost(cost: float, inscribed_radius_m: float, cost_scaling_factor: float) -> float:
    """Where the exponential decay reaches ``cost`` (1 <= cost <= 252)."""
    return inscribed_radius_m + math.log(MAX_NON_OBSTACLE / cost) / cost_scaling_factor


def inflate(
    master: NDArray[np.uint8],
    resolution: float,
    inscribed_radius_m: float,
    cost_scaling_factor: float,
    inflation_radius_m: float,
) -> NDArray[np.uint8]:
    """Return a new costmap with the inflation layer applied to ``master``."""
    rows, cols = master.shape
    cell_radius = math.ceil(inflation_radius_m / resolution)
    nearest = np.full(master.shape, np.inf)  # distance in cells to the nearest lethal cell
    rr, cc = np.nonzero(master == LETHAL_OBSTACLE)
    for r, c in zip(rr, cc):
        r0, r1 = max(0, r - cell_radius), min(rows, r + cell_radius + 1)
        c0, c1 = max(0, c - cell_radius), min(cols, c + cell_radius + 1)
        dr, dc = np.mgrid[r0:r1, c0:c1]
        d = np.hypot(dr - r, dc - c)
        nearest[r0:r1, c0:c1] = np.minimum(nearest[r0:r1, c0:c1], d)
    out = master.copy()
    for r, c in zip(*np.nonzero(nearest <= cell_radius)):
        cost = inflation_cost(float(nearest[r, c]) * resolution, inscribed_radius_m, cost_scaling_factor)
        old = int(master[r, c])
        if old == NO_INFORMATION:
            if cost >= INSCRIBED_INFLATED_OBSTACLE:
                out[r, c] = cost
        else:
            out[r, c] = max(old, cost)
    return out
