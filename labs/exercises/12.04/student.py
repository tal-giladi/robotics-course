"""12.04 — The inflation layer, exactly like Nav2.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 12.04``.
Only the standard library and numpy are needed.

Cost values (nav2_costmap_2d/cost_values.hpp) are given below. A costmap is a 2D ``np.uint8`` array
indexed ``[row, col]``; distances between cells are Euclidean between cell CENTERS, in cells
(multiply by ``resolution`` for meters).
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
    """(inscribed, circumscribed) radius of a closed polygon around base_link, as Nav2 computes them.

    ``footprint`` lists the vertices in order, e.g. ``[(0.125, 0.105), (0.125, -0.105), ...]``; the last
    vertex connects back to the first. Nav2 takes, over every vertex AND every edge (segment),
    inscribed = the smallest distance from the origin, circumscribed = the largest.
    """
    # TODO(student): point-to-segment distance from (0, 0) for each edge, plus each vertex distance.
    raise NotImplementedError("footprint_radii")


def inflation_cost(distance_m: float, inscribed_radius_m: float, cost_scaling_factor: float) -> int:
    """Nav2 InflationLayer::computeCost for a cell ``distance_m`` from the nearest lethal cell:

    * distance 0                        -> LETHAL_OBSTACLE (254)
    * distance <= inscribed radius      -> INSCRIBED_INFLATED_OBSTACLE (253)
    * otherwise                         -> (253 - 1) * exp(-cost_scaling_factor * (distance - inscribed)),
                                           TRUNCATED to an integer (C++ ``static_cast<unsigned char>``)
    """
    # TODO(student)
    raise NotImplementedError("inflation_cost")


def distance_for_cost(cost: float, inscribed_radius_m: float, cost_scaling_factor: float) -> float:
    """The distance at which the exponential part of ``inflation_cost`` equals ``cost`` (1..252).

    Tuning question this answers: "how far from the wall does the cost drop below 128?"
    """
    # TODO(student): solve 252 * exp(-k (d - r)) = cost for d.
    raise NotImplementedError("distance_for_cost")


def inflate(
    master: NDArray[np.uint8],
    resolution: float,
    inscribed_radius_m: float,
    cost_scaling_factor: float,
    inflation_radius_m: float,
) -> NDArray[np.uint8]:
    """Return a NEW costmap: ``master`` with the inflation layer applied.

    1. Sources are the cells equal to LETHAL_OBSTACLE (254).
    2. For every cell, find the distance (in cells, between centers) to the nearest source.
    3. Only cells within ``ceil(inflation_radius_m / resolution)`` cells of a source are touched
       (Nav2's Costmap2D::cellDistance rounds UP to whole cells).
    4. new cost = inflation_cost(distance_cells * resolution, ...)
    5. Combine like Nav2: if the old cell is NO_INFORMATION (255) it only takes the new cost when that is
       >= 253; otherwise the cell becomes max(old, new).
    Do not modify ``master``.
    """
    # TODO(student): a loop over lethal cells updating a "nearest distance" array in a
    # (2 * cell_radius + 1)^2 window around each is plenty fast for these grids.
    raise NotImplementedError("inflate")
