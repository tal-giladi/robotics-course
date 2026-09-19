"""11.02 — Occupancy grid mapping with known poses (log-odds and Bresenham rays).

Fill in every ``TODO(student)``. Check your work with ``python course.py check 11.02``.
Only the standard library and numpy are needed.

Conventions (the same as ROS nav_msgs/OccupancyGrid and robotlab's OccupancyGrid):
* ``logodds[row, col]``: ``col`` counts cells along +x, ``row`` along +y, row 0 is the bottom.
* ``origin`` is the world (x, y) of the bottom-left corner of cell (row 0, col 0).
* A cell is written ``(col, row)`` when passed to the ray functions, like an (x, y) pair.
"""

from __future__ import annotations

import math

import numpy as np

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


def bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """All integer cells on the line from (x0, y0) to (x1, y1), in order, BOTH ends included.

    Must work in every direction (all eight octants). Consecutive cells touch (8-connected), and
    the list has exactly max(|x1 - x0|, |y1 - y0|) + 1 cells.

    >>> bresenham(0, 0, 5, 2)
    [(0, 0), (1, 0), (2, 1), (3, 1), (4, 2), (5, 2)]
    """
    # TODO(student): integer-only Bresenham. One classic form:
    #   dx = |x1-x0|, dy = -|y1-y0|, sx/sy = step signs, err = dx + dy
    #   loop: append (x, y); stop at the end; e2 = 2*err;
    #         if e2 >= dy: err += dy; x += sx
    #         if e2 <= dx: err += dx; y += sy
    raise NotImplementedError("bresenham")


def logodds(p: float) -> float:
    """Probability -> log-odds, ln(p / (1 - p)). logodds(0.5) == 0."""
    # TODO(student)
    raise NotImplementedError("logodds")


def probability(l: float | np.ndarray) -> float | np.ndarray:
    """Log-odds -> probability, 1 - 1 / (1 + exp(l)). Works element-wise on numpy arrays."""
    # TODO(student)
    raise NotImplementedError("probability")


class LogOddsMap:
    """A log-odds occupancy grid. Every cell starts at l = 0 (p = 0.5, unknown)."""

    def __init__(
        self,
        width: int,
        height: int,
        resolution: float,
        origin: tuple[float, float] = (0.0, 0.0),
        p_hit: float = 0.7,
        p_miss: float = 0.35,
        l_clamp: float = 4.0,
    ) -> None:
        self.width, self.height = width, height  # cells along x (columns) and y (rows)
        self.resolution = resolution  # meters per cell
        self.origin = origin
        self.p_hit, self.p_miss, self.l_clamp = p_hit, p_miss, l_clamp
        self.logodds = np.zeros((height, width))

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        """World point -> ``(col, row)`` of the cell containing it (may be outside the grid)."""
        return (math.floor((x - self.origin[0]) / self.resolution), math.floor((y - self.origin[1]) / self.resolution))

    def update_ray(self, start: tuple[int, int], end: tuple[int, int], hit: bool) -> None:
        """Apply the inverse sensor model along one beam, from cell ``start`` to cell ``end``.

        * every cell of the Bresenham line BEFORE ``end``: add logodds(p_miss) (the beam passed)
        * the ``end`` cell: add logodds(p_hit) if ``hit`` (the beam stopped there),
          otherwise logodds(p_miss) (the beam reached max range without hitting anything)
        * skip cells outside the grid; clamp every updated cell to [-l_clamp, +l_clamp]
        """
        # TODO(student): walk bresenham(*start, *end); remember logodds[row, col] = logodds[y, x].
        raise NotImplementedError("LogOddsMap.update_ray")

    def integrate_scan(
        self, pose: tuple[float, float, float], ranges: np.ndarray, angles: np.ndarray, max_range: float
    ) -> None:
        """Add one 2D LiDAR scan taken by a sensor at ``pose = (x, y, theta)`` in the map frame.

        ``angles`` are beam directions in the SENSOR frame (add theta for the map frame).
        * finite range <= max_range: a hit at that distance
        * +inf, or a range beyond max_range: no hit; the beam is free up to max_range
        * nan: a dropout; ignore the beam
        """
        # TODO(student): for each beam compute the end point in the map frame, convert start and
        #   end to cells with world_to_cell, and call update_ray.
        raise NotImplementedError("LogOddsMap.integrate_scan")

    def to_trinary(self, free_thresh: float = 0.25, occupied_thresh: float = 0.65) -> np.ndarray:
        """int8 array like nav_msgs/OccupancyGrid: p >= occupied_thresh -> 100, p <= free_thresh -> 0,
        anything in between -> -1 (unknown)."""
        # TODO(student)
        raise NotImplementedError("LogOddsMap.to_trinary")
