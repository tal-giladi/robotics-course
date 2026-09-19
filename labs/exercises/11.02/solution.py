"""11.02 — Occupancy grid mapping with known poses (reference solution)."""

from __future__ import annotations

import math

import numpy as np

UNKNOWN, FREE, OCCUPIED = -1, 0, 100


def bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """All integer cells on the line from (x0, y0) to (x1, y1), in order, both ends included."""
    cells = []
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx, sy = (1 if x1 > x0 else -1), (1 if y1 > y0 else -1)
    err = dx + dy
    x, y = x0, y0
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            return cells
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def logodds(p: float) -> float:
    return math.log(p / (1.0 - p))


def probability(l: float | np.ndarray) -> float | np.ndarray:
    return 1.0 - 1.0 / (1.0 + np.exp(l))


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
        self.width, self.height = width, height
        self.resolution = resolution
        self.origin = origin
        self.p_hit, self.p_miss, self.l_clamp = p_hit, p_miss, l_clamp
        self.logodds = np.zeros((height, width))

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (math.floor((x - self.origin[0]) / self.resolution), math.floor((y - self.origin[1]) / self.resolution))

    def _add(self, col: int, row: int, delta: float) -> None:
        if 0 <= col < self.width and 0 <= row < self.height:
            value = self.logodds[row, col] + delta
            self.logodds[row, col] = min(self.l_clamp, max(-self.l_clamp, value))

    def update_ray(self, start: tuple[int, int], end: tuple[int, int], hit: bool) -> None:
        l_hit, l_miss = logodds(self.p_hit), logodds(self.p_miss)
        cells = bresenham(start[0], start[1], end[0], end[1])
        for col, row in cells[:-1]:
            self._add(col, row, l_miss)
        self._add(end[0], end[1], l_hit if hit else l_miss)

    def integrate_scan(
        self, pose: tuple[float, float, float], ranges: np.ndarray, angles: np.ndarray, max_range: float
    ) -> None:
        x, y, theta = pose
        start = self.world_to_cell(x, y)
        for r, a in zip(np.asarray(ranges, dtype=float), np.asarray(angles, dtype=float)):
            if math.isnan(r):
                continue
            hit = r <= max_range
            d = r if hit else max_range
            end = self.world_to_cell(x + d * math.cos(theta + a), y + d * math.sin(theta + a))
            self.update_ray(start, end, hit)

    def to_trinary(self, free_thresh: float = 0.25, occupied_thresh: float = 0.65) -> np.ndarray:
        p = probability(self.logodds)
        out = np.full(self.logodds.shape, UNKNOWN, dtype=np.int8)
        out[p <= free_thresh] = FREE
        out[p >= occupied_thresh] = OCCUPIED
        return out
