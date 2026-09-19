"""Provided helpers for 10.08 — the likelihood field, beam subsampling and the tour log.

    from mcl_sim import build_likelihood_field, subsample_scan, drive_tour

``build_likelihood_field`` precomputes, once, the distance from every grid cell to the nearest
obstacle — exactly what Nav2's AMCL does with ``laser_likelihood_max_dist``. Looking a scan endpoint
up in that grid is a numpy index, so weighting 1000 particles x 60 beams costs about a millisecond,
while ray casting the same thing costs about 200 ms.
"""

from __future__ import annotations

import importlib.util
import sys
from dataclasses import dataclass
from pathlib import Path
from types import ModuleType

import numpy as np
from numpy.typing import ArrayLike, NDArray

from robotlab.sim import World

HERE = Path(__file__).resolve().parent


def _tour_sim() -> ModuleType:
    """Import ``labs/exercises/10.06/tour_sim.py`` (the apartment tour 10.06-10.10 share)."""
    name = "course_exercise_10_06_tour_sim"
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, HERE.parent / "10.06" / "tour_sim.py")
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def drive_tour(*args, **kwargs):
    """``tour_sim.drive_tour``: karmel tours the apartment and logs ticks, gyro, scans and truth."""
    return _tour_sim().drive_tour(*args, **kwargs)


def wheel_travels(ticks, config=None):
    """(T-1, 2) left/right wheel travel per step in meters, from a tick log."""
    return _tour_sim().wheel_travels(ticks, config)


@dataclass(frozen=True)
class LikelihoodField:
    """Distance from each cell center to the nearest obstacle, clipped at ``max_dist``."""

    dist: NDArray[np.floating]  # (rows, cols), meters; rows index y, cols index x
    resolution: float
    origin: tuple[float, float]  # world (x, y) of the center of cell (0, 0)
    max_dist: float

    def query(self, points: ArrayLike) -> NDArray[np.floating]:
        """Distance to the nearest obstacle for each ``(N, 2)`` world point (``max_dist`` outside)."""
        p = np.asarray(points, dtype=float).reshape(-1, 2)
        col = np.rint((p[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = np.rint((p[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = ((row >= 0) & (row < self.dist.shape[0]) & (col >= 0) & (col < self.dist.shape[1]))
        out = np.full(p.shape[0], self.max_dist)
        out[inside] = self.dist[row[inside], col[inside]]
        return out


def build_likelihood_field(world: World, resolution: float = 0.05, max_dist: float = 2.0,
                           margin: float = 0.3) -> LikelihoodField:
    """Precompute the obstacle-distance grid covering ``world`` (a few hundred ms, done once)."""
    x0, x1, y0, y1 = world.bounds
    x0, y0, x1, y1 = x0 - margin, y0 - margin, x1 + margin, y1 + margin
    xs = np.arange(x0, x1 + resolution, resolution)
    ys = np.arange(y0, y1 + resolution, resolution)
    gx, gy = np.meshgrid(xs, ys)
    d = world.distance_to_obstacles(np.column_stack([gx.ravel(), gy.ravel()]))
    return LikelihoodField(np.minimum(d, max_dist).reshape(gy.shape), resolution, (float(xs[0]), float(ys[0])), max_dist)


def subsample_scan(scan, beams: int = 60) -> tuple[NDArray[np.floating], NDArray[np.floating]]:
    """``beams`` evenly spaced VALID (range, angle) pairs from a LaserScan, in the sensor frame.

    Dropping beams is not an approximation you apologise for: neighbouring LiDAR returns hit the same
    wall and are strongly correlated, so 60 beams carry nearly all the information of 360 and keep the
    filter from becoming absurdly overconfident. AMCL's ``max_beams`` default is 60 for this reason.
    """
    idx = np.flatnonzero(np.asarray(scan.valid))
    if idx.size == 0:
        return np.empty(0), np.empty(0)
    take = idx[np.linspace(0, idx.size - 1, min(beams, idx.size)).astype(int)]
    return np.asarray(scan.ranges)[take], np.asarray(scan.angles)[take]
