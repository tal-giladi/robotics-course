"""Checker for 11.02 — Occupancy grid mapping with known poses.

Run: ``python course.py check 11.02`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robotlab.sim import World, box_segments

approx = pytest.approx


# --- bresenham -------------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("line", "expected"),
    [
        ((0, 0, 5, 2), [(0, 0), (1, 0), (2, 1), (3, 1), (4, 2), (5, 2)]),  # shallow, +x +y
        ((0, 0, 2, 5), [(0, 0), (0, 1), (1, 2), (1, 3), (2, 4), (2, 5)]),  # steep: y is the major axis
        ((0, 0, -3, 0), [(0, 0), (-1, 0), (-2, 0), (-3, 0)]),  # horizontal, backwards
        ((2, 7, 2, 4), [(2, 7), (2, 6), (2, 5), (2, 4)]),  # vertical, downwards
        ((0, 0, 3, -3), [(0, 0), (1, -1), (2, -2), (3, -3)]),  # diagonal
        ((4, 4, 4, 4), [(4, 4)]),  # a single cell
    ],
)
def test_bresenham_known_lines(impl, line, expected):
    assert impl.bresenham(*line) == expected


def test_bresenham_is_connected_in_every_octant(impl):
    rng = np.random.default_rng(11)
    for _ in range(300):
        x0, y0, x1, y1 = (int(v) for v in rng.integers(-30, 30, 4))
        cells = impl.bresenham(x0, y0, x1, y1)
        assert cells[0] == (x0, y0) and cells[-1] == (x1, y1), "both end cells must be included, in order"
        assert len(cells) == max(abs(x1 - x0), abs(y1 - y0)) + 1, f"wrong number of cells for {(x0, y0, x1, y1)}"
        steps = np.abs(np.diff(np.array(cells), axis=0)) if len(cells) > 1 else np.zeros((0, 2))
        assert (steps <= 1).all(), f"gap in the line {(x0, y0, x1, y1)}: consecutive cells must touch"


def test_bresenham_stays_close_to_the_true_line(impl):
    # Every cell center is within half a cell (perpendicular) of the ideal line (1, 1) -> (13, 5).
    cells = np.array(impl.bresenham(1, 1, 13, 5), dtype=float)
    d = np.array([12.0, 4.0]) / math.hypot(12.0, 4.0)
    off = (cells - [1.0, 1.0]) @ np.array([-d[1], d[0]])
    assert np.abs(off).max() <= 0.5 + 1e-9


# --- log-odds ----------------------------------------------------------------------------------------
def test_logodds_values(impl):
    assert impl.logodds(0.5) == approx(0.0)
    assert impl.logodds(0.7) == approx(0.8473, abs=1e-4)  # ln(0.7 / 0.3)
    assert impl.logodds(0.35) == approx(-0.6190, abs=1e-4)
    assert impl.probability(0.0) == approx(0.5)
    assert impl.probability(impl.logodds(0.9)) == approx(0.9)
    assert np.allclose(impl.probability(np.array([-4.0, 0.0, 4.0])), [0.01799, 0.5, 0.98201], atol=1e-5)


def make_row_map(impl, **kwargs):
    return impl.LogOddsMap(width=10, height=3, resolution=0.1, **kwargs)


def test_update_ray_with_hit(impl):
    m = make_row_map(impl)
    m.update_ray((0, 1), (4, 1), hit=True)
    l_hit, l_miss = math.log(0.7 / 0.3), math.log(0.35 / 0.65)
    assert m.logodds[1, :4] == approx([l_miss] * 4), "cells the beam passed through get the miss update"
    assert m.logodds[1, 4] == approx(l_hit), "the end cell of a hit gets the hit update"
    assert (m.logodds[1, 5:] == 0).all() and (m.logodds[[0, 2]] == 0).all(), "no other cell changes"


def test_update_ray_without_hit_marks_the_end_free(impl):
    m = make_row_map(impl)
    m.update_ray((6, 1), (2, 1), hit=False)
    assert m.logodds[1, 2:7] == approx([math.log(0.35 / 0.65)] * 5)


def test_update_ray_hit_then_miss_sequence(impl):
    # hit, hit, miss on the same cell: 2 * 0.8473 - 0.6190 = 1.0756 -> p = 0.746
    m = make_row_map(impl)
    m.update_ray((0, 1), (3, 1), hit=True)
    m.update_ray((0, 1), (3, 1), hit=True)
    m.update_ray((0, 1), (5, 1), hit=True)
    assert m.logodds[1, 3] == approx(1.0756, abs=1e-4)
    assert impl.probability(m.logodds[1, 3]) == approx(0.746, abs=1e-3)


def test_update_ray_clamps_and_ignores_cells_outside(impl):
    m = make_row_map(impl)
    for _ in range(20):
        m.update_ray((-5, 1), (8, 1), hit=True)  # starts outside the grid: no IndexError
    assert m.logodds[1, 8] == approx(4.0), "20 hits must be clamped to +l_clamp"
    assert m.logodds[1, 0] == approx(-4.0), "20 misses must be clamped to -l_clamp"
    m.update_ray((0, 1), (15, 1), hit=True)  # ends outside the grid
    assert np.isfinite(m.logodds).all()


def test_to_trinary_thresholds(impl):
    m = make_row_map(impl)
    m.logodds[0, :5] = [-1.2, -1.0, 0.0, 0.5, 0.7]  # p = 0.23, 0.27, 0.50, 0.62, 0.67
    out = m.to_trinary()
    assert out.dtype == np.int8
    assert out[0, :5].tolist() == [0, -1, -1, -1, 100]
    assert (out[1:] == -1).all(), "untouched cells (p = 0.5) are unknown"


def test_integrate_scan_single_beams(impl):
    m = impl.LogOddsMap(width=40, height=40, resolution=0.1, origin=(-2.0, -2.0))
    # Sensor at (0.05, 0.05) = center of cell (20, 20), facing +y (theta = 90 deg).
    ranges = np.array([1.0, np.inf, np.nan])
    angles = np.array([0.0, math.pi / 2, math.pi])  # sensor frame: ahead (+y in map), left (-x), behind
    m.integrate_scan((0.05, 0.05, math.pi / 2), ranges, angles, max_range=1.5)
    assert m.logodds[30, 20] > 0.8, "beam 1 hits 1.0 m ahead = +y in the map: cell (col 20, row 30)"
    assert (m.logodds[20:30, 20] < 0).all(), "cells before the hit are free"
    assert (m.logodds[20, 5:20] < 0).all(), "the +inf beam (map -x) is free up to max_range = 15 cells"
    assert m.logodds[20, 4] == 0, "... and not beyond"
    assert (m.logodds[11:20, 20] == 0).all(), "the nan beam (map -y) is ignored"


# --- a known small world -----------------------------------------------------------------------------
def small_world() -> World:
    walls = np.vstack([box_segments(0.0, 0.0, 3.0, 2.0), [[1.5, 0.0, 1.5, 1.2]]])
    return World.from_segments(walls, circles=[[2.3, 1.3, 0.2]])


def test_maps_a_known_world(impl):
    world = small_world()
    truth = world.to_occupancy_grid(resolution=0.05, margin=0.25)
    m = impl.LogOddsMap(truth.width, truth.height, truth.resolution, truth.origin)
    angles = -math.pi + np.arange(360) * (2 * math.pi / 360)
    poses = [(0.7, 0.6, 0.0), (0.8, 1.6, -1.0), (2.2, 0.5, 2.0), (2.6, 1.7, 3.0), (1.5, 1.6, 0.5)]
    for pose in poses:
        ranges = world.raycast(pose[:2], angles + pose[2], max_range=6.0)
        m.integrate_scan(pose, ranges, angles, max_range=6.0)
    out = m.to_trinary()
    assert out.shape == truth.data.shape

    mapped_occ, mapped_free = out == 100, out == 0
    assert mapped_occ.sum() > 200, "the walls and the round obstacle should appear as occupied cells"
    clearance_occ = world.distance_to_obstacles(np.column_stack(truth.cell_to_world(*np.nonzero(mapped_occ))))
    assert (clearance_occ < 0.075).mean() > 0.97, "occupied cells must lie on real walls (within 1.5 cells)"
    clearance_free = world.distance_to_obstacles(np.column_stack(truth.cell_to_world(*np.nonzero(mapped_free))))
    assert (clearance_free > 0.01).mean() > 0.97, "free cells must not contain walls"
    x, y = truth.cell_to_world(*np.nonzero(truth.data == 0))
    inside = world.distance_to_obstacles(np.column_stack([x, y])) > 0.1
    inside &= (x > 0) & (x < 3) & (y > 0) & (y < 2)
    rows, cols = np.nonzero(truth.data == 0)
    seen = (out[rows[inside], cols[inside]] == 0).mean()
    assert seen > 0.9, f"most of the open floor should be mapped as free (got {seen:.0%})"
