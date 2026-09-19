"""Tests for the lesson 03.01 ecosystem tour — no hardware, no network.

    python -m pytest 03-robot-software/code/test_l0301_ecosystem.py
"""

from __future__ import annotations

import math

import numpy as np
import pytest

import l0301_ecosystem_tour as tour


def test_library_versions_lists_the_whole_stack() -> None:
    rows = tour.library_versions()
    names = [name for name, _, _ in rows]
    assert names == [name for name, _ in tour.STACK]
    assert dict((n, v) for n, v, _ in rows)["numpy"] != "not installed"
    # A missing optional package must read as "not installed", never raise.
    assert all(isinstance(version, str) and version for _, version, _ in rows)


def test_make_scan_covers_the_full_circle() -> None:
    ranges, angles = tour.make_scan(360)
    assert len(ranges) == len(angles) == 360
    assert angles[0] == pytest.approx(-math.pi)
    assert angles[1] - angles[0] == pytest.approx(2 * math.pi / 360)
    assert all(0.5 <= r <= 1.5 for r in ranges)


def test_the_two_transforms_compute_the_same_points() -> None:
    """The whole point of the lesson: vectorizing changes the speed, not the answer."""
    ranges, angles = tour.make_scan(120)
    pose = (1.0, 1.3, 0.4)
    slow = np.asarray(tour.transform_python(ranges, angles, pose))
    fast = np.asarray(tour.transform_numpy(np.asarray(ranges), np.asarray(angles), pose))
    assert fast.shape == (120, 2)
    np.testing.assert_allclose(slow, fast, atol=1e-12)


def test_transform_matches_a_hand_computed_point() -> None:
    # One beam: range 2 m straight ahead (angle 0), robot at (1, 1) rotated by 90 deg.
    # Sensor frame (2, 0) -> rotated (0, 2) -> translated (1, 3).
    points = tour.transform_python([2.0], [0.0], (1.0, 1.0, math.pi / 2))
    assert points[0][0] == pytest.approx(1.0, abs=1e-12)
    assert points[0][1] == pytest.approx(3.0, abs=1e-12)


def test_kdtree_and_brute_force_agree() -> None:
    rng = np.random.default_rng(3)
    points = rng.uniform(-2.0, 2.0, size=(40, 2))
    world = rng.uniform(-3.0, 3.0, size=(300, 2))
    np.testing.assert_array_equal(
        tour.nearest_bruteforce(points, world), tour.nearest_kdtree(points, world)
    )


def test_run_produces_one_timing_per_workload() -> None:
    results = tour.run(scans=2, samples=60, map_points=200, quick=True)
    names = [r.name for r in results]
    assert names.count("transform") == 2 and "associate" in names
    assert all(r.seconds >= 0.0 or math.isnan(r.seconds) for r in results)
    assert all(r.ms == pytest.approx(r.seconds * 1000.0) for r in results if not math.isnan(r.seconds))


def test_main_runs_end_to_end(capsys: pytest.CaptureFixture[str]) -> None:
    results = tour.main(["--scans", "2", "--samples", "60", "--map-points", "200", "--quick"])
    out = capsys.readouterr().out
    assert "numpy" in out and "workload" in out
    assert len(results) >= 3
