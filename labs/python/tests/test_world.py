from __future__ import annotations

import math
import time
from pathlib import Path

import numpy as np
import pytest

from robotlab.sim import FREE, OCCUPIED, UNKNOWN, OccupancyGrid, World


def test_raycast_square_room_from_center():
    room = World.rectangle_room(4.0, 4.0)
    angles = np.array([0.0, math.pi / 2, math.pi, -math.pi / 2, math.pi / 4, 3 * math.pi / 4])
    ranges = room.raycast((2.0, 2.0), angles)
    assert ranges == pytest.approx([2.0, 2.0, 2.0, 2.0, 2 * math.sqrt(2), 2 * math.sqrt(2)])


def test_raycast_max_range_gives_inf():
    room = World.rectangle_room(10.0, 2.0)
    ranges = room.raycast((1.0, 1.0), [0.0, math.pi], max_range=5.0)
    assert np.isinf(ranges[0]) and ranges[1] == pytest.approx(1.0)


def test_raycast_empty_world_is_inf():
    assert np.all(np.isinf(World().raycast((0, 0), np.linspace(0, 6, 10))))


def test_raycast_circle_front_and_inside():
    world = World.from_segments(np.zeros((0, 4)), circles=[[3.0, 0.0, 1.0]])
    assert world.raycast((0.0, 0.0), [0.0])[0] == pytest.approx(2.0)
    assert world.raycast((3.0, 0.0), [0.0])[0] == pytest.approx(1.0)  # from inside: exit distance
    assert np.isinf(world.raycast((0.0, 0.0), [math.pi])[0])


def test_raycast_one_origin_per_ray():
    room = World.rectangle_room(4.0, 4.0)
    origins = np.array([[1.0, 2.0], [3.0, 2.0], [2.0, 0.5]])
    assert room.raycast(origins, [0.0, 0.0, -math.pi / 2]) == pytest.approx([3.0, 1.0, 0.5])


def test_raycast_is_fast_enough_for_lidar_labs():
    world = World.apartment()
    angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
    start = time.perf_counter()
    for _ in range(600):  # 60 s of scans at 10 Hz
        world.raycast((1.0, 1.3), angles, 12.0)
    assert time.perf_counter() - start < 3.0


def test_apartment_scan_is_closed_from_every_room():
    world = World.apartment()
    for x, y in [(1.0, 1.3), (1.0, 4.0), (4.8, 2.0), (4.0, 4.0)]:
        assert not world.collides(x, y, 0.16)
        ranges = world.raycast((x, y), np.linspace(-math.pi, math.pi, 360, endpoint=False))
        assert np.all(np.isfinite(ranges)) and ranges.max() < 8.0


def test_collides_and_clearance():
    room = World.rectangle_room(2.0, 2.0)
    assert not room.collides(1.0, 1.0, 0.5)
    assert room.collides(1.8, 1.0, 0.25)
    assert room.distance_to_obstacles([[1.0, 1.0], [0.1, 1.0]]) == pytest.approx([1.0, 0.1])


def test_landmark_ids_default_and_validation():
    world = World.rectangle_room(2.0, 2.0, landmarks=[[0, 1], [2, 1]])
    assert list(world.landmark_ids) == [0, 1]
    with pytest.raises(ValueError):
        World.from_segments(np.zeros((0, 4)), landmarks=[[0, 0]], landmark_ids=[1, 2])


def test_occupancy_grid_of_room():
    grid = World.rectangle_room(2.0, 1.0).to_occupancy_grid(resolution=0.1, margin=0.5)
    assert (grid.height, grid.width) == (20, 30)
    assert grid.origin == pytest.approx((-0.5, -0.5))
    assert grid.is_occupied(0.0, 0.5) and grid.is_occupied(1.0, 1.0)
    assert not grid.is_occupied(1.0, 0.5)
    row, col = grid.world_to_cell(1.0, 0.5)
    assert grid.cell_to_world(row, col) == pytest.approx((1.05, 0.55))


def test_occupancy_grid_walls_do_not_leak_diagonally():
    world = World.from_segments([[0.0, 0.0, 3.0, 2.2]])
    data = world.to_occupancy_grid(0.05, 0.2).data == OCCUPIED
    diagonal_gap = ~data[:-1, :-1] & ~data[1:, 1:] & data[:-1, 1:] & data[1:, :-1]
    anti_gap = data[:-1, :-1] & data[1:, 1:] & ~data[:-1, 1:] & ~data[1:, :-1]
    assert not diagonal_gap.any() and not anti_gap.any()


def test_map_server_round_trip(tmp_path: Path):
    grid = World.apartment().to_occupancy_grid(0.05)
    grid.data[:3, :3] = UNKNOWN
    yaml_path = grid.save(tmp_path / "apartment.yaml")
    assert (tmp_path / "apartment.pgm").read_bytes().startswith(b"P5")
    loaded = OccupancyGrid.load(yaml_path)
    assert loaded.resolution == pytest.approx(grid.resolution)
    assert loaded.origin == pytest.approx(grid.origin)
    assert np.array_equal(loaded.data, grid.data)


def test_loads_ascii_pgm_with_comment_in_scale_mode(tmp_path: Path):
    (tmp_path / "m.pgm").write_text("P2\n# made by hand\n3 1\n255\n0 255 128\n")
    (tmp_path / "m.yaml").write_text(
        "image: m.pgm\nmode: scale\nresolution: 0.5\norigin: [1.0, 2.0, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n"
    )
    grid = OccupancyGrid.load(tmp_path / "m.yaml")
    assert grid.data[0, 0] == OCCUPIED and grid.data[0, 1] == FREE
    assert 0 < grid.data[0, 2] < 100
    assert grid.origin == (1.0, 2.0)
