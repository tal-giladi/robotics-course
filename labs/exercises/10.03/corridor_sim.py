"""Provided (not an exercise): karmel drives down a simulated corridor with doors.

The corridor is 10 m long and 1.2 m wide, cut into 0.25 m cells. The bottom (right-hand) wall is
solid; the top (left-hand) wall has doorways. The robot:

* follows the right-hand wall using two LiDAR beams (so it drives straight down the middle),
* measures how far it has travelled with its wheel encoders (odometry, 09.04),
* each time odometry says "one more cell", looks left with the LiDAR beams around +90 degrees
  and reports ``True`` (a doorway: no wall within 1 m), ``False`` (a wall), or ``None`` (all beams
  dropped out).

This gives exactly the inputs of a 1D histogram filter: one ``move`` of +1 cell and one door
reading per cell, plus the true cell for grading.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from robotlab.config import KarmelConfig, load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World

CELL_M = 0.25
N_CELLS = 40
WIDTH_M = 1.2
DOOR_CELLS = (3, 4, 9, 16, 17, 23, 30, 31, 32, 36)
SPEED_M_S = 0.4
DT = 0.05


def door_map() -> NDArray[np.bool_]:
    """``True`` for the cells that have a doorway on the left wall."""
    doors = np.zeros(N_CELLS, dtype=bool)
    doors[list(DOOR_CELLS)] = True
    return doors


def corridor_world() -> World:
    doors = door_map()
    length = N_CELLS * CELL_M
    segments = [[0.0, 0.0, length, 0.0], [0.0, 0.0, 0.0, WIDTH_M], [length, 0.0, length, WIDTH_M]]
    i = 0
    while i < N_CELLS:  # the left wall: one segment per run of door-free cells
        if doors[i]:
            i += 1
            continue
        j = i
        while j < N_CELLS and not doors[j]:
            j += 1
        segments.append([i * CELL_M, WIDTH_M, j * CELL_M, WIDTH_M])
        i = j
    return World.from_segments(segments)


@dataclass(frozen=True)
class CorridorRun:
    moves: list[int]            # +1 per cell crossed (from odometry)
    readings: list[bool | None]  # door seen on the left?
    true_cells: list[int]        # ground-truth cell index when the reading was taken
    start_cell: int


def _beam(ranges: NDArray[np.floating], angles: NDArray[np.floating], angle: float) -> float:
    i = int(np.argmin(np.abs(np.angle(np.exp(1j * (angles - angle))))))
    return float(ranges[i])


def drive_corridor(
    start_cell: int, n_cells: int, seed: int, realistic: bool = True, config: KarmelConfig | None = None
) -> CorridorRun:
    """Drive ``n_cells`` cells from the centre of ``start_cell`` and record moves and door readings."""
    cfg = config or load_config()
    params = DiffDriveParams.realistic(cfg) if realistic else DiffDriveParams.ideal(cfg)
    sensors = SensorParams.realistic(cfg) if realistic else SensorParams.ideal(cfg)
    if start_cell < 0 or start_cell + n_cells >= N_CELLS:
        raise ValueError("the robot would hit the end of the corridor")
    x0 = (start_cell + 0.5) * CELL_M
    sim = DiffDriveSim(corridor_world(), params, sensors, pose=(x0, WIDTH_M / 2, 0.0), seed=seed, config=cfg)
    r, b = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
    m_per_tick = cfg.drive.meters_per_tick
    ticks0 = sim.ticks
    omega = 0.0
    moves: list[int] = []
    readings: list[bool | None] = []
    truth: list[int] = []
    step = 0
    while len(moves) < n_cells:
        wheel = SPEED_M_S / r
        sim.set_velocity(wheel - omega * b / (2 * r), wheel + omega * b / (2 * r))
        sim.step(DT)
        step += 1
        left, right = sim.ticks
        travelled = ((left - ticks0[0]) + (right - ticks0[1])) / 2 * m_per_tick
        crossed = travelled >= (len(moves) + 1) * CELL_M
        if step % 2 and not crossed:
            continue
        scan = sim.lidar_scan()
        angles = scan.angles
        right_90 = _beam(scan.ranges, angles, -math.pi / 2)
        right_60 = _beam(scan.ranges, angles, -math.pi / 3)
        if math.isfinite(right_90) and math.isfinite(right_60):
            # Two points on the right wall (sensor frame) -> wall direction and perpendicular distance.
            p1 = np.array([0.0, -right_90])
            p2 = right_60 * np.array([math.cos(-math.pi / 3), math.sin(-math.pi / 3)])
            d = p2 - p1
            wall_angle = math.atan2(d[1], d[0])
            wall_dist = abs(p1[0] * d[1] - p1[1] * d[0]) / math.hypot(d[0], d[1])
            omega = 2.0 * (WIDTH_M / 2 - wall_dist) + 3.0 * wall_angle
        if crossed:
            left_beams = np.array([_beam(scan.ranges, angles, math.pi / 2 + math.radians(a)) for a in (-4, -2, 0, 2, 4)])
            finite = left_beams[~np.isnan(left_beams)]
            reading = None if finite.size == 0 else bool(np.median(finite) > WIDTH_M / 2 + 0.4)
            moves.append(1)
            readings.append(reading)
            truth.append(int(sim.pose.x // CELL_M))
    return CorridorRun(moves, readings, truth, start_cell)
