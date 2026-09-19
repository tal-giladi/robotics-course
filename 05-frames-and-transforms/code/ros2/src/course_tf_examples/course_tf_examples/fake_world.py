"""A 4 m x 3 m empty room for fake LiDAR scans (lesson 05.09). No rclpy import.

The room's walls are x = 0, x = 4, y = 0, y = 3 in the map frame. fake_scan looks up where the
laser is in the map and ray-casts every beam against the walls, so the scan it publishes is
consistent with TF: in RViz (fixed frame map) the walls stand still while the robot drives.
"""

from __future__ import annotations

import math

import numpy as np

ROOM = (0.0, 4.0, 0.0, 3.0)          # xmin, xmax, ymin, ymax  [m]


def beam_angles(n: int) -> np.ndarray:
    """n beams over a full turn, counter-clockwise from the laser's +x (LaserScan convention)."""
    return -math.pi + np.arange(n) * (2.0 * math.pi / n)


def raycast_room(x: float, y: float, yaw: float, angles: np.ndarray,
                 room: tuple[float, float, float, float] = ROOM, max_range: float = 12.0) -> np.ndarray:
    """Range of each beam from a laser at (x, y, yaw) in map to the room's walls.

    Beams from outside the room, or that miss, return inf (LaserScan's "no return").
    """
    xmin, xmax, ymin, ymax = room
    out = np.full(len(angles), math.inf)
    if not (xmin < x < xmax and ymin < y < ymax):
        return out
    d = np.stack([np.cos(yaw + angles), np.sin(yaw + angles)], axis=1)
    with np.errstate(divide='ignore', invalid='ignore'):
        tx = np.where(d[:, 0] > 0, (xmax - x) / d[:, 0], np.where(d[:, 0] < 0, (xmin - x) / d[:, 0], np.inf))
        ty = np.where(d[:, 1] > 0, (ymax - y) / d[:, 1], np.where(d[:, 1] < 0, (ymin - y) / d[:, 1], np.inf))
    r = np.minimum(tx, ty)
    out = np.where(r <= max_range, r, math.inf)
    return out
