"""Occupancy grids, with save/load in the ROS ``map_server`` format (PGM image + YAML).

Cell values follow ``nav_msgs/OccupancyGrid``: 0 = free, 100 = occupied, -1 = unknown.
``data[row, col]``: row 0 is the *bottom* of the map (smallest y), col 0 the smallest x, and
``origin`` is the world position of the bottom-left corner of cell (0, 0) — the same layout as
the ROS message. (The PGM image is stored top row first, so it is flipped on save/load.)
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from numpy.typing import ArrayLike, NDArray

FREE = 0
OCCUPIED = 100
UNKNOWN = -1

# Pixel values map_saver writes for trinary maps.
_PGM_OCCUPIED = 0
_PGM_FREE = 254
_PGM_UNKNOWN = 205


@dataclass
class OccupancyGrid:
    """A 2D grid of occupancy values with its placement in the world."""

    data: NDArray[np.int8]
    resolution: float  # meters per cell
    origin: tuple[float, float]  # world (x, y) of the bottom-left corner of cell (0, 0)

    FREE = FREE
    OCCUPIED = OCCUPIED
    UNKNOWN = UNKNOWN

    @property
    def height(self) -> int:
        """Number of rows (y direction)."""
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        """Number of columns (x direction)."""
        return int(self.data.shape[1])

    @property
    def extent(self) -> tuple[float, float, float, float]:
        """``(x_min, x_max, y_min, y_max)`` — handy for ``imshow(..., extent=grid.extent)``."""
        x0, y0 = self.origin
        return (x0, x0 + self.width * self.resolution, y0, y0 + self.height * self.resolution)

    def world_to_cell(self, x: ArrayLike, y: ArrayLike) -> tuple[Any, Any]:
        """World coordinates -> ``(row, col)`` integer indices (may be out of bounds)."""
        col = np.floor((np.asarray(x, dtype=float) - self.origin[0]) / self.resolution)
        row = np.floor((np.asarray(y, dtype=float) - self.origin[1]) / self.resolution)
        if np.ndim(col) == 0:
            return int(row), int(col)
        return row.astype(int), col.astype(int)

    def cell_to_world(self, row: ArrayLike, col: ArrayLike) -> tuple[Any, Any]:
        """``(row, col)`` -> world coordinates of the cell *center*."""
        x = self.origin[0] + (np.asarray(col, dtype=float) + 0.5) * self.resolution
        y = self.origin[1] + (np.asarray(row, dtype=float) + 0.5) * self.resolution
        if np.ndim(x) == 0:
            return float(x), float(y)
        return x, y

    def in_bounds(self, row: ArrayLike, col: ArrayLike) -> Any:
        r, c = np.asarray(row), np.asarray(col)
        return (r >= 0) & (r < self.height) & (c >= 0) & (c < self.width)

    def is_occupied(self, x: float, y: float) -> bool:
        """True if the world point lies in an occupied cell (outside the grid counts as free)."""
        row, col = self.world_to_cell(x, y)
        return bool(self.in_bounds(row, col)) and self.data[row, col] == OCCUPIED

    # --- ROS map_server format -----------------------------------------------------------------
    def save(self, yaml_path: str | os.PathLike[str]) -> Path:
        """Write ``<name>.yaml`` + ``<name>.pgm`` (trinary mode, like ``map_saver_cli``)."""
        yaml_path = Path(yaml_path)
        pgm_path = yaml_path.with_suffix(".pgm")
        image = np.full(self.data.shape, _PGM_UNKNOWN, dtype=np.uint8)
        image[self.data == FREE] = _PGM_FREE
        image[self.data == OCCUPIED] = _PGM_OCCUPIED
        _write_pgm(pgm_path, np.flipud(image))
        meta = {
            "image": pgm_path.name,
            "mode": "trinary",
            "resolution": float(self.resolution),
            "origin": [float(self.origin[0]), float(self.origin[1]), 0.0],
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,  # unknown pixels (205) must load as unknown, not free
        }
        yaml_path.write_text(yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")
        return yaml_path

    @classmethod
    def load(cls, yaml_path: str | os.PathLike[str]) -> OccupancyGrid:
        """Read a map_server map (``trinary``, ``scale`` or ``raw`` mode). Origin yaw is ignored."""
        yaml_path = Path(yaml_path)
        meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        image, maxval = _read_pgm(yaml_path.parent / meta["image"])
        image = np.flipud(image).astype(float)
        mode = meta.get("mode", "trinary")
        if mode == "raw":
            data = image
        else:
            p = image / maxval if meta.get("negate", 0) else (maxval - image) / maxval
            occ, free = float(meta["occupied_thresh"]), float(meta["free_thresh"])
            data = np.full(image.shape, UNKNOWN, dtype=float)
            if mode == "scale":
                between = (p >= free) & (p <= occ)
                data[between] = np.rint(99.0 * (p[between] - free) / (occ - free))
            data[p > occ] = OCCUPIED
            data[p < free] = FREE
        origin = meta.get("origin", [0.0, 0.0, 0.0])
        return cls(data.astype(np.int8), float(meta["resolution"]), (float(origin[0]), float(origin[1])))


def _write_pgm(path: Path, image: NDArray[np.uint8]) -> None:
    header = f"P5\n{image.shape[1]} {image.shape[0]}\n255\n".encode("ascii")
    path.write_bytes(header + np.ascontiguousarray(image).tobytes())


def _read_pgm(path: Path) -> tuple[NDArray[np.integer], int]:
    """Minimal PGM reader: binary (P5, 8/16-bit) and ASCII (P2), with ``#`` comments."""
    raw = path.read_bytes()
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while raw[pos : pos + 1].isspace():
            pos += 1
        if raw[pos : pos + 1] == b"#":
            pos = raw.index(b"\n", pos) + 1
            continue
        start = pos
        while not raw[pos : pos + 1].isspace():
            pos += 1
        tokens.append(raw[start:pos])
    magic, width, height, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == b"P5":
        dtype = np.dtype(">u2") if maxval > 255 else np.dtype(np.uint8)
        pixels = np.frombuffer(raw, dtype=dtype, count=width * height, offset=pos + 1)
    elif magic == b"P2":
        pixels = np.array(raw[pos:].split()[: width * height], dtype=int)
    else:
        raise ValueError(f"{path}: unsupported image format {magic!r} (need PGM P5 or P2)")
    return pixels.reshape(height, width), maxval


def grid_shape(extent: tuple[float, float, float, float], resolution: float) -> tuple[int, int]:
    """Rows and columns needed to cover ``(x_min, x_max, y_min, y_max)`` at ``resolution``."""
    x_min, x_max, y_min, y_max = extent
    return math.ceil((y_max - y_min) / resolution - 1e-9), math.ceil((x_max - x_min) / resolution - 1e-9)
