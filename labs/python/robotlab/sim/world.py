"""The 2D world: wall segments, round obstacles and point landmarks.

Everything geometric the simulator needs lives here — vectorized ray casting (LiDAR, ToF,
landmark visibility), clearance/collision queries, and rasterization to an occupancy grid for
the mapping and planning labs.

>>> world = World.apartment()
>>> world.raycast((1.0, 1.3), np.linspace(-np.pi, np.pi, 360, endpoint=False), max_range=12.0)
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray

from robotlab.sim.occupancy import FREE, OCCUPIED, OccupancyGrid, grid_shape

_CHUNK = 8192  # rays / points processed per batch, bounds memory to a few MB per call
_EPS = 1e-12


def _as_array(rows: ArrayLike | None, width: int) -> NDArray[np.floating]:
    if rows is None:
        return np.zeros((0, width))
    arr = np.asarray(rows, dtype=float)
    return arr.reshape(-1, width)


def box_segments(x_min: float, y_min: float, x_max: float, y_max: float) -> NDArray[np.floating]:
    """The four walls of an axis-aligned rectangle as ``(4, 4)`` segments ``[x0, y0, x1, y1]``."""
    return np.array(
        [
            [x_min, y_min, x_max, y_min],
            [x_max, y_min, x_max, y_max],
            [x_max, y_max, x_min, y_max],
            [x_min, y_max, x_min, y_min],
        ],
        dtype=float,
    )


@dataclass
class World:
    """Static 2D environment.

    Attributes:
        segments: ``(M, 4)`` wall segments ``[x0, y0, x1, y1]`` in meters.
        circles: ``(K, 3)`` round obstacles ``[cx, cy, radius]``.
        landmarks: ``(L, 2)`` point landmark positions (not obstacles; e.g. AprilTags on walls).
        landmark_ids: ``(L,)`` integer ids, one per landmark.
    """

    segments: NDArray[np.floating] = field(default_factory=lambda: np.zeros((0, 4)))
    circles: NDArray[np.floating] = field(default_factory=lambda: np.zeros((0, 3)))
    landmarks: NDArray[np.floating] = field(default_factory=lambda: np.zeros((0, 2)))
    landmark_ids: NDArray[np.integer] = field(default_factory=lambda: np.zeros(0, dtype=int))

    def __post_init__(self) -> None:
        self.segments = _as_array(self.segments, 4)
        self.circles = _as_array(self.circles, 3)
        self.landmarks = _as_array(self.landmarks, 2)
        ids = np.asarray(self.landmark_ids, dtype=int).reshape(-1)
        if ids.size == 0 and len(self.landmarks):
            ids = np.arange(len(self.landmarks))
        if ids.size != len(self.landmarks):
            raise ValueError("landmark_ids must have one id per landmark")
        self.landmark_ids = ids

    # --- constructors ----------------------------------------------------------------------------
    @classmethod
    def from_segments(
        cls,
        segments: ArrayLike,
        circles: ArrayLike | None = None,
        landmarks: ArrayLike | None = None,
        landmark_ids: Sequence[int] | None = None,
    ) -> World:
        """Build a world from raw arrays (see the class attributes for the shapes)."""
        return cls(
            _as_array(segments, 4),
            _as_array(circles, 3),
            _as_array(landmarks, 2),
            np.asarray(landmark_ids if landmark_ids is not None else [], dtype=int),
        )

    @classmethod
    def rectangle_room(cls, width: float, height: float, landmarks: ArrayLike | None = None) -> World:
        """An empty room spanning ``[0, width] x [0, height]``."""
        return cls.from_segments(box_segments(0.0, 0.0, width, height), landmarks=landmarks)

    @classmethod
    def apartment(cls) -> World:
        """A 6 m x 5 m apartment spanning ``[0, 6] x [0, 5]``: four rooms, doorways, furniture
        and 10 wall-mounted landmarks. Good start pose: ``SE2(1.0, 1.3, 0.0)`` (living room).

        ::

            y=5 +----------------+----------------------+
                | study  [desk]  |      [   bed   ]      |
            3.2 +----  ----------+ door                 |
                |                |                      |
                | living room    +-------   ------------+ y=2.8
                |                door    kitchen  (tbl) |
                |        (tbl)   |                [ctr] |
                | [sofa]         |                      |
            y=0 +----------------+----------------------+
               x=0             x=3.5                   x=6
        """
        walls = [
            box_segments(0.0, 0.0, 6.0, 5.0),
            # vertical wall between left and right halves, doorways at y 1.2-2.1 and 3.6-4.4
            [[3.5, 0.0, 3.5, 1.2], [3.5, 2.1, 3.5, 3.6], [3.5, 4.4, 3.5, 5.0]],
            # living room | study, doorway at x 1.6-2.5
            [[0.0, 3.2, 1.6, 3.2], [2.5, 3.2, 3.5, 3.2]],
            # kitchen | bedroom, doorway at x 4.5-5.3
            [[3.5, 2.8, 4.5, 2.8], [5.3, 2.8, 6.0, 2.8]],
            # furniture
            box_segments(0.2, 0.2, 1.6, 0.8),  # sofa
            box_segments(0.2, 4.3, 1.4, 4.8),  # desk
            box_segments(4.4, 3.6, 5.8, 4.8),  # bed
            box_segments(5.4, 0.2, 5.8, 2.0),  # kitchen counter
        ]
        circles = [
            [2.7, 0.8, 0.25],  # coffee table
            [4.4, 1.2, 0.35],  # kitchen table
            [3.2, 4.75, 0.15],  # plant
        ]
        landmarks = [
            [0.0, 1.8], [1.0, 3.2], [3.5, 0.6], [2.0, 0.0],  # living room
            [0.0, 4.0], [2.8, 5.0],  # study
            [6.0, 3.5], [4.0, 5.0],  # bedroom
            [6.0, 2.4], [4.5, 0.0],  # kitchen
        ]
        return cls.from_segments(np.vstack(walls), circles, landmarks, list(range(len(landmarks))))

    # --- queries ---------------------------------------------------------------------------------
    @property
    def bounds(self) -> tuple[float, float, float, float]:
        """``(x_min, x_max, y_min, y_max)`` of all obstacles and landmarks."""
        xs = [self.segments[:, 0], self.segments[:, 2], self.landmarks[:, 0],
              self.circles[:, 0] - self.circles[:, 2], self.circles[:, 0] + self.circles[:, 2]]
        ys = [self.segments[:, 1], self.segments[:, 3], self.landmarks[:, 1],
              self.circles[:, 1] - self.circles[:, 2], self.circles[:, 1] + self.circles[:, 2]]
        x, y = np.concatenate(xs), np.concatenate(ys)
        if x.size == 0:
            return (0.0, 0.0, 0.0, 0.0)
        return (float(x.min()), float(x.max()), float(y.min()), float(y.max()))

    def raycast(self, origins: ArrayLike, angles: ArrayLike, max_range: float = np.inf) -> NDArray[np.floating]:
        """Distance along each ray to the first obstacle; ``inf`` if nothing within ``max_range``.

        Args:
            origins: ``(2,)`` one origin for all rays, or ``(N, 2)`` one per ray (e.g. particles).
            angles: ``(N,)`` world-frame ray directions in radians.
            max_range: hits farther than this are reported as ``inf``.
        """
        angles = np.atleast_1d(np.asarray(angles, dtype=float)).reshape(-1)
        origins = np.broadcast_to(np.asarray(origins, dtype=float), (angles.size, 2))
        out = np.empty(angles.size)
        for start in range(0, angles.size, _CHUNK):
            sl = slice(start, start + _CHUNK)
            dirs = np.stack([np.cos(angles[sl]), np.sin(angles[sl])], axis=-1)
            out[sl] = np.minimum(
                _ray_segment_hits(origins[sl], dirs, self.segments),
                _ray_circle_hits(origins[sl], dirs, self.circles),
            )
        out[out > max_range] = np.inf
        return out

    def distance_to_obstacles(self, points: ArrayLike) -> NDArray[np.floating]:
        """Clearance from each ``(N, 2)`` point to the nearest obstacle (0 inside a circle)."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        out = np.empty(len(pts))
        for start in range(0, len(pts), _CHUNK):
            chunk = pts[start : start + _CHUNK]
            out[start : start + _CHUNK] = np.minimum(
                _point_segment_distances(chunk, self.segments),
                _point_circle_distances(chunk, self.circles),
            )
        return out

    def collides(self, x: float, y: float, radius: float) -> bool:
        """True if a disc of ``radius`` centered at ``(x, y)`` touches any obstacle."""
        return bool(self.distance_to_obstacles([x, y])[0] < radius)

    def to_occupancy_grid(self, resolution: float = 0.05, margin: float = 0.5) -> OccupancyGrid:
        """Rasterize obstacles: a cell is occupied if an obstacle passes within half a cell
        diagonal of its center (walls come out 1-2 cells thick and never leak diagonally).
        Everything else is free. ``margin`` pads the world bounds on every side.
        """
        x_min, x_max, y_min, y_max = self.bounds
        extent = (x_min - margin, x_max + margin, y_min - margin, y_max + margin)
        rows, cols = grid_shape(extent, resolution)
        grid = OccupancyGrid(np.full((rows, cols), FREE, dtype=np.int8), resolution, (extent[0], extent[2]))
        r, c = np.mgrid[0:rows, 0:cols]
        cx, cy = grid.cell_to_world(r.ravel(), c.ravel())
        clearance = self.distance_to_obstacles(np.column_stack([cx, cy]))
        grid.data[(clearance <= resolution * np.sqrt(0.5)).reshape(rows, cols)] = OCCUPIED
        return grid

    def with_landmarks(self, landmarks: ArrayLike, ids: Iterable[int] | None = None) -> World:
        """A copy of this world with a different landmark set."""
        return World.from_segments(self.segments, self.circles, landmarks, None if ids is None else list(ids))


# --- vectorized geometry kernels (rays: (N, 2) origins + (N, 2) unit directions) ---------------------
def _cross(a: NDArray[np.floating], b: NDArray[np.floating]) -> NDArray[np.floating]:
    return a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]


def _ray_segment_hits(p: NDArray[np.floating], d: NDArray[np.floating], seg: NDArray[np.floating]) -> NDArray[np.floating]:
    """Solve p + t*d = a + u*(b - a) for every ray/segment pair; nearest t >= 0 with u in [0, 1]."""
    if len(seg) == 0:
        return np.full(len(p), np.inf)
    a, e = seg[:, :2], seg[:, 2:] - seg[:, :2]  # (M, 2)
    w = a[None, :, :] - p[:, None, :]  # (N, M, 2)
    denom = _cross(d[:, None, :], e[None, :, :])  # (N, M)
    parallel = np.abs(denom) < _EPS
    safe = np.where(parallel, 1.0, denom)
    t = _cross(w, e[None, :, :]) / safe
    u = _cross(w, d[:, None, :]) / safe
    t = np.where(~parallel & (t >= 0.0) & (u >= 0.0) & (u <= 1.0), t, np.inf)
    return t.min(axis=1)


def _ray_circle_hits(p: NDArray[np.floating], d: NDArray[np.floating], circ: NDArray[np.floating]) -> NDArray[np.floating]:
    """|p + t*d - c|^2 = r^2 -> t^2 + 2bt + c0 = 0; nearest non-negative root."""
    if len(circ) == 0:
        return np.full(len(p), np.inf)
    f = p[:, None, :] - circ[None, :, :2]  # (N, K, 2)
    b = np.einsum("nkj,nj->nk", f, d)
    c0 = np.einsum("nkj,nkj->nk", f, f) - circ[None, :, 2] ** 2
    disc = b * b - c0
    root = np.sqrt(np.maximum(disc, 0.0))
    t_near, t_far = -b - root, -b + root
    t = np.where(t_near >= 0.0, t_near, t_far)
    t = np.where((disc >= 0.0) & (t >= 0.0), t, np.inf)
    return t.min(axis=1)


def _point_segment_distances(pts: NDArray[np.floating], seg: NDArray[np.floating]) -> NDArray[np.floating]:
    if len(seg) == 0:
        return np.full(len(pts), np.inf)
    a, e = seg[:, :2], seg[:, 2:] - seg[:, :2]
    length2 = np.maximum(np.einsum("mj,mj->m", e, e), _EPS)
    w = pts[:, None, :] - a[None, :, :]  # (N, M, 2)
    u = np.clip(np.einsum("nmj,mj->nm", w, e) / length2, 0.0, 1.0)
    closest = w - u[..., None] * e[None, :, :]
    return np.sqrt(np.einsum("nmj,nmj->nm", closest, closest)).min(axis=1)


def _point_circle_distances(pts: NDArray[np.floating], circ: NDArray[np.floating]) -> NDArray[np.floating]:
    if len(circ) == 0:
        return np.full(len(pts), np.inf)
    centers = np.linalg.norm(pts[:, None, :] - circ[None, :, :2], axis=-1)
    return np.maximum(centers - circ[None, :, 2], 0.0).min(axis=1)
