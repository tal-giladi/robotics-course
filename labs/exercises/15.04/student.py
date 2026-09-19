"""15.04 — tabletop perception: RANSAC plane, voxel clustering, and a graspable pose.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.04``.
Only the standard library and numpy are needed.

The pipeline, in the order the code runs it:

    points (base frame) -> RANSAC table plane -> remove the table -> cluster -> PCA -> ObjectPose

Frames: the table is roughly the plane z = 0, x forward, y left, z up (REP-103). Units: metres.

The reference implementation lives at ``15-manipulation/code/object_pose.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given ---------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Plane:
    """n . p + d = 0 with a **unit** normal. ``signed_distance`` is positive on the normal's side."""

    normal: tuple[float, float, float]
    d: float
    n_inliers: int = 0

    def signed_distance(self, pts: ArrayLike) -> Array:
        return np.asarray(pts, dtype=float) @ np.asarray(self.normal, dtype=float) + self.d

    @property
    def height(self) -> float:
        """Height of the plane above the origin along its own normal (z of the table, if level)."""
        return -self.d / self.normal[2] if abs(self.normal[2]) > 1e-9 else float("nan")


@dataclass(frozen=True)
class ObjectPose:
    """A graspable pose.

    centre:   (x, y, z) of the *visible* bounding box centre, in the base frame
    yaw:      rotation of the long footprint axis about z, wrapped to (-pi/2, pi/2]
    extents:  (long, short, height) in metres — long/short are the footprint axes
    top_z:    height of the highest point above the origin (table height + object height)
    n_points: how many points the estimate is based on
    """

    centre: tuple[float, float, float]
    yaw: float
    extents: tuple[float, float, float]
    top_z: float
    n_points: int

    @property
    def grasp_width(self) -> float:
        """What the jaws must span for a top-down grasp across the short axis."""
        return self.extents[1]


# --- implement these -------------------------------------------------------------------------------
def fit_plane_ransac(points: ArrayLike, *, threshold_m: float = 0.006, iterations: int = 200,
                     rng: np.random.Generator | None = None, sample_size: int = 20_000,
                     refit_iterations: int = 2, up_hint: ArrayLike | None = (0.0, 0.0, 1.0),
                     max_tilt_deg: float = 25.0) -> Plane:
    """RANSAC plane fit, then a least-squares refit on the inliers.

    Algorithm:

    1. If there are more than ``sample_size`` points, score candidates on a random subset of that
       size (the winner is the same and it is an order of magnitude faster).
    2. ``iterations`` times: draw 3 distinct points, take ``n = cross(b - a, c - a)`` normalised
       (skip near-collinear triples), reject the candidate when ``up_hint`` is given and
       ``|n . up| < cos(max_tilt_deg)``, set ``d = -n . a``, and count inliers within
       ``threshold_m``. Keep the best.
    3. Refit ``refit_iterations`` times on the **full** point set: take the current inliers, centre
       them, and use the last right-singular vector of ``svd(inliers - centroid)`` as the new
       normal (the direction of least variance). Flip it to agree with ``up_hint``, recompute
       ``d = -n . centroid``, and re-select the inliers.

    The refit matters: the RANSAC normal comes from 3 noisy points, this one from all of them.
    Raise ``RuntimeError`` if no candidate plane was accepted at all.

    Return a ``Plane`` with a unit normal and ``n_inliers`` set to the final inlier count.
    """
    raise NotImplementedError("fit_plane_ransac")  # TODO(student)


def cluster_points(points: ArrayLike, *, voxel_m: float = 0.010, min_points: int = 150) -> list[Array]:
    """Voxel-grid connected components: the poor man's DBSCAN, and fast enough on a Pi.

    Drop each point into a ``voxel_m`` grid (``floor(p / voxel_m)``), then flood-fill occupied
    voxels that touch in the 26-neighbourhood. Each connected set of voxels is one cluster.

    Drop clusters with fewer than ``min_points`` points. Return the survivors **sorted by size,
    largest first**. An empty input returns an empty list.
    """
    raise NotImplementedError("cluster_points")  # TODO(student)


def wrap_axis_angle(a: float) -> float:
    """Wrap an *axis* angle (not a direction) to (-pi/2, pi/2].

    PCA cannot tell an axis from its opposite, so ``a`` and ``a + pi`` must map to the same value.
    Note the half-open interval: +pi/2 maps to +pi/2, and -pi/2 must also map to **+pi/2**.
    """
    raise NotImplementedError("wrap_axis_angle")  # TODO(student)


def pose_from_cluster(points: ArrayLike, table: Plane) -> ObjectPose:
    """Principal-axis pose of one cluster, assuming the object stands on ``table``.

    1. Heights above the table: ``table.signed_distance(points)``.
    2. PCA on the **footprint** (x, y only — a top-down grasp does not care about z): centre the
       xy points, form the 2x2 covariance ``xy.T @ xy / len(xy)``, and take ``np.linalg.eigh``.
       The eigenvector with the larger eigenvalue is the long axis; ``yaw`` is
       ``wrap_axis_angle(atan2(v_y, v_x))``.
    3. Extents: rotate the centred footprint into the yaw frame and take max - min along each axis.
       **Not** the eigenvalues — those are variances, and a variance is not a width.
    4. ``centre`` is the midpoint of that oriented bounding box, rotated back into the base frame,
       at height ``table.height + top / 2`` where ``top = max(heights)``.
    5. ``top_z = table.height + top``; ``n_points = len(points)``.

    Raise ``ValueError`` for fewer than 3 points.
    """
    raise NotImplementedError("pose_from_cluster")  # TODO(student)


def segment_tabletop(points: ArrayLike, *, plane_threshold_m: float = 0.006, voxel_m: float = 0.010,
                     min_points: int = 150, min_height_m: float = 0.005,
                     rng: np.random.Generator | None = None) -> tuple[Plane, list[ObjectPose]]:
    """The whole pipeline: base-frame points -> (table plane, one ObjectPose per object).

    Keep the points whose signed distance above the plane exceeds
    ``max(plane_threshold_m, min_height_m)``, cluster them, and build a pose per cluster.
    """
    raise NotImplementedError("segment_tabletop")  # TODO(student)
