"""15.05 — grasp planning: generate top-down candidates, filter the impossible, rank the rest.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.05``.
Only the standard library, numpy and scipy are needed.

Inputs come from 15.04 (an ``ObjectPose`` and the cluster's points) and the physics from 15.01
(``epsilon_quality`` of the two antipodal contacts, given here so this exercise stands alone).

The reference implementation lives at ``15-manipulation/code/grasp_planning.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial import ConvexHull

Array = NDArray[np.float64]


# --- given: the 15.01 physics -------------------------------------------------------------------
@dataclass(frozen=True)
class Contact:
    """A planar point contact: position, **inward** unit normal, friction coefficient."""

    position: tuple[float, float]
    normal: tuple[float, float]
    mu: float = 0.5

    @property
    def p(self) -> Array:
        return np.asarray(self.position, dtype=float)

    @property
    def n(self) -> Array:
        n = np.asarray(self.normal, dtype=float)
        return n / float(np.linalg.norm(n))


def epsilon_quality(contacts: list[Contact], com: ArrayLike = (0.0, 0.0), *,
                    length_scale_m: float = 0.05) -> float:
    """Ferrari-Canny epsilon: the radius of the biggest wrench ball inside the grasp wrench space."""
    com = np.asarray(com, dtype=float)
    rows = []
    for c in contacts:
        a = math.atan(c.mu)
        for t in (-a, a):
            rot = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
            f = rot @ c.n
            r = c.p - com
            rows.append([f[0], f[1], (r[0] * f[1] - r[1] * f[0]) / length_scale_m])
    w = np.asarray(rows, dtype=float)
    if len(w) < 4:
        return 0.0
    try:
        hull = ConvexHull(w)
    except Exception:  # noqa: BLE001
        return 0.0
    offsets = hull.equations[:, -1]
    return 0.0 if float(offsets.max()) >= 0.0 else float(-offsets.max())


# --- given: the 15.02 gripper and the 15.04 pose -------------------------------------------------
@dataclass(frozen=True)
class JawGripper:
    name: str
    stroke_m: float
    max_force_n: float
    pad_mu: float = 0.6
    pad_radius_m: float = 0.006
    finger_length_m: float = 0.05


@dataclass(frozen=True)
class ObjectPose:
    centre: tuple[float, float, float]
    yaw: float
    extents: tuple[float, float, float]
    top_z: float
    n_points: int


@dataclass(frozen=True)
class ScoreWeights:
    """Every term is 0..1 and the score is their weighted mean. Tune these, don't hide them."""

    width_margin: float = 0.30      # how much jaw travel is left after closing
    stability: float = 0.30         # epsilon metric of the two contacts (15.01)
    centred: float = 0.20           # how close the closing line passes to the footprint centroid
    clearance: float = 0.20         # room around the pads for the fingers to come down


@dataclass(frozen=True)
class Grasp:
    """A top-down parallel-jaw grasp.

    position:  (x, y, z) of the point midway between the pads at the moment of closing, base frame
    yaw:       rotation about z of the **closing direction** (the direction the pads travel)
    width:     the object's width along that direction, metres
    contacts:  the two footprint points the pads will touch, base frame (x, y)
    score:     0..1, higher is better; ``terms`` explains it
    """

    position: tuple[float, float, float]
    yaw: float
    width: float
    contacts: tuple[tuple[float, float], tuple[float, float]]
    score: float
    terms: dict[str, float] = field(default_factory=dict)

    @property
    def pre_grasp(self) -> tuple[float, float, float]:
        """Where the gripper waits before the final straight-down approach (see 15.07)."""
        return (self.position[0], self.position[1], self.position[2] + 0.08)


# --- implement these -------------------------------------------------------------------------------
def footprint_hull(points: ArrayLike) -> Array:
    """(M, 2) convex hull of a cluster's footprint, counter-clockwise.

    The hull is the right model for a jaw: the pads touch the outside of the object and can never
    enter a concavity. Take the (x, y) columns and return ``xy[ConvexHull(xy).vertices]``.
    """
    raise NotImplementedError("footprint_hull")  # TODO(student)


def width_along(hull: Array, direction: ArrayLike) -> tuple[float, float, float]:
    """(width, min, max) of the hull projected on a unit ``direction``."""
    raise NotImplementedError("width_along")  # TODO(student)


def contact_points(hull: Array, centre: ArrayLike, direction: ArrayLike) -> tuple[Array, Array]:
    """Where the two pads first touch the hull when closing along ``direction`` through ``centre``.

    Take the hull vertex furthest along ``-d`` and the one furthest along ``+d``, then **slide each
    onto the closing line through ``centre``**: with ``perp = (-d_y, d_x)``, replace ``p`` by
    ``p - ((p - centre) . perp) * perp``. Without that step the two pads act along parallel but
    different lines and the epsilon metric is wrong.

    Returns (low-side contact, high-side contact), in that order.
    """
    raise NotImplementedError("contact_points")  # TODO(student)


def hull_normal_at(hull: Array, point: ArrayLike) -> Array:
    """Inward unit normal of the hull edge nearest to ``point`` — the direction a pad pushes.

    For each edge (a, b): project ``point`` onto the segment (clamping the parameter to [0, 1]),
    keep the nearest edge, and return its unit normal flipped so it points **into** the hull
    (test it against the hull's mean point). Getting this sign wrong gives epsilon = 0 everywhere,
    which looks like a physics result and is a bug.
    """
    raise NotImplementedError("hull_normal_at")  # TODO(student)


def grasp_candidates(points: ArrayLike, pose: ObjectPose, gripper: JawGripper, *,
                     table_z: float = 0.0, yaw_step_deg: float = 5.0,
                     obstacles: list[Array] | None = None, mu: float = 0.6,
                     grasp_depth_m: float = 0.015, open_margin_m: float = 0.010,
                     clearance_m: float = 0.008,
                     weights: ScoreWeights = ScoreWeights()) -> list[Grasp]:
    """All feasible top-down grasps of one object cluster, best first.

    Closing height: ``z = max(table_z + 0.004, pose.top_z - grasp_depth_m)``.

    For every yaw from 0 to 180 degrees (exclusive) in ``yaw_step_deg`` steps, with
    ``d = (cos yaw, sin yaw)``:

    * **Filter**: skip the candidate entirely when ``width + open_margin_m > gripper.stroke_m``.
      An impossible grasp is not a low-scoring grasp.
    * ``c1, c2 = contact_points(hull, hull_centre, d)`` and ``mid = (c1 + c2) / 2``.
    * ``width``  term: ``clip((stroke - width) / stroke, 0, 1)``
    * ``stab``   term: ``clip(epsilon / 0.3, 0, 1)`` where epsilon is ``epsilon_quality`` of the two
      contacts expressed **relative to mid** (positions ``c1 - mid`` and ``c2 - mid``), with their
      ``hull_normal_at`` normals, friction ``mu``, ``com=(0, 0)`` and ``length_scale_m=0.05``.
    * ``centre`` term: ``clip(1 - |mid - hull_centre| / 0.02, 0, 1)``
    * ``clear``  term: 1.0 with no obstacles. Otherwise, for each obstacle and each of the two pad
      positions ``c1 - d * clearance_m`` and ``c2 + d * clearance_m`` (just OUTSIDE each contact),
      take the obstacle points whose z is within 0.03 m of the closing height, find the nearest in
      (x, y), and keep the minimum of ``clip(dist / (2 * clearance_m), 0, 1)`` over everything.

    ``score`` is the weighted mean of the four terms; ``terms`` is
    ``{"width": ..., "stab": ..., "centre": ..., "clear": ...}``.

    Return the surviving grasps sorted by score, highest first.
    """
    raise NotImplementedError("grasp_candidates")  # TODO(student)
