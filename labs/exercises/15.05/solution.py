"""15.05 — reference solution. Same public names as student.py.

Mirrors ``15-manipulation/code/grasp_planning.py``.

Inputs come from 15.04 (an ``ObjectPose`` and the cluster's points) and the physics from 15.01
(``epsilon_quality`` of the two antipodal contacts, given here so this exercise stands alone).

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


# --- implemented ---------------------------------------------------------------------------------
def footprint_hull(points: ArrayLike) -> Array:
    """(M, 2) convex hull of a cluster's footprint, counter-clockwise."""
    xy = np.asarray(points, dtype=float)[:, :2]
    hull = ConvexHull(xy)
    return xy[hull.vertices]


def width_along(hull: Array, direction: ArrayLike) -> tuple[float, float, float]:
    """(width, min, max) of the hull projected on a unit ``direction``."""
    d = np.asarray(direction, dtype=float)
    p = hull @ d
    return float(p.max() - p.min()), float(p.min()), float(p.max())


def contact_points(hull: Array, centre: ArrayLike, direction: ArrayLike) -> tuple[Array, Array]:
    """Where the two pads first touch the hull when closing along ``direction`` through ``centre``."""
    d = np.asarray(direction, dtype=float)
    c = np.asarray(centre, dtype=float)
    perp = np.array([-d[1], d[0]])
    proj = hull @ d
    lo, hi = hull[int(np.argmin(proj))], hull[int(np.argmax(proj))]
    return (lo - float((lo - c) @ perp) * perp, hi - float((hi - c) @ perp) * perp)


def hull_normal_at(hull: Array, point: ArrayLike) -> Array:
    """Inward unit normal of the hull edge nearest to ``point``."""
    p = np.asarray(point, dtype=float)
    centre = hull.mean(axis=0)
    best, best_d = None, 1e9
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        ab = b - a
        t = float(np.clip((p - a) @ ab / max(float(ab @ ab), 1e-12), 0.0, 1.0))
        dist = float(np.linalg.norm(p - (a + t * ab)))
        if dist < best_d:
            n = np.array([ab[1], -ab[0]], dtype=float)
            n = n / max(float(np.linalg.norm(n)), 1e-12)
            if float(n @ (centre - (a + t * ab))) < 0:
                n = -n                                   # make it point into the object
            best, best_d = n, dist
    return best


def grasp_candidates(points: ArrayLike, pose: ObjectPose, gripper: JawGripper, *,
                     table_z: float = 0.0, yaw_step_deg: float = 5.0,
                     obstacles: list[Array] | None = None, mu: float = 0.6,
                     grasp_depth_m: float = 0.015, open_margin_m: float = 0.010,
                     clearance_m: float = 0.008,
                     weights: ScoreWeights = ScoreWeights()) -> list[Grasp]:
    """All feasible top-down grasps of one object cluster, best first."""
    pts = np.asarray(points, dtype=float)
    hull = footprint_hull(pts)
    centre_xy = hull.mean(axis=0)
    z = max(table_z + 0.004, pose.top_z - grasp_depth_m)
    obstacles = [] if obstacles is None else [np.asarray(o, dtype=float) for o in obstacles]

    out: list[Grasp] = []
    for deg in np.arange(0.0, 180.0, yaw_step_deg):
        yaw = math.radians(float(deg))
        d = np.array([math.cos(yaw), math.sin(yaw)])
        width, _, _ = width_along(hull, d)
        if width + open_margin_m > gripper.stroke_m:
            continue                                     # the jaws cannot open that far

        c1, c2 = contact_points(hull, centre_xy, d)
        mid = (c1 + c2) / 2.0

        width_margin = float(np.clip((gripper.stroke_m - width) / gripper.stroke_m, 0.0, 1.0))

        n1, n2 = hull_normal_at(hull, c1), hull_normal_at(hull, c2)
        eps = epsilon_quality([Contact(tuple(c1 - mid), tuple(n1), mu),
                               Contact(tuple(c2 - mid), tuple(n2), mu)],
                              com=(0.0, 0.0), length_scale_m=0.05)
        stability = float(np.clip(eps / 0.3, 0.0, 1.0))

        offset = float(np.linalg.norm(mid - centre_xy))
        centred = float(np.clip(1.0 - offset / 0.02, 0.0, 1.0))

        clearance = 1.0
        for obs in obstacles:
            for pad in (c1 - d * clearance_m, c2 + d * clearance_m):
                near = obs[np.abs(obs[:, 2] - z) < 0.03][:, :2] if len(obs) else obs
                if len(near) == 0:
                    continue
                dist = float(np.min(np.linalg.norm(near - pad, axis=1)))
                clearance = min(clearance, float(np.clip(dist / (2 * clearance_m), 0.0, 1.0)))

        terms = {"width": width_margin, "stab": stability, "centre": centred, "clear": clearance}
        w = weights
        total = (w.width_margin * width_margin + w.stability * stability +
                 w.centred * centred + w.clearance * clearance)
        total /= (w.width_margin + w.stability + w.centred + w.clearance)
        out.append(Grasp((float(mid[0]), float(mid[1]), float(z)), yaw, width,
                         (tuple(c1), tuple(c2)), float(total), terms))
    return sorted(out, key=lambda g: g.score, reverse=True)
