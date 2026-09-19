"""Lesson 15.05 — generate top-down grasp candidates, score them, rank them.

The heuristic planner the course uses before any learned model: enumerate the closing directions
of a parallel jaw around a vertical approach axis, throw away the ones that don't fit or collide,
and rank the rest with a score whose terms you can argue about one by one.

    py grasp_planning.py                  every table printed in the lesson
    py grasp_planning.py --only clutter

Inputs come from 15.04 (``ObjectPose`` and the cluster's points) and the physics from 15.01
(``epsilon_quality`` of the two antipodal contacts). Nothing here needs a robot.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.spatial import ConvexHull

from grasp_physics import Contact, epsilon_quality
from grippers import JawGripper, so101_jaw, wide_jaw
from object_pose import ObjectPose, cluster_points, fit_plane_ransac, pose_from_cluster
from tabletop_scene import (Camera, camera_above, cluttered_scene, demo_scene, depth_to_points,
                            noisy_depth, transform_points)

Array = NDArray[np.float64]


# ============================================================================== the candidate
@dataclass(frozen=True)
class Grasp:
    """A top-down parallel-jaw grasp.

    position:  (x, y, z) of the point midway between the pads at the moment of closing, base frame
    yaw:       rotation about z of the **closing direction** (the direction the pads travel)
    width:     the object's width along that closing direction, metres
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

    def describe(self) -> str:
        t = " ".join(f"{k}={v:.2f}" for k, v in self.terms.items())
        return (f"({self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}) "
                f"yaw {math.degrees(self.yaw):+6.1f} deg  width {self.width * 1000:5.1f} mm  "
                f"score {self.score:.3f}  [{t}]")


# ============================================================================== footprint geometry
def footprint_hull(points: ArrayLike) -> Array:
    """(M, 2) convex hull of a cluster's footprint, counter-clockwise. The hull is the right model
    for a jaw: the pads touch the outside of the object, never a concavity they cannot enter."""
    xy = np.asarray(points, dtype=float)[:, :2]
    hull = ConvexHull(xy)
    return xy[hull.vertices]


def width_along(hull: Array, direction: ArrayLike) -> tuple[float, float, float]:
    """(width, min, max) of the hull projected on a unit ``direction``."""
    d = np.asarray(direction, dtype=float)
    p = hull @ d
    return float(p.max() - p.min()), float(p.min()), float(p.max())


def contact_points(hull: Array, centre: ArrayLike, direction: ArrayLike) -> tuple[Array, Array]:
    """Where the two pads first touch the hull when closing along ``direction`` through ``centre``.

    Simple and robust: the hull vertex furthest along +d and the one furthest along -d, both
    projected onto the closing line through the centre. Good enough for a convex footprint, and it
    degrades gracefully (a rounded object gives its extreme points, which is what the pads meet).
    """
    d = np.asarray(direction, dtype=float)
    c = np.asarray(centre, dtype=float)
    perp = np.array([-d[1], d[0]])
    proj = hull @ d
    lo, hi = hull[int(np.argmin(proj))], hull[int(np.argmax(proj))]
    # slide each contact onto the closing line so both pads act along the same line
    return (lo - float((lo - c) @ perp) * perp, hi - float((hi - c) @ perp) * perp)


def hull_normal_at(hull: Array, point: ArrayLike) -> Array:
    """Inward unit normal of the hull edge nearest to ``point`` (the direction a pad pushes)."""
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


# ============================================================================== scoring
@dataclass(frozen=True)
class ScoreWeights:
    """Every term is 0..1 and the score is their weighted mean. Tune these, don't hide them."""

    width_margin: float = 0.30      # how much jaw travel is left after closing
    stability: float = 0.30         # epsilon metric of the two contacts (15.01)
    centred: float = 0.20           # how close the closing line passes to the footprint centroid
    clearance: float = 0.20         # room around the pads for the fingers to come down


def grasp_candidates(points: ArrayLike, pose: ObjectPose, gripper: JawGripper, *,
                     table_z: float = 0.0, yaw_step_deg: float = 5.0,
                     obstacles: list[Array] | None = None, mu: float = 0.6,
                     grasp_depth_m: float = 0.015, open_margin_m: float = 0.010,
                     clearance_m: float = 0.008,
                     weights: ScoreWeights = ScoreWeights()) -> list[Grasp]:
    """All feasible top-down grasps of one object cluster, best first.

    ``grasp_depth_m`` is how far below the object's top the pads close — deep enough to get a real
    contact patch, shallow enough not to hit the table. ``open_margin_m`` is how much wider than the
    object the jaws must open to come down around it without brushing it. ``clearance_m`` is the
    room each finger needs beside the object; ``obstacles`` are the other clusters, whose points
    are checked against both pad positions.
    """
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

        # --- term 1: how much travel is left in the jaw
        width_margin = float(np.clip((gripper.stroke_m - width) / gripper.stroke_m, 0.0, 1.0))

        # --- term 2: force closure quality of the two contacts (15.01)
        n1, n2 = hull_normal_at(hull, c1), hull_normal_at(hull, c2)
        eps = epsilon_quality([Contact(tuple(c1 - mid), tuple(n1), mu),
                               Contact(tuple(c2 - mid), tuple(n2), mu)],
                              com=(0.0, 0.0), length_scale_m=0.05)
        stability = float(np.clip(eps / 0.3, 0.0, 1.0))   # 0.3 is force closure on a clean pinch

        # --- term 3: does the closing line pass through the centre of mass? (torque, 15.01)
        offset = float(np.linalg.norm(mid - centre_xy))
        centred = float(np.clip(1.0 - offset / 0.02, 0.0, 1.0))

        # --- term 4: is there room beside the object for the fingers?
        clearance = 1.0
        for obs in obstacles:
            for pad in (c1 - d * clearance_m, c2 + d * clearance_m):   # just OUTSIDE each contact
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


def plan_scene(depth: Array, cam: Camera, T_base_optical: Array, gripper: JawGripper,
               rng: np.random.Generator | None = None, voxel_m: float = 0.010, **kw
               ) -> list[tuple[ObjectPose, list[Grasp]]]:
    """15.04 + 15.05 end to end: depth image in, ranked grasps per object out."""
    pts_optical, _ = depth_to_points(depth, cam)
    pts = transform_points(T_base_optical, pts_optical)
    table = fit_plane_ransac(pts, rng=rng)
    above = pts[table.signed_distance(pts) > 0.006]
    clusters = cluster_points(above, voxel_m=voxel_m)
    out = []
    for i, c in enumerate(clusters):
        pose = pose_from_cluster(c, table)
        obstacles = [o for j, o in enumerate(clusters) if j != i]
        out.append((pose, grasp_candidates(c, pose, gripper, table_z=table.height,
                                           obstacles=obstacles, **kw)))
    return out


# ============================================================================== demos
def _named(pose: ObjectPose) -> str:
    """Nearest object name in the demo scene, for readable output only."""
    best, best_d = "?", 1e9
    for obj in demo_scene().objects:
        c = obj.centre_xy if hasattr(obj, "centre_xy") else obj.centre[:2]
        dist = math.hypot(pose.centre[0] - c[0], pose.centre[1] - c[1])
        if dist < best_d:
            best, best_d = obj.name, dist
    return best


def demo_ranking() -> None:
    cam = Camera()
    T = camera_above(tilt_deg=75.0)
    rng = np.random.default_rng(3)
    depth = noisy_depth(demo_scene().depth(cam, T), rng)
    for gripper in (so101_jaw(), wide_jaw()):
        print(f"--- {gripper.name}: stroke {gripper.stroke_m * 1000:.0f} mm, "
              f"{gripper.max_force_n:.1f} N per pad ---")
        for pose, grasps in plan_scene(depth, cam, T, gripper, rng=np.random.default_rng(3)):
            name = _named(pose)
            if not grasps:
                print(f"{name:<14} no feasible top-down grasp "
                      f"(narrowest footprint {pose.extents[1] * 1000:.0f} mm)")
                continue
            print(f"{name:<14} {len(grasps):2d} candidates, best 2:")
            for g in grasps[:2]:
                print(f"               {g.describe()}")
        print()


def demo_terms() -> None:
    print("--- what each term does: the wooden block (60 x 30 mm), 80 mm jaw so nothing is filtered ---")
    cam = Camera()
    T = camera_above(tilt_deg=75.0)
    rng = np.random.default_rng(3)
    depth = noisy_depth(demo_scene().depth(cam, T), rng)
    for pose, grasps in plan_scene(depth, cam, T, wide_jaw(), rng=np.random.default_rng(3)):
        if _named(pose) != "wooden block":
            continue
        print(f"{'yaw deg':>8}{'width mm':>10}{'width':>8}{'stab':>7}{'centre':>8}{'clear':>7}{'score':>8}")
        for g in sorted(grasps, key=lambda g: g.yaw)[::2]:
            t = g.terms
            print(f"{math.degrees(g.yaw):>8.0f}{g.width * 1000:>10.1f}{t['width']:>8.2f}"
                  f"{t['stab']:>7.2f}{t['centre']:>8.2f}{t['clear']:>7.2f}{g.score:>8.3f}")
    print("  the block's long axis is at 25 deg, so the narrow closing direction is 115 deg.\n")


def demo_clutter() -> None:
    print("--- clutter: two 60 x 30 mm blocks side by side, closing across the gap is blocked ---")
    print("    (voxel 4 mm so the two blocks stay separate clusters even at a 12 mm gap)")
    cam = Camera()
    T = camera_above(tilt_deg=80.0)
    print(f"{'gap mm':>7}{'objects':>9}{'candidates':>12}{'blocked':>9}{'best yaw':>10}{'best score':>12}")
    for gap_mm in (12, 24, 40, 80):
        scene = cluttered_scene(gap_m=gap_mm / 1000)
        rng = np.random.default_rng(5)
        depth = noisy_depth(scene.depth(cam, T), rng)
        plans = plan_scene(depth, cam, T, so101_jaw(), rng=np.random.default_rng(5), voxel_m=0.004)
        all_g = [g for _, gs in plans for g in gs]
        best = max(all_g, key=lambda g: g.score, default=None)
        blocked = sum(1 for g in all_g if g.terms["clear"] < 0.5)
        print(f"{gap_mm:>7}{len(plans):>9}{len(all_g):>12}{blocked:>9}"
              f"{math.degrees(best.yaw) if best else float('nan'):>10.0f}"
              f"{best.score if best else 0.0:>12.3f}")
    print("  the blocks are side by side along y, and the only closing direction the 45 mm jaw can use is\n"
          "  across their 30 mm short axis -- which is along y, straight into the neighbour. At a 12 mm gap\n"
          "  EVERY candidate is blocked and the best score drops from 0.634 to 0.489. There is no better yaw\n"
          "  to pick: closing along x needs 60 mm of stroke. Clutter this tight is not a planning problem,\n"
          "  it is a 'move the neighbour first, or use thinner fingers' problem (15.08).\n")


def demo_friction() -> None:
    print("--- the stability term is the 15.01 epsilon metric: best score vs. pad friction ---")
    cam = Camera()
    T = camera_above(tilt_deg=75.0)
    rng = np.random.default_rng(3)
    depth = noisy_depth(demo_scene().depth(cam, T), rng)
    print(f"{'mu':>6}{'block: best score':>20}{'stab term':>12}")
    for mu in (0.1, 0.25, 0.4, 0.6, 0.9):
        for pose, grasps in plan_scene(depth, cam, T, so101_jaw(), rng=np.random.default_rng(3), mu=mu):
            if _named(pose) != "wooden block" or not grasps:
                continue
            g = grasps[0]
            print(f"{mu:>6.2f}{g.score:>20.3f}{g.terms['stab']:>12.2f}")
    print()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["ranking", "terms", "clutter", "friction"])
    args = ap.parse_args(argv)
    if args.only in (None, "ranking"):
        demo_ranking()
    if args.only in (None, "terms"):
        demo_terms()
    if args.only in (None, "clutter"):
        demo_clutter()
    if args.only in (None, "friction"):
        demo_friction()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
