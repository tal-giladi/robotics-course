"""Lesson 15.07 — detect an object and put the gripper at a pre-grasp above it.

The whole open-loop chain of module 15 in one file, with the two things that decide whether it
works on a real arm rather than on paper:

  * every waypoint is checked against **IK** before anything moves, and
  * the **error budget** is computed instead of hoped for.

    py reach_pipeline.py                  every table printed in the lesson
    py reach_pipeline.py --only standoff  the pre-grasp height / approach angle grid

Frames follow the rest of the module: base_link is x forward, y left, z up (REP-103); the tool
frame's **z axis is the approach direction** (out of the jaws) and its **x axis is the closing
direction** (the way the pads travel). Everything imports from the earlier lessons:

    tabletop_scene.py   15.04   the synthetic depth camera
    object_pose.py      15.04   depth -> table plane -> clusters -> ObjectPose
    grasp_planning.py   15.05   ObjectPose -> ranked Grasp candidates
    arm_kinematics.py   14.04-14.06  the SO-101 chain, its Jacobian and its IK
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from numpy.typing import ArrayLike, NDArray

_ARM_CODE = Path(__file__).resolve().parents[2] / "14-robotic-arm" / "code"
if str(_ARM_CODE) not in sys.path:
    sys.path.insert(0, str(_ARM_CODE))
import arm_kinematics as ak  # noqa: E402

from grasp_planning import Grasp, plan_scene  # noqa: E402
from grippers import JawGripper, so101_jaw  # noqa: E402
from object_pose import ObjectPose  # noqa: E402
from tabletop_scene import Camera, camera_above, demo_scene, noisy_depth  # noqa: E402

Array = NDArray[np.float64]

# The SO-101's tool frame in the URDF of 14.08. If you glue longer pads on, this is the number
# that changes, and forgetting it is worth a few millimetres of "mysterious" offset.
PAD_OFFSET_M = 0.0


# ============================================================================== the tool pose
def approach_axis(pitch_rad: float, azimuth_rad: float) -> Array:
    """Unit vector the gripper travels ALONG as it closes on the object.

    ``pitch_rad`` is measured below the horizontal, matching ``ak.approach_pitch``: pi/2 is
    straight down, 0 is horizontal. ``azimuth_rad`` is the compass direction the gripper leans
    from — normally ``atan2(y, x)`` of the object, so the arm reaches over its own shoulder.
    """
    c, s = math.cos(pitch_rad), math.sin(pitch_rad)
    return np.array([c * math.cos(azimuth_rad), c * math.sin(azimuth_rad), -s])


def grasp_tool_pose(position: ArrayLike, jaw_yaw: float, *, pitch_rad: float = math.pi / 2,
                    azimuth_rad: float | None = None) -> Array:
    """The 4x4 tool pose that puts the pads at ``position`` closing along ``jaw_yaw``.

    z = the approach axis, x = the closing direction (projected perpendicular to z, because the
    two are only independent when the approach is exactly vertical), y = z x x.

    Raises ValueError when the closing direction is parallel to the approach axis: at pitch 0 and
    a yaw pointing straight at the object there is no valid frame, which is a real geometric fact
    and not an edge case to paper over.
    """
    p = np.asarray(position, dtype=float).reshape(3)
    azimuth = math.atan2(p[1], p[0]) if azimuth_rad is None else azimuth_rad
    z = approach_axis(pitch_rad, azimuth)
    d = np.array([math.cos(jaw_yaw), math.sin(jaw_yaw), 0.0])
    x = d - float(d @ z) * z
    n = float(np.linalg.norm(x))
    if n < 1e-6:
        raise ValueError("closing direction is parallel to the approach axis")
    x = x / n
    y = np.cross(z, x)
    return ak.se3(np.column_stack((x, y, z)), p)


def jaw_yaw_of(T_base_tool: ArrayLike) -> float:
    """The closing direction of the jaws, projected on the table, wrapped to (-pi/2, pi/2].

    An *axis*, not a direction: a parallel jaw closing at theta and theta + pi is the same grasp
    (15.05). Comparing these without wrapping is how a correct arm looks 180 degrees wrong.
    """
    x = np.asarray(T_base_tool, dtype=float)[:3, 0]
    a = math.atan2(x[1], x[0])
    a = (a + math.pi / 2) % math.pi - math.pi / 2
    return a if a > -math.pi / 2 else a + math.pi


# ============================================================================== waypoints
@dataclass(frozen=True)
class Waypoint:
    """One pose on the way to the object, with the reason it exists and how fast to go there."""

    name: str
    T_base_tool: Array
    why: str
    speed_scale: float = 1.0          # fraction of the configured joint speed (14.11)

    @property
    def position(self) -> Array:
        return self.T_base_tool[:3, 3]


def approach_waypoints(T_grasp: ArrayLike, *, standoff_m: float = 0.080, lift_m: float = 0.060,
                       table_z: float = 0.0, clearance_m: float = 0.004) -> list[Waypoint]:
    """The four poses of a top-down reach, in order.

    ``pre_grasp``  standoff_m back along the approach axis — far enough to see the object with a
                   wrist camera, close enough that the descent is short and straight.
    ``approach``   the last 20 mm, at a quarter speed: this is the segment that hits things.
    ``grasp``      pads at the closing height chosen by 15.05.
    ``lift``       straight up, so a grasp that slipped drops the object on the table and not
                   into the next object.

    Raises ValueError if the grasp pose is below the table: a plan that scrapes is not a plan.
    """
    T = np.asarray(T_grasp, dtype=float)
    p, z = T[:3, 3], T[:3, 2]
    if p[2] < table_z + clearance_m:
        raise ValueError(f"grasp is {(table_z - p[2]) * 1000:.1f} mm into the table")
    pre = T.copy()
    pre[:3, 3] = p - z * standoff_m
    mid = T.copy()
    mid[:3, 3] = p - z * 0.020
    lift = T.copy()
    lift[:3, 3] = p + np.array([0.0, 0.0, lift_m])
    return [
        Waypoint("pre_grasp", pre, f"{standoff_m * 1000:.0f} mm back along the approach axis", 1.0),
        Waypoint("approach", mid, "20 mm out, slow: the last segment that can collide", 0.25),
        Waypoint("grasp", T, "pads at the closing height from 15.05", 0.25),
        Waypoint("lift", lift, f"straight up {lift_m * 1000:.0f} mm, before any lateral motion", 0.5),
    ]


# ============================================================================== IK for a grasp
@dataclass
class WaypointIK:
    name: str
    converged: bool
    position_error_m: float
    pitch_error_rad: float
    jaw_yaw_error_rad: float
    q: Array
    reason: str

    @property
    def ok(self) -> bool:
        return (self.converged and self.position_error_m < 0.002
                and abs(self.jaw_yaw_error_rad) < math.radians(10.0))


def align_jaw_roll(chain: ak.SerialChain, q: ArrayLike, target_yaw: float,
                   step_deg: float = 1.0) -> Array:
    """Set the last joint so the jaws close along ``target_yaw``, leaving the others alone.

    ``wrist_roll`` rotates the jaw about the approach axis without changing the approach
    direction at all (check it: ``ak.approach_pitch`` is identical for every roll), so a 1-D scan
    over its range is exact for the angle and only nudges the tool point by a few millimetres —
    which the next IK round cleans up.
    """
    q = np.asarray(q, dtype=float).copy()
    best, best_err = q[-1], 1e9
    lo, hi = chain.lower[-1], chain.upper[-1]
    for roll in np.arange(lo, hi, math.radians(step_deg)):
        q[-1] = roll
        err = abs(_wrap_axis(jaw_yaw_of(chain.fk(q)) - target_yaw))
        if err < best_err:
            best, best_err = float(roll), err
    q[-1] = best
    return q


def _wrap_axis(a: float) -> float:
    a = (a + math.pi / 2) % math.pi - math.pi / 2
    return a if a > -math.pi / 2 else a + math.pi


def ik_for_pose(chain: ak.SerialChain, T_target: ArrayLike, q0: ArrayLike, *,
                name: str = "", rounds: int = 3) -> WaypointIK:
    """IK for a full grasp pose on a 5-DOF arm: position + approach pitch + jaw yaw.

    The SO-101 cannot serve a general 6-DOF pose (14.05, 15.06), so the task is the five numbers
    that a top-down grasp actually needs: (x, y, z, pitch, jaw yaw). Position and pitch go to
    ``ak.ik_dls``; the jaw yaw is the last joint, set by ``align_jaw_roll``. Alternating the two
    converges in two or three rounds because the coupling between them is about 6 mm.
    """
    T = np.asarray(T_target, dtype=float)
    target_pitch = ak.approach_pitch(T)
    target_yaw = jaw_yaw_of(T)
    q = np.asarray(q0, dtype=float).copy()
    res = None
    for _ in range(rounds):
        res = ak.ik_dls(chain, T[:3, 3], q, target_pitch=target_pitch)
        q = align_jaw_roll(chain, res.q, target_yaw)
    res = ak.ik_dls(chain, T[:3, 3], q, target_pitch=target_pitch)
    q = res.q
    yaw_err = _wrap_axis(jaw_yaw_of(chain.fk(q)) - target_yaw)
    return WaypointIK(name or "?", res.converged, res.position_error_m, res.pitch_error_rad,
                      yaw_err, q, res.reason)


# ============================================================================== the plan
@dataclass
class ReachPlan:
    """Everything needed to execute — or to explain a refusal — for one object."""

    object_name: str
    pose: ObjectPose | None
    grasp: Grasp | None
    waypoints: list[Waypoint] = field(default_factory=list)
    ik: list[WaypointIK] = field(default_factory=list)
    refusal: str | None = None

    @property
    def ok(self) -> bool:
        return self.refusal is None and bool(self.ik) and all(s.ok for s in self.ik)

    def report(self) -> str:
        if self.refusal:
            return f"{self.object_name:<14} REFUSED: {self.refusal}"
        lines = [f"{self.object_name:<14} grasp {self.grasp.describe()}"]
        for wp, s in zip(self.waypoints, self.ik, strict=True):
            mark = "ok " if s.ok else "NO "
            lines.append(f"    {mark}{wp.name:<10}"
                         f"({wp.position[0]:.3f}, {wp.position[1]:.3f}, {wp.position[2]:.3f})  "
                         f"pos err {s.position_error_m * 1000:5.2f} mm  "
                         f"pitch err {math.degrees(s.pitch_error_rad):5.2f} deg  "
                         f"jaw err {math.degrees(s.jaw_yaw_error_rad):6.2f} deg  {s.reason}")
        return "\n".join(lines)


def plan_reach(depth: Array, cam: Camera, T_base_optical: Array, gripper: JawGripper, *,
               chain: ak.SerialChain | None = None, q_home: ArrayLike | None = None,
               pitch_deg: float = 90.0, standoff_m: float = 0.080, lift_m: float = 0.060,
               rng: np.random.Generator | None = None,
               names: dict[int, str] | None = None) -> list[ReachPlan]:
    """15.04 + 15.05 + IK: a depth image in, an executable (or refused) plan per object out."""
    chain = chain or ak.load_so101()
    q_home = np.radians([0.0, -30.0, 60.0, 30.0, 0.0]) if q_home is None else np.asarray(q_home)
    plans: list[ReachPlan] = []
    for i, (pose, grasps) in enumerate(plan_scene(depth, cam, T_base_optical, gripper, rng=rng)):
        name = (names or {}).get(i, f"object {i}")
        if not grasps:
            plans.append(ReachPlan(name, pose, None,
                                   refusal=f"no feasible grasp (narrowest footprint "
                                           f"{pose.extents[1] * 1000:.0f} mm, stroke "
                                           f"{gripper.stroke_m * 1000:.0f} mm)"))
            continue
        g = grasps[0]
        try:
            T_grasp = grasp_tool_pose(g.position, g.yaw, pitch_rad=math.radians(pitch_deg))
            wps = approach_waypoints(T_grasp, standoff_m=standoff_m, lift_m=lift_m)
        except ValueError as exc:
            plans.append(ReachPlan(name, pose, g, refusal=str(exc)))
            continue
        q = np.asarray(q_home, dtype=float)
        checks = []
        for wp in wps:
            s = ik_for_pose(chain, wp.T_base_tool, q, name=wp.name)
            checks.append(s)
            q = s.q                                  # seed the next waypoint from this one
        plans.append(ReachPlan(name, pose, g, wps, checks))
    return plans


# ============================================================================== error budget
@dataclass(frozen=True)
class ErrorSources:
    """The four places millimetres come from, with the values 15.03 and 15.04 actually measured."""

    hand_eye_mm: float = 1.5          # 15.03, 15 well-spread poses, realistic noise
    hand_eye_deg: float = 0.35
    object_pose_mm: float = 3.0       # 15.04, centre error at a 60 deg view
    fk_deg: float = 0.30              # 14.04 + servo backlash, per joint
    fk_mm: float = 0.50
    execution_mm: float = 1.0         # what the controller actually lands on (14.08)


def reach_error_budget(sources: ErrorSources = ErrorSources(), *, n: int = 4000,
                       view_distance_m: float = 0.30, chain: ak.SerialChain | None = None,
                       q_view: ArrayLike | None = None,
                       rng: np.random.Generator | None = None) -> dict[str, float]:
    """Monte Carlo: how far from the object do the pads actually close?

    The chain the error travels down, once:

        p_cam  (what the camera measured, with 15.04 noise)
          -> p_base = T_base_tool(q, with FK error) @ X(with calibration error) @ p_cam
          -> the arm is commanded there and lands with its own execution error

    The term people underestimate is the **hand-eye rotation**: 0.35 deg is 1.8 mm at a 300 mm
    view distance, and it is a *lever*, so it grows with how far the camera stands off.
    """
    rng = np.random.default_rng(0) if rng is None else rng
    chain = chain or ak.load_so101()
    q_view = np.radians([5.0, -25.0, 55.0, 35.0, 0.0]) if q_view is None else np.asarray(q_view)

    X_true = _default_hand_eye()
    T_bt_true = chain.fk(np.asarray(q_view, dtype=float))
    T_bc_true = T_bt_true @ X_true
    p_obj = T_bc_true @ np.array([0.0, 0.0, view_distance_m, 1.0])      # object on the optical axis
    p_obj = p_obj[:3]
    p_cam_true = (ak.se3(T_bc_true[:3, :3].T, -T_bc_true[:3, :3].T @ T_bc_true[:3, 3])
                  @ np.append(p_obj, 1.0))[:3]

    out = np.zeros((n, 3))
    for k in range(n):
        p_cam = p_cam_true + rng.normal(0.0, sources.object_pose_mm / 1000.0, 3)
        X = _perturb(X_true, sources.hand_eye_mm, sources.hand_eye_deg, rng)
        dq = rng.normal(0.0, math.radians(sources.fk_deg), chain.n_dof)
        T_bt = chain.fk(np.asarray(q_view, dtype=float) + dq)
        T_bt[:3, 3] += rng.normal(0.0, sources.fk_mm / 1000.0, 3)
        p_hat = (T_bt @ X @ np.append(p_cam, 1.0))[:3]
        landed = p_hat + rng.normal(0.0, sources.execution_mm / 1000.0, 3)
        out[k] = landed - p_obj

    d = np.linalg.norm(out, axis=1) * 1000.0
    lat = np.linalg.norm(out[:, :2], axis=1) * 1000.0
    return {"median_mm": float(np.median(d)), "p90_mm": float(np.percentile(d, 90)),
            "max_mm": float(d.max()), "lateral_mm": float(np.median(lat)),
            "lateral_p90_mm": float(np.percentile(lat, 90))}


def _default_hand_eye() -> Array:
    """Camera 35 mm above the tool point, 30 mm behind it, tilted 20 deg down (15.03's default X)."""
    R = ak.rpy_to_matrix(0.0, math.radians(20.0), 0.0) @ ak.rpy_to_matrix(math.radians(-90), 0, 0)
    return ak.se3(R, (0.0, -0.030, 0.035))


def _perturb(T: Array, sigma_mm: float, sigma_deg: float, rng: np.random.Generator) -> Array:
    out = T.copy()
    out[:3, :3] = T[:3, :3] @ ak.axis_angle_to_matrix(
        *_axis_angle(rng.normal(0.0, math.radians(sigma_deg), 3)))
    out[:3, 3] = T[:3, 3] + rng.normal(0.0, sigma_mm / 1000.0, 3)
    return out


def _axis_angle(v: Array) -> tuple[Array, float]:
    n = float(np.linalg.norm(v))
    return (v / n if n > 1e-12 else np.array([0.0, 0.0, 1.0])), n


# ============================================================================== demos
NAMES = {0: "drinks can", 1: "phone", 2: "wooden block", 3: "eraser", 4: "golf ball"}


def _scene_depth(tilt_deg: float = 75.0, seed: int = 3) -> tuple[Array, Camera, Array]:
    cam = Camera()
    T = camera_above(tilt_deg=tilt_deg)
    return noisy_depth(demo_scene().depth(cam, T), np.random.default_rng(seed)), cam, T


def _named(plans: list[ReachPlan]) -> list[ReachPlan]:
    """Label the plans by the nearest object in the demo scene, for readable output only."""
    for p in plans:
        if p.pose is None:
            continue
        best, best_d = "?", 1e9
        for obj in demo_scene().objects:
            c = obj.centre_xy if hasattr(obj, "centre_xy") else obj.centre[:2]
            d = math.hypot(p.pose.centre[0] - c[0], p.pose.centre[1] - c[1])
            if d < best_d:
                best, best_d = obj.name, d
        p.object_name = best
    return plans


def demo_plan() -> None:
    print("--- the full chain: depth image -> pose -> grasp -> 4 waypoints -> IK ---")
    print("    SO-101 jaw (45 mm), straight-down approach, 80 mm pre-grasp standoff\n")
    depth, cam, T = _scene_depth()
    plans = _named(plan_reach(depth, cam, T, so101_jaw(), rng=np.random.default_rng(3)))
    for p in plans:
        print(p.report())
    ok = sum(p.ok for p in plans)
    print(f"\n  {ok} of {len(plans)} objects have a fully reachable plan.")
    print("  Read the failures: a grasp the planner likes is not a grasp the ARM can make, and the")
    print("  waypoint that fails is the PRE-GRASP, not the grasp. The arm can touch the block; it")
    print("  cannot hover 80 mm above it pointing straight down.\n")

    print("--- the same scene with the approach tilted to 80 degrees ---")
    plans = _named(plan_reach(depth, cam, T, so101_jaw(), pitch_deg=80.0,
                              rng=np.random.default_rng(3)))
    for p in plans:
        if p.refusal:
            continue
        print(p.report())
    ok = sum(p.ok for p in plans)
    print(f"\n  {ok} of {len(plans)} objects now have a fully reachable plan. Ten degrees of tilt")
    print("  bought the whole reach, and cost the grasp a little: the closing direction is no")
    print("  longer exactly the footprint's short axis (15.05 assumed a vertical approach).\n")


def demo_standoff() -> None:
    print("--- pre-grasp standoff x approach pitch: can the arm actually get there? ---")
    print("    (wooden block at (0.237, 0.059), pads closing 26 mm above the table)")
    chain = ak.load_so101()
    q_home = np.radians([0.0, -30.0, 60.0, 30.0, 0.0])
    print(f"{'standoff mm':>12}" + "".join(f"{f'{p} deg':>12}" for p in (90, 80, 70, 60, 45)))
    for standoff_mm in (40, 60, 80, 100, 120):
        row = [f"{standoff_mm:>12}"]
        for pitch in (90, 80, 70, 60, 45):
            T_grasp = grasp_tool_pose((0.237, 0.059, 0.026), math.radians(115.0),
                                      pitch_rad=math.radians(pitch))
            wps = approach_waypoints(T_grasp, standoff_m=standoff_mm / 1000)
            q = q_home
            verdict = "ok"
            for wp in wps:
                s = ik_for_pose(chain, wp.T_base_tool, q, name=wp.name)
                q = s.q
                if not s.ok:
                    verdict = wp.name[:9]
                    break
            row.append(f"{verdict:>12}")
        print("".join(row))
    print("  A straight-down (90 deg) pre-grasp is NOT reachable at any useful standoff: the SO-101")
    print("  cannot put its tool 80-100 mm above a point 240 mm out AND point it at the floor. The")
    print("  fix is not a better IK seed, it is to tilt the approach - which then costs you the")
    print("  'straight down' property that made the grasp simple (15.05).\n")


def demo_budget() -> None:
    print("--- where the millimetres come from (4000 samples, object 300 mm from the camera) ---")
    print(f"{'error sources':<38}{'median mm':>11}{'p90 mm':>9}{'worst mm':>10}{'lateral mm':>12}")
    zero = ErrorSources(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    cases = [
        ("everything perfect", zero),
        ("hand-eye only (1.5 mm, 0.35 deg)", ErrorSources(1.5, 0.35, 0.0, 0.0, 0.0, 0.0)),
        ("  ... its rotation alone (0.35 deg)", ErrorSources(0.0, 0.35, 0.0, 0.0, 0.0, 0.0)),
        ("object pose only (3 mm)", ErrorSources(0.0, 0.0, 3.0, 0.0, 0.0, 0.0)),
        ("FK + backlash only (0.3 deg)", ErrorSources(0.0, 0.0, 0.0, 0.30, 0.5, 0.0)),
        ("execution only (1 mm)", ErrorSources(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)),
        ("all of it (the realistic case)", ErrorSources()),
        ("sloppy: 5 mm / 1 deg hand-eye", ErrorSources(5.0, 1.0, 5.0, 0.5, 1.0, 1.5)),
    ]
    for label, src in cases:
        b = reach_error_budget(src, rng=np.random.default_rng(7))
        print(f"{label:<38}{b['median_mm']:>11.2f}{b['p90_mm']:>9.2f}{b['max_mm']:>10.2f}"
              f"{b['lateral_mm']:>12.2f}")
    print()
    print("  the same budget as the object moves away from the camera (realistic sources):")
    print(f"{'view distance mm':>18}{'median mm':>11}{'p90 mm':>9}")
    for dist_mm in (150, 250, 350, 500):
        b = reach_error_budget(rng=np.random.default_rng(7), view_distance_m=dist_mm / 1000)
        print(f"{dist_mm:>18}{b['median_mm']:>11.2f}{b['p90_mm']:>9.2f}")
    print("  The hand-eye ROTATION is a lever: its contribution grows with the view distance while")
    print("  everything else stays put. Look from closer before you calibrate harder.\n")


def demo_tolerance() -> None:
    print("--- what the budget has to fit inside (SO-101 jaw, 45 mm stroke) ---")
    jaw = so101_jaw()
    for width_mm in (20, 30, 40):
        free = jaw.stroke_m * 1000 - width_mm
        print(f"  object {width_mm:2d} mm wide: {free:4.1f} mm of total free stroke "
              f"-> +/-{free / 2:4.1f} mm of lateral error before a pad touches first")
    b = reach_error_budget(rng=np.random.default_rng(7))
    print(f"\n  measured LATERAL error, realistic sources: median {b['lateral_mm']:.2f} mm, "
          f"p90 {b['lateral_p90_mm']:.2f} mm")
    print("  A 30 mm block leaves +/-7.5 mm of lateral slack. The median fits comfortably; the p90")
    print("  is right at the edge, and a 40 mm object (+/-2.5 mm) does not fit at all. That is the")
    print("  honest reason this lesson exists: open-loop reaching works most of the time, and")
    print("  'most of the time' is what 15.08's verification and retries are for.\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["plan", "standoff", "budget", "tolerance"])
    args = ap.parse_args(argv)
    if args.only in (None, "plan"):
        demo_plan()
    if args.only in (None, "standoff"):
        demo_standoff()
    if args.only in (None, "budget"):
        demo_budget()
    if args.only in (None, "tolerance"):
        demo_tolerance()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
