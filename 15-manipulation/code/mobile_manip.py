"""Lesson 15.10 — an arm on a moving base: where to park, and whether it tips.

Three questions that do not exist when the arm is bolted to a table:

  * **Where should the base stand?** Not "in front of the object" — in the part of the workspace
    where the arm is far from its limits and far from a singularity.
  * **What does the navigation error do?** Nav2 lands within a tolerance; that tolerance becomes a
    position error in the arm's frame, and it is an order of magnitude bigger than 15.07's budget.
  * **Does it tip?** A 0.63 kg arm on a 1.6 kg robot whose support polygon is a triangle — two
    wheels on the axle and one ball caster 100 mm in front of them — is not a rhetorical question.
    The polygon's REAR edge is the axle, so the danger is not the extended arm; it is the stowed
    one, and acceleration.

    py mobile_manip.py                    every table printed in the lesson
    py mobile_manip.py --only stability   the tipping analysis

Everything is computed from the course's own files: ``labs/config/karmel.yaml`` for the base and
``14-robotic-arm/code/data/so101_new_calib.urdf`` for the arm's link masses.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from numpy.typing import ArrayLike, NDArray

_ROOT = Path(__file__).resolve().parents[2]
_ARM_CODE = _ROOT / "14-robotic-arm" / "code"
if str(_ARM_CODE) not in sys.path:
    sys.path.insert(0, str(_ARM_CODE))
import arm_kinematics as ak  # noqa: E402

Array = NDArray[np.float64]
G = 9.81

KARMEL_YAML = _ROOT / "labs" / "config" / "karmel.yaml"
SO101_URDF = _ARM_CODE / "data" / "so101_new_calib.urdf"


# ============================================================================== the robot
def load_karmel() -> dict:
    return yaml.safe_load(KARMEL_YAML.read_text(encoding="utf-8"))


def link_masses(urdf_path: Path = SO101_URDF) -> dict[str, tuple[float, Array]]:
    """{link_name: (mass_kg, com_in_link_frame)} straight out of the URDF's <inertial> blocks."""
    root = ET.parse(urdf_path).getroot()
    out: dict[str, tuple[float, Array]] = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = float(inertial.find("mass").get("value"))
        origin = inertial.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz").split()]) if origin is not None \
            else np.zeros(3)
        out[link.get("name")] = (mass, xyz)
    return out


@dataclass(frozen=True)
class MobileManipulator:
    """A differential-drive base with an arm bolted on at ``mount_xyz`` (base_link frame).

    ``support`` is the support polygon in the base frame — for karmel that is a **triangle**: two
    drive wheels on the axle at x = 0 and one caster 100 mm in front of them. There is nothing
    behind the wheels, which is the single most important fact in this lesson.
    """

    base_mass_kg: float
    base_com: tuple[float, float, float]
    support: tuple[tuple[float, float], ...]
    mount_xyz: tuple[float, float, float] = (0.0, 0.0, 0.070)
    mount_yaw: float = 0.0
    chain: ak.SerialChain = field(default_factory=ak.load_so101)
    masses: dict[str, tuple[float, Array]] = field(default_factory=link_masses)

    @property
    def arm_mass_kg(self) -> float:
        return sum(m for m, _ in self.masses.values())

    def T_base_arm(self) -> Array:
        return ak.se3(ak.rpy_to_matrix(0.0, 0.0, self.mount_yaw), self.mount_xyz)

    def arm_com(self, q: ArrayLike) -> tuple[float, Array]:
        """(total arm mass, its centre of mass in the ROBOT base frame) at configuration q."""
        frames = dict(self.chain.frames(q))
        frames["base_link"] = np.eye(4)
        frames.setdefault("moving_jaw_so101_v1_link", frames["gripper_link"])
        total, moment = 0.0, np.zeros(3)
        T_base_arm = self.T_base_arm()
        for name, (mass, com_local) in self.masses.items():
            T = frames.get(name)
            if T is None:
                continue
            p_arm = T[:3, :3] @ com_local + T[:3, 3]
            p_base = T_base_arm[:3, :3] @ p_arm + T_base_arm[:3, 3]
            total += mass
            moment += mass * p_base
        return total, moment / total

    def com(self, q: ArrayLike, payload_kg: float = 0.0) -> tuple[float, Array]:
        """(total mass, centre of mass in the base frame) including the base and any payload."""
        m_arm, c_arm = self.arm_com(q)
        p_tool = self.T_base_arm() @ np.append(self.chain.fk(q)[:3, 3], 1.0)
        m = self.base_mass_kg + m_arm + payload_kg
        moment = (self.base_mass_kg * np.asarray(self.base_com, dtype=float)
                  + m_arm * c_arm + payload_kg * p_tool[:3])
        return m, moment / m

    def tool_in_base(self, q: ArrayLike) -> Array:
        return (self.T_base_arm() @ np.append(self.chain.fk(q)[:3, 3], 1.0))[:3]


def karmel_with_so101(mount_x: float = 0.0, extra_caster_x: float | None = None,
                      cfg: dict | None = None) -> MobileManipulator:
    """The course robot with the arm mounted ``mount_x`` forward of the wheel axle.

    ``extra_caster_x`` fits a *pair* of extra casters at that x, at the same +/- y as the wheels.
    Negative values put them behind the axle, which is the end karmel has nothing on.
    """
    cfg = cfg or load_karmel()
    half_track = cfg["drive"]["wheel_separation_m"] / 2.0
    caster_x = cfg["chassis"]["caster_offset_x_m"]
    support = [(0.0, +half_track), (0.0, -half_track), (caster_x, 0.0)]
    if extra_caster_x is not None:
        support += [(extra_caster_x, +half_track), (extra_caster_x, -half_track)]
    return MobileManipulator(
        base_mass_kg=cfg["robot"]["mass_kg"],
        base_com=(0.0, 0.0, cfg["chassis"]["height_m"] / 2.0),
        support=tuple(convex_hull_2d(support)),
        mount_xyz=(mount_x, 0.0, cfg["chassis"]["height_m"]),
    )


# ============================================================================== stability
def convex_hull_2d(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Counter-clockwise convex hull (monotone chain). The support polygon is the hull of the
    ground contacts — a wheel inside the hull contributes nothing to stability."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list[tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper: list[tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def stability_margin(support: tuple[tuple[float, float], ...], com_xy: ArrayLike) -> float:
    """Distance from the CoM's ground projection to the nearest edge of the support polygon.

    Positive inside (the robot is stable, by that many metres), negative outside (it tips). This
    is the static criterion of FP.07; it says nothing about what happens when the arm *accelerates*,
    which is why the practical rule is "margin > 30 mm AND move the arm slowly".
    """
    p = np.asarray(com_xy, dtype=float)[:2]
    n = len(support)
    best = float("inf")
    inside = True
    for i in range(n):
        a = np.asarray(support[i], dtype=float)
        b = np.asarray(support[(i + 1) % n], dtype=float)
        edge = b - a
        length = float(np.linalg.norm(edge))
        if length < 1e-12:
            continue
        # counter-clockwise polygon: the inward normal is the left normal of the edge
        normal = np.array([-edge[1], edge[0]]) / length
        signed = float(normal @ (p - a))
        inside &= signed >= 0.0
        best = min(best, abs(signed))
    return best if inside else -best


def reach_forward(chain: ak.SerialChain, distance_m: float, height_m: float = 0.10,
                  q0: ArrayLike | None = None) -> ak.IKResult:
    """The arm reaching straight forward to (distance, 0, height) in its OWN base frame."""
    q0 = np.radians([0.0, -30.0, 60.0, 30.0, 0.0]) if q0 is None else q0
    return ak.ik_dls(chain, (distance_m, 0.0, height_m), q0, target_pitch=math.radians(20.0))


# ============================================================================== base placement
@dataclass(frozen=True)
class BasePlacement:
    x: float
    y: float
    yaw: float
    reachable: bool
    manipulability: float
    position_error_m: float


def object_in_arm_frame(robot: MobileManipulator, base_xy_yaw: tuple[float, float, float],
                        object_map_xyz: ArrayLike) -> Array:
    """Where the object is, in the ARM's base frame, for a given base pose in the map."""
    x, y, yaw = base_xy_yaw
    T_map_base = ak.se3(ak.rpy_to_matrix(0.0, 0.0, yaw), (x, y, 0.0))
    T_map_arm = T_map_base @ robot.T_base_arm()
    return (ak.se3(T_map_arm[:3, :3].T, -T_map_arm[:3, :3].T @ T_map_arm[:3, 3])
            @ np.append(np.asarray(object_map_xyz, dtype=float), 1.0))[:3]


#: Distance from the SO-101's base_link origin to its tool, over 20000 random configurations
#: (``ak.reach_limits_so101``). Anything outside this cannot be reached, and checking it first
#: saves ~0.45 s of futile damped-least-squares per out-of-range candidate.
SO101_REACH_M: tuple[float, float] = (0.05, 0.545)


def evaluate_placement(robot: MobileManipulator, base_xy_yaw: tuple[float, float, float],
                       object_map_xyz: ArrayLike, *, pitch_deg: float = 80.0,
                       q0: ArrayLike | None = None,
                       max_iterations: int = 200) -> BasePlacement:
    """Can the arm reach the object from here, and how comfortably?

    ``manipulability`` is 14.06's sqrt(det(J J^T)) on the position rows: it is small near a
    singularity and near full extension, which is exactly where you do not want to be working.
    """
    p_arm = object_in_arm_frame(robot, base_xy_yaw, object_map_xyz)
    lo, hi = SO101_REACH_M
    if not (lo <= float(np.linalg.norm(p_arm)) <= hi):        # cheap gate before the solver
        return BasePlacement(*base_xy_yaw, False, 0.0, float("inf"))
    q0 = np.radians([0.0, -30.0, 60.0, 30.0, 0.0]) if q0 is None else q0
    res = ak.ik_dls(robot.chain, p_arm, q0, target_pitch=math.radians(pitch_deg),
                    max_iterations=max_iterations)
    w = ak.manipulability(robot.chain.geometric_jacobian(res.q)[:3]) if res.converged else 0.0
    return BasePlacement(base_xy_yaw[0], base_xy_yaw[1], base_xy_yaw[2],
                         res.converged, float(w), res.position_error_m)


def placement_map(robot: MobileManipulator, object_map_xyz: ArrayLike, *,
                  half_extent_m: float = 0.50, step_m: float = 0.05,
                  face_object: bool = True, max_iterations: int = 60) -> list[BasePlacement]:
    """Evaluate a grid of base positions around the object, each facing it (as Nav2 would)."""
    obj = np.asarray(object_map_xyz, dtype=float)
    out: list[BasePlacement] = []
    n = int(round(half_extent_m / step_m))
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = obj[0] + i * step_m, obj[1] + j * step_m
            yaw = math.atan2(obj[1] - y, obj[0] - x) if face_object else 0.0
            if abs(x - obj[0]) < 1e-9 and abs(y - obj[1]) < 1e-9:
                yaw = 0.0
            out.append(evaluate_placement(robot, (x, y, yaw), obj,
                                          max_iterations=max_iterations))
    return out


# ============================================================================== demos
def demo_stability() -> None:
    cfg = load_karmel()
    robot = karmel_with_so101()
    print("--- the support polygon of karmel, from labs/config/karmel.yaml ---")
    print(f"  drive wheels at x = 0.000, y = +/-{cfg['drive']['wheel_separation_m'] / 2:.3f} m")
    print(f"  one caster at x = {cfg['chassis']['caster_offset_x_m']:+.3f} m")
    print(f"  support polygon: {[(round(a, 3), round(b, 3)) for a, b in robot.support]}")
    print(f"  base {robot.base_mass_kg:.2f} kg, arm {robot.arm_mass_kg:.3f} kg "
          f"(sum of the SO-101 URDF link masses)")
    print("  NOTE: the caster is in FRONT, so nothing supports the robot BEHIND the wheel axle.")
    print("  The rear edge of the support polygon IS the line x = 0, and the base's own CoM sits")
    print("  on it. Reaching forward BUYS margin here; stowing the arm and accelerating spends it.\n")

    print("--- tipping margin while reaching forward (arm mounted at the axle, x = 0) ---")
    print(f"{'reach mm':>10}{'tool x mm':>11}{'CoM x mm':>10}{'margin mm':>11}{'verdict':>14}")
    for reach_mm in (150, 200, 250, 300, 350, 400):
        res = reach_forward(robot.chain, reach_mm / 1000)
        if not res.converged:
            print(f"{reach_mm:>10}{'-':>11}{'-':>10}{'-':>11}{'ARM CANNOT':>14}")
            continue
        m, com = robot.com(res.q)
        margin = stability_margin(robot.support, com)
        tool = robot.tool_in_base(res.q)
        print(f"{reach_mm:>10}{tool[0] * 1000:>11.0f}{com[0] * 1000:>10.1f}"
              f"{margin * 1000:>11.1f}{('STABLE' if margin > 0 else 'TIPS'):>14}")
    print("  Two separate limits, and the near one surprises people: at a 20 deg approach pitch")
    print("  the SO-101 cannot reach anything CLOSER than about 250 mm either - it folds up")
    print("  against its own joint limits. The workspace is an annulus, not a disc (see")
    print("  --only placement).\n")

    print("--- where to mount the arm: extended at 300 mm, and stowed ---")
    print(f"{'mount x mm':>12}{'reaching 300 mm':>18}{'arm folded for driving':>24}"
          f"{'folded + rear casters':>24}")
    #: the travel pose: shoulder back to its limit, elbow and wrist folded, tool over the deck
    stow = np.radians([0.0, -100.0, 96.0, -95.0, 0.0])
    for mount_mm in (-80, -40, 0, 40, 80):
        r = karmel_with_so101(mount_x=mount_mm / 1000)
        rc = karmel_with_so101(mount_x=mount_mm / 1000, extra_caster_x=-0.100)
        res = reach_forward(r.chain, 0.300)
        _, com_out = r.com(res.q)
        _, com_in = r.com(stow)
        print(f"{mount_mm:>12}{f'{stability_margin(r.support, com_out) * 1000:+.0f} mm':>18}"
              f"{f'{stability_margin(r.support, com_in) * 1000:+.0f} mm':>24}"
              f"{f'{stability_margin(rc.support, com_in) * 1000:+.0f} mm':>24}")
    print("  The middle column is the one that bites, and it is the OPPOSITE of what a rear-caster")
    print("  base does. Reaching forward is safe on this robot; parking the arm over or behind the")
    print("  axle walks the CoM onto the rear edge, where there is no contact at all. A pair of")
    print("  rear casters - two-shekel parts - is what buys that back.\n")

    print("--- payload, at a 250 mm reach, arm mounted 40 mm behind the axle ---")
    r = karmel_with_so101(mount_x=-0.040)
    res = reach_forward(r.chain, 0.250)
    print(f"{'payload g':>11}{'total kg':>10}{'CoM x mm':>10}{'margin mm':>11}{'verdict':>14}")
    for payload_g in (0, 100, 200, 400, 800, 1500):
        m, com = r.com(res.q, payload_kg=payload_g / 1000)
        margin = stability_margin(r.support, com)
        print(f"{payload_g:>11}{m:>10.2f}{com[0] * 1000:>10.1f}{margin * 1000:>11.1f}"
              f"{('STABLE' if margin > 0 else 'TIPS'):>14}")
    print("  The static margin is not the whole story: accelerating the arm outward adds an")
    print("  inertial term the same sign as gravity. Keep 30 mm of static margin AND move slowly")
    print("  when extended (14.11). If you cannot, the answer is another contact or a longer base.\n")


def demo_placement() -> None:
    print("--- where should the base park? (object at (1.50, 0.40, 0.03) in the map) ---")
    robot = karmel_with_so101(mount_x=-0.040)
    obj = (1.50, 0.40, 0.030)
    places = placement_map(robot, obj, half_extent_m=0.45, step_m=0.05)
    grid = {(round(p.x, 3), round(p.y, 3)): p for p in places}
    xs = sorted({round(p.x, 3) for p in places})
    ys = sorted({round(p.y, 3) for p in places})
    best = max(places, key=lambda p: p.manipulability)
    print("    . unreachable   - reachable, poor manipulability   o good   O best   X object")
    for y in reversed(ys):
        row = []
        for x in xs:
            p = grid[(x, y)]
            if abs(x - obj[0]) < 1e-9 and abs(y - obj[1]) < 1e-9:
                row.append("X")
            elif not p.reachable:
                row.append(".")
            elif p is best:
                row.append("O")
            elif p.manipulability > 0.7 * best.manipulability:
                row.append("o")
            else:
                row.append("-")
        print(f"    y={y:+.2f}  " + " ".join(row))
    print(f"    columns: x from {xs[0]:.2f} m (left) to {xs[-1]:.2f} m (right), step 0.05 m; "
          f"the object is at x = {obj[0]:.2f}")
    n_reach = sum(p.reachable for p in places)
    print(f"\n  {n_reach} of {len(places)} base positions can reach the object at all.")
    d = math.hypot(best.x - obj[0], best.y - obj[1])
    print(f"  best: ({best.x:.2f}, {best.y:.2f}) facing the object, {d * 1000:.0f} mm away, "
          f"manipulability {best.manipulability:.4f}")
    print("  The reachable set is an annulus, not a disc: too close is as bad as too far, because")
    print("  the arm folds up against its own joint limits. Nav2 goals should aim at the MIDDLE of")
    print("  that annulus, not at its edge - the edge is where a 50 mm docking error costs you.\n")


def demo_nav_error() -> None:
    print("--- what the navigation tolerance does to the arm's job (50 dockings each) ---")
    robot = karmel_with_so101(mount_x=-0.040)
    obj = np.array([1.50, 0.40, 0.030])
    nominal = (obj[0] - 0.30, obj[1], 0.0)
    n = 50
    print(f"{'xy tol mm':>10}{'yaw tol deg':>13}{'object err in arm frame mm':>29}"
          f"{'still reachable':>17}")
    for xy_mm, yaw_deg in ((0, 0), (25, 5), (50, 10), (100, 15), (250, 30)):
        rng = np.random.default_rng(7)
        errs, ok = [], 0
        for _ in range(n):
            pose = (nominal[0] + rng.normal(0, xy_mm / 1000 / 2),
                    nominal[1] + rng.normal(0, xy_mm / 1000 / 2),
                    nominal[2] + rng.normal(0, math.radians(yaw_deg) / 2))
            p_actual = object_in_arm_frame(robot, pose, obj)
            p_nominal = object_in_arm_frame(robot, nominal, obj)
            errs.append(float(np.linalg.norm(p_actual - p_nominal)) * 1000)
            ok += int(evaluate_placement(robot, pose, obj, max_iterations=60).reachable)
        print(f"{xy_mm:>10}{yaw_deg:>13}"
              f"{f'{np.median(errs):.0f} med / {np.percentile(errs, 90):.0f} p90':>29}"
              f"{f'{ok / n * 100:.0f}%':>17}")
    print("  Compare the middle column with 15.07's 5 mm reach budget. A 50 mm / 10 deg docking")
    print("  tolerance - a GOOD Nav2 result - puts the object 60-100 mm from where the pre-computed")
    print("  grasp says it is. Navigating and then executing a grasp planned before you moved is")
    print("  not a tuning problem, it is a category error: ALWAYS re-perceive after docking.\n")


def demo_decision() -> None:
    print("--- move the base, or move the arm? (karmel as shipped, 200 g payload) ---")
    robot = karmel_with_so101(mount_x=-0.040)
    obj = np.array([1.50, 0.40, 0.030])
    print(f"{'object at mm':>13}{'reach with arm':>16}{'manipulability':>16}"
          f"{'margin mm':>11}  decision")
    for dist_mm in (200, 250, 300, 350, 400, 450):
        docked = (obj[0] - dist_mm / 1000, obj[1], 0.0)
        p = evaluate_placement(robot, docked, obj, max_iterations=60)
        if not p.reachable:
            print(f"{dist_mm:>13}{'no':>16}{'-':>16}{'-':>11}  re-dock the base")
            continue
        res = ak.ik_dls(robot.chain, object_in_arm_frame(robot, docked, obj),
                        np.radians([0, -30, 60, 30, 0.0]), target_pitch=math.radians(80.0),
                        max_iterations=60)
        _, com = robot.com(res.q, payload_kg=0.2)
        margin = stability_margin(robot.support, com) * 1000
        good = p.manipulability > 0.010 and margin > 30
        print(f"{dist_mm:>13}{'yes':>16}{p.manipulability:>16.4f}{margin:>11.1f}  "
              f"{'use the arm' if good else 're-dock the base'}")
    print("\n  the same 250 mm reach, swung sideways instead of straight ahead:")
    docked = (obj[0] - 0.250, obj[1], 0.0)
    for lateral_mm in (0, 50, 100):
        target = np.array([obj[0], obj[1] + lateral_mm / 1000, obj[2]])
        p = evaluate_placement(robot, docked, target, max_iterations=60)
        res = ak.ik_dls(robot.chain, object_in_arm_frame(robot, docked, target),
                        np.radians([0, -30, 60, 30, 0.0]), target_pitch=math.radians(80.0),
                        max_iterations=60)
        _, com = robot.com(res.q, payload_kg=0.2)
        print(f"    lateral {lateral_mm:>3} mm: manipulability {p.manipulability:.4f}, "
              f"CoM y {com[1] * 1000:+.1f} mm, margin {stability_margin(robot.support, com) * 1000:.1f} mm")
    print("  The arm's first joint is vertical, so panning costs nothing in manipulability - it")
    print("  even helps slightly. On a four-contact base it would cost nothing in margin either.")
    print("  But karmel's polygon narrows towards the caster, so swinging 100 mm to the side")
    print("  throws away 15 mm of margin that the same reach straight ahead keeps.")
    print("  The rule that falls out: use the arm while the manipulability stays healthy AND the")
    print("  stability margin stays above 30 mm; re-dock otherwise. Note that the two constraints")
    print("  close in from OPPOSITE ends - 200 mm fails on manipulability, 300 mm on margin - so")
    print("  the usable band here is narrow. Re-docking costs 5-15 s and a fresh perception")
    print("  cycle, and it is almost always cheaper than a tipped robot.\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["stability", "placement", "nav", "decision"])
    args = ap.parse_args(argv)
    if args.only in (None, "stability"):
        demo_stability()
    if args.only in (None, "placement"):
        demo_placement()
    if args.only in (None, "nav"):
        demo_nav_error()
    if args.only in (None, "decision"):
        demo_decision()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
