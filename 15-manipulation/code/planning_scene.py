"""Lesson 15.09 — keeping the planner's world model true, and recovering when it is not.

Three things that decide whether a manipulation pipeline survives an hour of running:

  * **identity** — the same physical object must keep the same collision-object id across frames,
    or ``attach("object_2")`` attaches the wrong box;
  * **padding** — how much you inflate every shape before checking, and what it costs;
  * **recovery** — an escalation ladder that ends in a state you can plan from.

    py planning_scene.py                  every table printed in the lesson
    py planning_scene.py --only ghost     the forgotten-detach experiment

The collision model is deliberately small: upright boxes for the world, a handful of spheres for
the gripper, and distance checks with padding. It is not MoveIt's FCL, and it is not trying to be
— it is the arithmetic *underneath* the parts of MoveIt this lesson is about
(14.09 covers the ROS side: CollisionObject, is_diff, AttachedCollisionObject, padding).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field, replace
from enum import Enum

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# ============================================================================== the world model
@dataclass(frozen=True)
class CollisionBox:
    """An upright box in the base frame — a table, an object, a wall.

    ``object_id`` is the string MoveIt keys the scene on. It is the *identity* of the thing, and
    keeping it stable across perception frames is the hard part (see ``reconcile``).
    """

    object_id: str
    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0

    def distance_to(self, point: ArrayLike) -> float:
        """Distance from a point to the box surface; negative (approximately) inside."""
        p = np.asarray(point, dtype=float).reshape(3) - np.asarray(self.centre, dtype=float)
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        local = np.array([c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]])
        half = np.asarray(self.size, dtype=float) / 2.0
        d = np.abs(local) - half
        outside = float(np.linalg.norm(np.maximum(d, 0.0)))
        inside = float(min(d.max(), 0.0))
        return outside + inside


@dataclass
class PlanningScene:
    """What the planner believes. ``attached`` is the object currently riding on the gripper."""

    objects: dict[str, CollisionBox] = field(default_factory=dict)
    attached: str | None = None
    #: links allowed to touch the attached object — without these, holding it IS a collision
    touch_links: frozenset[str] = frozenset({"gripper_link", "gripper_frame_link",
                                             "jaw_left", "jaw_right"})

    def add(self, box: CollisionBox) -> None:
        self.objects[box.object_id] = box

    def remove(self, object_id: str) -> None:
        self.objects.pop(object_id, None)
        if self.attached == object_id:
            self.attached = None

    def attach(self, object_id: str) -> None:
        if object_id not in self.objects:
            raise KeyError(f"cannot attach unknown object {object_id!r}")
        self.attached = object_id

    def detach(self, object_id: str) -> None:
        if self.attached == object_id:
            self.attached = None

    def obstacles(self) -> list[CollisionBox]:
        """Everything the gripper must avoid. The attached object is NOT an obstacle to itself."""
        return [b for i, b in self.objects.items() if i != self.attached]


# ============================================================================== identity
@dataclass(frozen=True)
class SceneDiff:
    """What to send to ``/planning_scene`` as an ``is_diff=True`` update."""

    added: tuple[CollisionBox, ...] = ()
    moved: tuple[CollisionBox, ...] = ()
    removed: tuple[str, ...] = ()

    @property
    def is_empty(self) -> bool:
        return not (self.added or self.moved or self.removed)


#: Scene objects that perception never reports and must never be removed for not being reported.
STATIC_IDS: frozenset[str] = frozenset({"table", "wall", "arm_mount"})


def reconcile(scene: PlanningScene, detections: list[CollisionBox], *,
              match_radius_m: float = 0.04, move_threshold_m: float = 0.003,
              protect_attached: bool = True,
              static_ids: frozenset[str] = STATIC_IDS) -> SceneDiff:
    """Match this frame's detections to the ids already in the scene, and return the diff.

    The naive version — clear the scene and add ``object_0 ... object_n`` every frame — is wrong
    in a way that only shows up once something is attached: the ids are assigned by whatever order
    the clustering returned, so ``object_2`` is a different physical block from one frame to the
    next, and ``attach("object_2")`` attaches the wrong box.

    This version matches greedily by nearest centre within ``match_radius_m``, keeps the existing
    id, and only reports a ``moved`` when the centre actually shifted by more than
    ``move_threshold_m`` (otherwise every frame of perception noise is a scene update, and at
    10 Hz that is 10 scene diffs a second for a table that is not moving).

    ``protect_attached`` keeps the object in the gripper out of the reconciliation entirely: it is
    no longer on the table, so perception will not see it, and "not seen" must not mean "removed".
    ``static_ids`` does the same for the table and any fixture — perception never reports them, and
    a reconciler that deletes whatever it did not see will delete the table on the first frame.
    """
    detections = list(detections)
    unmatched_ids = [i for i in scene.objects
                     if i not in static_ids
                     and not (protect_attached and i == scene.attached)]
    added: list[CollisionBox] = []
    moved: list[CollisionBox] = []
    taken: set[str] = set()

    for det in detections:
        best, best_d = None, match_radius_m
        for oid in unmatched_ids:
            if oid in taken:
                continue
            d = float(np.linalg.norm(np.asarray(scene.objects[oid].centre)
                                     - np.asarray(det.centre)))
            if d < best_d:
                best, best_d = oid, d
        if best is None:
            added.append(det)
            continue
        taken.add(best)
        if best_d > move_threshold_m or scene.objects[best].size != det.size:
            moved.append(replace(det, object_id=best))

    removed = tuple(i for i in unmatched_ids if i not in taken)
    return SceneDiff(tuple(added), tuple(moved), removed)


def apply_diff(scene: PlanningScene, diff: SceneDiff) -> None:
    for box in diff.added + diff.moved:
        scene.add(box)
    for oid in diff.removed:
        scene.remove(oid)


# ============================================================================== collision
#: The SO-101 gripper as spheres in the TOOL frame (z = approach, x = closing). Crude on purpose:
#: a real checker uses the meshes, but the arithmetic and the padding argument are identical.
GRIPPER_SPHERES: tuple[tuple[tuple[float, float, float], float], ...] = (
    ((0.000, 0.0, -0.035), 0.022),        # the palm / wrist body
    ((+0.022, 0.0, -0.012), 0.010),       # right finger, upper
    ((-0.022, 0.0, -0.012), 0.010),       # left finger, upper
    ((+0.022, 0.0, +0.006), 0.008),       # right pad
    ((-0.022, 0.0, +0.006), 0.008),       # left pad
)


def held_spheres(size: tuple[float, float, float], n: int = 3
                 ) -> tuple[tuple[tuple[float, float, float], float], ...]:
    """An attached object, as spheres in the TOOL frame: it sits between the pads, at the tool z.

    This is what "attached" means geometrically — the object's collision geometry is rigidly bolted
    to the gripper and travels with every plan. Forget the detach and these spheres never go away.
    """
    long_m, short_m, tall_m = size
    r = max(short_m, 0.006) / 2.0
    offsets = np.linspace(-long_m / 2 + r, long_m / 2 - r, n) if long_m > 2 * r else [0.0]
    # The pads grip ~15 mm below the top, so the object hangs FURTHER along the approach axis.
    z = max(tall_m / 2.0 - 0.015, 0.0)
    return tuple(((0.0, float(o), z), r) for o in offsets)


def gripper_points(T_base_tool: ArrayLike, held_size: tuple[float, float, float] | None = None
                   ) -> list[tuple[Array, float]]:
    """(centre, radius) of every gripper sphere — plus the attached object's, if it is holding one."""
    T = np.asarray(T_base_tool, dtype=float)
    spheres = GRIPPER_SPHERES + (held_spheres(held_size) if held_size else ())
    return [(T[:3, :3] @ np.asarray(c, dtype=float) + T[:3, 3], r) for c, r in spheres]


def collides(T_base_tool: ArrayLike, obstacles: list[CollisionBox], *,
             padding_m: float = 0.005, ignore: frozenset[str] = frozenset(),
             held_size: tuple[float, float, float] | None = None) -> str | None:
    """The id of the first obstacle the gripper is inside, or None.

    ``padding_m`` inflates every obstacle. It is the cheap way to buy margin for the difference
    between your box model and the real object, and unlike a finer check resolution it costs
    nothing at run time (14.09).
    """
    pts = gripper_points(T_base_tool, held_size)
    for box in obstacles:
        if box.object_id in ignore:
            continue
        for centre, radius in pts:
            if box.distance_to(centre) < radius + padding_m:
                return box.object_id
    return None


def grasp_allowances(target_id: str, table_id: str = "table") -> frozenset[str]:
    """The Allowed Collision Matrix entries you must add **for the grasp segment only**.

    Two of them, and both are physics rather than a workaround:

    * the **target**: grasping an object means touching it. A checker that forbids contact with
      the thing you are picking refuses every grasp pose there is.
    * the **table**: the pads close a few millimetres above it (15.05's 4 mm clamp), and with any
      padding at all that is inside the padded table. A 24 mm-tall eraser is un-graspable with
      5 mm of padding until you allow this.

    Add them when the approach starts, **remove them when the lift ends**. An allowance left on is
    the quiet cousin of the forgotten detach: the arm will happily drive through the table later.
    """
    return frozenset({target_id, table_id})


def path_collides(waypoints: list[Array], obstacles: list[CollisionBox], *,
                  padding_m: float = 0.005, steps: int = 20,
                  ignore: frozenset[str] = frozenset(),
                  held_size: tuple[float, float, float] | None = None) -> str | None:
    """Check a straight-line tool path at ``steps`` samples per segment (14.09's resolution)."""
    for a, b in zip(waypoints[:-1], waypoints[1:], strict=True):
        for k in range(steps + 1):
            T = np.asarray(a, dtype=float).copy()
            T[:3, 3] = a[:3, 3] + (b[:3, 3] - a[:3, 3]) * (k / steps)
            hit = collides(T, obstacles, padding_m=padding_m, ignore=ignore, held_size=held_size)
            if hit:
                return hit
    return None


# ============================================================================== recovery
class Failure(str, Enum):
    PLAN_FAILED = "plan_failed"
    COLLISION_IN_MOTION = "collision_in_motion"
    EMPTY_GRASP = "empty_grasp"
    DROPPED = "dropped"
    ATTACHED_BUT_EMPTY = "attached_but_empty"     # the scene says held, the gripper says no
    SCENE_STALE = "scene_stale"
    JOINT_LIMIT = "joint_limit"


@dataclass(frozen=True)
class RecoveryStep:
    name: str
    why: str
    changes_world: bool = False


#: The escalation ladder. Cheapest and least destructive first; every rung ends in a state you can
#: plan from, and the last rung is always a human.
LADDER: dict[Failure, tuple[RecoveryStep, ...]] = {
    Failure.PLAN_FAILED: (
        RecoveryStep("replan", "sampling planners are randomised; a second seed often succeeds"),
        RecoveryStep("relax_tolerance", "widen the goal tolerance to 5 mm / 5 deg"),
        RecoveryStep("clear_stale_objects", "an object removed from the table may still be in the scene"),
        RecoveryStep("go_home", "plan from a known-good configuration instead of wherever you are",
                     changes_world=False),
    ),
    Failure.COLLISION_IN_MOTION: (
        RecoveryStep("stop", "stop before anything else; a controller still tracking is a hazard"),
        RecoveryStep("retreat_along_approach", "back out the way you came in, 50 mm, slowly",
                     changes_world=True),
        RecoveryStep("reperceive", "you just touched something; the table is not what you thought"),
        RecoveryStep("grow_padding", "if it happens twice in the same place, your model is too small"),
    ),
    Failure.EMPTY_GRASP: (
        RecoveryStep("open_gripper", "leave the jaws in a known state before anything else"),
        RecoveryStep("retreat_and_reperceive", "the pads moved the object; re-detect before planning",
                     changes_world=True),
        RecoveryStep("next_best_grasp", "try 15.05's second candidate, not the same one again"),
        RecoveryStep("nudge_or_skip", "change the world, or hand this object back to the caller",
                     changes_world=True),
    ),
    Failure.DROPPED: (
        RecoveryStep("detach_in_scene", "the planner still thinks it is holding a box; fix that FIRST"),
        RecoveryStep("open_gripper", "a half-closed jaw is a collision geometry you did not model"),
        RecoveryStep("reperceive", "the object is on the table again, somewhere new",
                     changes_world=True),
    ),
    Failure.ATTACHED_BUT_EMPTY: (
        RecoveryStep("detach_in_scene", "the scene and the gripper disagree; the gripper is right"),
        RecoveryStep("reperceive", "resynchronise the whole world model, do not patch it"),
    ),
    Failure.SCENE_STALE: (
        RecoveryStep("clear_world_objects", "remove everything except the attached object"),
        RecoveryStep("reperceive", "rebuild from one frame rather than merging into a bad state"),
    ),
    Failure.JOINT_LIMIT: (
        RecoveryStep("stop", "a joint against its stop is a stalled servo and heat (14.11)"),
        RecoveryStep("back_off_joint", "move the offending joint 10 deg inside its limit"),
        RecoveryStep("go_home", "re-establish a configuration your IK seeds are valid for"),
        RecoveryStep("abort_to_human", "three limit hits in a row is a workspace problem, not a bug"),
    ),
}


def recovery_plan(failure: Failure, attempt: int, *, holding: bool = False) -> list[RecoveryStep]:
    """The steps to run for the ``attempt``-th occurrence of ``failure``, cheapest first.

    Each attempt runs one more rung of the ladder than the last, so the first try is cheap and the
    third is decisive. A step that ``changes_world`` invalidates the perception, which is exactly
    the distinction 15.08's retry policy switches on.

    ``holding`` inserts the two steps that must come before anything else when there is an object
    in the gripper: put it down somewhere safe, and tell the scene about it.
    """
    rungs = LADDER[failure]
    steps = list(rungs[:min(attempt + 1, len(rungs))])
    if holding and failure in (Failure.PLAN_FAILED, Failure.COLLISION_IN_MOTION,
                               Failure.JOINT_LIMIT):
        steps = [RecoveryStep("place_held_object_safely",
                              "never run a recovery that assumes an empty gripper while holding",
                              changes_world=True),
                 RecoveryStep("detach_in_scene", "keep the scene honest about what is held")] + steps
    return steps


# ============================================================================== scenes
def table_scene() -> tuple[PlanningScene, list[CollisionBox]]:
    """The 15.04 demo table as collision objects, plus the table itself."""
    scene = PlanningScene()
    scene.add(CollisionBox("table", (0.25, 0.0, -0.010), (0.70, 0.70, 0.020)))
    truth = [
        CollisionBox("block", (0.237, 0.059, 0.020), (0.060, 0.030, 0.040), yaw=math.radians(25)),
        CollisionBox("eraser", (0.188, 0.161, 0.012), (0.045, 0.024, 0.024), yaw=math.radians(-40)),
        CollisionBox("can", (0.299, -0.101, 0.0575), (0.066, 0.066, 0.115)),
        CollisionBox("neighbour", (0.237, 0.101, 0.020), (0.060, 0.030, 0.040)),
    ]
    for b in truth:
        scene.add(b)
    return scene, truth


def tool_pose(position: ArrayLike, yaw: float = 0.0) -> Array:
    """A straight-down tool pose: z = (0, 0, -1), x = the closing direction (15.07)."""
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    T = np.eye(4)
    T[:3, :3] = np.column_stack((x, np.cross(z, x), z))
    T[:3, 3] = np.asarray(position, dtype=float)
    return T


# ============================================================================== demos
def demo_identity() -> None:
    print("--- object identity across frames: why 'clear and re-add' breaks attaching ---")
    rng = np.random.default_rng(3)
    scene, truth = table_scene()
    print(f"{'frame':>6}{'naive id of the block':>26}{'reconciled id':>16}{'diff sent':>36}")
    naive_wrong = 0
    for frame in range(1, 7):
        # perception returns the same objects in a DIFFERENT ORDER each frame, with noise
        order = rng.permutation(len(truth))
        dets = [replace(truth[i], object_id=f"object_{k}",
                        centre=(truth[i].centre[0] + rng.normal(0, 0.002),
                                truth[i].centre[1] + rng.normal(0, 0.002),
                                truth[i].centre[2]))
                for k, i in enumerate(order)]
        naive_id = next(d.object_id for d, i in zip(dets, order, strict=True)
                        if truth[i].object_id == "block")
        naive_wrong += int(naive_id != "object_0")
        diff = reconcile(scene, dets)
        apply_diff(scene, diff)
        matched = min(scene.objects.values(),
                      key=lambda b: float(np.linalg.norm(np.asarray(b.centre)
                                                         - np.asarray(truth[0].centre))))
        summary = (f"+{len(diff.added)} ~{len(diff.moved)} -{len(diff.removed)}")
        print(f"{frame:>6}{naive_id:>26}{matched.object_id:>16}{summary:>36}")
    print("  The naive id of the SAME physical block changes whenever the clustering order does.")
    print("  Attach 'object_2' on one frame and the next frame's 'object_2' is a different block:")
    print("  the planner then avoids the box you are holding and holds the box you meant to avoid.\n")


def demo_attached() -> None:
    print("--- attaching: three states of the same gripper at the same pose ---")
    scene, _ = table_scene()
    T = tool_pose((0.237, 0.059, 0.026), math.radians(115))
    print(f"{'scene state':<40}{'collides with':>16}")
    print(f"{'block in the world, gripper on it':<40}"
          f"{str(collides(T, scene.obstacles())):>16}")
    scene.attach("block")
    print(f"{'block ATTACHED to the gripper':<40}"
          f"{str(collides(T, scene.obstacles())):>16}")
    scene.detach("block")
    scene.remove("block")
    print(f"{'block removed from the scene entirely':<40}"
          f"{str(collides(T, scene.obstacles())):>16}")
    print("  Attaching is not cosmetic: while the block is a world object, every pose that grasps")
    print("  it is 'in collision' and the planner will refuse to go there. Removing it works too")
    print("  and is WRONG - the planner then drives the held block through the can (14.09).")
    print()
    print("  --- the other half: allowed collisions during the grasp (5 mm padding) ---")
    print(f"{'object (height)':<24}{'no allowances':>16}{'grasp allowances':>19}")
    for oid in ("eraser", "block", "can"):
        sc, _ = table_scene()
        obj = sc.objects[oid]
        z = obj.centre[2] + obj.size[2] / 2 - 0.015
        T = tool_pose((obj.centre[0], obj.centre[1], z), obj.yaw + math.pi / 2)
        plain = collides(T, sc.obstacles(), padding_m=0.005) or "clear"
        allowed = collides(T, sc.obstacles(), padding_m=0.005,
                           ignore=grasp_allowances(oid)) or "clear"
        print(f"{f'{oid} ({obj.size[2] * 1000:.0f} mm tall)':<24}{plain:>16}{allowed:>19}")
    print("  Without allowances EVERY grasp is a collision with the object you are grasping, and")
    print("  a short object is additionally a collision with the padded table. Add both for the")
    print("  approach-and-lift segment only, and REMOVE them afterwards - an allowance left on is")
    print("  the quiet cousin of the forgotten detach.\n")


def demo_ghost() -> None:
    print("--- the forgotten detach: a ghost object bolted to the wrist, forever ---")
    print("    the block was placed elsewhere; the question is what the gripper can still do")
    block_size = (0.060, 0.030, 0.040)
    table = [CollisionBox("table", (0.25, 0.0, -0.010), (0.70, 0.70, 0.020))]

    def lowest_z(held) -> float:
        for z_mm in range(0, 120):
            if collides(tool_pose((0.25, 0.0, z_mm / 1000)), table, padding_m=0.005,
                        held_size=held) is None:
                return z_mm / 1000
        return float("nan")

    print(f"{'what the gripper can still do':<44}{'empty':>10}{'ghost':>10}")
    print(f"{'lowest tool height over a bare table':<44}"
          f"{f'{lowest_z(None) * 1000:.0f} mm':>10}{f'{lowest_z(block_size) * 1000:.0f} mm':>10}")
    print()
    print("  and therefore, for the next object (grasp height = top - 15 mm, table NOT allowed):")
    print(f"{'object height':>16}{'grasp height':>15}{'empty':>10}{'ghost':>10}")
    for h_mm in (15, 20, 30, 40, 60):
        z = h_mm / 1000 - 0.015
        box = CollisionBox("target", (0.25, 0.0, h_mm / 2000), (0.040, 0.030, h_mm / 1000))
        verdicts = []
        for held in (None, block_size):
            hit = collides(tool_pose((0.25, 0.0, z)), table + [box], padding_m=0.005,
                           held_size=held, ignore=frozenset({"target"}))
            verdicts.append("clear" if hit is None else hit)
        print(f"{f'{h_mm} mm':>16}{f'{z * 1000:.0f} mm':>15}{verdicts[0]:>10}{verdicts[1]:>10}")
    print("  A ghost is not a box left where the object used to be. It is 60 x 30 x 40 mm of")
    print("  collision geometry rigidly attached to the TOOL FRAME, travelling with every plan for")
    print("  the rest of the session: it hangs below the pads, so it raises the lowest height the")
    print("  gripper can reach from 20 to 26 mm and moves the smallest graspable object from 40 mm")
    print("  to 60 mm. The symptom is 'the planner has started refusing things it used to do', it")
    print("  gets worse the lower you reach, and it survives restarting YOUR node but not")
    print("  move_group - which is the clue that the state does not live in your node.\n")


def demo_padding() -> None:
    print("--- padding: margin you buy, and reach you lose (approach beside a neighbour) ---")
    scene, _ = table_scene()
    obstacles = [b for b in scene.obstacles() if b.object_id != "block"]
    print(f"{'padding mm':>11}{'gap 12 mm':>12}{'gap 24 mm':>12}{'gap 40 mm':>12}{'gap 80 mm':>12}")
    for pad_mm in (0, 2, 5, 10, 20):
        row = [f"{pad_mm:>11}"]
        for gap_mm in (12, 24, 40, 80):
            sc, _ = table_scene()
            sc.remove("neighbour")
            y = 0.059 + 0.030 + gap_mm / 1000.0
            sc.add(CollisionBox("neighbour", (0.237, y, 0.020), (0.060, 0.030, 0.040)))
            obs = [b for b in sc.obstacles() if b.object_id != "block"]
            path = [tool_pose((0.237, 0.059, 0.106), math.radians(115)),
                    tool_pose((0.237, 0.059, 0.026), math.radians(115))]
            hit = path_collides(path, obs, padding_m=pad_mm / 1000.0)
            row.append(f"{(hit or 'clear'):>12}")
        print("".join(row))
    print("  0 mm of padding says every gap is fine, which is a claim about a box model you drew")
    print("  from a noisy point cloud (15.04). 20 mm refuses grasps that would have worked. 5 mm is")
    print("  the course default and it refuses exactly the 12 mm gap that 15.05 also refuses - two")
    print("  independent checks agreeing is the sign you want, not a coincidence.\n")


def demo_recovery() -> None:
    print("--- the escalation ladder: one more rung per attempt ---")
    for failure in (Failure.EMPTY_GRASP, Failure.COLLISION_IN_MOTION, Failure.PLAN_FAILED):
        print(f"  {failure.value}:")
        for attempt in range(3):
            steps = recovery_plan(failure, attempt)
            marks = ", ".join(s.name + ("*" if s.changes_world else "") for s in steps)
            print(f"    attempt {attempt}: {marks}")
        print()
    print("  while HOLDING an object, plan_failed gets two steps prepended:")
    for s in recovery_plan(Failure.PLAN_FAILED, 0, holding=True):
        print(f"    {s.name:<28}{s.why}")
    print("\n  (* = the step changes the world, so the perception is stale afterwards: 15.08's")
    print("   retry policy must route through REPERCEIVE, not RETRY.)\n")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", choices=["identity", "attached", "ghost", "padding", "recovery"])
    args = ap.parse_args(argv)
    if args.only in (None, "identity"):
        demo_identity()
    if args.only in (None, "attached"):
        demo_attached()
    if args.only in (None, "ghost"):
        demo_ghost()
    if args.only in (None, "padding"):
        demo_padding()
    if args.only in (None, "recovery"):
        demo_recovery()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
