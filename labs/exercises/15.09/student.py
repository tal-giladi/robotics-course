"""15.09 — keeping the planner's world model true: identity, allowances, collisions, recovery.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 15.09``.
Only the standard library and numpy are needed — no ROS, no MoveIt.

Four things, and each one is a bug you will otherwise meet at 23:00:

    box_distance       the arithmetic under every collision check
    reconcile          the same physical object keeps the same id across frames
    grasp_allowances   grasping something means touching it; say so, and say it only then
    recovery_plan      an escalation ladder that ends in a state you can plan from

The reference implementation lives at ``15-manipulation/code/planning_scene.py``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum

import numpy as np
from numpy.typing import ArrayLike, NDArray

Array = NDArray[np.float64]


# --- given -----------------------------------------------------------------------------------
@dataclass(frozen=True)
class CollisionBox:
    """An upright box in the base frame, yawed about z. ``object_id`` is its identity."""

    object_id: str
    centre: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0


@dataclass
class PlanningScene:
    """What the planner believes. ``attached`` is the object currently riding on the gripper."""

    objects: dict[str, CollisionBox] = field(default_factory=dict)
    attached: str | None = None

    def add(self, box: CollisionBox) -> None:
        self.objects[box.object_id] = box

    def remove(self, object_id: str) -> None:
        self.objects.pop(object_id, None)
        if self.attached == object_id:
            self.attached = None

    def obstacles(self) -> list[CollisionBox]:
        """Everything to avoid. The attached object is not an obstacle to itself."""
        return [b for i, b in self.objects.items() if i != self.attached]


@dataclass(frozen=True)
class SceneDiff:
    """What to send to ``/planning_scene`` as an ``is_diff=True`` update."""

    added: tuple[CollisionBox, ...] = ()
    moved: tuple[CollisionBox, ...] = ()
    removed: tuple[str, ...] = ()

    @property
    def is_empty(self) -> bool:
        return not (self.added or self.moved or self.removed)


#: Scene objects perception never reports, and that must never be removed for not being reported.
STATIC_IDS: frozenset[str] = frozenset({"table", "wall", "arm_mount"})

#: The gripper as spheres in the TOOL frame (z = approach, x = closing): ((cx, cy, cz), radius).
GRIPPER_SPHERES: tuple[tuple[tuple[float, float, float], float], ...] = (
    ((0.000, 0.0, -0.035), 0.022),
    ((+0.022, 0.0, -0.012), 0.010),
    ((-0.022, 0.0, -0.012), 0.010),
    ((+0.022, 0.0, +0.006), 0.008),
    ((-0.022, 0.0, +0.006), 0.008),
)


class Failure(str, Enum):
    PLAN_FAILED = "plan_failed"
    COLLISION_IN_MOTION = "collision_in_motion"
    EMPTY_GRASP = "empty_grasp"
    DROPPED = "dropped"
    ATTACHED_BUT_EMPTY = "attached_but_empty"
    SCENE_STALE = "scene_stale"
    JOINT_LIMIT = "joint_limit"


@dataclass(frozen=True)
class RecoveryStep:
    name: str
    why: str
    changes_world: bool = False


#: The escalation ladder: cheapest and least destructive first.
LADDER: dict[Failure, tuple[RecoveryStep, ...]] = {
    Failure.PLAN_FAILED: (
        RecoveryStep("replan", "sampling planners are randomised; another seed often succeeds"),
        RecoveryStep("relax_tolerance", "widen the goal tolerance"),
        RecoveryStep("clear_stale_objects", "an object off the table may still be in the scene"),
        RecoveryStep("go_home", "plan from a known-good configuration"),
    ),
    Failure.COLLISION_IN_MOTION: (
        RecoveryStep("stop", "a controller still tracking is a hazard"),
        RecoveryStep("retreat_along_approach", "back out the way you came in", changes_world=True),
        RecoveryStep("reperceive", "you just touched something"),
        RecoveryStep("grow_padding", "twice in the same place means your model is too small"),
    ),
    Failure.EMPTY_GRASP: (
        RecoveryStep("open_gripper", "leave the jaws in a known state"),
        RecoveryStep("retreat_and_reperceive", "the pads moved it", changes_world=True),
        RecoveryStep("next_best_grasp", "15.05's second candidate, not the same one"),
        RecoveryStep("nudge_or_skip", "change the world, or hand it back", changes_world=True),
    ),
    Failure.DROPPED: (
        RecoveryStep("detach_in_scene", "the planner still thinks it is holding a box"),
        RecoveryStep("open_gripper", "a half-closed jaw is geometry you did not model"),
        RecoveryStep("reperceive", "it is on the table again, somewhere new", changes_world=True),
    ),
    Failure.ATTACHED_BUT_EMPTY: (
        RecoveryStep("detach_in_scene", "the scene and the gripper disagree; the gripper is right"),
        RecoveryStep("reperceive", "resynchronise the world model, do not patch it"),
    ),
    Failure.SCENE_STALE: (
        RecoveryStep("clear_world_objects", "remove everything except the attached object"),
        RecoveryStep("reperceive", "rebuild from one frame"),
    ),
    Failure.JOINT_LIMIT: (
        RecoveryStep("stop", "a joint on its stop is a stalled servo and heat (14.11)"),
        RecoveryStep("back_off_joint", "move it 10 deg inside its limit"),
        RecoveryStep("go_home", "re-establish a configuration your IK seeds are valid for"),
        RecoveryStep("abort_to_human", "three limit hits is a workspace problem"),
    ),
}

#: The two steps that must come first when there is an object in the gripper.
HOLDING_PREFIX: tuple[RecoveryStep, ...] = (
    RecoveryStep("place_held_object_safely",
                 "never run a recovery that assumes an empty gripper while holding",
                 changes_world=True),
    RecoveryStep("detach_in_scene", "keep the scene honest about what is held"),
)

#: Failures whose recovery assumes an empty gripper, so they need ``HOLDING_PREFIX``.
NEEDS_EMPTY_GRIPPER: frozenset[Failure] = frozenset({
    Failure.PLAN_FAILED, Failure.COLLISION_IN_MOTION, Failure.JOINT_LIMIT})


def gripper_points(T_base_tool: ArrayLike,
                   spheres: tuple[tuple[tuple[float, float, float], float], ...] = GRIPPER_SPHERES
                   ) -> list[tuple[Array, float]]:
    """(centre in the base frame, radius) for every sphere."""
    T = np.asarray(T_base_tool, dtype=float)
    return [(T[:3, :3] @ np.asarray(c, dtype=float) + T[:3, 3], r) for c, r in spheres]


# --- implement these ---------------------------------------------------------------------------
def box_distance(box: CollisionBox, point: ArrayLike) -> float:
    """Distance from ``point`` to the surface of ``box``; negative inside.

    Rotate the point into the box's own frame first (the box is yawed about z, so rotate by
    ``-box.yaw``), then with ``d = abs(local) - half_size``:

        outside = norm(maximum(d, 0))        distance to the surface when the point is outside
        inside  = min(max(d), 0)             a negative depth when every component is inside

    and the answer is ``outside + inside``. Exactly one of the two terms is non-zero.
    """
    raise NotImplementedError("box_distance")  # TODO(student)


def collides(T_base_tool: ArrayLike, obstacles: list[CollisionBox], *,
             padding_m: float = 0.005, ignore: frozenset[str] = frozenset(),
             spheres: tuple[tuple[tuple[float, float, float], float], ...] = GRIPPER_SPHERES
             ) -> str | None:
    """The ``object_id`` of the first obstacle the gripper is inside, or ``None``.

    For every obstacle not in ``ignore`` and every sphere from ``gripper_points``, the pair is in
    collision when ``box_distance(box, centre) < radius + padding_m``. Return the obstacle's id on
    the first hit — a checker that returns a bool makes every failure undiagnosable.
    """
    raise NotImplementedError("collides")  # TODO(student)


def reconcile(scene: PlanningScene, detections: list[CollisionBox], *,
              match_radius_m: float = 0.04, move_threshold_m: float = 0.003,
              protect_attached: bool = True,
              static_ids: frozenset[str] = STATIC_IDS) -> SceneDiff:
    """Match this frame's detections to the ids already in the scene, and return the diff.

    The ids in ``detections`` come from the clustering and are meaningless — ``object_0`` is a
    different physical block from one frame to the next. Your job is to keep the *scene's* ids.

    Algorithm:

    * Candidate ids = everything in ``scene.objects`` except ``static_ids`` and (when
      ``protect_attached``) the attached object. The table is never detected and must never be
      removed for not being detected; the attached object is in the gripper and likewise.
    * For each detection, take the nearest unclaimed candidate whose centre is within
      ``match_radius_m``. Claim it.
      - No match → the detection is **added**, keeping its own id.
      - Matched → it is **moved** (re-issued with the *scene's* id) only when the centre shifted by
        more than ``move_threshold_m`` or the size changed. Otherwise send nothing: at 10 Hz,
        re-publishing every object every frame is 10 scene updates a second for a still table.
    * Candidates nobody claimed are **removed**.

    Use ``dataclasses.replace(detection, object_id=matched_id)`` to re-issue a moved box.
    """
    raise NotImplementedError("reconcile")  # TODO(student)


def grasp_allowances(target_id: str, table_id: str = "table") -> frozenset[str]:
    """The Allowed Collision Matrix entries needed **for the grasp segment only**.

    Two, and both are physics rather than a workaround: the **target** (grasping something means
    touching it) and the **table** (the pads close a few millimetres above it, which is inside the
    padded table). Return them as a frozenset.
    """
    raise NotImplementedError("grasp_allowances")  # TODO(student)


def recovery_plan(failure: Failure, attempt: int, *, holding: bool = False) -> list[RecoveryStep]:
    """The steps for the ``attempt``-th occurrence of ``failure``, cheapest first.

    Take the first ``attempt + 1`` rungs of ``LADDER[failure]`` (never more than the ladder has),
    so attempt 0 is one cheap step and attempt 2 is three increasingly decisive ones.

    When ``holding`` and the failure is in ``NEEDS_EMPTY_GRIPPER``, prepend ``HOLDING_PREFIX``:
    a recovery that assumes an empty gripper must not run with an object in it.
    """
    raise NotImplementedError("recovery_plan")  # TODO(student)
