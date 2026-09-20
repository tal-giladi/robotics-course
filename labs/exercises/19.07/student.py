"""19.07 — verify every step: was the postcondition actually established?

The dataclass, the constants and the helpers are given. You write the five functions that decide
whether the world changed and what to do when it did not.

The rule that makes this exercise worth doing: a verifier must use a **different channel** from the
one that did the work. ``pick`` returning SUCCEEDED is the arm's opinion of itself;
``get_robot_state`` is proprioception; ``detect_objects`` is the camera. They disagree, and the
disagreement is the information.

Fill in every ``TODO(student)``; check with ``python course.py check 19.07``.
"""

from __future__ import annotations

import math
import sys
from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "19-llm-robot-agents" / "code", _REPO / "labs" / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from robot_agent.skills import RobotSkills, SkillResult  # noqa: E402

JsonDict = dict[str, Any]

PLACE_TOLERANCE_M = 0.30
MIN_CONFIDENCE = 0.4
#: Failure codes meaning "the observation this step was planned from is no longer true".
INVALIDATES_BINDING = frozenset({"GRASP_FAILED", "OBJECT_NOT_FOUND"})


# ----------------------------------------------------------------------------------------------
# Given
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class Verification:
    skill: str
    ok: bool
    code: str  # VERIFIED | NOT_VERIFIED | UNCHECKABLE
    method: str  # "result" | "state" | "perception" | "-"
    message: str = ""
    observed: JsonDict = field(default_factory=dict)
    cost_s: float = 0.0
    repeatable: bool = True


def label_matches(label: str, wanted: str) -> bool:
    label, wanted = label.lower(), wanted.lower()
    return wanted in label or label in wanted or bool(set(wanted.split()) & set(label.split()))


def read_state(skills: RobotSkills) -> JsonDict:
    """Proprioception. Costs 0 simulated seconds — and it can be wrong."""
    return skills.call("get_robot_state").data


def look(skills: RobotSkills) -> list[JsonDict]:
    """Exteroception. Costs 0.2 s — and it misses things."""
    r = skills.call("detect_objects", {"min_confidence": MIN_CONFIDENCE})
    return list(r.data.get("detections", [])) if r.ok else []


# ----------------------------------------------------------------------------------------------
# Yours
# ----------------------------------------------------------------------------------------------
def verify_navigate(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult) -> Verification:
    """Did the robot arrive? Use ``read_state``, never the return code.

    * ``args["place"]`` is not in ``skills.world.places`` -> NOT_VERIFIED, method ``"state"``,
      message ``f"'{place}' is not on the map"``.
    * otherwise compute the distance from ``state["pose"]`` (``x_m``, ``y_m``) to that ``Place``
      (``.x``, ``.y``). Verified when ``state["at_place"] == place`` **or** the distance is within
      ``PLACE_TOLERANCE_M``.
    * ``observed`` must carry ``{"at_place": ..., "distance_m": round(distance, 2)}``, and the
      message must contain both, e.g. ``f"at_place={at!r}, {distance:.2f} m from '{place}'"``.
    * ``cost_s`` is 0.0: a state read does not move the robot.
    """
    # TODO(student): implement.
    raise NotImplementedError("verify_navigate")


def verify_pick(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                level: str = "perception") -> Verification:
    """Is the object actually in the gripper?

    1. ``read_state``. If ``holding`` is not ``args["object_id"]`` -> NOT_VERIFIED, method
       ``"state"``, ``observed={"holding": holding}``. (This catches a gripper with no force
       sensor: the skill said SUCCEEDED and the hand is empty.)
    2. If ``level != "perception"``, stop here and return VERIFIED, method ``"state"``.
    3. Otherwise ``look``. A held object cannot still be standing in the scene, so a detection of
       that ``object_id`` -> NOT_VERIFIED, method ``"perception"``, ``cost_s=0.2``,
       ``observed={"holding": holding, "detection": <that detection>}``, and — this is the
       subtle part — **``repeatable=False``**: looking again can only *miss* the object, which
       would turn a real failure into a pass.
    4. Nothing seen -> VERIFIED, method ``"perception"``, ``cost_s=0.2``.
    """
    # TODO(student): implement.
    raise NotImplementedError("verify_pick")


def verify_place(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                 level: str = "perception") -> Verification:
    """Is the object on the surface?

    1. ``read_state``: ``holding`` is not ``None`` -> NOT_VERIFIED, method ``"state"``,
       ``observed={"holding": holding}``.
    2. ``level != "perception"`` -> VERIFIED, method ``"state"``, ``observed={"holding": None}``.
    3. ``look``: no detection whose ``near_place`` equals ``args["location"]`` -> NOT_VERIFIED,
       method ``"perception"``, ``cost_s=0.2`` (leave ``repeatable`` at its default True — here a
       second look *removes* a detector miss).
    4. Otherwise VERIFIED, ``observed={"detection": <first hit>}``, ``cost_s=0.2``.
    """
    # TODO(student): implement.
    raise NotImplementedError("verify_place")


def verify_detect(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                  label: str | None = None) -> Verification:
    """A detect step's postcondition is about its own result: did it find what the step was for?

    * ``label is None`` -> UNCHECKABLE, method ``"result"``.
    * otherwise count the detections in ``result.data["detections"]`` whose label matches
      (``label_matches``). Any hit -> VERIFIED with ``observed={"detection": <first hit>}``;
      none -> NOT_VERIFIED. Method is ``"result"`` and ``cost_s`` is 0.0 either way: no new
      observation was taken.
    """
    # TODO(student): implement.
    raise NotImplementedError("verify_detect")


def verify_step(skills: RobotSkills, skill: str, args: Mapping[str, Any], result: SkillResult,
                level: str = "perception", label: str | None = None) -> Verification:
    """Dispatch, with the two cases that come before any verifier.

    1. ``level == "none"`` -> ``Verification(skill, result.ok, "UNCHECKABLE", "-", ...)``.
    2. ``not result.ok`` -> NOT_VERIFIED, method ``"result"``, message
       ``f"the call itself failed: {result.code}"`` — and **no extra observation**: a failed call
       has already told you what went wrong, and looking costs robot time.
    3. otherwise the verifier for ``skill`` (``navigate_to``, ``pick``, ``place``,
       ``detect_objects``); anything else is UNCHECKABLE with method ``"-"``.
    """
    # TODO(student): implement.
    raise NotImplementedError("verify_step")


def choose_repair(skill: str, result_code: str, verification: Verification, attempt: int,
                  budget: int, idempotent: bool, can_repair: bool) -> str:
    """One rung of the ladder. Return ``"ok"``, ``"retry"``, ``"repair"`` or ``"replan"``.

    * verified, or UNCHECKABLE -> ``"ok"``.
    * ``"retry"`` (same call, unchanged) only when **all** of: attempts remain
      (``attempt < budget``), the skill is ``idempotent``, and the step's precondition has not
      been invalidated. It is invalidated when ``skill == "pick"`` and either ``result_code`` is
      in ``INVALIDATES_BINDING`` or the verification failed — a grasp that did not work has
      moved the object, so repeating the same ``pick`` cannot succeed.
    * ``"repair"`` when ``can_repair`` and the skill is ``pick``: go back and re-establish the
      precondition (re-detect) before trying again.
    * otherwise ``"replan"``.
    """
    # TODO(student): implement.
    raise NotImplementedError("choose_repair")


def should_look_again(verification: Verification, looks_used: int, max_looks: int) -> bool:
    """Is another 0.2 s observation worth taking before believing a failed verification?

    True only when the verification failed, ``verification.repeatable`` is True, the method is
    ``"perception"``, and ``looks_used < max_looks``.
    """
    # TODO(student): implement.
    raise NotImplementedError("should_look_again")
