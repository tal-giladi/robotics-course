"""19.07 — reference solution: independent verification and the repair ladder."""

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
INVALIDATES_BINDING = frozenset({"GRASP_FAILED", "OBJECT_NOT_FOUND"})


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
    return skills.call("get_robot_state").data


def look(skills: RobotSkills) -> list[JsonDict]:
    r = skills.call("detect_objects", {"min_confidence": MIN_CONFIDENCE})
    return list(r.data.get("detections", [])) if r.ok else []


def verify_navigate(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult) -> Verification:
    place = str(args.get("place"))
    state = read_state(skills)
    at = state.get("at_place")
    target = skills.world.places.get(place)
    if target is None:
        return Verification("navigate_to", False, "NOT_VERIFIED", "state", f"'{place}' is not on the map")
    pose = state["pose"]
    distance = math.hypot(target.x - pose["x_m"], target.y - pose["y_m"])
    ok = at == place or distance <= PLACE_TOLERANCE_M
    return Verification("navigate_to", ok, "VERIFIED" if ok else "NOT_VERIFIED", "state",
                        f"at_place={at!r}, {distance:.2f} m from '{place}'",
                        {"at_place": at, "distance_m": round(distance, 2)})


def verify_pick(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                level: str = "perception") -> Verification:
    object_id = str(args.get("object_id"))
    holding = read_state(skills).get("holding")
    if holding != object_id:
        return Verification("pick", False, "NOT_VERIFIED", "state",
                            f"the skill reported success but the robot holds {holding!r}",
                            {"holding": holding})
    if level != "perception":
        return Verification("pick", True, "VERIFIED", "state", f"holding {object_id}", {"holding": holding})
    still_there = [d for d in look(skills) if d["object_id"] == object_id]
    if still_there:
        return Verification("pick", False, "NOT_VERIFIED", "perception",
                            f"the state says 'holding {object_id}' and the camera still sees it",
                            {"holding": holding, "detection": still_there[0]}, 0.2, repeatable=False)
    return Verification("pick", True, "VERIFIED", "perception", f"holding {object_id}",
                        {"holding": holding}, 0.2)


def verify_place(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                 level: str = "perception") -> Verification:
    location = str(args.get("location"))
    holding = read_state(skills).get("holding")
    if holding is not None:
        return Verification("place", False, "NOT_VERIFIED", "state",
                            f"the gripper still holds {holding!r}", {"holding": holding})
    if level != "perception":
        return Verification("place", True, "VERIFIED", "state", "the gripper is empty", {"holding": None})
    hits = [d for d in look(skills) if d.get("near_place") == location]
    if not hits:
        return Verification("place", False, "NOT_VERIFIED", "perception",
                            f"nothing is visible on '{location}' after placing", {}, 0.2)
    return Verification("place", True, "VERIFIED", "perception",
                        f"{hits[0]['label']} is on '{location}'", {"detection": hits[0]}, 0.2)


def verify_detect(skills: RobotSkills, args: Mapping[str, Any], result: SkillResult,
                  label: str | None = None) -> Verification:
    if label is None:
        return Verification("detect_objects", True, "UNCHECKABLE", "result",
                            "no label to match: this step has no postcondition")
    hits = [d for d in result.data.get("detections", []) if label_matches(d["label"], label)]
    return Verification("detect_objects", bool(hits), "VERIFIED" if hits else "NOT_VERIFIED", "result",
                        f"{len(hits)} detection(s) matching '{label}'",
                        {"detection": hits[0]} if hits else {})


def verify_step(skills: RobotSkills, skill: str, args: Mapping[str, Any], result: SkillResult,
                level: str = "perception", label: str | None = None) -> Verification:
    if level == "none":
        return Verification(skill, result.ok, "UNCHECKABLE", "-", "verification disabled")
    if not result.ok:
        return Verification(skill, False, "NOT_VERIFIED", "result", f"the call itself failed: {result.code}")
    if skill == "navigate_to":
        return verify_navigate(skills, args, result)
    if skill == "pick":
        return verify_pick(skills, args, result, level)
    if skill == "place":
        return verify_place(skills, args, result, level)
    if skill == "detect_objects":
        return verify_detect(skills, args, result, label)
    return Verification(skill, True, "UNCHECKABLE", "-", f"no verifier for '{skill}'")


def choose_repair(skill: str, result_code: str, verification: Verification, attempt: int,
                  budget: int, idempotent: bool, can_repair: bool) -> str:
    """One rung of the ladder: "ok" | "retry" | "repair" | "replan"."""
    if verification.ok or verification.code == "UNCHECKABLE":
        return "ok"
    invalidated = skill == "pick" and (result_code in INVALIDATES_BINDING
                                       or verification.code == "NOT_VERIFIED")
    if attempt < budget and not invalidated and idempotent:
        return "retry"
    if can_repair and skill == "pick":
        return "repair"
    return "replan"


def should_look_again(verification: Verification, looks_used: int, max_looks: int) -> bool:
    """Is another observation worth 0.2 s? Only for a "did not see what I expected" failure."""
    return (not verification.ok and verification.repeatable
            and verification.method == "perception" and looks_used < max_looks)
