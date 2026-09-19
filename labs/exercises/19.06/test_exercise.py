"""Checker for 19.06 — the plan validator.

Run: ``python course.py check 19.06`` (``--solution`` runs the reference).
Offline: the plans are JSON, the "robot" is only consulted for its skill schemas.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "19-llm-robot-agents" / "code", _REPO / "labs" / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from robot_agent.sim_world import HomeWorld  # noqa: E402
from robot_agent.skills import RobotSkills  # noqa: E402


@pytest.fixture
def skills():
    return RobotSkills(HomeWorld(seed=1))


def codes(errors):
    return [e.code for e in errors]


GOOD = {"goal": "the water bottle is on the kitchen_table", "steps": [
    {"skill": "navigate_to", "args": {"place": "kitchen"}},
    {"skill": "detect_objects", "args": {}, "bind": "target", "label": "water bottle"},
    {"skill": "navigate_to", "args": {"place": "$target"}},
    {"skill": "detect_objects", "args": {}, "bind": "target", "label": "water bottle"},
    {"skill": "pick", "args": {"object_id": "$target"}},
    {"skill": "navigate_to", "args": {"place": "kitchen_table"}},
    {"skill": "place", "args": {"location": "kitchen_table"}},
]}


def goal(impl):
    return impl.Goal("water bottle", "kitchen_table")


# --- substitute -------------------------------------------------------------------------------
def test_substitute_uses_the_parameters_own_schema(impl, skills):
    spec = skills.specs["navigate_to"]
    out = impl.substitute({"place": "$target", "timeout_s": 30.0}, spec)
    assert out["place"] in spec.params[0].enum, "a variable must become a value the enum accepts"
    assert out["timeout_s"] == 30.0, "literal arguments are untouched"


def test_substitute_keeps_unknown_keys_so_the_schema_can_report_them(impl, skills):
    out = impl.substitute({"place": "kitchen", "speed": "$fast"}, skills.specs["navigate_to"])
    assert out["speed"] == "$fast", "an unknown key has no parameter to substitute for"


def test_substitute_satisfies_the_object_id_pattern(impl, skills):
    from robot_agent.skills import validate_arguments
    spec = skills.specs["pick"]
    out = impl.substitute({"object_id": "$target"}, spec)
    assert validate_arguments(spec.input_schema(), out) == [], f"placeholder rejected: {out}"


# --- the good plan ----------------------------------------------------------------------------
def test_a_good_plan_is_accepted(impl, skills):
    plan, errors = impl.validate_plan(GOOD, skills, goal(impl))
    assert errors == [] and plan is not None
    assert len(plan.steps) == 7


def test_validation_never_touches_the_robot(impl, skills):
    impl.validate_plan(GOOD, skills, goal(impl))
    impl.validate_plan({"goal": "g", "steps": [{"skill": "navigate_to", "args": {"place": "kitchen"}}]}, skills)
    assert skills.log == [] and skills.world.t == 0.0, "a rejected plan must cost zero robot time"


# --- the checks, one at a time ----------------------------------------------------------------
def test_malformed_plans_never_reach_the_other_checks(impl, skills):
    for raw in ("not json", [1, 2], {"steps": []}, {"goal": "g", "steps": []},
                {"goal": "g", "steps": [{"skill": "list_places", "args": {}}] * 13}):
        plan, errors = impl.validate_plan(raw, skills)
        assert plan is None and codes(errors) == ["MALFORMED"] * len(errors)


def test_hallucinated_skills_and_places(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "open_fridge", "args": {}},
        {"skill": "navigate_to", "args": {"place": "garage"}},
        {"skill": "grasp", "args": {"target": "water bottle"}},
    ]}
    plan, errors = impl.validate_plan(raw, skills)
    assert plan is None
    assert codes(errors) == ["UNKNOWN_SKILL", "INVALID_ARGUMENTS", "UNKNOWN_SKILL"]
    assert errors[0].step == 1 and errors[2].step == 3


def test_a_skill_outside_the_allowlist_is_not_available(impl):
    """A skill that exists but is not offered to this caller must not be plannable either."""
    limited = RobotSkills(HomeWorld(seed=1), allowlist={"list_places", "navigate_to"})
    raw = {"goal": "g", "steps": [{"skill": "pick", "args": {"object_id": "$x"}}]}
    errors = impl.validate_plan(raw, limited)[1]
    assert codes(errors) == ["UNKNOWN_SKILL"]
    assert "pick" not in errors[0].message.split("have: ")[1]


def test_arguments_are_checked_with_the_same_schema_the_model_was_given(impl, skills):
    raw = {"goal": "g", "steps": [{"skill": "navigate_to", "args": {"place": "kitchen", "timeout_s": 900}}]}
    errors = impl.validate_plan(raw, skills)[1]
    assert codes(errors) == ["INVALID_ARGUMENTS"] and "above the maximum" in errors[0].message


def test_an_invented_object_id_is_caught(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "pick", "args": {"object_id": "bottle-1"}},
        {"skill": "navigate_to", "args": {"place": "kitchen_table"}},
        {"skill": "place", "args": {"location": "kitchen_table"}},
    ]}
    plan, errors = impl.validate_plan(raw, skills)
    assert plan is None
    assert codes(errors) == ["LITERAL_OBJECT_ID", "PRECONDITION"], \
        "the rejected pick leaves the abstract gripper empty, so the place fails too"
    assert errors[0].step == 2 and errors[1].step == 4


def test_an_unbound_variable_is_caught(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "pick", "args": {"object_id": "$ghost"}},
    ]}
    assert codes(impl.validate_plan(raw, skills)[1]) == ["UNBOUND_VARIABLE"]


def test_a_variable_on_the_wrong_parameter(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "detect_objects", "args": {}, "bind": "a", "label": "water bottle"},
        {"skill": "navigate_to", "args": {"place": "kitchen", "timeout_s": "$a"}},
    ]}
    errors = impl.validate_plan(raw, skills)[1]
    assert "INVALID_ARGUMENTS" in codes(errors)


# --- symbolic execution -----------------------------------------------------------------------
def test_place_before_arriving(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "detect_objects", "args": {}, "bind": "t", "label": "water bottle"},
        {"skill": "pick", "args": {"object_id": "$t"}},
        {"skill": "place", "args": {"location": "kitchen_table"}},
        {"skill": "navigate_to", "args": {"place": "kitchen_table"}},
    ]}
    plan, errors = impl.validate_plan(raw, skills)
    assert plan is None and codes(errors) == ["PRECONDITION"]
    assert errors[0].step == 3 and "not at 'kitchen_table'" in errors[0].message


def test_place_with_an_empty_gripper(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen_table"}},
        {"skill": "place", "args": {"location": "kitchen_table"}},
    ]}
    errors = impl.validate_plan(raw, skills)[1]
    assert codes(errors) == ["PRECONDITION"] and "hold something" in errors[0].message


def test_picking_twice_without_placing(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "detect_objects", "args": {}, "bind": "a", "label": "water bottle"},
        {"skill": "pick", "args": {"object_id": "$a"}},
        {"skill": "detect_objects", "args": {}, "bind": "c", "label": "red cup"},
        {"skill": "pick", "args": {"object_id": "$c"}},
    ]}
    errors = impl.validate_plan(raw, skills)[1]
    assert codes(errors) == ["PRECONDITION"] and "still holds" in errors[0].message


def test_driving_away_makes_a_detection_stale(impl, skills):
    """The `fresh` rule: a detection taken before the last navigate_to is out of reach."""
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "detect_objects", "args": {}, "bind": "b", "label": "water bottle"},
        {"skill": "navigate_to", "args": {"place": "study"}},
        {"skill": "pick", "args": {"object_id": "$b"}},
    ]}
    plan, errors = impl.validate_plan(raw, skills)
    assert plan is None and codes(errors) == ["PRECONDITION"]
    assert errors[0].step == 4 and "detect_objects again" in errors[0].message


def test_re_detecting_after_arriving_makes_the_plan_legal(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "detect_objects", "args": {}, "bind": "b", "label": "water bottle"},
        {"skill": "navigate_to", "args": {"place": "$b"}},
        {"skill": "detect_objects", "args": {}, "bind": "b", "label": "water bottle"},
        {"skill": "pick", "args": {"object_id": "$b"}},
    ]}
    assert impl.validate_plan(raw, skills)[1] == []


def test_read_only_skills_change_nothing(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "list_places", "args": {}},
        {"skill": "get_robot_state", "args": {}},
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "detect_objects", "args": {}, "bind": "b", "label": "water bottle"},
        {"skill": "list_places", "args": {}},
        {"skill": "pick", "args": {"object_id": "$b"}},
    ]}
    assert impl.validate_plan(raw, skills)[1] == [], "a read-only step must not invalidate a detection"


# --- the goal check ---------------------------------------------------------------------------
def test_a_legal_plan_that_never_achieves_the_goal(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "detect_objects", "args": {}, "bind": "t", "label": "water bottle"},
        {"skill": "get_robot_state", "args": {}},
    ]}
    assert codes(impl.validate_plan(raw, skills, goal(impl))[1]) == ["GOAL_NOT_REACHED"]
    assert impl.validate_plan(raw, skills, None)[0] is not None, "no goal, nothing to compare against"


def test_the_right_object_on_the_wrong_surface_fails_the_goal(impl, skills):
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "detect_objects", "args": {}, "bind": "t", "label": "water bottle"},
        {"skill": "navigate_to", "args": {"place": "$t"}},
        {"skill": "detect_objects", "args": {}, "bind": "t", "label": "water bottle"},
        {"skill": "pick", "args": {"object_id": "$t"}},
        {"skill": "navigate_to", "args": {"place": "bed"}},
        {"skill": "place", "args": {"location": "bed"}},
    ]}
    assert codes(impl.validate_plan(raw, skills, goal(impl))[1]) == ["GOAL_NOT_REACHED"], \
        "the goal comes from the user, not from the plan"


def test_the_wrong_object_on_the_right_surface_fails_the_goal(impl, skills):
    raw = dict(GOOD)
    raw["steps"] = [dict(s) for s in GOOD["steps"]]
    for s in raw["steps"]:
        if s.get("label"):
            s["label"] = "red cup"
    assert codes(impl.validate_plan(raw, skills, goal(impl))[1]) == ["GOAL_NOT_REACHED"]


def test_a_broken_plan_gets_no_goal_verdict(impl, skills):
    raw = {"goal": "g", "steps": [{"skill": "open_fridge", "args": {}}]}
    assert codes(impl.validate_plan(raw, skills, goal(impl))[1]) == ["UNKNOWN_SKILL"]
