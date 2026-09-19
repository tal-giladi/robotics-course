"""Tests for 19.06: the plan language, the validator and the plan -> behavior tree compiler."""

from __future__ import annotations

import pytest
from py_trees.common import Status

from robot_agent.bt import run_tree
from robot_agent.llm import AssistantMessage, ScriptedLLM, ToolCall
from robot_agent.planning import (
    MAX_STEPS,
    Goal,
    MockPlanner,
    PlanExecConfig,
    parse_plan,
    plan_and_repair,
    plan_to_tree,
    validate_plan,
)
from robot_agent.sim_world import ArmParams, HomeWorld
from robot_agent.skills import RobotSkills

GOAL = Goal("water bottle", "kitchen_table")


@pytest.fixture
def skills():
    return RobotSkills(HomeWorld(seed=1))


def codes(errors):
    return [e.code for e in errors]


def plan_json(name: str, **kwargs):
    return MockPlanner(**kwargs).plans()[name]


# --- shape ------------------------------------------------------------------------------------
@pytest.mark.parametrize(("raw", "fragment"), [
    ("not json at all", "not valid JSON"),
    ([{"skill": "navigate_to"}], "must be an object"),
    ({"steps": [{"skill": "list_places", "args": {}}]}, "'goal' must be"),
    ({"goal": "g", "steps": []}, "non-empty list"),
    ({"goal": "g", "steps": [{"args": {}}]}, "'skill' string"),
    ({"goal": "g", "steps": [{"skill": "detect_objects", "args": {}, "bind": "X"}]}, "lower-case"),
    ({"goal": "g", "steps": [{"skill": "detect_objects", "args": {}, "bind": "x"}]}, "'label'"),
    ({"goal": "g", "steps": [{"skill": "list_places", "args": {}}] * (MAX_STEPS + 1)}, "more than the limit"),
])
def test_malformed_plans_are_rejected_before_anything_else(raw, fragment):
    plan, errors = parse_plan(raw)
    assert plan is None
    assert any(fragment in e.message for e in errors), errors


def test_a_good_plan_is_accepted_and_round_trips(skills):
    plan, errors = validate_plan(plan_json("good"), skills, GOAL)
    assert errors == []
    assert plan is not None and len(plan.steps) == 7
    again, errors2 = validate_plan(plan.to_json(), skills, GOAL)
    assert errors2 == [] and again == plan


# --- semantics --------------------------------------------------------------------------------
def test_invented_object_id_is_caught(skills):
    plan, errors = validate_plan(plan_json("literal_id"), skills, GOAL)
    assert plan is None
    assert "LITERAL_OBJECT_ID" in codes(errors)
    assert errors[0].step == 2


def test_hallucinated_skills_and_places(skills):
    plan, errors = validate_plan(plan_json("hallucinated"), skills, GOAL)
    assert plan is None
    assert codes(errors) == ["UNKNOWN_SKILL", "INVALID_ARGUMENTS", "UNKNOWN_SKILL"]


def test_wrong_order_is_caught_without_moving_the_robot(skills):
    plan, errors = validate_plan(plan_json("bad_order"), skills, GOAL)
    assert plan is None and "PRECONDITION" in codes(errors)
    assert skills.world.t == 0.0 and skills.log == []


def test_a_valid_plan_that_misses_the_goal(skills):
    plan, errors = validate_plan(plan_json("sightseeing"), skills, GOAL)
    assert plan is None and codes(errors) == ["GOAL_NOT_REACHED"]
    assert validate_plan(plan_json("sightseeing"), skills, goal=None)[0] is not None  # no goal: it is legal


def test_stale_detection_before_pick(skills):
    """Detect in the kitchen, drive somewhere else, then pick: the object is not in reach."""
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "detect_objects", "args": {}, "bind": "b", "label": "water bottle"},
        {"skill": "navigate_to", "args": {"place": "study"}},
        {"skill": "pick", "args": {"object_id": "$b"}},
    ]}
    plan, errors = validate_plan(raw, skills)
    assert plan is None and codes(errors) == ["PRECONDITION"]
    assert errors[0].step == 4 and "detect_objects again" in errors[0].message


def test_unbound_variable_and_double_pick(skills):
    raw = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "pick", "args": {"object_id": "$ghost"}},
    ]}
    assert codes(validate_plan(raw, skills)[1]) == ["UNBOUND_VARIABLE"]
    raw2 = {"goal": "g", "steps": [
        {"skill": "navigate_to", "args": {"place": "kitchen"}},
        {"skill": "detect_objects", "args": {}, "bind": "a", "label": "water bottle"},
        {"skill": "pick", "args": {"object_id": "$a"}},
        {"skill": "detect_objects", "args": {}, "bind": "c", "label": "red cup"},
        {"skill": "pick", "args": {"object_id": "$c"}},
    ]}
    errors = validate_plan(raw2, skills)[1]
    assert codes(errors) == ["PRECONDITION"] and "still holds" in errors[0].message


def test_bad_arguments_use_the_same_schema_as_the_tools(skills):
    raw = {"goal": "g", "steps": [{"skill": "navigate_to", "args": {"place": "kitchen", "timeout_s": 900}}]}
    errors = validate_plan(raw, skills)[1]
    assert codes(errors) == ["INVALID_ARGUMENTS"] and "above the maximum" in errors[0].message


def test_a_skill_outside_the_allowlist_cannot_be_planned():
    skills = RobotSkills(HomeWorld(seed=1), allowlist={"list_places", "get_robot_state", "navigate_to"})
    raw = {"goal": "g", "steps": [{"skill": "pick", "args": {"object_id": "$x"}}]}
    assert codes(validate_plan(raw, skills)[1]) == ["UNKNOWN_SKILL"]


# --- planning with a model --------------------------------------------------------------------
def test_mock_planner_repairs_after_the_validator_rejects(skills):
    planner = MockPlanner(variant="bad_order", repair=True)
    plan, attempts = plan_and_repair(planner, skills, "Find the water bottle and put it on the kitchen_table.", GOAL)
    assert plan is not None
    assert [a.plan is None for a in attempts] == [True, False]
    assert skills.log == []  # validation alone never calls a skill


def test_a_planner_that_never_repairs_gives_up(skills):
    planner = MockPlanner(variant="literal_id", repair=False)
    plan, attempts = plan_and_repair(planner, skills, "task", GOAL, max_attempts=3)
    assert plan is None and len(attempts) == 3


def test_a_reply_without_a_plan_is_an_error_not_a_crash(skills):
    llm = ScriptedLLM([AssistantMessage("Which table do you mean?", (), stop_reason="end_turn")])
    plan, attempts = plan_and_repair(llm, skills, "put it on the table", GOAL, max_attempts=1)
    assert plan is None and codes(attempts[0].errors) == ["MALFORMED"]
    assert attempts[0].text == "Which table do you mean?"


def test_the_planner_sees_submit_plan_and_every_skill(skills):
    seen = {}

    class Spy:
        name = "spy"

        def complete(self, system, messages, tools):
            seen["tools"] = [t["name"] for t in tools]
            seen["system"] = system
            return AssistantMessage("", (ToolCall("c1", "submit_plan", plan_json("good")),), stop_reason="tool_use")

    plan, _ = plan_and_repair(Spy(), skills, "task", GOAL, max_attempts=1)
    assert plan is not None
    assert seen["tools"][0] == "submit_plan" and "pick" in seen["tools"]
    assert "$<name>" in seen["system"]


# --- execution --------------------------------------------------------------------------------
def test_an_accepted_plan_executes_and_achieves_the_goal():
    skills = RobotSkills(HomeWorld(seed=1))
    plan, errors = validate_plan(plan_json("good"), skills, GOAL)
    assert errors == []
    root, bindings = plan_to_tree(plan, skills)
    run = run_tree(root, skills)
    assert run.status == Status.SUCCESS
    assert skills.world.objects["bottle-1"].on == "kitchen_table"
    assert bindings["target"]["object_id"] == "bottle-1"


def test_a_failed_grasp_re_runs_the_detection_that_bound_the_variable():
    """The compiler fuses a bind step with the step that consumes it, so retries are not stale."""
    skills = RobotSkills(HomeWorld(seed=1, arm=ArmParams(grasp_success_p=0.5)))
    plan, _ = validate_plan(plan_json("good"), skills, GOAL)
    root, _ = plan_to_tree(plan, skills)
    run_tree(root, skills)
    picks = [c for c in skills.log if c.skill == "pick"]
    assert any(c.result.code == "GRASP_FAILED" for c in picks)
    assert not any(c.result.code == "OBJECT_NOT_FOUND" for c in picks), "a retry used a stale detection"


def test_the_estop_guard_stops_plan_execution():
    skills = RobotSkills(HomeWorld(seed=1))
    plan, _ = validate_plan(plan_json("good"), skills, GOAL)
    root, _ = plan_to_tree(plan, skills, PlanExecConfig())
    run = run_tree(root, skills, on_tick=lambda tick, _root: setattr(skills.world, "estop", skills.world.t >= 5.0))
    assert run.status == Status.FAILURE
    assert skills.world.t < 12.0 and skills.world.holding is None


def test_the_deadline_uses_simulated_time():
    skills = RobotSkills(HomeWorld(seed=1))
    plan, _ = validate_plan(plan_json("good"), skills, GOAL)
    root, _ = plan_to_tree(plan, skills, PlanExecConfig(deadline_s=8.0))
    run = run_tree(root, skills)
    assert run.status == Status.FAILURE and 8.0 <= skills.world.t < 10.0
