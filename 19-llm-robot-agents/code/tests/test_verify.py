"""Tests for 19.07 — verification and closed-loop execution. Offline, no model, seconds to run."""

from __future__ import annotations

import pytest

from robot_agent.planning import Goal, MockPlanner, Plan, Step, plan_and_repair
from robot_agent.sim_world import ArmParams, HomeWorld
from robot_agent.skills import RobotSkills
from robot_agent.verify import (
    ClosedLoopExecutor,
    FaultySkills,
    LoopConfig,
    SearchReplanner,
    VerifyConfig,
    verify_goal,
    verify_step,
)

BOTTLE_GOAL = Goal("water bottle", "kitchen_table")


def fetch_plan(obj: str = "water bottle", where: str = "kitchen", target: str = "kitchen_table") -> Plan:
    home = HomeWorld(seed=1)
    plan, _ = plan_and_repair(MockPlanner(obj=obj, where=where, target=target), RobotSkills(home),
                              f"Find the {obj} and put it on the {target}.", Goal(obj, target), max_attempts=1)
    assert plan is not None
    return plan


def build(seed: int = 1, fault: str = "none", grasp_p: float = 0.85):
    home = HomeWorld(seed=seed, arm=ArmParams(grasp_success_p=grasp_p))
    skills = RobotSkills(home)
    return home, (FaultySkills(skills, fault) if fault != "none" else skills)


# ----------------------------------------------------------------------------------------------
# Verifiers
# ----------------------------------------------------------------------------------------------
def test_navigate_verification_uses_state_not_the_return_code() -> None:
    _, skills = build(fault="short_nav")
    result = skills.call("navigate_to", {"place": "kitchen"})
    assert result.ok and result.code == "SUCCEEDED"  # the skill lied
    v = verify_step(skills, "navigate_to", {"place": "kitchen"}, result)
    assert not v.ok and v.code == "NOT_VERIFIED" and v.method == "state"
    assert v.observed["distance_m"] > 1.0


def test_state_verification_is_free_and_perception_is_not() -> None:
    home, skills = build()
    skills.call("navigate_to", {"place": "kitchen"})
    t0 = home.t
    v = verify_step(skills, "navigate_to", {"place": "kitchen"}, skills.log[-1].result)
    assert v.ok and v.cost_s == 0.0 and home.t == pytest.approx(t0)


def test_phantom_grasp_is_caught_by_proprioception() -> None:
    home, skills = build(fault="phantom_grasp")
    skills.call("navigate_to", {"place": "kitchen_counter"})
    skills.call("detect_objects")
    result = skills.call("pick", {"object_id": "bottle-1"})
    assert result.ok  # the gripper reported a grasp
    assert home.holding is None  # ... and the hand is empty
    v = verify_step(skills, "pick", {"object_id": "bottle-1"}, result, VerifyConfig(level="state"))
    assert not v.ok and v.method == "state"


def test_a_lying_gripper_sensor_needs_the_camera() -> None:
    home, skills = build(fault="blind_gripper")
    skills.call("navigate_to", {"place": "kitchen_counter"})
    skills.call("detect_objects")
    result = skills.call("pick", {"object_id": "bottle-1"})
    assert home.holding is None
    # Proprioception agrees with the lie: the limit switch is the thing that broke.
    assert verify_step(skills, "pick", {"object_id": "bottle-1"}, result, VerifyConfig(level="state")).ok
    # The camera does not: the bottle is still standing on the counter.
    v = verify_step(skills, "pick", {"object_id": "bottle-1"}, result, VerifyConfig(level="perception"))
    assert not v.ok and v.method == "perception" and v.cost_s > 0


def test_a_failed_call_is_reported_as_not_verified_without_extra_observation() -> None:
    home, skills = build()
    result = skills.call("pick", {"object_id": "bottle-1"})  # never detected: OBJECT_NOT_FOUND
    t0 = home.t
    v = verify_step(skills, "pick", {"object_id": "bottle-1"}, result)
    assert not v.ok and v.method == "result" and home.t == pytest.approx(t0)


def test_detect_verification_is_about_the_step_not_the_skill() -> None:
    _, skills = build()
    skills.call("navigate_to", {"place": "kitchen"})
    result = skills.call("detect_objects")
    assert result.ok  # the detector ran
    v = verify_step(skills, "detect_objects", {}, result, VerifyConfig(), label="grand piano")
    assert not v.ok and v.code == "NOT_VERIFIED"  # ... and the step still failed


def test_goal_verification_looks_at_the_world() -> None:
    home, skills = build()
    assert not verify_goal(skills, BOTTLE_GOAL).ok
    home.objects["bottle-1"].x, home.objects["bottle-1"].y = 4.4, 1.25
    home.objects["bottle-1"].on = "kitchen_table"
    assert verify_goal(skills, BOTTLE_GOAL).ok


# ----------------------------------------------------------------------------------------------
# The closed loop
# ----------------------------------------------------------------------------------------------
def test_the_happy_path_succeeds_and_the_goal_is_verified() -> None:
    home, skills = build()
    report = ClosedLoopExecutor(skills, BOTTLE_GOAL).run(fetch_plan())
    assert report.outcome == "succeeded"
    assert report.goal_check is not None and report.goal_check.ok
    assert home.objects["bottle-1"].on == "kitchen_table"


def test_open_loop_execution_of_the_same_plan_ends_wrong_and_the_goal_check_says_so() -> None:
    home, skills = build()
    cfg = LoopConfig(verify=VerifyConfig(level="none"))
    report = ClosedLoopExecutor(skills, BOTTLE_GOAL, cfg).run(fetch_plan())
    # seed 1 fails one grasp; without verification nothing notices until the end
    assert report.outcome == "goal_not_verified"
    assert home.objects["bottle-1"].on == "kitchen_counter"


def test_a_plan_aimed_at_the_wrong_room_is_repaired_by_replanning() -> None:
    home, skills = build()
    goal = Goal("red cup", "sofa")
    report = ClosedLoopExecutor(skills, goal, replanner=SearchReplanner()).run(
        fetch_plan("red cup", "kitchen", "sofa"))
    assert report.outcome == "succeeded" and report.replans == 1
    assert home.objects["cup-1"].on == "sofa"


def test_without_a_replanner_the_same_task_fails_and_says_why() -> None:
    _, skills = build()
    goal = Goal("red cup", "sofa")
    report = ClosedLoopExecutor(skills, goal, replanner=None).run(fetch_plan("red cup", "kitchen", "sofa"))
    assert report.outcome == "failed" and "no replanner" in report.message


def test_an_unverifiable_grasp_repairs_by_re_detecting_before_retrying() -> None:
    _, skills = build(fault="blind_gripper")
    report = ClosedLoopExecutor(skills, BOTTLE_GOAL, replanner=SearchReplanner()).run(fetch_plan())
    picks = [s for s in report.steps if s.skill == "pick"]
    assert len(picks) > 1
    # the step after a failed pick is always a fresh detection, never the same pick again
    for a, b in zip(report.steps, report.steps[1:]):
        if a.skill == "pick" and a.repair == "repair":
            assert b.skill == "detect_objects"


def test_verification_cost_is_bounded_and_reported() -> None:
    home, skills = build()
    report = ClosedLoopExecutor(skills, BOTTLE_GOAL).run(fetch_plan())
    assert 0.0 < report.verify_cost_s < 0.1 * report.sim_time_s  # under 10 % of robot time
    assert report.sim_time_s == pytest.approx(home.t, abs=0.01)


def test_the_robot_is_stopped_on_every_path() -> None:
    for fault in ("none", "phantom_grasp", "short_nav"):
        home, skills = build(fault=fault)
        ClosedLoopExecutor(skills, BOTTLE_GOAL, replanner=SearchReplanner()).run(fetch_plan())
        state = home.base.read()
        assert abs(state.left_rad_s) < 1e-3 and abs(state.right_rad_s) < 1e-3


def test_the_estop_stops_the_loop_between_steps() -> None:
    home, skills = build()
    plan = Plan("stop", (Step("navigate_to", {"place": "kitchen"}), Step("navigate_to", {"place": "bedroom"})))
    executor = ClosedLoopExecutor(skills, BOTTLE_GOAL, LoopConfig(check_goal=False))
    home.estop = True
    report = executor.run(plan)
    assert report.outcome == "estop" and not report.steps
