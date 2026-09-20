"""Checker for 19.07 — verification and the repair ladder.

Run: ``python course.py check 19.07`` (``--solution`` runs the reference). Offline, seconds.
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
from robot_agent.skills import RobotSkills, SkillResult  # noqa: E402
from robot_agent.verify import FaultySkills  # noqa: E402


@pytest.fixture
def skills() -> RobotSkills:
    return RobotSkills(HomeWorld(seed=0))


def at_the_counter(fault: str = "none"):
    home = HomeWorld(seed=0)
    skills = RobotSkills(home)
    wrapped = FaultySkills(skills, fault) if fault != "none" else skills
    wrapped.call("navigate_to", {"place": "kitchen_counter"})
    wrapped.call("detect_objects")
    return home, wrapped


# ----------------------------------------------------------------------------------------------
# navigate
# ----------------------------------------------------------------------------------------------
def test_navigate_verified_after_a_real_drive(impl, skills: RobotSkills) -> None:
    result = skills.call("navigate_to", {"place": "kitchen"})
    v = impl.verify_navigate(skills, {"place": "kitchen"}, result)
    assert v.ok and v.code == "VERIFIED" and v.method == "state"
    assert v.observed["at_place"] == "kitchen" and v.observed["distance_m"] < 0.3


def test_navigate_not_verified_when_the_skill_lies(impl) -> None:
    skills = FaultySkills(RobotSkills(HomeWorld(seed=0)), "short_nav")
    result = skills.call("navigate_to", {"place": "kitchen"})
    assert result.ok  # the skill claims success
    v = impl.verify_navigate(skills, {"place": "kitchen"}, result)
    assert not v.ok and v.code == "NOT_VERIFIED" and v.observed["distance_m"] > 1.0


def test_navigate_verification_costs_no_robot_time(impl, skills: RobotSkills) -> None:
    result = skills.call("navigate_to", {"place": "kitchen"})
    t0 = skills.world.t
    v = impl.verify_navigate(skills, {"place": "kitchen"}, result)
    assert v.cost_s == 0.0 and skills.world.t == pytest.approx(t0)


def test_an_unknown_place_is_not_verified(impl, skills: RobotSkills) -> None:
    v = impl.verify_navigate(skills, {"place": "garage"}, SkillResult("navigate_to", True, "SUCCEEDED", ""))
    assert not v.ok and "map" in v.message


# ----------------------------------------------------------------------------------------------
# pick
# ----------------------------------------------------------------------------------------------
def test_pick_is_verified_when_the_object_is_really_held(impl) -> None:
    home, skills = at_the_counter()
    result = skills.call("pick", {"object_id": "bottle-1"})
    while not result.ok:  # the arm misses sometimes; that is not what this test is about
        skills.call("detect_objects")
        result = skills.call("pick", {"object_id": "bottle-1"})
    v = impl.verify_pick(skills, {"object_id": "bottle-1"}, result)
    assert v.ok and home.holding == "bottle-1"


def test_a_phantom_grasp_is_caught_by_proprioception(impl) -> None:
    home, skills = at_the_counter("phantom_grasp")
    result = skills.call("pick", {"object_id": "bottle-1"})
    assert result.ok and home.holding is None
    v = impl.verify_pick(skills, {"object_id": "bottle-1"}, result, level="state")
    assert not v.ok and v.method == "state" and v.observed["holding"] is None


def test_a_lying_gripper_sensor_is_caught_only_by_the_camera(impl) -> None:
    home, skills = at_the_counter("blind_gripper")
    result = skills.call("pick", {"object_id": "bottle-1"})
    assert home.holding is None
    assert impl.verify_pick(skills, {"object_id": "bottle-1"}, result, level="state").ok
    v = impl.verify_pick(skills, {"object_id": "bottle-1"}, result, level="perception")
    assert not v.ok and v.method == "perception" and v.cost_s > 0


def test_a_still_visible_object_makes_the_check_non_repeatable(impl) -> None:
    _, skills = at_the_counter("blind_gripper")
    result = skills.call("pick", {"object_id": "bottle-1"})
    v = impl.verify_pick(skills, {"object_id": "bottle-1"}, result, level="perception")
    if not v.ok and v.method == "perception":
        assert v.repeatable is False


# ----------------------------------------------------------------------------------------------
# place and detect
# ----------------------------------------------------------------------------------------------
def test_place_is_not_verified_while_the_gripper_is_full(impl) -> None:
    home, skills = at_the_counter()
    home.holding = "bottle-1"
    v = impl.verify_place(skills, {"location": "kitchen_table"},
                          SkillResult("place", True, "SUCCEEDED", ""), level="state")
    assert not v.ok and v.observed["holding"] == "bottle-1"


def test_detect_verification_separates_the_skill_from_the_step(impl, skills: RobotSkills) -> None:
    skills.call("navigate_to", {"place": "kitchen"})
    result = skills.call("detect_objects")
    assert result.ok  # the detector ran
    v = impl.verify_detect(skills, {}, result, label="grand piano")
    assert not v.ok and v.code == "NOT_VERIFIED" and v.method == "result" and v.cost_s == 0.0


def test_detect_without_a_label_has_no_postcondition(impl, skills: RobotSkills) -> None:
    result = skills.call("detect_objects")
    assert impl.verify_detect(skills, {}, result, label=None).code == "UNCHECKABLE"


def test_a_matching_detection_is_returned_for_binding(impl) -> None:
    _, skills = at_the_counter()
    result = skills.call("detect_objects")
    v = impl.verify_detect(skills, {}, result, label="water bottle")
    if v.ok:
        assert v.observed["detection"]["object_id"] == "bottle-1"


# ----------------------------------------------------------------------------------------------
# dispatch
# ----------------------------------------------------------------------------------------------
def test_a_failed_call_is_not_verified_without_spending_robot_time(impl, skills: RobotSkills) -> None:
    result = skills.call("pick", {"object_id": "bottle-1"})  # never detected
    assert not result.ok
    t0 = skills.world.t
    v = impl.verify_step(skills, "pick", {"object_id": "bottle-1"}, result)
    assert not v.ok and v.method == "result" and skills.world.t == pytest.approx(t0)


def test_verification_can_be_switched_off(impl, skills: RobotSkills) -> None:
    result = skills.call("list_places")
    v = impl.verify_step(skills, "list_places", {}, result, level="none")
    assert v.code == "UNCHECKABLE"


def test_a_read_only_skill_has_nothing_to_verify(impl, skills: RobotSkills) -> None:
    result = skills.call("list_places")
    assert impl.verify_step(skills, "list_places", {}, result).code == "UNCHECKABLE"


# ----------------------------------------------------------------------------------------------
# the ladder
# ----------------------------------------------------------------------------------------------
def verified(impl, ok: bool = True, method: str = "state", repeatable: bool = True):
    return impl.Verification("x", ok, "VERIFIED" if ok else "NOT_VERIFIED", method,
                             repeatable=repeatable)


def test_a_verified_step_moves_on(impl) -> None:
    assert impl.choose_repair("navigate_to", "SUCCEEDED", verified(impl), 1, 2, True, False) == "ok"


def test_an_idempotent_skill_with_budget_left_is_retried(impl) -> None:
    v = verified(impl, ok=False)
    assert impl.choose_repair("navigate_to", "BLOCKED", v, 1, 2, True, False) == "retry"


def test_a_non_idempotent_skill_is_never_retried(impl) -> None:
    v = verified(impl, ok=False)
    assert impl.choose_repair("place", "SUCCEEDED", v, 1, 3, False, False) == "replan"


def test_a_failed_grasp_skips_the_retry_rung(impl) -> None:
    v = verified(impl, ok=False)
    assert impl.choose_repair("pick", "GRASP_FAILED", v, 1, 3, True, True) == "repair"
    assert impl.choose_repair("pick", "SUCCEEDED", v, 1, 3, True, True) == "repair"


def test_without_a_binding_step_in_front_a_failed_pick_replans(impl) -> None:
    v = verified(impl, ok=False)
    assert impl.choose_repair("pick", "GRASP_FAILED", v, 1, 3, True, False) == "replan"


def test_the_last_attempt_drops_to_the_next_rung(impl) -> None:
    v = verified(impl, ok=False)
    assert impl.choose_repair("navigate_to", "NO_PATH", v, 2, 2, True, False) == "replan"


# ----------------------------------------------------------------------------------------------
# looking again
# ----------------------------------------------------------------------------------------------
def test_a_missed_detection_is_worth_a_second_look(impl) -> None:
    assert impl.should_look_again(verified(impl, ok=False, method="perception"), 1, 2)


def test_a_state_check_is_not_worth_repeating(impl) -> None:
    assert not impl.should_look_again(verified(impl, ok=False, method="state"), 1, 2)


def test_a_non_repeatable_check_is_never_repeated(impl) -> None:
    v = verified(impl, ok=False, method="perception", repeatable=False)
    assert not impl.should_look_again(v, 1, 2)


def test_a_passing_check_is_not_repeated(impl) -> None:
    assert not impl.should_look_again(verified(impl, ok=True, method="perception"), 1, 2)


def test_the_look_budget_is_respected(impl) -> None:
    assert not impl.should_look_again(verified(impl, ok=False, method="perception"), 2, 2)
