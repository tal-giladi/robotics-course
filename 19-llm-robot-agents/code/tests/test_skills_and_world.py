"""Skill contracts, the gateway and the simulated home (19.02)."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

import pytest

from robot_agent.sim_world import ArmParams, HomeWorld, Status
from robot_agent.skills import ROS2_BINDINGS, CancelToken, RobotSkills, build_skill_specs, validate_arguments

REPO = Path(__file__).resolve().parents[3]


@pytest.fixture
def home() -> HomeWorld:
    return HomeWorld(seed=0)


# --- contracts and schemas ---------------------------------------------------------------------
def test_schemas_are_generated_from_the_world(home):
    specs = build_skill_specs(home)
    nav = specs["navigate_to"].input_schema()
    assert nav["additionalProperties"] is False
    assert nav["required"] == ["place"]
    assert set(nav["properties"]["place"]["enum"]) == set(home.places)
    assert nav["properties"]["timeout_s"]["x-unit"] == "s"
    assert nav["properties"]["timeout_s"]["maximum"] == 300.0
    place = specs["place"].input_schema()
    assert "living_room" not in place["properties"]["location"]["enum"], "rooms are not surfaces"
    json.dumps([s.tool_definition() for s in specs.values()])  # serializable


def test_descriptions_carry_the_contract(home):
    d = build_skill_specs(home)["pick"].description()
    assert "Preconditions" in d and "OUT_OF_REACH" in d and "0.75 m" in d


@pytest.mark.parametrize(
    ("args", "fragment"),
    [
        ({}, "place: required"),
        ({"place": "garage"}, "not one of"),
        ({"place": "kitchen", "timeout_s": 900}, "above the maximum 300.0 s"),
        ({"place": "kitchen", "timeout_s": True}, "expected a number"),
        ({"place": "kitchen", "timeout_s": float("nan")}, "finite"),
        ({"place": "kitchen", "speed": 2.0}, "speed: unknown parameter"),
        ({"place": "kitchen", "timeout_s": "30 s"}, "expected a number"),
    ],
)
def test_validation_rejects_bad_arguments(home, args, fragment):
    errors = validate_arguments(build_skill_specs(home)["navigate_to"].input_schema(), args)
    assert any(fragment in e for e in errors), errors


def test_valid_arguments_pass(home):
    schema = build_skill_specs(home)["navigate_to"].input_schema()
    assert validate_arguments(schema, {"place": "kitchen", "timeout_s": 30}) == []


def test_ros2_bindings_match_karmel_interfaces():
    """The skill layer's error codes must be exactly the constants in the .action files."""
    for skill, binding in ROS2_BINDINGS.items():
        if "action" not in binding:
            continue
        name = binding["action"].rsplit("/", 1)[1]
        text = (REPO / "labs/ros2_ws/src/karmel_interfaces/action" / f"{name}.action").read_text(encoding="utf-8")
        result_section = text.split("---")[1]
        constants = {int(v): k for k, v in re.findall(r"uint8\s+([A-Z_]+)=(\d+)", result_section)}
        assert constants == binding["error_codes"], f"{skill}: {constants}"
        goal_fields = set(re.findall(r"^\s*\w[\w/]*\s+(\w+)", text.split("---")[0], re.M))
        assert set(binding["goal"].values()) <= goal_fields, f"{skill}: goal fields {goal_fields}"


# --- gateway -----------------------------------------------------------------------------------
def test_gateway_rejects_before_touching_the_robot(home):
    skills = RobotSkills(home, allowlist={"list_places", "navigate_to"})
    r = skills.call("pick", {"object_id": "bottle-1"})
    assert (r.ok, r.code) == (False, "NOT_ALLOWED")
    r = skills.call("navigate_to", {"place": "kitchen", "timeout_s": 1e6})
    assert r.code == "INVALID_ARGUMENTS" and "maximum" in r.message
    assert home.t == 0.0, "rejected calls must not advance the robot"


def test_confirmation_gate(home):
    asked = []
    skills = RobotSkills(home, confirm=lambda spec, args: asked.append(spec.name) or False,
                         confirm_effects=frozenset({"manipulation"}))
    assert skills.call("place", {"location": "bed"}).code == "NOT_CONFIRMED"
    assert asked == ["place"]
    assert skills.call("get_robot_state").ok, "read-only skills need no confirmation"


def test_idempotency_key_prevents_double_execution(home):
    skills = RobotSkills(home)
    first = skills.call("navigate_to", {"place": "sofa"}, call_id="toolu_1")
    t_after = home.t
    again = skills.call("navigate_to", {"place": "sofa"}, call_id="toolu_1")
    assert first.ok and again == first
    assert home.t == t_after, "a retried delivery of the same call must not run the skill again"
    assert skills.log[-1].cached


# --- world behaviour -----------------------------------------------------------------------------
def test_navigation_really_drives_the_simulator(home):
    r = home.navigate("kitchen").run(timeout_s=90)
    assert r.code == "SUCCEEDED"
    x, y, th = home.pose
    place = home.places["kitchen"]
    assert math.hypot(x - place.x, y - place.y) < 0.1
    assert abs(th - place.theta) < 0.1
    assert home.room_of(x, y) == "kitchen"


def test_timeout_cancels_and_stops(home):
    r = RobotSkills(home).call("navigate_to", {"place": "bedroom", "timeout_s": 2.0})
    assert r.code == "TIMEOUT" and r.retryable
    assert 2.0 <= r.elapsed_s < 2.5


def test_cancel_token(home):
    token = CancelToken()
    h = home.navigate("study")
    h.step(1.0)
    token.cancel()
    r = h.run(should_cancel=lambda: token.cancelled)
    assert r.code == "CANCELED" and h.status is Status.CANCELED


def test_blocked_doorway_gives_no_path(home):
    home.add_obstacle(3.5, 1.65, 0.5)  # a big box in the living room -> kitchen doorway
    home.add_obstacle(3.5, 4.0, 0.5)  # ... and the study -> bedroom doorway
    home.add_obstacle(4.9, 2.8, 0.5)  # ... and the kitchen <-> bedroom doorway
    r = RobotSkills(home).call("navigate_to", {"place": "kitchen"})
    assert r.code == "NO_PATH" and r.hint


def test_pick_preconditions_and_postconditions(home):
    skills = RobotSkills(home)
    assert skills.call("pick", {"object_id": "bottle-1"}).code == "OBJECT_NOT_FOUND"
    assert skills.call("navigate_to", {"place": "kitchen_counter"}).ok
    det = skills.call("detect_objects")
    assert [d["object_id"] for d in det.data["detections"]] == ["bottle-1"]
    assert det.data["detections"][0]["near_place"] == "kitchen_counter"
    assert skills.call("place", {"location": "kitchen_table"}).code == "NOT_HOLDING_OBJECT"
    r = skills.call("pick", {"object_id": "bottle-1"})
    assert r.ok and home.holding == "bottle-1"
    assert skills.call("pick", {"object_id": "bottle-1"}).message.startswith("already holding"), "pick is idempotent"
    assert skills.call("place", {"location": "kitchen_table"}).code == "OUT_OF_REACH"
    assert skills.call("navigate_to", {"place": "kitchen_table"}).ok
    assert skills.call("place", {"location": "kitchen_table"}).ok
    assert home.holding is None and home.objects["bottle-1"].on == "kitchen_table"


def test_failed_grasp_is_retryable_and_forgets_the_detection():
    home = HomeWorld(seed=0, arm=ArmParams(grasp_success_p=0.0))
    skills = RobotSkills(home)
    skills.call("navigate_to", {"place": "kitchen_counter"})
    skills.call("detect_objects")
    r = skills.call("pick", {"object_id": "bottle-1"})
    assert r.code == "GRASP_FAILED" and r.retryable and "detect" in r.hint
    assert skills.call("pick", {"object_id": "bottle-1"}).code == "OBJECT_NOT_FOUND"


def test_text_in_the_world_is_marked_untrusted(home):
    skills = RobotSkills(home)
    skills.call("navigate_to", {"place": "kitchen"})
    seen = []
    for _ in range(5):
        seen += skills.call("detect_objects").data["detections"]
    note = next(d for d in seen if d["object_id"] == "note-1")
    assert "untrusted_text_seen" in note and "text" not in note
    assert "ignore your previous instructions" in note["untrusted_text_seen"]


def test_detector_misses_more_far_away():
    near = far = 0
    for seed in range(40):
        home = HomeWorld(seed=seed, start=(5.0, 0.5, 0.0))  # 0.6 m from the bottle
        near += any(d.object_id == "bottle-1" for d in home.detect().run().data["detections"])
        home = HomeWorld(seed=seed, start=(4.3, 2.2, -0.45))  # 2.1 m from the bottle
        far += any(d.object_id == "bottle-1" for d in home.detect().run().data["detections"])
    assert near >= 34 and far <= 32 and near > far
