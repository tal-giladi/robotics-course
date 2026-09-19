"""Checker for 19.02 — the robot's skill API: generated schemas and enforced arguments.

Run: ``python course.py check 19.02`` (``--solution`` runs the reference).
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_REPO = Path(__file__).resolve().parents[3]
for _p in (_REPO / "19-llm-robot-agents" / "code", _REPO / "labs" / "python"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

PLACES =("bed", "bedroom", "charging_dock", "coffee_table", "desk", "kitchen",
          "kitchen_counter", "kitchen_table", "living_room", "sofa", "study")


def navigate_schema(impl):
    """The real navigate_to input schema, assembled from the student's param_schema."""
    return {
        "type": "object",
        "properties": {
            "place": impl.param_schema("place", "string", "Exact place name from list_places.", enum=PLACES),
            "timeout_s": impl.param_schema("timeout_s", "number", "Give up (and stop the robot) after this long.",
                                           unit="s", minimum=1.0, maximum=300.0, default=90.0),
        },
        "required": ["place"],
        "additionalProperties": False,
    }


# --- param_schema -----------------------------------------------------------------------------
def test_minimal_parameter_is_just_type_and_description(impl):
    assert impl.param_schema("text", "string", "What to say.") == {
        "type": "string", "description": "What to say."}


def test_units_ranges_and_defaults_all_reach_the_schema(impl):
    s = impl.param_schema("timeout_s", "number", "Give up (and stop the robot) after this long.",
                          unit="s", minimum=1.0, maximum=300.0, default=90.0)
    assert s["type"] == "number"
    assert s["x-unit"] == "s", "the unit must survive into the schema, not only into the prose"
    assert (s["minimum"], s["maximum"], s["default"]) == (1.0, 300.0, 90.0)
    assert s["description"] == ("Give up (and stop the robot) after this long. Unit: s. "
                                "Range: [1.0, 300.0]. Default: 90.0.")


def test_an_enum_is_a_list_so_it_can_be_serialised(impl):
    s = impl.param_schema("place", "string", "Exact place name from list_places.", enum=PLACES)
    assert s["enum"] == list(PLACES) and isinstance(s["enum"], list)
    assert s["description"] == "Exact place name from list_places.", "no range/default sentence here"


def test_a_half_open_range_still_documents_itself(impl):
    s = impl.param_schema("min_confidence", "number", "Drop detections below this.", minimum=0.0)
    assert s["minimum"] == 0.0 and "maximum" not in s
    assert s["description"].endswith("Range: [0.0, None].")


def test_pattern_and_no_spurious_keys(impl):
    s = impl.param_schema("object_id", "string", "From a recent detect_objects result.", pattern=r"[a-z]+-\d+")
    assert s["pattern"] == r"[a-z]+-\d+"
    assert set(s) == {"type", "pattern", "description"}, "only declared constraints belong in the schema"


# --- validate_arguments: the happy path -------------------------------------------------------
@pytest.mark.parametrize("args", [
    {"place": "kitchen"},                       # optional parameter omitted
    {"place": "kitchen", "timeout_s": 30.0},
    {"place": "kitchen", "timeout_s": 30},      # an int is an acceptable "number"
    {"place": "kitchen", "timeout_s": 1.0},     # on the boundary
    {"place": "kitchen", "timeout_s": 300.0},   # on the boundary
])
def test_valid_calls_produce_no_errors(impl, args):
    assert impl.validate_arguments(navigate_schema(impl), args) == []


# --- validate_arguments: the rejections -------------------------------------------------------
@pytest.mark.parametrize(("args", "fragment"), [
    ({}, "place: required"),
    ({"place": "garage"}, "'garage' is not one of"),
    ({"place": "kitchen", "timeout_s": 900}, "timeout_s: 900 s is above the maximum 300.0 s"),
    ({"place": "kitchen", "timeout_s": 0.5}, "timeout_s: 0.5 s is below the minimum 1.0 s"),
    ({"place": "kitchen", "timeout_s": "30 s"}, "timeout_s: expected a number, got '30 s'"),
    ({"place": "kitchen", "timeout_s": float("nan")}, "timeout_s: must be finite"),
    ({"place": "kitchen", "timeout_s": float("inf")}, "timeout_s: must be finite"),
    ({"place": "kitchen", "timeout_s": True}, "timeout_s: expected a number, got True"),
    ({"place": 7}, "place: expected a string, got 7"),
    ({"place": "kitchen", "speed": 2.0}, "speed: unknown parameter"),
])
def test_bad_calls_are_rejected_with_a_message_that_teaches(impl, args, fragment):
    errors = impl.validate_arguments(navigate_schema(impl), args)
    assert any(fragment in e for e in errors), f"expected {fragment!r} among {errors}"


def test_a_range_error_names_the_unit(impl):
    """'900 is above the maximum 300' is ambiguous; '900 s ... 300.0 s' is not."""
    errors = impl.validate_arguments(navigate_schema(impl), {"place": "kitchen", "timeout_s": 900})
    assert errors and all(" s " in e for e in errors)


def test_a_type_error_does_not_also_raise_a_range_error(impl):
    errors = impl.validate_arguments(navigate_schema(impl), {"place": "kitchen", "timeout_s": "30 s"})
    assert len(errors) == 1, f"comparing a string with 1.0 must not happen at all: {errors}"


def test_every_problem_is_reported_not_just_the_first(impl):
    errors = impl.validate_arguments(navigate_schema(impl), {"place": "garage", "timeout_s": 900, "speed": 2.0})
    assert len(errors) == 3, f"the caller should learn about all three mistakes at once: {errors}"


def test_arguments_that_are_not_an_object_at_all(impl):
    assert impl.validate_arguments(navigate_schema(impl), ["kitchen"]) == [
        "arguments must be an object, got list"]
    assert impl.validate_arguments(navigate_schema(impl), None) == [
        "arguments must be an object, got NoneType"]


def test_a_pattern_is_enforced(impl):
    schema = {"type": "object", "additionalProperties": False, "required": ["object_id"],
              "properties": {"object_id": impl.param_schema("object_id", "string", "From detect_objects.",
                                                            pattern=r"[a-z]+-\d+")}}
    assert impl.validate_arguments(schema, {"object_id": "bottle-1"}) == []
    assert impl.validate_arguments(schema, {"object_id": "the water bottle"}) != []


def test_integers_and_booleans_are_not_interchangeable(impl):
    schema = {"type": "object", "additionalProperties": False, "required": [],
              "properties": {"n": impl.param_schema("n", "integer", "How many."),
                             "loud": impl.param_schema("loud", "boolean", "Shout.")}}
    assert impl.validate_arguments(schema, {"n": 3, "loud": False}) == []
    assert impl.validate_arguments(schema, {"n": True}) == ["n: expected an integer, got True"]
    assert impl.validate_arguments(schema, {"n": 3.0}) == ["n: expected an integer, got 3.0"]
    assert impl.validate_arguments(schema, {"loud": 1}) == ["loud: expected true/false, got 1"]


def test_extra_keys_are_reported_once_and_not_type_checked(impl):
    errors = impl.validate_arguments(navigate_schema(impl), {"place": "kitchen", "speed": "fast"})
    assert errors == ["speed: unknown parameter"]


def test_it_matches_the_course_gateway(impl):
    """The same call, judged by the student's validator and by robot_agent's — identically."""
    reference = pytest.importorskip("robot_agent.skills", reason="module 19 code not on sys.path")
    schema = navigate_schema(impl)
    for args in ({"place": "kitchen"}, {"place": "garage"}, {"place": "kitchen", "timeout_s": 900},
                 {"place": "kitchen", "speed": 2.0}, {}):
        assert sorted(impl.validate_arguments(schema, args)) == sorted(
            reference.validate_arguments(schema, args)), f"disagreement on {args}"
