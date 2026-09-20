"""Checker for 19.09 — the safety layer's rules. Pure functions: no robot, no model."""

from __future__ import annotations

import pytest

EFFECTS = {"list_places": "none", "get_robot_state": "none", "detect_objects": "none",
           "navigate_to": "motion", "pick": "manipulation", "place": "manipulation"}
ALL_SKILLS = tuple(EFFECTS)


def ctx(impl, **kw):
    kw.setdefault("skills", ALL_SKILLS)
    kw.setdefault("effects", EFFECTS)
    kw.setdefault("mode_skills", ALL_SKILLS)
    return impl.Context(**kw)


def bedroom(impl):
    return impl.Zone("bedroom", 3.5, 2.9, 6.0, 5.0, "someone is asleep in there")


# ----------------------------------------------------------------------------------------------
# Order and the default
# ----------------------------------------------------------------------------------------------
def test_an_ordinary_call_is_allowed(impl) -> None:
    d = impl.check("navigate_to", {"place": "kitchen"}, ctx(impl))
    assert d.allowed and d.code == "OK"


def test_the_lock_beats_every_other_rule(impl) -> None:
    c = ctx(impl, locked=True, mode_skills=(), zones=(bedroom(impl),), destination=(5.0, 3.3))
    assert impl.check("navigate_to", {"place": "bed"}, c).rule == "estop"


def test_the_mode_is_checked_before_the_geofence(impl) -> None:
    c = ctx(impl, mode_skills=("list_places",), zones=(bedroom(impl),), destination=(5.0, 3.3))
    assert impl.check("navigate_to", {"place": "bed"}, c).rule == "mode"


def test_the_geofence_is_checked_before_the_budget(impl) -> None:
    budget = impl.Budget(distance_m=1000.0)
    c = ctx(impl, zones=(bedroom(impl),), destination=(5.0, 3.3), budget=budget)
    assert impl.check("navigate_to", {"place": "bed"}, c).rule == "geofence"


def test_the_goal_lock_is_checked_before_confirmation(impl) -> None:
    c = ctx(impl, goal_surface="kitchen_table", goal_label="water bottle",
            confirm_effects=("manipulation",), confirmed=False)
    assert impl.check("place", {"location": "bed"}, c).rule == "goal_lock"


# ----------------------------------------------------------------------------------------------
# Mode
# ----------------------------------------------------------------------------------------------
def test_a_skill_outside_the_mode_is_refused_with_the_allowed_list(impl) -> None:
    c = ctx(impl, mode_skills=("list_places", "detect_objects", "navigate_to"))
    d = impl.check("pick", {"object_id": "bottle-1"}, c)
    assert not d.allowed and d.code == "NOT_ALLOWED" and "navigate_to" in d.message


def test_an_empty_mode_refuses_everything_and_says_nothing_is_allowed(impl) -> None:
    d = impl.check("list_places", {}, ctx(impl, mode_skills=()))
    assert not d.allowed and "nothing" in d.message


def test_visible_tools_is_the_intersection_sorted(impl) -> None:
    c = ctx(impl, mode_skills=("navigate_to", "list_places", "fly"))
    assert impl.visible_tools(c) == ["list_places", "navigate_to"]


# ----------------------------------------------------------------------------------------------
# Geofence
# ----------------------------------------------------------------------------------------------
def test_a_destination_inside_the_zone_is_refused(impl) -> None:
    c = ctx(impl, zones=(bedroom(impl),), destination=(5.0, 3.3))
    d = impl.check("navigate_to", {"place": "bed"}, c)
    assert not d.allowed and d.code == "GEOFENCE" and "asleep" in d.message


def test_a_permitted_destination_reached_through_the_zone_is_refused(impl) -> None:
    c = ctx(impl, zones=(bedroom(impl),), destination=(2.0, 4.0),
            path=[(1.0, 1.3), (4.0, 3.5), (2.0, 4.0)])
    assert impl.check("navigate_to", {"place": "study"}, c).code == "GEOFENCE"


def test_a_route_that_stays_outside_is_allowed(impl) -> None:
    c = ctx(impl, zones=(bedroom(impl),), destination=(2.0, 4.0),
            path=[(1.0, 1.3), (1.5, 2.5), (2.0, 4.0)])
    assert impl.check("navigate_to", {"place": "study"}, c).allowed


def test_the_margin_keeps_the_robot_off_the_boundary(impl) -> None:
    c = ctx(impl, zones=(bedroom(impl),), destination=(3.4, 3.0), margin_m=0.15)
    assert impl.check("navigate_to", {"place": "edge"}, c).code == "GEOFENCE"


def test_the_geofence_only_constrains_motion(impl) -> None:
    c = ctx(impl, zones=(bedroom(impl),), destination=(5.0, 3.3))
    assert impl.check("detect_objects", {}, c).allowed


def test_no_zones_means_no_denials(impl) -> None:
    assert impl.zone_violated(ctx(impl, destination=(5.0, 3.3))) is None


# ----------------------------------------------------------------------------------------------
# Goal lock
# ----------------------------------------------------------------------------------------------
def test_the_goal_lock_allows_the_surface_the_user_named(impl) -> None:
    c = ctx(impl, goal_surface="kitchen_table", goal_label="water bottle")
    assert impl.check("place", {"location": "kitchen_table"}, c).allowed


def test_the_goal_lock_refuses_any_other_surface(impl) -> None:
    c = ctx(impl, goal_surface="kitchen_table", goal_label="water bottle")
    d = impl.check("place", {"location": "bed"}, c)
    assert d.code == "GOAL_VIOLATION" and "kitchen_table" in d.message and "bed" in d.message


def test_the_goal_lock_does_not_constrain_driving_or_looking(impl) -> None:
    c = ctx(impl, goal_surface="kitchen_table", goal_label="water bottle")
    assert impl.check("navigate_to", {"place": "bedroom"}, c).allowed
    assert impl.check("detect_objects", {}, c).allowed


def test_without_a_goal_there_is_nothing_to_violate(impl) -> None:
    assert impl.check("place", {"location": "bed"}, ctx(impl)).allowed


# ----------------------------------------------------------------------------------------------
# Budget
# ----------------------------------------------------------------------------------------------
def test_distance_is_bounded(impl) -> None:
    c = ctx(impl, budget=impl.Budget(max_distance_m=10.0, distance_m=10.5))
    d = impl.check("navigate_to", {"place": "kitchen"}, c)
    assert d.code == "BUDGET_EXCEEDED" and "m driven" in d.message


def test_manipulations_are_counted_apart_from_motion(impl) -> None:
    budget = impl.Budget(max_manipulations=2, manipulations=2)
    c = ctx(impl, budget=budget)
    assert impl.check("pick", {"object_id": "bottle-1"}, c).code == "BUDGET_EXCEEDED"
    assert impl.check("navigate_to", {"place": "kitchen"}, c).allowed


def test_a_single_skill_cannot_be_called_forever(impl) -> None:
    c = ctx(impl, budget=impl.Budget(max_calls_per_skill=3, calls={"detect_objects": 3}))
    assert impl.check("detect_objects", {}, c).code == "BUDGET_EXCEEDED"


def test_robot_time_is_bounded(impl) -> None:
    c = ctx(impl, budget=impl.Budget(max_robot_time_s=100.0, robot_time_s=101.0))
    assert "robot time" in impl.check("navigate_to", {"place": "kitchen"}, c).message


def test_a_fresh_budget_blocks_nothing(impl) -> None:
    assert impl.budget_exceeded("navigate_to", "motion", impl.Budget()) is None


# ----------------------------------------------------------------------------------------------
# Confirmation
# ----------------------------------------------------------------------------------------------
def test_confirmation_is_required_per_effect_class(impl) -> None:
    c = ctx(impl, confirm_effects=("manipulation",), confirmed=False)
    assert impl.check("pick", {"object_id": "bottle-1"}, c).code == "NOT_CONFIRMED"
    assert impl.check("navigate_to", {"place": "kitchen"}, c).allowed


def test_a_confirmed_call_goes_through(impl) -> None:
    c = ctx(impl, confirm_effects=("manipulation",), confirmed=True)
    assert impl.check("pick", {"object_id": "bottle-1"}, c).allowed


# ----------------------------------------------------------------------------------------------
# Every rule must hold on its own
# ----------------------------------------------------------------------------------------------
@pytest.mark.parametrize("rule,skill,args,extra", [
    ("estop", "list_places", {}, {"locked": True}),
    ("mode", "pick", {"object_id": "bottle-1"}, {"mode_skills": ("list_places",)}),
    ("geofence", "navigate_to", {"place": "bed"}, {"destination": (5.0, 3.3)}),
    ("goal_lock", "place", {"location": "bed"}, {"goal_surface": "kitchen_table",
                                                 "goal_label": "water bottle"}),
    ("budget", "pick", {"object_id": "bottle-1"}, {"budget_manip": True}),
    ("confirmation", "pick", {"object_id": "bottle-1"}, {"confirm_effects": ("manipulation",)}),
])
def test_each_rule_denies_on_its_own(impl, rule, skill, args, extra) -> None:
    kw = dict(extra)
    if kw.pop("budget_manip", False):
        kw["budget"] = impl.Budget(max_manipulations=0)
    if rule == "geofence":
        kw["zones"] = (bedroom(impl),)
    d = impl.check(skill, args, ctx(impl, **kw))
    assert not d.allowed and d.rule == rule, f"{rule} alone did not deny: {d}"
