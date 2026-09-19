"""Tests for 15.09 — planning-scene identity, collision arithmetic, allowances and recovery."""

from __future__ import annotations

import math

import numpy as np
import pytest


def _tool(position, yaw: float = 0.0):
    x = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    T = np.eye(4)
    T[:3, :3] = np.column_stack((x, np.cross(z, x), z))
    T[:3, 3] = np.asarray(position, dtype=float)
    return T


def _table(impl):
    return impl.CollisionBox("table", (0.25, 0.0, -0.010), (0.70, 0.70, 0.020))


# ---------------------------------------------------------------- box distance
def test_distance_on_a_face(impl):
    box = impl.CollisionBox("b", (0.0, 0.0, 0.0), (0.100, 0.060, 0.040))
    assert impl.box_distance(box, (0.05, 0.0, 0.0)) == pytest.approx(0.0, abs=1e-12)
    assert impl.box_distance(box, (0.08, 0.0, 0.0)) == pytest.approx(0.03)
    assert impl.box_distance(box, (0.0, 0.05, 0.0)) == pytest.approx(0.02)


def test_distance_is_negative_inside(impl):
    box = impl.CollisionBox("b", (0.0, 0.0, 0.0), (0.100, 0.060, 0.040))
    assert impl.box_distance(box, (0.0, 0.0, 0.0)) == pytest.approx(-0.020)
    assert impl.box_distance(box, (0.04, 0.0, 0.0)) == pytest.approx(-0.010)


def test_distance_at_a_corner_is_euclidean(impl):
    box = impl.CollisionBox("b", (0.0, 0.0, 0.0), (0.100, 0.060, 0.040))
    d = impl.box_distance(box, (0.08, 0.06, 0.02))
    assert d == pytest.approx(math.hypot(0.03, 0.03))


def test_yaw_is_respected(impl):
    box = impl.CollisionBox("b", (0.0, 0.0, 0.0), (0.100, 0.020, 0.040), yaw=math.pi / 2)
    # rotated 90 deg: the long axis now points along y
    assert impl.box_distance(box, (0.0, 0.05, 0.0)) == pytest.approx(0.0, abs=1e-12)
    assert impl.box_distance(box, (0.05, 0.0, 0.0)) == pytest.approx(0.04)


def test_translation_is_respected(impl):
    box = impl.CollisionBox("b", (0.25, 0.10, 0.02), (0.100, 0.060, 0.040))
    assert impl.box_distance(box, (0.25, 0.10, 0.02)) == pytest.approx(-0.020)
    assert impl.box_distance(box, (0.35, 0.10, 0.02)) == pytest.approx(0.05)


# ---------------------------------------------------------------- collides
def test_clear_above_the_table(impl):
    assert impl.collides(_tool((0.25, 0.0, 0.20)), [_table(impl)]) is None


def test_the_pads_hit_the_padded_table(impl):
    assert impl.collides(_tool((0.25, 0.0, 0.005)), [_table(impl)]) == "table"


def test_collides_returns_the_id_not_a_bool(impl):
    box = impl.CollisionBox("mug", (0.25, 0.0, 0.05), (0.08, 0.08, 0.10))
    hit = impl.collides(_tool((0.25, 0.0, 0.05)), [_table(impl), box])
    assert hit == "mug"


def test_ignore_suppresses_that_object_only(impl):
    box = impl.CollisionBox("mug", (0.25, 0.0, 0.05), (0.08, 0.08, 0.10))
    obstacles = [_table(impl), box]
    assert impl.collides(_tool((0.25, 0.0, 0.05)), obstacles, ignore=frozenset({"mug"})) is None
    assert impl.collides(_tool((0.25, 0.0, 0.005)), obstacles,
                         ignore=frozenset({"mug"})) == "table"


def test_more_padding_means_more_collisions(impl):
    box = impl.CollisionBox("wall", (0.25, 0.10, 0.05), (0.02, 0.02, 0.10))
    T = _tool((0.25, 0.0, 0.05))
    assert impl.collides(T, [box], padding_m=0.000) is None
    assert impl.collides(T, [box], padding_m=0.080) == "wall"


# ---------------------------------------------------------------- reconcile
def _scene_with(impl, *boxes):
    scene = impl.PlanningScene()
    for b in boxes:
        scene.add(b)
    return scene


def test_a_still_table_produces_no_diff(impl):
    a = impl.CollisionBox("block", (0.24, 0.06, 0.02), (0.06, 0.03, 0.04))
    scene = _scene_with(impl, _table(impl), a)
    diff = impl.reconcile(scene, [impl.CollisionBox("object_0", a.centre, a.size)])
    assert diff.is_empty, "re-publishing an unmoved object every frame floods /planning_scene"


def test_the_table_is_never_removed(impl):
    scene = _scene_with(impl, _table(impl))
    assert impl.reconcile(scene, []).removed == ()


def test_identity_survives_a_reordered_detection_list(impl):
    a = impl.CollisionBox("block", (0.24, 0.06, 0.02), (0.06, 0.03, 0.04))
    b = impl.CollisionBox("eraser", (0.19, 0.16, 0.012), (0.045, 0.024, 0.024))
    scene = _scene_with(impl, _table(impl), a, b)
    dets = [impl.CollisionBox("object_0", (0.19, 0.16, 0.012), b.size),   # the eraser, first now
            impl.CollisionBox("object_1", (0.24, 0.072, 0.02), a.size)]   # the block, moved 12 mm
    diff = impl.reconcile(scene, dets)
    assert diff.added == ()
    assert diff.removed == ()
    assert [m.object_id for m in diff.moved] == ["block"]
    assert diff.moved[0].centre == pytest.approx((0.24, 0.072, 0.02))


def test_a_new_object_is_added_with_its_own_id(impl):
    scene = _scene_with(impl, _table(impl))
    det = impl.CollisionBox("object_0", (0.30, -0.10, 0.05), (0.06, 0.06, 0.10))
    diff = impl.reconcile(scene, [det])
    assert [a.object_id for a in diff.added] == ["object_0"]
    assert diff.moved == () and diff.removed == ()


def test_a_vanished_object_is_removed(impl):
    a = impl.CollisionBox("block", (0.24, 0.06, 0.02), (0.06, 0.03, 0.04))
    scene = _scene_with(impl, _table(impl), a)
    assert impl.reconcile(scene, []).removed == ("block",)


def test_the_attached_object_is_not_removed_for_being_unseen(impl):
    a = impl.CollisionBox("block", (0.24, 0.06, 0.02), (0.06, 0.03, 0.04))
    scene = _scene_with(impl, _table(impl), a)
    scene.attached = "block"
    diff = impl.reconcile(scene, [])
    assert diff.removed == (), "it is in the gripper; 'not seen' must not mean 'gone'"


def test_a_far_detection_is_a_new_object_not_a_move(impl):
    a = impl.CollisionBox("block", (0.24, 0.06, 0.02), (0.06, 0.03, 0.04))
    scene = _scene_with(impl, _table(impl), a)
    far = impl.CollisionBox("object_0", (0.24, 0.20, 0.02), a.size)      # 140 mm away
    diff = impl.reconcile(scene, [far], match_radius_m=0.04)
    assert [b.object_id for b in diff.added] == ["object_0"]
    assert diff.removed == ("block",)


def test_two_detections_cannot_claim_the_same_id(impl):
    a = impl.CollisionBox("block", (0.24, 0.06, 0.02), (0.06, 0.03, 0.04))
    scene = _scene_with(impl, _table(impl), a)
    dets = [impl.CollisionBox("object_0", (0.245, 0.062, 0.02), a.size),
            impl.CollisionBox("object_1", (0.250, 0.065, 0.02), a.size)]
    diff = impl.reconcile(scene, dets)
    ids = [b.object_id for b in diff.added + diff.moved]
    assert len(ids) == len(set(ids))
    assert len(diff.added) == 1, "one detection matches, the other is genuinely new"


def test_a_size_change_counts_as_a_move(impl):
    a = impl.CollisionBox("block", (0.24, 0.06, 0.02), (0.06, 0.03, 0.04))
    scene = _scene_with(impl, _table(impl), a)
    det = impl.CollisionBox("object_0", a.centre, (0.06, 0.045, 0.04))
    assert [m.object_id for m in impl.reconcile(scene, [det]).moved] == ["block"]


# ---------------------------------------------------------------- allowances
def test_allowances_cover_the_target_and_the_table(impl):
    a = impl.grasp_allowances("block")
    assert "block" in a and "table" in a
    assert len(a) == 2


def test_allowances_do_not_cover_anything_else(impl):
    assert "neighbour" not in impl.grasp_allowances("block")


def test_a_short_object_needs_the_table_allowance(impl):
    """A 24 mm eraser is grasped 9 mm above the table — inside the padded table."""
    eraser = impl.CollisionBox("eraser", (0.25, 0.0, 0.012), (0.045, 0.024, 0.024))
    obstacles = [_table(impl), eraser]
    T = _tool((0.25, 0.0, 0.009))
    assert impl.collides(T, obstacles, padding_m=0.005) is not None
    assert impl.collides(T, obstacles, padding_m=0.005,
                         ignore=impl.grasp_allowances("eraser")) is None


# ---------------------------------------------------------------- recovery
def test_the_ladder_grows_one_rung_per_attempt(impl):
    assert len(impl.recovery_plan(impl.Failure.PLAN_FAILED, 0)) == 1
    assert len(impl.recovery_plan(impl.Failure.PLAN_FAILED, 1)) == 2
    assert len(impl.recovery_plan(impl.Failure.PLAN_FAILED, 2)) == 3


def test_the_ladder_never_runs_off_the_end(impl):
    steps = impl.recovery_plan(impl.Failure.DROPPED, 99)
    assert len(steps) == len(impl.LADDER[impl.Failure.DROPPED])


def test_the_cheapest_step_comes_first(impl):
    assert impl.recovery_plan(impl.Failure.COLLISION_IN_MOTION, 0)[0].name == "stop"
    assert impl.recovery_plan(impl.Failure.DROPPED, 0)[0].name == "detach_in_scene"


def test_holding_prepends_putting_it_down(impl):
    steps = impl.recovery_plan(impl.Failure.PLAN_FAILED, 0, holding=True)
    assert [s.name for s in steps[:2]] == ["place_held_object_safely", "detach_in_scene"]
    assert steps[2].name == "replan"


def test_holding_does_not_change_recoveries_that_already_assume_it(impl):
    """DROPPED already starts by detaching; prepending 'place the object' would be nonsense."""
    steps = impl.recovery_plan(impl.Failure.DROPPED, 0, holding=True)
    assert steps[0].name == "detach_in_scene"


def test_world_changing_steps_are_flagged(impl):
    steps = impl.recovery_plan(impl.Failure.EMPTY_GRASP, 3)
    by_name = {s.name: s for s in steps}
    assert by_name["retreat_and_reperceive"].changes_world
    assert not by_name["open_gripper"].changes_world
