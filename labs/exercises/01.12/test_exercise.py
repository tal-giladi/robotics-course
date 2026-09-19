"""Checker for 01.12 — ticks to distances and angles, and back.

Run: ``python course.py check 01.12`` (or ``--solution`` to see the reference pass).

The hand-computed cases use a round robot (r = 0.05 m, 1000 ticks/rev, track 0.2 m) so you can
redo the arithmetic on paper. The last tests use YOUR robot's numbers from
``labs/config/karmel.yaml`` and drive the course simulator around the square your plan describes.
"""

from __future__ import annotations

import math

import pytest

from robotlab.config import load_config
from robotlab.geometry import SE2, angle_diff
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World
from robotlab.sim.robot import arc_update

approx = pytest.approx

# A deliberately round robot: circumference 0.1*pi m, so one tick is 0.1*pi/1000 m.
R, TPR, B = 0.05, 1000, 0.2
CIRCUMFERENCE = 2 * math.pi * R


# --- meters_per_tick / ticks_to_distance --------------------------------------------------------
def test_meters_per_tick_is_circumference_over_ticks(impl):
    assert impl.meters_per_tick(R, TPR) == approx(CIRCUMFERENCE / 1000)


def test_karmel_tick_is_a_tenth_of_a_millimetre(impl):
    cfg = load_config()
    # 2*pi*0.045 / 2464 = 0.2827 m / 2464 = 0.11475 mm
    value = impl.meters_per_tick(cfg.drive.wheel_radius_m, cfg.drive.ticks_per_wheel_rev)
    assert value == approx(cfg.drive.meters_per_tick), "must match robotlab's own conversion"


def test_one_revolution_is_one_circumference(impl):
    assert impl.ticks_to_distance(TPR, R, TPR) == approx(CIRCUMFERENCE)


def test_negative_ticks_are_backwards(impl):
    assert impl.ticks_to_distance(-500, R, TPR) == approx(-CIRCUMFERENCE / 2)


# --- distance_to_ticks --------------------------------------------------------------------------
def test_distance_to_ticks_round_trips(impl):
    assert impl.distance_to_ticks(CIRCUMFERENCE, R, TPR) == 1000
    assert impl.distance_to_ticks(-CIRCUMFERENCE / 4, R, TPR) == -250


def test_distance_to_ticks_returns_a_whole_number_of_ticks(impl):
    ticks = impl.distance_to_ticks(1.0, R, TPR)
    assert isinstance(ticks, int), "encoders count whole ticks — round, don't return a float"
    # 1.0 m / (0.1*pi/1000 m per tick) = 3183.098... -> 3183
    assert ticks == 3183


def test_one_metre_on_karmel(impl):
    cfg = load_config()
    # 1 m / 0.00011475 m per tick = 8714.6 -> 8715 ticks per wheel
    ticks = impl.distance_to_ticks(1.0, cfg.drive.wheel_radius_m, cfg.drive.ticks_per_wheel_rev)
    assert ticks == 8715


def test_drive_straight_moves_both_wheels_equally(impl):
    assert impl.drive_straight_ticks(1.0, R, TPR) == (3183, 3183)
    assert impl.drive_straight_ticks(-0.5, R, TPR) == (-1592, -1592)


# --- turn_in_place_ticks ------------------------------------------------------------------------
def test_quarter_turn_left(impl):
    # Each wheel follows a circle of radius 0.1 m: arc = (pi/2)*0.1 = 0.15708 m = 500 ticks.
    assert impl.turn_in_place_ticks(math.pi / 2, R, B, TPR) == (-500, 500)


def test_quarter_turn_right_is_the_mirror_image(impl):
    assert impl.turn_in_place_ticks(-math.pi / 2, R, B, TPR) == (500, -500)


def test_full_turn(impl):
    assert impl.turn_in_place_ticks(2 * math.pi, R, B, TPR) == (-2000, 2000)


def test_ninety_degrees_on_karmel(impl):
    cfg = load_config()
    # (pi/2)*0.1 m = 0.15708 m / 0.00011475 = 1368.9 -> 1369 ticks per wheel
    left, right = impl.turn_in_place_ticks(
        math.pi / 2, cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m,
        cfg.drive.ticks_per_wheel_rev,
    )
    assert (left, right) == (-1369, 1369)


# --- arc_ticks ----------------------------------------------------------------------------------
def test_arc_left_inner_wheel_is_the_left_one(impl):
    # R = 0.5 m, quarter turn left: left wheel 0.4*pi/2 = 0.6283 m, right 0.6*pi/2 = 0.9425 m.
    assert impl.arc_ticks(0.5, math.pi / 2, R, B, TPR) == (2000, 3000)


def test_arc_right_swaps_the_wheels(impl):
    assert impl.arc_ticks(0.5, -math.pi / 2, R, B, TPR) == (-2000, -3000)


def test_arc_with_zero_radius_is_a_turn_in_place(impl):
    assert impl.arc_ticks(0.0, math.pi / 2, R, B, TPR) == impl.turn_in_place_ticks(math.pi / 2, R, B, TPR)


def test_tight_arc_runs_the_inner_wheel_backwards(impl):
    # Radius 0.05 m is inside the 0.1 m half-track: the left wheel must reverse.
    left, right = impl.arc_ticks(0.05, math.pi / 2, R, B, TPR)
    assert left < 0 < right
    assert (left, right) == (-250, 750)


# --- reading motion back out of the ticks -------------------------------------------------------
def test_distance_from_ticks_is_the_mean(impl):
    assert impl.distance_from_ticks(1000, 1000, R, TPR) == approx(CIRCUMFERENCE)
    assert impl.distance_from_ticks(-500, 500, R, TPR) == approx(0.0)  # turning in place goes nowhere
    assert impl.distance_from_ticks(2000, 3000, R, TPR) == approx(2500 * CIRCUMFERENCE / 1000)


def test_heading_change_of_a_turn_in_place(impl):
    assert impl.heading_change_from_ticks(-500, 500, R, B, TPR) == approx(math.pi / 2)
    assert impl.heading_change_from_ticks(500, -500, R, B, TPR) == approx(-math.pi / 2)


def test_heading_change_is_not_wrapped(impl):
    # Three full left turns really is +6*pi, not 0: a turn controller needs the total.
    assert impl.heading_change_from_ticks(-6000, 6000, R, B, TPR) == approx(6 * math.pi)


def test_straight_driving_changes_no_heading(impl):
    assert impl.heading_change_from_ticks(3183, 3183, R, B, TPR) == approx(0.0)


def test_conversions_are_inverses(impl):
    """turn_in_place_ticks -> heading_change_from_ticks must come back to the same angle."""
    cfg = load_config()
    r, b, tpr = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m, cfg.drive.ticks_per_wheel_rev
    for angle in (0.1, 0.5, math.pi / 2, math.pi, -2.0):
        left, right = impl.turn_in_place_ticks(angle, r, b, tpr)
        assert impl.heading_change_from_ticks(left, right, r, b, tpr) == approx(angle, abs=2e-3)


# --- the square plan ----------------------------------------------------------------------------
def test_square_plan_has_eight_segments_alternating_straight_and_turn(impl):
    plan = impl.square_plan(1.0, R, B, TPR)
    assert len(plan) == 8
    for straight, turn in zip(plan[0::2], plan[1::2]):
        assert straight[0] == straight[1] > 0, "a side drives both wheels forward equally"
        assert turn[0] == -turn[1] != 0, "a corner turns in place"


def test_square_plan_turns_left_by_default_and_right_when_asked(impl):
    assert impl.square_plan(1.0, R, B, TPR)[1] == (-500, 500)
    assert impl.square_plan(1.0, R, B, TPR, clockwise=True)[1] == (500, -500)


def test_square_plan_closes_on_paper(impl):
    """Integrating the plan's tick deltas along exact arcs must return to the start pose."""
    cfg = load_config()
    r, b, tpr = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m, cfg.drive.ticks_per_wheel_rev
    pose = SE2(0.0, 0.0, 0.0)
    for left, right in impl.square_plan(1.0, r, b, tpr):
        pose = arc_update(pose, impl.ticks_to_distance(left, r, tpr),
                          impl.ticks_to_distance(right, r, tpr), b)
    # Not exactly zero: each target is rounded to a whole tick (0.11 mm), so a millimetre is plenty.
    assert math.hypot(pose.x, pose.y) < 0.002, f"the square should close, ended at {pose}"
    assert abs(angle_diff(pose.theta, 0.0)) < math.radians(0.5)


# --- against the simulator ----------------------------------------------------------------------
def run_segment(base: SimBase, left_target: int, right_target: int, meters_per_tick: float) -> None:
    """Drive until both wheels have turned their target number of ticks (closed loop on ticks).

    The same controller as ``labs/robot/drive_square.py``: full speed far from the target, then
    slow down over the last 10 cm of wheel travel so the motors are almost stopped when we
    arrive. Without that ramp the wheels coast past the target by several percent.
    """
    target_m = 0.5 * (abs(left_target) + abs(right_target)) * meters_per_tick
    start = base.read()
    for _ in range(3000):  # 3000 * 0.02 s = 60 s of simulated time, far more than any segment
        state = base.read()
        travelled = 0.5 * meters_per_tick * (
            abs(state.left_ticks - start.left_ticks) + abs(state.right_ticks - start.right_ticks)
        )
        remaining = target_m - travelled
        if remaining <= 0.0005:
            break
        speed = max(0.02, min(0.2, 0.2 * remaining / 0.10)) / 0.045  # m/s -> rad/s at the wheel
        base.set_wheel_velocity(
            math.copysign(speed, left_target) if left_target else 0.0,
            math.copysign(speed, right_target) if right_target else 0.0,
        )
    base.stop()
    for _ in range(25):  # let the wheels come to rest (0.5 s)
        base.set_wheel_velocity(0.0, 0.0)
        base.read()


@pytest.mark.parametrize("clockwise", [False, True], ids=["counter-clockwise", "clockwise"])
def test_plan_drives_a_square_in_the_simulator(impl, clockwise):
    cfg = load_config()
    start = (0.0, 0.0, 0.0)
    base = SimBase(DiffDriveSim(World(), DiffDriveParams.ideal(cfg), SensorParams.ideal(cfg),
                                pose=start, seed=0))
    plan = impl.square_plan(1.0, cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m,
                            cfg.drive.ticks_per_wheel_rev, clockwise=clockwise)
    corners = []
    for index, (left, right) in enumerate(plan):
        run_segment(base, left, right, cfg.drive.meters_per_tick)
        if index % 2 == 1:  # after each corner
            corners.append((base.sim.pose.x, base.sim.pose.y))
    truth = base.sim.pose
    assert math.hypot(truth.x - start[0], truth.y - start[1]) < 0.05, (
        f"a perfect robot following your plan should come back within 5 cm, ended at {truth}"
    )
    assert abs(angle_diff(truth.theta, start[2])) < math.radians(5)
    # The corners really are 1 m apart — it drove a square, not a small circle.
    side = math.hypot(corners[0][0] - start[0], corners[0][1] - start[1])
    assert side == approx(1.0, abs=0.05), f"first side was {side:.3f} m, expected 1 m"
