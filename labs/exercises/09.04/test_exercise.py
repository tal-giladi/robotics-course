"""Checker for 09.04 — Build an odometry system from scratch.

Run: ``python course.py check 09.04`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import dataclasses
import math

import pytest

from robotlab.config import load_config
from robotlab.geometry import angle_diff
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

B = 0.2  # wheel separation used by the hand-computed cases
approx = pytest.approx


# --- tick_delta ---------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("previous", "current", "bits", "expected"),
    [
        (100, 150, None, 50),
        (150, 100, None, -50),
        (-7, -7, None, 0),
        (127, -128, 8, 1),  # 8-bit counter wraps forward
        (-128, 127, 8, -1),  # ... and backward
        (100, -100, 8, 56),
        (32_000, -32_000, 16, 1536),
        (5, 9, 16, 4),  # no wrap: same as plain subtraction
    ],
)
def test_tick_delta(impl, previous, current, bits, expected):
    assert impl.tick_delta(previous, current, bits) == expected


# --- ticks_to_distance --------------------------------------------------------------------------
def test_one_revolution_is_one_circumference(impl):
    assert impl.ticks_to_distance(2464, 0.045, 2464) == approx(2 * math.pi * 0.045)


def test_negative_ticks_move_backwards(impl):
    assert impl.ticks_to_distance(-1320, 0.05, 2640) == approx(-math.pi * 0.05)


# --- integrate_pose -----------------------------------------------------------------------------
def test_straight_line_along_x(impl):
    assert impl.integrate_pose((0.0, 0.0, 0.0), 1.0, 1.0, B) == approx((1.0, 0.0, 0.0))


def test_straight_line_follows_heading(impl):
    assert impl.integrate_pose((1.0, 2.0, math.pi / 2), 0.5, 0.5, B) == approx((1.0, 2.5, math.pi / 2))


def test_quarter_circle_to_the_left_is_an_exact_arc(impl):
    # Center of rotation at (0, 1): wheels on circles of radius 0.9 (left) and 1.1 (right).
    quarter = math.pi / 2
    pose = impl.integrate_pose((0.0, 0.0, 0.0), 0.9 * quarter, 1.1 * quarter, B)
    assert pose == approx((1.0, 1.0, quarter), abs=1e-9)


def test_reversing_arc(impl):
    quarter = math.pi / 2
    pose = impl.integrate_pose((0.0, 0.0, 0.0), -0.9 * quarter, -1.1 * quarter, B)
    # Backing up with the right wheel faster: clockwise around (0, 1), ending at (-1, 1).
    assert pose == approx((-1.0, 1.0, -quarter), abs=1e-9)


def test_rotation_in_place(impl):
    d = B / 2 * (math.pi / 2)
    assert impl.integrate_pose((0.3, -0.4, 0.0), -d, d, B) == approx((0.3, -0.4, math.pi / 2), abs=1e-12)


def test_nearly_straight_motion_does_not_blow_up(impl):
    x, y, theta = impl.integrate_pose((0.0, 0.0, 0.0), 1.0, 1.0 + 1e-13, B)
    assert (x, y, theta) == approx((1.0, 0.0, 0.0), abs=1e-9)
    assert all(math.isfinite(v) for v in (x, y, theta))


def test_heading_is_wrapped(impl):
    d = B / 2 * 0.5
    _, _, theta = impl.integrate_pose((0.0, 0.0, 3.0), -d, d, B)
    assert theta == approx(3.5 - 2 * math.pi)
    assert -math.pi < theta <= math.pi


# --- Odometry -----------------------------------------------------------------------------------
def make_odometry(impl, **kwargs):
    return impl.Odometry(wheel_radius_m=0.05, wheel_separation_m=B, ticks_per_rev=1000, **kwargs)


def test_first_update_only_remembers_the_counts(impl):
    odom = make_odometry(impl)
    assert odom.update(5000, -3000) == approx((0.0, 0.0, 0.0))
    assert odom.update(5000, -3000) == approx((0.0, 0.0, 0.0))


def test_first_update_returns_the_initial_pose(impl):
    odom = make_odometry(impl, initial_pose=(1.0, 2.0, 0.5))
    assert odom.update(123, 456) == approx((1.0, 2.0, 0.5))


def test_one_wheel_revolution_forward(impl):
    odom = make_odometry(impl)
    odom.update(250, 250)
    assert odom.update(1250, 1250) == approx((2 * math.pi * 0.05, 0.0, 0.0))
    assert odom.pose == approx((2 * math.pi * 0.05, 0.0, 0.0))


def test_updates_use_deltas_not_totals(impl):
    odom = make_odometry(impl)
    odom.update(0, 0)
    for ticks in range(100, 1001, 100):
        pose = odom.update(ticks, ticks)
    assert pose == approx((2 * math.pi * 0.05, 0.0, 0.0))


def test_spin_in_place_then_drive(impl):
    odom = make_odometry(impl)
    odom.update(0, 0)
    # A quarter turn: each wheel travels (B/2) * pi/2 = 0.05*pi m = 500 ticks (r=0.05, 1000 tpr).
    odom.update(-500, 500)
    x, y, theta = odom.update(-500 + 500, 500 + 500)  # then 0.05*pi m straight along +y
    assert (x, y, theta) == approx((0.0, math.pi * 0.05, math.pi / 2), abs=1e-9)


def test_counter_rollover(impl):
    odom = make_odometry(impl, encoder_bits=16)
    odom.update(32_700, 32_700)
    pose = odom.update(-32_736, -32_736)  # +100 ticks through the wrap
    assert pose == approx((2 * math.pi * 0.05 * 0.1, 0.0, 0.0))


# --- against the simulator ----------------------------------------------------------------------
PLAN = [  # (seconds, (left_rad_s, right_rad_s)): straight, arcs, spins, reversing
    (3.0, (8.0, 8.0)),
    (3.0, (4.0, 9.0)),
    (2.0, (-5.0, 5.0)),
    (3.0, (9.0, 3.0)),
    (2.0, (-6.0, -6.0)),
    (2.0, (6.0, 6.0)),
]
START = (0.5, -0.2, 0.3)


def drive_and_compare(impl, params: DiffDriveParams, sensors: SensorParams, seed: int) -> tuple[float, float]:
    """Drive PLAN with SimBase, run the odometry on the telemetry; return (position, heading) error."""
    cfg = load_config()
    base = SimBase(DiffDriveSim(World(), params, sensors, pose=START, seed=seed))
    odom = impl.Odometry(
        cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m, cfg.drive.ticks_per_wheel_rev,
        encoder_bits=params.encoder_bits, initial_pose=START,
    )
    state = base.read()
    odom.update(state.left_ticks, state.right_ticks)
    for seconds, (left, right) in PLAN:
        for _ in range(round(seconds / base.dt)):
            base.set_wheel_velocity(left, right)
            state = base.read()
            odom.update(state.left_ticks, state.right_ticks)
    x, y, theta = odom.pose
    truth = base.sim.pose
    return math.hypot(x - truth.x, y - truth.y), abs(angle_diff(theta, truth.theta))


@pytest.mark.parametrize("encoder_bits", [None, 12], ids=["unbounded", "12-bit-rollover"])
def test_matches_noise_free_simulator(impl, encoder_bits):
    cfg = load_config()
    params = dataclasses.replace(DiffDriveParams.ideal(cfg), encoder_bits=encoder_bits)
    position_error, heading_error = drive_and_compare(impl, params, SensorParams.ideal(cfg), seed=0)
    assert position_error < 0.005, "about 4 m of driving on a perfect robot should match truth within 5 mm"
    assert heading_error < math.radians(0.5)


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_stays_close_on_realistic_robot(impl, seed):
    cfg = load_config()
    position_error, heading_error = drive_and_compare(
        impl, DiffDriveParams.realistic(cfg), SensorParams.realistic(cfg), seed=seed
    )
    # Mismatched wheels and slip make odometry drift; it must still be roughly right.
    assert position_error < 0.25
    assert heading_error < math.radians(15)
