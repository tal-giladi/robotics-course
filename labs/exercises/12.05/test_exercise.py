"""Checker for 12.05 — A pure pursuit path follower.

Run: ``python course.py check 12.05`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import math

import pytest

from robotlab.config import load_config
from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

approx = pytest.approx


def densify(waypoints, spacing=0.05):
    out = [tuple(waypoints[0])]
    for (ax, ay), (bx, by) in zip(waypoints, waypoints[1:]):
        n = max(1, math.ceil(math.hypot(bx - ax, by - ay) / spacing))
        out += [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(1, n + 1)]
    return out


def cross_track_error(path, x, y) -> float:
    best = math.inf
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        dx, dy = bx - ax, by - ay
        length2 = dx * dx + dy * dy
        t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length2))
        best = min(best, math.hypot(ax + t * dx - x, ay + t * dy - y))
    return best


# --- geometry ------------------------------------------------------------------------------------
def test_to_robot_frame(impl):
    assert impl.to_robot_frame((1.0, 2.0, math.pi / 2), (1.0, 3.0)) == approx((1.0, 0.0), abs=1e-12)
    assert impl.to_robot_frame((1.0, 2.0, math.pi / 2), (0.0, 2.0)) == approx((0.0, 1.0), abs=1e-12)  # to the left
    assert impl.to_robot_frame((0.0, 0.0, math.pi), (-2.0, -0.5)) == approx((2.0, 0.5), abs=1e-12)
    assert impl.to_robot_frame((0.5, -1.0, 0.3), (0.5, -1.0)) == approx((0.0, 0.0), abs=1e-12)


@pytest.mark.parametrize(
    ("x_r", "y_r", "expected"),
    [
        (1.0, 0.0, 0.0),  # straight ahead: no turn
        (0.3, 0.1, 2.0),  # 2 * 0.1 / 0.1 -> radius 0.5 m, to the left
        (0.3, -0.1, -2.0),
        (0.0, 0.5, 4.0),  # a point beside the robot: a half circle of radius 0.25 m
        (0.0, 0.0, 0.0),
    ],
)
def test_curvature(impl, x_r, y_r, expected):
    assert impl.curvature(x_r, y_r) == approx(expected)


def test_twist_to_wheel_speeds(impl):
    assert impl.twist_to_wheel_speeds(0.2, 0.0, 0.045, 0.2) == approx((0.2 / 0.045, 0.2 / 0.045))
    assert impl.twist_to_wheel_speeds(0.0, 1.0, 0.045, 0.2) == approx((-0.1 / 0.045, 0.1 / 0.045))
    assert impl.twist_to_wheel_speeds(0.2, 1.0, 0.05, 0.2) == approx((2.0, 6.0))  # (0.2 -+ 0.1) / 0.05


# --- lookahead_point -------------------------------------------------------------------------------
def test_lookahead_on_a_straight_path(impl):
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    point, index = impl.lookahead_point(path, (0.5, 0.1), 0.5)
    assert point == approx((0.5 + math.sqrt(0.24), 0.0))  # 0.5^2 = 0.1^2 + dx^2
    assert index == 0


def test_lookahead_around_a_corner(impl):
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    point, index = impl.lookahead_point(path, (0.9, 0.0), 0.5)
    assert point == approx((1.0, math.sqrt(0.24)))  # (1, 0) is only 0.1 m away: the carrot is on the next segment
    assert index == 1


def test_lookahead_respects_start_index(impl):
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    point, index = impl.lookahead_point(path, (1.5, 0.0), 0.2, start_index=1)
    assert point == approx((1.7, 0.0)) and index == 1


def test_lookahead_past_the_end_returns_the_goal(impl):
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    point, index = impl.lookahead_point(path, (1.9, 0.05), 0.5, start_index=1)
    assert point == approx((2.0, 0.0)) and index == 1


# --- controller rules ------------------------------------------------------------------------------
STRAIGHT = densify([(0.0, 0.0), (2.0, 0.0)])


def test_steers_back_onto_the_path(impl):
    v, w = impl.PurePursuitController(STRAIGHT).compute((0.5, 0.1, 0.0))
    assert v == approx(0.2)
    assert w < 0, "the path is to the robot's right: turn clockwise (negative w)"
    # carrot at (0.5 + sqrt(0.08), 0) -> base_link (0.2828, -0.1) -> kappa = -0.2 / 0.09 -> w = 0.2 * kappa
    assert w == approx(0.2 * -0.2 / 0.09, rel=1e-3)


def test_rotates_in_place_when_facing_away(impl):
    v, w = impl.PurePursuitController(STRAIGHT).compute((0.5, 0.0, math.pi - 0.2))
    assert v == 0.0 and abs(w) == approx(1.0)


def test_slows_down_near_the_goal_and_stops(impl):
    ctrl = impl.PurePursuitController(STRAIGHT)
    assert ctrl.compute((1.8, 0.0, 0.0))[0] == approx(0.1)  # 0.2 * 0.2 / 0.4
    assert ctrl.compute((1.94, 0.0, 0.0))[0] == approx(0.05)  # 0.2 * 0.06 / 0.4 = 0.03, but never below min_speed_m_s
    assert ctrl.done is False
    assert ctrl.compute((1.98, 0.01, 0.0)) == (0.0, 0.0)
    assert ctrl.done is True
    assert ctrl.compute((1.9, 0.0, 0.0)) == (0.0, 0.0), "once done, stay stopped"


def test_angular_limit_keeps_the_arc(impl):
    ctrl = impl.PurePursuitController(STRAIGHT, lookahead_m=0.1, speed_m_s=0.3, max_angular_rad_s=0.5)
    v, w = ctrl.compute((0.5, 0.05, 0.0))
    assert abs(w) == approx(0.5)
    # carrot (0.5 + sqrt(0.0075), 0) -> base_link (0.0866, -0.05): kappa = 2 * -0.05 / 0.01 = -10 1/m
    assert w / v == approx(-10.0), "scale v down with w so the robot still drives the same arc"
    assert v == approx(0.05)


# --- against the simulator -----------------------------------------------------------------------
PATHS = {
    "kitchen": [(1.0, 1.3), (3.0, 1.65), (4.0, 1.65), (5.0, 2.3)],  # through the doorway, gentle bends
    "study": [(1.0, 1.3), (2.05, 1.3), (2.05, 3.8), (0.8, 3.8)],  # two 90-degree corners
}
MAX_CTE = {"kitchen": 0.08, "study": 0.15}


def follow(impl, waypoints, params, sensors, seed):
    cfg = load_config()
    path = densify(waypoints)
    heading = math.atan2(waypoints[1][1] - waypoints[0][1], waypoints[1][0] - waypoints[0][0])
    base = SimBase(DiffDriveSim(World.apartment(), params, sensors, pose=(*waypoints[0], heading), seed=seed))
    ctrl = impl.PurePursuitController(path)
    worst, collided, t_done, pose_done = 0.0, False, None, None
    while base.sim.t < 60.0:
        pose = tuple(base.true_pose)  # localization = ground truth here (AMCL on the robot)
        v, w = ctrl.compute(pose)
        base.set_wheel_velocity(*impl.twist_to_wheel_speeds(v, w, cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m))
        base.read()
        x, y, _ = base.true_pose
        worst = max(worst, cross_track_error(path, x, y))
        collided |= base.sim.collided
        if ctrl.done and t_done is None:
            t_done, pose_done = base.sim.t, (x, y)
        if t_done is not None and base.sim.t >= t_done + 1.0:
            break
    return ctrl, base, worst, collided, t_done, pose_done


@pytest.mark.parametrize(
    ("room", "realistic", "seed"),
    [("kitchen", False, 0), ("kitchen", True, 1), ("kitchen", True, 2), ("study", False, 0), ("study", True, 3)],
)
def test_follows_a_path_in_the_apartment(impl, room, realistic, seed):
    cfg = load_config()
    params = DiffDriveParams.realistic(cfg) if realistic else DiffDriveParams.ideal(cfg)
    sensors = SensorParams.realistic(cfg) if realistic else SensorParams.ideal(cfg)
    ctrl, base, worst, collided, t_done, pose_done = follow(impl, PATHS[room], params, sensors, seed)
    goal = PATHS[room][-1]
    assert t_done is not None and t_done < 45.0, "the controller must reach the goal and set done (within 45 s)"
    assert not collided, "the robot touched an obstacle"
    assert worst < MAX_CTE[room], f"max cross-track error {worst * 100:.1f} cm (limit {MAX_CTE[room] * 100:.0f} cm)"
    x, y, _ = base.true_pose
    assert math.hypot(x - goal[0], y - goal[1]) < 0.08, "the robot must stop within 8 cm of the goal"
    assert math.hypot(x - pose_done[0], y - pose_done[1]) < 0.02, "after done the robot must stay put"
    assert max(abs(s) for s in base.sim.wheel_rad_s) < 0.2, "the wheels must be stopped at the end"
