"""Checker for 17.07 — the course simulator as a Gymnasium environment.

Run: ``python course.py check 17.07`` (or ``--solution`` to see the reference pass).
No training here: the tests check the API contract, spaces, seeding, termination and reward with
short scripted episodes (a few seconds in total).
"""

from __future__ import annotations

import math
import warnings

import numpy as np
import pytest

gymnasium = pytest.importorskip("gymnasium", reason="py -m pip install gymnasium")
from gymnasium import spaces  # noqa: E402
from gymnasium.utils.env_checker import check_env  # noqa: E402

from robotlab.config import load_config  # noqa: E402

approx = pytest.approx


# --- pure functions -----------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("v", "w", "expected"),
    [
        (0.5, 0.0, (11.1111, 11.1111)),  # 0.5 / 0.045
        (0.0, 2.5, (-5.5556, 5.5556)),  # (0 -+ 2.5 * 0.1) / 0.045: spin in place, CCW
        (0.2, 1.0, (2.2222, 6.6667)),  # (0.2 - 0.1) / 0.045, (0.2 + 0.1) / 0.045
        (-0.3, -1.0, (-4.4444, -8.8889)),
    ],
)
def test_twist_to_wheels(impl, v, w, expected):
    assert impl.twist_to_wheels(v, w, 0.045, 0.2) == approx(expected, abs=1e-3)


def test_observation_goal_straight_ahead(impl):
    obs = impl.make_observation((1.0, 1.0, 0.0), (3.0, 1.0), 0.25, -1.25, d_max=4.0, v_max=0.5, w_max=2.5)
    assert isinstance(obs, np.ndarray) and obs.dtype == np.float32 and obs.shape == (5,)
    assert obs == approx([0.5, 1.0, 0.0, 0.5, -0.5], abs=1e-6)


def test_observation_goal_behind_left_uses_wrapped_error(impl):
    # Robot at 170 deg; goal bearing is -135 deg -> error = wrap(-135 - 170) = +55 deg (turn left).
    theta = math.radians(170)
    obs = impl.make_observation((0.0, 0.0, theta), (-1.0, -1.0), 0.0, 0.0, 5.0, 0.5, 2.5)
    e = math.radians(55)
    assert obs[:3] == approx([math.sqrt(2) / 5, math.cos(e), math.sin(e)], abs=1e-5)


def test_observation_is_clipped(impl):
    obs = impl.make_observation((0.0, 0.0, 0.0), (100.0, 0.0), 0.8, -4.0, 5.0, 0.5, 2.5)
    assert obs[0] == approx(1.0) and obs[3] == approx(1.0) and obs[4] == approx(-1.0)


@pytest.mark.parametrize(
    ("d_prev", "d", "success", "collided", "expected"),
    [
        (2.0, 1.95, False, False, 0.45),  # 10 * 0.05 - 0.05
        (2.0, 2.02, False, False, -0.25),  # moving away costs
        (0.17, 0.14, True, False, 10.25),  # 0.3 - 0.05 + 10
        (1.0, 1.0, False, True, -10.05),
    ],
)
def test_step_reward(impl, d_prev, d, success, collided, expected):
    assert impl.step_reward(d_prev, d, success, collided) == approx(expected)


# --- the environment ----------------------------------------------------------------------------
@pytest.fixture
def env(impl):
    e = impl.GoToGoalEnv()
    yield e
    e.close()


def test_spaces(env):
    cfg = load_config()
    assert isinstance(env.action_space, spaces.Box) and isinstance(env.observation_space, spaces.Box)
    assert env.action_space.shape == (2,) and env.action_space.dtype == np.float32
    assert env.action_space.low == approx([-1, -1]) and env.action_space.high == approx([1, 1])
    assert env.observation_space.shape == (5,) and env.observation_space.dtype == np.float32
    assert env.observation_space.low == approx([0, -1, -1, -1, -1])
    assert env.observation_space.high == approx([1, 1, 1, 1, 1])
    assert env.v_max == approx(cfg.drive.max_linear_speed_m_s)


def test_passes_gymnasium_check_env(impl):
    env = impl.GoToGoalEnv()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        check_env(env, skip_render_check=True)
    messages = [str(w.message) for w in caught if "gymnasium" in str(w.filename).lower() or "env_checker" in str(w.message)]
    assert not messages, f"check_env warned: {messages}"


def test_reset_returns_obs_and_info(env):
    obs, info = env.reset(seed=3)
    assert env.observation_space.contains(obs), f"{obs} is not inside the observation space"
    assert isinstance(info, dict) and {"is_success", "collided", "distance"} <= set(info)


def test_same_seed_same_episode_different_seed_different(env, impl):
    a, _ = env.reset(seed=42)
    b, _ = env.reset(seed=42)
    c, _ = env.reset(seed=43)
    other = impl.GoToGoalEnv()
    d, _ = other.reset(seed=42)
    assert np.array_equal(a, b) and np.array_equal(a, d), "reset(seed=s) must fully determine the episode"
    assert not np.array_equal(a, c)


def test_sampled_start_and_goal_are_valid(env):
    for seed in range(25):
        env.reset(seed=seed)
        x, y, theta = env.sim.pose
        gx, gy = env.goal
        for coord in (x, y, gx, gy):
            assert 0.4 - 1e-9 <= coord <= 3.6 + 1e-9, "start and goal must stay 0.4 m away from the walls"
        assert math.hypot(gx - x, gy - y) > 1.0
        assert -math.pi <= theta <= math.pi


def test_driving_straight_to_a_goal_terminates_with_success(env):
    env.reset(seed=0, options={"start": (1.0, 1.0, 0.0), "goal": (1.6, 1.0)})
    total = 0.0
    for step in range(40):
        obs, reward, terminated, truncated, info = env.step(np.array([1.0, 0.0], dtype=np.float32))
        total += reward
        assert type(terminated) is bool and type(truncated) is bool and isinstance(reward, float)
        if terminated or truncated:
            break
    assert terminated and not truncated and info["is_success"] and not info["collided"]
    assert step < 20, "0.45 m at up to 0.5 m/s should take well under 2 s (20 steps)"
    assert reward > 10.0, "the final step includes the +10 success bonus"
    assert total == approx(10.0 * (0.6 - info["distance"]) - 0.05 * (step + 1) + 10.0, abs=1e-6)


def test_hitting_a_wall_terminates_with_collision(env):
    # 0.3 m from the x = 0 wall, facing it; the robot's radius is ~0.16 m.
    env.reset(seed=0, options={"start": (0.3, 2.0, math.pi), "goal": (3.0, 2.0)})
    for step in range(30):
        obs, reward, terminated, truncated, info = env.step(np.array([1.0, 0.0], dtype=np.float32))
        if terminated or truncated:
            break
    assert terminated and info["collided"] and not info["is_success"]
    assert reward < -9.0


def test_time_limit_truncates(impl):
    env = impl.GoToGoalEnv(max_episode_steps=5)
    env.reset(seed=1, options={"start": (1.0, 1.0, 0.0), "goal": (3.0, 3.0)})
    for i in range(5):
        _, _, terminated, truncated, _ = env.step(np.zeros(2, dtype=np.float32))
        assert not terminated, "standing still is neither success nor collision"
        assert truncated == (i == 4), "truncated exactly when the 5th step is done"


def test_actions_are_clipped(impl):
    runs = []
    for action in ([1.0, 0.3], [7.0, 0.3]):
        env = impl.GoToGoalEnv()
        env.reset(seed=5, options={"start": (2.0, 2.0, 0.0), "goal": (3.5, 3.5)})
        for _ in range(3):
            obs, *_ = env.step(np.array(action, dtype=np.float32))
        runs.append(obs)
    assert np.allclose(runs[0], runs[1]), "an action of 7.0 must behave exactly like 1.0"


def test_turning_left_changes_heading_error_as_expected(env):
    # Goal straight to the left (+90 deg). Turning CCW must shrink sin(error) toward 0.
    obs0, _ = env.reset(seed=0, options={"start": (2.0, 1.0, 0.0), "goal": (2.0, 3.0)})
    assert obs0[1:3] == approx([0.0, 1.0], abs=1e-5)
    for _ in range(4):
        obs, *_ = env.step(np.array([0.0, 1.0], dtype=np.float32))
    assert obs[2] < 0.95 and obs[1] > 0.3, "after 0.4 s at +2.5 rad/s the robot faces the goal much better"
    assert obs[4] > 0.5, "the measured yaw rate is positive (CCW) and large"
