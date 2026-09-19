"""Fast tests for the module-17 code (no training runs): py -m pytest 17-reinforcement-learning/code"""

from __future__ import annotations

import math
import pickle
import warnings

import numpy as np
import pytest

import bandits
import corridor_mdp
import rl_tools

gymnasium = pytest.importorskip("gymnasium")
from gymnasium.utils.env_checker import check_env  # noqa: E402

import karmel_env  # noqa: E402
import safety_layer as sl  # noqa: E402
from karmel_env import KarmelGoToGoalEnv, Randomization, RobotVariant  # noqa: E402


# --- 17.01 / 17.02 --------------------------------------------------------------------------------
def test_corridor_transition_probabilities_sum_to_one():
    mdp = corridor_mdp.CorridorMDP()
    for s in range(1, mdp.n_cells - 1):
        for a in (corridor_mdp.LEFT, corridor_mdp.RIGHT):
            assert sum(p for p, *_ in mdp.transitions(s, a)) == pytest.approx(1.0)


def test_discounted_return():
    assert bandits.discounted_return([-1, -1, -1, 10], 0.9) == pytest.approx(-1 - 0.9 - 0.81 + 10 * 0.729)
    assert bandits.discounted_return([-1, -1, -1, 10], 1.0) == pytest.approx(7.0)


def test_exact_policy_value_matches_hand_solution():
    mdp = corridor_mdp.CorridorMDP()
    V = bandits.policy_value_exact(mdp, np.tile([0.0, 1.0], (mdp.n_cells, 1)))
    # V(4) = 0.8 * 10 + 0.2 * (-1 + 0.9 V(4))  ->  V(4) = 7.8 / 0.82
    assert V[4] == pytest.approx(7.8 / 0.82)
    assert V[0] == 0.0 and V[-1] == 0.0


def test_monte_carlo_agrees_with_exact_value():
    mdp = corridor_mdp.CorridorMDP()
    env = corridor_mdp.CorridorEnv(mdp)
    mc = np.mean([corridor_mdp.run_episode(env, lambda s, rng: corridor_mdp.RIGHT, seed)[1] for seed in range(2000)])
    V = bandits.policy_value_exact(mdp, np.tile([0.0, 1.0], (mdp.n_cells, 1)))
    assert mc == pytest.approx(V[mdp.start], abs=0.15)


def test_epsilon_greedy_bandit_beats_greedy():
    probs = bandits.DOCKING_SUCCESS
    greedy = bandits.bandit_experiment(probs, epsilon=0.0, steps=300, runs=300)
    explore = bandits.bandit_experiment(probs, epsilon=0.1, steps=300, runs=300)
    assert explore["optimal_last100"] > greedy["optimal_last100"] + 0.2


# --- 17.04 / 17.05 --------------------------------------------------------------------------------
def test_random_goal_grid_shortest_path_and_dqn_smoke():
    dqn = pytest.importorskip("dqn")  # needs torch
    env = dqn.RandomGoalGrid()
    env.reset(seed=0)
    env.pos, env.goal = (2, 1), (4, 1)  # the wall row 3 blocks cols 1-4: go around via col 0
    assert env.shortest_path() == 4
    dqn.torch.set_num_threads(1)
    q, _ = dqn.train_dqn(env, dqn.DQNConfig(steps=150, warmup=100, batch=16), seed=0, log_every=50)
    assert q(dqn.torch.zeros(env.obs_size)).shape == (4,)


def test_softmax_policy_gradient_matches_finite_differences():
    reinforce = pytest.importorskip("reinforce")
    theta, R = np.array([0.5, 0.0, -0.5]), np.array([1.0, 3.0, 0.0])
    pi = reinforce.softmax(theta)
    exact = pi * (R - pi @ R)
    eps = 1e-6
    fd = [(reinforce.softmax(theta + eps * np.eye(3)[k]) @ R - reinforce.softmax(theta - eps * np.eye(3)[k]) @ R) / (2 * eps)
          for k in range(3)]
    assert exact == pytest.approx(fd, abs=1e-6)
    assert exact == pytest.approx([-0.2168, 0.4829, -0.2661], abs=1e-4)


def test_gaussian_score_function():
    # d/dmu log N(a; mu, sigma^2) = (a - mu) / sigma^2, checked numerically
    def logpdf(a, mu, sigma):
        return -0.5 * ((a - mu) / sigma) ** 2 - math.log(sigma * math.sqrt(2 * math.pi))

    a, mu, sigma, eps = 0.42, 0.30, 0.1, 1e-6
    numeric = (logpdf(a, mu + eps, sigma) - logpdf(a, mu - eps, sigma)) / (2 * eps)
    assert numeric == pytest.approx((a - mu) / sigma**2, rel=1e-5)
    numeric_ls = (logpdf(a, mu, math.exp(math.log(sigma) + eps)) - logpdf(a, mu, math.exp(math.log(sigma) - eps))) / (2 * eps)
    assert numeric_ls == pytest.approx((a - mu) ** 2 / sigma**2 - 1.0, rel=1e-5)


# --- rl_tools -------------------------------------------------------------------------------------
def test_monitor_reader_and_curve_table(tmp_path):
    for i, rows in enumerate([[(1.0, 10, 0.5, True), (3.0, 20, 1.5, False)], [(2.0, 5, 1.0, True)]]):
        lines = ['#{"t_start": 0, "env_id": "x"}', "r,l,t,is_success"] + [f"{r},{l},{t},{s}" for r, l, t, s in rows]
        (tmp_path / f"{i}.monitor.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")
    log = rl_tools.read_monitor_dir(tmp_path)
    assert log["r"].tolist() == [1.0, 2.0, 3.0]  # sorted by finish time
    assert log["steps"].tolist() == [10, 15, 35]
    table = rl_tools.curve_table(log, [15, 35], window=2)
    assert table[0]["return"] == pytest.approx(1.5) and table[0]["success"] == pytest.approx(1.0)
    assert table[1]["return"] == pytest.approx(2.5) and table[1]["success"] == pytest.approx(0.5)
    assert rl_tools.moving_average(np.array([1.0, 3.0, 5.0]), 2).tolist() == [1.0, 2.0, 4.0]


# --- 17.07 – 17.09: the Gymnasium environment -----------------------------------------------------
@pytest.mark.parametrize("kwargs", [{}, {"n_obstacles": 3}, {"randomize": True}, {"reward_mode": "naive_heading"}])
def test_env_passes_check_env(kwargs):
    env = KarmelGoToGoalEnv(**kwargs)
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        check_env(env, skip_render_check=True)
    assert not [str(w.message) for w in caught if "gymnasium" in str(w.filename).lower()]


def test_observation_shape_with_rays():
    env = KarmelGoToGoalEnv(n_obstacles=2)
    obs, _ = env.reset(seed=0)
    assert obs.shape == (5 + 16,) and env.observation_space.contains(obs)
    assert np.all(obs[5:] <= 1.0) and np.all(obs[5:] > 0.0)


def test_env_is_deterministic_per_seed():
    runs = []
    for _ in range(2):
        env = KarmelGoToGoalEnv(randomize=True)
        obs, _ = env.reset(seed=11)
        trace = [obs]
        for _ in range(10):
            trace.append(env.step(np.array([0.7, -0.2], dtype=np.float32))[0])
        runs.append(np.array(trace))
    assert np.array_equal(runs[0], runs[1])


def spin_rewards(mode: str, steps: int) -> list[float]:
    env = KarmelGoToGoalEnv(reward_mode=mode, max_episode_steps=1000)
    env.reset(seed=0, options={"start": (2.0, 2.0, 0.0), "goal": (3.5, 2.0)})
    return [env.step(np.array([0.0, 1.0], dtype=np.float32))[1] for _ in range(steps)]


def test_naive_heading_reward_pays_for_spinning_and_fixed_does_not():
    # 2.5 rad/s for 10 s = 25 rad ~ 4 turns in place, the goal 1.5 m away never gets closer
    naive, fixed = sum(spin_rewards("naive_heading", 100)), sum(spin_rewards("heading_fixed", 100))
    time_cost = -0.05 * 100
    assert naive > time_cost + 25.0, "spinning farms roughly 3 * pi per turn in the naive reward"
    assert fixed == pytest.approx(time_cost, abs=3.5), "a potential-based term sums to ~0 over whole turns"


def test_action_latency_delays_commands():
    env = KarmelGoToGoalEnv(variant=RobotVariant(action_delay_steps=2))
    env.reset(seed=0, options={"start": (2.0, 2.0, 0.0), "goal": (3.5, 2.0)})
    for _ in range(2):
        env.step(np.array([1.0, 0.0], dtype=np.float32))
    assert env.sim.pose.x == pytest.approx(2.0, abs=1e-9), "first 2 steps execute the initial zero commands"
    env.step(np.array([1.0, 0.0], dtype=np.float32))
    assert env.sim.pose.x > 2.0


def test_randomization_samples_within_ranges():
    R = Randomization()
    env = KarmelGoToGoalEnv(randomize=True, randomization=R)
    seen = set()
    for seed in range(30):
        env.reset(seed=seed)
        v = env.variant
        assert R.motor_gain[0] <= v.motor_gain_left <= R.motor_gain[1]
        assert R.wheel_separation_scale[0] <= v.wheel_separation_scale <= R.wheel_separation_scale[1]
        assert R.action_delay_steps[0] <= v.action_delay_steps <= R.action_delay_steps[1]
        assert env.sim.params.wheel_separation_scale == v.wheel_separation_scale
        seen.add(v.action_delay_steps)
    assert len(seen) >= 3


def test_hand_written_controller_reaches_goal():
    env = KarmelGoToGoalEnv()
    obs, _ = env.reset(seed=3)
    for _ in range(200):
        obs, r, terminated, truncated, info = env.step(karmel_env.heading_controller(obs))
        if terminated or truncated:
            break
    assert info["is_success"]


def test_env_factory_is_picklable():
    train = pytest.importorskip("train_go_to_goal")
    factory = train.EnvFactory({"n_obstacles": 1})
    env = pickle.loads(pickle.dumps(factory))()
    assert env.n_obstacles == 1


# --- 17.09: safety layer --------------------------------------------------------------------------
def shield(**overrides) -> sl.SafetyShield:
    cfg = sl.ShieldConfig(**{"max_v_m_s": 0.25, "max_w_rad_s": 1.5, "max_linear_accel_m_s2": 0.5,
                             "max_angular_accel_rad_s2": 3.0, **overrides})
    return sl.SafetyShield(cfg)


def step(s: sl.SafetyShield, now: float, v: float, w: float = 0.0, **kw):
    return s.filter(sl.ShieldInput(now=now, obs_stamp=kw.pop("obs_stamp", now), v_cmd=v, w_cmd=w, **kw))


def test_velocity_and_acceleration_limits():
    s = shield()
    step(s, 0.0, 0.0)
    v, w, fired = step(s, 0.1, 1.0, 5.0)
    assert (v, w) == pytest.approx((0.05, 0.3)), "0.5 m/s^2 and 3 rad/s^2 for 0.1 s"
    assert "velocity_limit" in fired and "accel_limit" in fired
    for k in range(2, 20):
        v, w, _ = step(s, 0.1 * k, 1.0, 5.0)
    assert (v, w) == pytest.approx((0.25, 1.5))


def test_non_finite_and_stale_observation_stop():
    s = shield()
    step(s, 0.0, 0.0)
    for k in range(1, 10):
        step(s, 0.1 * k, 0.2)
    v, _, fired = step(s, 1.0, float("nan"))
    assert "non_finite" in fired and v == pytest.approx(0.0)  # ramped at 4x: 0.2 -> 0 in one 0.1 s cycle
    v, _, fired = step(s, 1.1, 0.2, obs_stamp=0.7)
    assert "stale_observation" in fired and v == 0.0


def test_obstacle_stop_allows_turning():
    s = shield(max_linear_accel_m_s2=100.0, max_angular_accel_rad_s2=100.0)
    step(s, 0.0, 0.0)
    v, w, fired = step(s, 0.1, 0.2, 1.0, front_range_m=0.2)
    assert v == 0.0 and w == pytest.approx(1.0) and "obstacle_stop" in fired
    v, _, fired = step(s, 0.2, -0.1, 0.0, front_range_m=0.2)
    assert v == pytest.approx(-0.1) and "obstacle_stop" not in fired, "backing away is allowed"


def test_geofence_blocks_leaving_but_allows_returning():
    s = shield(geofence=(0.0, 0.0, 2.0, 2.0), max_linear_accel_m_s2=100.0)
    step(s, 0.0, 0.0)
    v, _, fired = step(s, 0.1, 0.2, x=2.1, y=1.0, theta=0.0)
    assert v == 0.0 and "geofence" in fired
    v, _, fired = step(s, 0.2, 0.2, x=2.1, y=1.0, theta=math.pi)
    assert v == pytest.approx(0.2) and "geofence" not in fired


def test_estop_latches_until_reset_and_violation_budget_trips():
    s = shield(max_consecutive_violations=3)
    step(s, 0.0, 0.0)
    s.request_stop()
    assert step(s, 0.1, 0.2)[:2] == (0.0, 0.0)
    assert step(s, 5.0, 0.2)[2] == ("estop_latched",)
    s.reset()
    step(s, 5.1, 0.0)
    for k in range(3):
        _, _, fired = step(s, 5.2 + 0.1 * k, float("inf"))
    assert "violation_budget" in fired and s.estop_latched


def test_shield_in_simulator_stops_before_wall():
    log = sl.run_with_shield(lambda k: (1.0, 0.0), steps=110)
    assert any("obstacle_stop" in fired for *_, fired in log)
    assert max(v for _, v, _, _ in log) <= 0.25 + 1e-9
    assert log[-1][1] == 0.0
