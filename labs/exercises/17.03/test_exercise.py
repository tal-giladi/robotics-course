"""Checker for 17.03 — Q-learning on a gridworld robot.

Run: ``python course.py check 17.03`` (or ``--solution`` to see the reference pass).
No training loops here: every test is a hand-computed update or a deterministic sweep (< 1 s).
"""

from __future__ import annotations

import numpy as np
import pytest

approx = pytest.approx
GAMMA = 0.9


# --- q_learning_update --------------------------------------------------------------------------
def test_update_bootstraps_from_the_best_next_action(impl):
    Q = np.zeros((24, 4))
    Q[1, 1] = 0.5
    Q[2] = [0.0, 2.0, 5.0, 1.0]
    # target = -1 + 0.9 * max(0, 2, 5, 1) = 3.5 ; td = 3.5 - 0.5 = 3.0 ; Q <- 0.5 + 0.5 * 3.0 = 2.0
    td = impl.q_learning_update(Q, 1, 1, -1.0, 2, False, alpha=0.5, gamma=GAMMA)
    assert td == approx(3.0)
    assert Q[1, 1] == approx(2.0), "Q[s, a] must be updated in place"


def test_update_does_not_bootstrap_from_terminal_state(impl):
    Q = np.zeros((24, 4))
    Q[4, 1] = 3.0
    Q[5] = 100.0  # garbage in the terminal row must be ignored
    # target = 10 (terminal) ; td = 7 ; Q <- 3 + 0.1 * 7 = 3.7
    td = impl.q_learning_update(Q, 4, 1, 10.0, 5, True, alpha=0.1, gamma=GAMMA)
    assert td == approx(7.0)
    assert Q[4, 1] == approx(3.7), "a terminal next state has no future: target = reward"


def test_update_touches_only_one_entry(impl):
    rng = np.random.default_rng(0)
    Q = rng.normal(size=(24, 4))
    before = Q.copy()
    impl.q_learning_update(Q, 7, 2, -1.0, 13, False, alpha=0.3, gamma=GAMMA)
    changed = np.argwhere(Q != before)
    assert changed.tolist() == [[7, 2]]


def test_update_returns_a_python_float(impl):
    Q = np.zeros((24, 4))
    assert isinstance(impl.q_learning_update(Q, 0, 1, -1.0, 1, False, 0.5, GAMMA), float)


def test_one_hand_driven_episode(impl):
    env = impl.GridWorld()
    Q = np.zeros((env.n_states, env.n_actions))
    # S -> right, right, down, down, right, right, up, up, right -> G (9 moves)
    plan = [1, 1, 2, 2, 1, 1, 0, 0, 1]
    s = env.reset()
    for a in plan:
        s2, r, terminated, truncated = env.step(a)
        impl.q_learning_update(Q, s, a, r, s2, terminated, alpha=0.5, gamma=GAMMA)
        s = s2
    assert terminated and env.cell(s) == "G"
    # Every non-final move: 0.5 * (-1 + 0.9 * 0) = -0.5 ; the last one: 0.5 * 10 = 5.0
    assert Q[0, 1] == approx(-0.5)
    assert Q[10, 0] == approx(-0.5)
    assert Q[4, 1] == approx(5.0), "the last move, (0,4) right into G, earns 0.5 * 10"
    assert np.count_nonzero(Q) == len(plan), "each (state, action) of the episode updated once"


# --- greedy_action / greedy_policy --------------------------------------------------------------
def test_greedy_action_picks_the_max(impl):
    Q = np.zeros((3, 4))
    Q[1] = [-3.0, -1.0, -2.0, -5.0]
    assert impl.greedy_action(Q, 1) == 1
    assert isinstance(impl.greedy_action(Q, 1), int)


def test_greedy_action_breaks_ties_to_lowest_index(impl):
    Q = np.zeros((3, 4))
    Q[2] = [1.0, 4.0, 4.0, 4.0]
    assert impl.greedy_action(Q, 2) == 1
    assert impl.greedy_action(Q, 0) == 0  # all zeros


def test_greedy_policy_is_vectorized_greedy_action(impl):
    rng = np.random.default_rng(3)
    Q = rng.integers(-3, 3, size=(24, 4)).astype(float)  # many ties on purpose
    policy = impl.greedy_policy(Q)
    assert np.asarray(policy).shape == (24,)
    assert [int(a) for a in policy] == [impl.greedy_action(Q, s) for s in range(24)]


# --- epsilon_greedy -----------------------------------------------------------------------------
def test_epsilon_zero_is_greedy(impl):
    Q = np.zeros((2, 4))
    Q[0, 3] = 1.0
    rng = np.random.default_rng(0)
    assert {impl.epsilon_greedy(Q, 0, 0.0, rng) for _ in range(200)} == {3}


def test_epsilon_one_is_uniform(impl):
    Q = np.zeros((2, 4))
    Q[0, 3] = 1.0
    rng = np.random.default_rng(1)
    counts = np.bincount([impl.epsilon_greedy(Q, 0, 1.0, rng) for _ in range(8000)], minlength=4)
    assert counts == approx([2000] * 4, rel=0.1), f"expected ~2000 of each action, got {counts}"


def test_epsilon_02_picks_greedy_85_percent(impl):
    # greedy with prob (1 - 0.2) + 0.2 / 4 = 0.85
    Q = np.zeros((2, 4))
    Q[0, 2] = 1.0
    rng = np.random.default_rng(2)
    picks = np.array([impl.epsilon_greedy(Q, 0, 0.2, rng) for _ in range(10000)])
    assert np.mean(picks == 2) == approx(0.85, abs=0.02)


def test_epsilon_greedy_uses_the_rng_as_specified(impl):
    Q = np.zeros((2, 4))
    Q[1, 1] = 1.0
    ours, reference = np.random.default_rng(7), np.random.default_rng(7)
    expected = []
    for _ in range(50):
        expected.append(int(reference.integers(4)) if reference.random() < 0.3 else 1)
    got = [impl.epsilon_greedy(Q, 1, 0.3, ours) for _ in range(50)]
    assert got == expected, "one rng.random() per call, plus one rng.integers(4) only when exploring"


# --- the fixed gridworld: sweeping updates reach the optimal Q --------------------------------
def optimal_q(env, gamma: float) -> np.ndarray:
    """Value iteration on the known model (independent of the student's code)."""
    Q = np.zeros((env.n_states, env.n_actions))
    for _ in range(200):
        for s in range(env.n_states):
            if env.is_terminal(s) or env.cell(s) == "#":
                continue
            for a in range(env.n_actions):
                s2, r, done = env.transition(s, a)
                Q[s, a] = r + (0.0 if done else gamma * Q[s2].max())
    return Q


def test_sweeps_of_updates_converge_to_optimal_q_and_policy(impl):
    env = impl.GridWorld()
    Q = np.zeros((env.n_states, env.n_actions))
    free = [s for s in range(env.n_states) if not env.is_terminal(s) and env.cell(s) != "#"]
    for _ in range(60):  # alpha = 1 on a deterministic world = value iteration, one entry at a time
        for s in free:
            for a in range(env.n_actions):
                s2, r, done = env.transition(s, a)
                impl.q_learning_update(Q, s, a, r, s2, done, alpha=1.0, gamma=GAMMA)
    Q_star = optimal_q(env, GAMMA)
    assert Q[free] == approx(Q_star[free], abs=1e-6)
    assert Q_star[0].max() == approx(-1 - 0.9 - 0.81 - 0.729 - 0.6561 - 0.59049 - 0.531441 - 0.4782969
                                    + 10 * 0.9 ** 8)  # 9 moves: 8 x (-1), then +10
    policy = impl.greedy_policy(Q)
    for s in free:  # compare only where the best action is unique
        best = np.flatnonzero(np.isclose(Q_star[s], Q_star[s].max(), atol=1e-6))
        assert policy[s] in best, f"state {divmod(s, env.n_cols)}: greedy action {policy[s]}, optimal {best}"
    # From S the robot goes right, around the wall, and not down the stairs.
    assert policy[env.start] == 1
