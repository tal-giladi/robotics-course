"""Checker for 17.04 — replay buffer, epsilon schedule and TD targets.

Run: ``python course.py check 17.04`` (or ``--solution`` to see the reference pass).
Every test is a hand-computed number or a tiny array: the whole file runs in well under a second.
"""

from __future__ import annotations

import numpy as np
import pytest

approx = pytest.approx


def transition(impl, i: int, terminated: bool = False):
    """A recognizable transition: obs = [i, i], action = i % 4, reward = -i."""
    return impl.Transition(
        obs=np.array([float(i), float(i)]), action=i % 4, reward=-float(i),
        next_obs=np.array([float(i) + 0.5, float(i)]), terminated=terminated,
    )


# --- ReplayBuffer ---------------------------------------------------------------------------------
def test_buffer_grows_until_capacity(impl):
    buf = impl.ReplayBuffer(capacity=3)
    assert len(buf) == 0
    for i in range(3):
        buf.add(transition(impl, i))
    assert len(buf) == 3
    assert [t.action for t in buf.all()] == [0, 1, 2]


def test_buffer_overwrites_the_oldest_transition(impl):
    buf = impl.ReplayBuffer(capacity=3)
    for i in range(5):
        buf.add(transition(impl, i))
    assert len(buf) == 3, "a circular buffer never grows past its capacity"
    assert [t.reward for t in buf.all()] == [-2.0, -3.0, -4.0], "oldest first, transitions 0 and 1 dropped"


def test_buffer_keeps_wrapping(impl):
    buf = impl.ReplayBuffer(capacity=4)
    for i in range(11):
        buf.add(transition(impl, i))
    assert [t.reward for t in buf.all()] == [-7.0, -8.0, -9.0, -10.0]


def test_sample_returns_stacked_arrays_with_the_right_dtypes(impl):
    buf = impl.ReplayBuffer(capacity=10)
    for i in range(10):
        buf.add(transition(impl, i, terminated=(i == 7)))
    batch = buf.sample(4, np.random.default_rng(0))
    assert set(batch) == {"obs", "action", "reward", "next_obs", "terminated"}
    assert batch["obs"].shape == (4, 2) and batch["next_obs"].shape == (4, 2)
    assert batch["action"].shape == (4,) and batch["reward"].shape == (4,)
    assert batch["obs"].dtype == np.float32 and batch["next_obs"].dtype == np.float32
    assert batch["reward"].dtype == np.float32 and batch["action"].dtype == np.int64
    assert batch["terminated"].dtype == np.float32, "the mask is multiplied into the target: use floats"
    assert set(np.unique(batch["terminated"])) <= {0.0, 1.0}


def test_sample_is_consistent_across_fields(impl):
    """obs, action and reward of a sampled transition must belong to the SAME transition."""
    buf = impl.ReplayBuffer(capacity=20)
    for i in range(20):
        buf.add(transition(impl, i))
    batch = buf.sample(8, np.random.default_rng(3))
    for obs, action, reward in zip(batch["obs"], batch["action"], batch["reward"], strict=True):
        i = obs[0]
        assert action == i % 4 and reward == approx(-i)


def test_sample_is_random_but_reproducible(impl):
    buf = impl.ReplayBuffer(capacity=50)
    for i in range(50):
        buf.add(transition(impl, i))
    a = buf.sample(5, np.random.default_rng(1))["reward"]
    b = buf.sample(5, np.random.default_rng(1))["reward"]
    c = buf.sample(5, np.random.default_rng(2))["reward"]
    assert a.tolist() == b.tolist(), "same generator seed -> same batch"
    assert a.tolist() != c.tolist(), "a different seed must give a different batch"
    assert len(set(a.tolist())) == 5, "sample without replacement"


def test_sample_rejects_a_batch_larger_than_the_buffer(impl):
    buf = impl.ReplayBuffer(capacity=10)
    for i in range(3):
        buf.add(transition(impl, i))
    with pytest.raises(ValueError):
        buf.sample(4, np.random.default_rng(0))


# --- epsilon_at -----------------------------------------------------------------------------------
def test_epsilon_schedule_hand_computed(impl):
    # dqn.py's defaults: 1.0 -> 0.05 over the first 40% of a 30 000-step run
    assert impl.epsilon_at(1, 30_000) == approx(1.0 - 0.95 / 12_000)
    assert impl.epsilon_at(6_000, 30_000) == approx(0.525)
    assert impl.epsilon_at(12_000, 30_000) == approx(0.05)


def test_epsilon_never_falls_below_the_floor(impl):
    for step in (12_001, 20_000, 30_000, 10**6):
        assert impl.epsilon_at(step, 30_000) == approx(0.05)


def test_epsilon_honours_its_arguments(impl):
    assert impl.epsilon_at(500, 10_000, eps_start=0.5, eps_end=0.1, fraction=0.2) == approx(0.4)
    assert impl.epsilon_at(2_000, 10_000, eps_start=0.5, eps_end=0.1, fraction=0.2) == approx(0.1)


# --- td_target ------------------------------------------------------------------------------------
GAMMA = 0.95
NEXT_Q = np.array([[2.1, 3.4, -0.7, 1.2], [0.4, 0.9, 1.9, -2.0], [5.0, 5.0, 5.0, 5.0]])
REWARD = np.array([-1.0, -1.0, 10.0])


def test_td_target_matches_the_lesson_example(impl):
    y = impl.td_target(REWARD, np.array([0.0, 0.0, 1.0]), NEXT_Q, GAMMA)
    assert np.asarray(y).shape == (3,)
    assert y == approx([2.23, 0.805, 10.0])


def test_td_target_masks_terminal_transitions(impl):
    y = impl.td_target(np.array([10.0]), np.array([1.0]), np.array([[100.0, 100.0, 100.0, 100.0]]), GAMMA)
    assert y == approx([10.0]), "a terminal state has no future: whatever the network says is ignored"


def test_td_target_accepts_boolean_flags(impl):
    y = impl.td_target(REWARD, np.array([False, False, True]), NEXT_Q, GAMMA)
    assert y == approx([2.23, 0.805, 10.0])


def test_td_target_bootstraps_truncated_transitions(impl):
    # a time limit is NOT a termination: terminated stays 0 and the bootstrap remains
    y = impl.td_target(np.array([-0.05]), np.array([0.0]), np.array([[2.0, 3.0, 1.0, 2.5]]), 0.99)
    assert y == approx([2.92])


# --- double_dqn_target ----------------------------------------------------------------------------
def test_double_dqn_uses_the_online_argmax_and_the_target_value(impl):
    online = np.array([[1.0, 5.0, 0.0, 0.0]])   # online network prefers action 1
    target = np.array([[9.0, 2.0, 0.0, 0.0]])   # target network's own max is action 0, value 9
    y = impl.double_dqn_target(np.array([0.0]), np.array([0.0]), online, target, gamma=1.0)
    assert y == approx([2.0]), "evaluate the ONLINE network's choice with the TARGET network"
    plain = impl.td_target(np.array([0.0]), np.array([0.0]), target, gamma=1.0)
    assert plain == approx([9.0]), "the plain target takes the target network's max: the overestimate"


def test_double_dqn_masks_terminal_transitions(impl):
    y = impl.double_dqn_target(np.array([10.0, -1.0]), np.array([1.0, 0.0]),
                               np.array([[0.0, 8.0], [3.0, 1.0]]), np.array([[7.0, 6.0], [2.0, 4.0]]), gamma=0.5)
    assert y == approx([10.0, 0.0])  # second: argmax_online = 0 -> target 2.0 -> -1 + 0.5 * 2 = 0
