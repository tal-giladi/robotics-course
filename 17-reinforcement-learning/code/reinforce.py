"""reinforce.py — policy gradients from the log-derivative trick to REINFORCE (lesson 17.05).

    py 17-reinforcement-learning/code/reinforce.py --part trick      # numbers for the derivation
    py 17-reinforcement-learning/code/reinforce.py --part dock        # Gaussian policy, numpy, ~10 s
    py 17-reinforcement-learning/code/reinforce.py --part cartpole   # PyTorch REINFORCE, a few minutes

Part "trick": a 3-action softmax policy (three turning speeds for a docking manoeuvre). The exact
gradient of the expected reward vs the score-function (log-derivative) estimate from samples,
with and without a baseline, and a finite-difference check.

Part "dock": karmel is x metres in front of its charger and must creep up to it. The policy is a
Gaussian over the speed command, a ~ N(k * x, sigma^2), with two parameters (k, log sigma).
REINFORCE with and without a baseline, gradients written out by hand.

Part "cartpole": the same algorithm with a neural network policy in PyTorch on CartPole-v1.
"""

from __future__ import annotations

import argparse
import time

import numpy as np


# ------------------------------------------------------------------------------------------------
# Part 1: the log-derivative trick on a 3-action softmax policy
# ------------------------------------------------------------------------------------------------
def softmax(z: np.ndarray) -> np.ndarray:
    e = np.exp(z - z.max())
    return e / e.sum()


def part_trick(seed: int = 0) -> None:
    theta = np.array([0.5, 0.0, -0.5])  # logits for: turn slow, turn medium, turn fast
    R = np.array([1.0, 3.0, 0.0])  # expected reward of each action (medium docks best)
    pi = softmax(theta)
    J = float(pi @ R)
    exact = pi * (R - J)  # dJ/dtheta_k = pi_k (R_k - J)
    print(f"pi = {np.round(pi, 4)}   J = sum_a pi(a) R(a) = {J:.4f}")
    print(f"exact gradient  pi_k (R_k - J)         = {np.round(exact, 4)}")
    eps = 1e-6
    fd = np.array([(softmax(theta + eps * np.eye(3)[k]) @ R - softmax(theta - eps * np.eye(3)[k]) @ R) / (2 * eps) for k in range(3)])
    print(f"finite differences                     = {np.round(fd, 4)}")

    # One sample worked by hand: a = 1 (medium), reward 3.
    score = np.eye(3)[1] - pi  # grad log pi(a=1) = onehot(1) - pi
    print(f"\none sample a=1, R=3: grad log pi(a) = onehot - pi = {np.round(score, 4)}; R * score = {np.round(3 * score, 4)}")

    rng = np.random.default_rng(seed)
    print(f"\n{'samples N':>9} {'estimate (no baseline)':>32} {'std of est.':>22} {'estimate (baseline b=J)':>32} {'std of est.':>22}")
    for n in (10, 100, 10_000):
        est, est_b = [], []
        for _ in range(500):  # 500 independent estimates to measure the spread
            a = rng.choice(3, size=n, p=pi)
            r = R[a] + rng.normal(0.0, 1.0, n)  # noisy rewards, like real docking attempts
            scores = np.eye(3)[a] - pi
            est.append((r[:, None] * scores).mean(axis=0))
            est_b.append(((r - J)[:, None] * scores).mean(axis=0))
        est, est_b = np.array(est), np.array(est_b)
        print(f"{n:>9} {np.array2string(est[0], precision=3):>32} {np.array2string(est.std(axis=0), precision=3):>22} "
              f"{np.array2string(est_b[0], precision=3):>32} {np.array2string(est_b.std(axis=0), precision=3):>22}")


# ------------------------------------------------------------------------------------------------
# Part 2: REINFORCE with a Gaussian policy on a 1-D docking task (numpy, gradients by hand)
# ------------------------------------------------------------------------------------------------
DT, HORIZON, V_MAX = 0.1, 30, 0.5


def dock_episode(k: float, log_sigma: float, rng: np.random.Generator):
    """Roll out one episode. Returns per-step (x, a, reward) arrays."""
    x = rng.uniform(0.5, 1.5)
    sigma = np.exp(log_sigma)
    xs, acts, rews = [], [], []
    for _ in range(HORIZON):
        a = rng.normal(k * x, sigma)  # sample the speed command
        v = np.clip(a, -V_MAX, V_MAX)
        x_next = x - v * DT
        reward = -abs(x_next) - 0.1 * a**2  # be close to the dock; don't slam the motors
        xs.append(x)
        acts.append(a)
        rews.append(reward)
        x = x_next
    return np.array(xs), np.array(acts), np.array(rews)


def reinforce_dock(iterations: int, batch: int, lr: float, baseline: bool, seed: int):
    rng = np.random.default_rng(seed)
    k, log_sigma = 0.0, np.log(0.3)
    history = []
    for it in range(iterations):
        episodes = [dock_episode(k, log_sigma, rng) for _ in range(batch)]
        # return-to-go G_t = sum_{t' >= t} r_t'   (no discount: 30-step horizon)
        G = np.array([np.cumsum(r[::-1])[::-1] for _, _, r in episodes])  # (batch, T)
        b = G.mean(axis=0) if baseline else np.zeros(HORIZON)  # time-dependent average return-to-go
        sigma = np.exp(log_sigma)
        g_k, g_ls = 0.0, 0.0
        for (x, a, _), G_i in zip(episodes, G, strict=True):
            mu = k * x
            dlogp_dk = (a - mu) / sigma**2 * x  # d/dk log N(a; kx, sigma^2)
            dlogp_dls = (a - mu) ** 2 / sigma**2 - 1.0  # d/d(log sigma) log N(...)
            g_k += np.sum((G_i - b) * dlogp_dk)
            g_ls += np.sum((G_i - b) * dlogp_dls)
        g_k, g_ls = g_k / batch, g_ls / batch
        k += lr * np.clip(g_k, -50, 50)  # gradient ASCENT on expected return
        log_sigma += lr * np.clip(g_ls, -50, 50)
        log_sigma = float(np.clip(log_sigma, np.log(0.02), np.log(1.0)))
        history.append((it, float(G[:, 0].mean()), k, float(np.exp(log_sigma)), g_k))
    return history


def evaluate_dock(k: float, episodes: int = 300, seed: int = 1000) -> float:
    """Mean return of the deterministic controller a = k x on fixed held-out start distances."""
    return float(np.mean([dock_episode(k, np.log(1e-9), np.random.default_rng(seed + i))[2].sum() for i in range(episodes)]))


def part_dock(seed: int = 0) -> None:
    print("1-D docking: a ~ N(k * x, sigma^2), 30 steps of 0.1 s, reward -|x| - 0.1 a^2, batch 16 episodes, lr 0.0005")
    for baseline in (False, True):
        t0 = time.perf_counter()
        hist = reinforce_dock(iterations=300, batch=16, lr=0.0005, baseline=baseline, seed=seed)
        print(f"\n{'with' if baseline else 'without'} baseline ({time.perf_counter() - t0:.1f} s):")
        print(f"{'iteration':>9} {'batch mean return':>18} {'k':>7} {'sigma':>7}")
        for it, ret, k, sigma, _ in hist[::50] + [hist[-1]]:
            print(f"{it:>9} {ret:>18.2f} {k:>7.2f} {sigma:>7.3f}")
        grads = np.array([h[4] for h in hist[-100:]])
        print(f"std of the k-gradient estimate over the last 100 iterations: {grads.std():.1f}")
        print(f"final mean action a = {hist[-1][2]:.2f} x on 300 held-out episodes: return {evaluate_dock(hist[-1][2]):.2f}")
    print("\nreference, hand-tuned P-controllers a = k x on the same 300 episodes:")
    print("  " + "   ".join(f"k={k}: {evaluate_dock(k):.2f}" for k in (0.5, 1.0, 1.5, 2.0, 3.0)))


# ------------------------------------------------------------------------------------------------
# Part 3: REINFORCE with a neural network on CartPole-v1 (PyTorch)
# ------------------------------------------------------------------------------------------------
def part_cartpole(episodes: int, seed: int = 0) -> None:
    import gymnasium as gym
    import torch
    from torch import nn

    torch.manual_seed(seed)
    torch.set_num_threads(1)
    env = gym.make("CartPole-v1")
    policy = nn.Sequential(nn.Linear(4, 64), nn.Tanh(), nn.Linear(64, 2))  # logits
    opt = torch.optim.Adam(policy.parameters(), lr=1e-2)
    gamma, lengths, t0 = 0.99, [], time.perf_counter()
    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        logps, rewards = [], []
        while True:
            dist = torch.distributions.Categorical(logits=policy(torch.as_tensor(obs, dtype=torch.float32)))
            a = dist.sample()
            logps.append(dist.log_prob(a))
            obs, r, term, trunc, _ = env.step(int(a))
            rewards.append(r)
            if term or trunc:
                break
        G, returns = 0.0, []
        for r in reversed(rewards):
            G = r + gamma * G
            returns.append(G)
        returns_t = torch.tensor(returns[::-1])
        advantages = (returns_t - returns_t.mean()) / (returns_t.std() + 1e-8)  # baseline + scaling
        loss = -(torch.stack(logps) * advantages).sum()  # minimizing -J == gradient ascent on J
        opt.zero_grad()
        loss.backward()
        opt.step()
        lengths.append(len(rewards))
        if (ep + 1) % max(1, episodes // 10) == 0:
            print(f"episode {ep + 1:>5}  mean length (last 50) {np.mean(lengths[-50:]):6.1f}   {time.perf_counter() - t0:5.0f} s")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--part", choices=["trick", "dock", "cartpole"], default="trick")
    ap.add_argument("--episodes", type=int, default=600)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()
    {"trick": lambda: part_trick(args.seed), "dock": lambda: part_dock(args.seed),
     "cartpole": lambda: part_cartpole(args.episodes, args.seed)}[args.part]()


if __name__ == "__main__":
    main()
