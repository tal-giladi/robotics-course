"""il_lab.py - a tiny imitation-learning lab for module 18 (lessons 18.02, 18.04, 18.05).

Three 2D toy experiments that show, in about a minute on a laptop CPU, the failure modes that
shaped modern robot-learning policies:

  bc-dagger   compounding error: behavior cloning from perfect demos vs DAgger     (18.02)
  multimodal  two valid demos (left / right of an obstacle): MSE regression averages
              them into a collision, a diffusion policy samples one; DDPM vs DDIM (18.02, 18.05)
  chunking    single-step policy vs action chunks (open loop) vs chunks with
              ACT-style temporal ensembling: jitter and tracking error            (18.04)

Run:   py il_lab.py all              (Linux/macOS: python3 il_lab.py all)
       py il_lab.py multimodal --plots       also writes PNGs to ./out/
Needs: numpy, torch (the CPU build is fine), matplotlib only for --plots.

The "robot" is a point in the plane: state s = (x, y) in metres, action a = (vx, vy) in m/s,
s_next = s + DT * a (+ noise). SI units throughout. All rollouts are batched: a policy maps an
(n, 2) array of states to n actions (or n action chunks), so 100 episodes cost one network call per step.
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import numpy as np
import torch
from torch import nn

DT = 0.1  # s, control period (10 Hz)
OUT = Path(__file__).resolve().parent / "out"

Policy = Callable[[np.ndarray], np.ndarray]       # states (n, 2) -> actions (n, 2)
ChunkPolicy = Callable[[np.ndarray], np.ndarray]  # states (n, 2) -> chunks (n, H, 2): H actions or waypoints


# ----------------------------------------------------------------------------------------------
# Shared helpers
# ----------------------------------------------------------------------------------------------
def mlp(n_in: int, n_out: int, hidden: int = 64, layers: int = 2) -> nn.Sequential:
    mods: list[nn.Module] = []
    width = n_in
    for _ in range(layers):
        mods += [nn.Linear(width, hidden), nn.ReLU()]
        width = hidden
    mods.append(nn.Linear(width, n_out))
    return nn.Sequential(*mods)


def fit_mse(x: np.ndarray, y: np.ndarray, seed: int, epochs: int = 800, hidden: int = 64) -> nn.Module:
    """Plain behavior cloning: full-batch regression with a mean-squared-error loss."""
    torch.manual_seed(seed)
    net = mlp(x.shape[1], y.shape[1], hidden)
    opt = torch.optim.Adam(net.parameters(), lr=3e-3)
    xt = torch.as_tensor(x, dtype=torch.float32)
    yt = torch.as_tensor(y, dtype=torch.float32)
    for _ in range(epochs):
        loss = ((net(xt) - yt) ** 2).mean()
        opt.zero_grad()
        loss.backward()
        opt.step()
    return net


def as_policy(net: nn.Module, out_shape: tuple[int, ...] = (2,)) -> Callable[[np.ndarray], np.ndarray]:
    def act(states: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            out = net(torch.as_tensor(states, dtype=torch.float32)).numpy()
        return out.reshape((len(states),) + out_shape)

    return act


def demos_to_chunks(states: list[np.ndarray], actions: list[np.ndarray], horizon: int) -> tuple[np.ndarray, np.ndarray]:
    """(state_t, a_t ... a_{t+H-1}) training pairs; the last action is repeated past the episode end."""
    obs, chunks = [], []
    for s_ep, a_ep in zip(states, actions):
        padded = np.concatenate([a_ep, np.repeat(a_ep[-1:], horizon, axis=0)])
        for t in range(len(s_ep)):
            obs.append(s_ep[t])
            chunks.append(padded[t:t + horizon].reshape(-1))
    return np.array(obs), np.array(chunks)


def lateral(n: int, std: float, rng: np.random.Generator) -> np.ndarray:
    """Noise that only acts sideways (the lane tasks keep forward speed exact)."""
    return np.stack([np.zeros(n), rng.normal(0.0, std, n)], axis=1)


# ----------------------------------------------------------------------------------------------
# Experiment 1 - compounding error: behavior cloning vs DAgger on a lane-following task
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class LaneTask:
    """Follow the centre line y = A sin(2 pi x / L) for x in [0, 4] m while staying in the lane."""

    amplitude_m: float = 0.5
    wavelength_m: float = 2.0
    speed_m_s: float = 0.5
    half_width_m: float = 0.2       # |cross-track error| >= 0.2 m at any step = failure
    steps: int = 80                 # 8 s at 10 Hz: x goes 0 -> 4 m
    act_noise_m_s: float = 0.3      # std of sideways actuation noise (slip, backlash, bumps)
    k_expert: float = 3.0           # the expert's feedback gain [1/s]

    def centre(self, x: np.ndarray) -> np.ndarray:
        return self.amplitude_m * np.sin(2 * np.pi * x / self.wavelength_m)

    def slope(self, x: np.ndarray) -> np.ndarray:
        return self.amplitude_m * 2 * np.pi / self.wavelength_m * np.cos(2 * np.pi * x / self.wavelength_m)

    def expert(self, s: np.ndarray) -> np.ndarray:
        """Feed-forward along the curve + proportional feedback toward the centre line."""
        x, y = s[:, 0], s[:, 1]
        vy = self.speed_m_s * self.slope(x) + self.k_expert * (self.centre(x) - y)
        return np.stack([np.full_like(x, self.speed_m_s), np.clip(vy, -1.5, 1.5)], axis=1)

    def rollout(self, policy: Policy, n: int, rng: np.random.Generator, noise: bool = True) -> np.ndarray:
        """Returns states with shape (n, steps + 1, 2)."""
        s = np.zeros((n, 2))
        traj = [s]
        for _ in range(self.steps):
            a = policy(s)
            if noise:
                a = a + lateral(n, self.act_noise_m_s, rng)
            s = s + DT * a
            traj.append(s)
        return np.stack(traj, axis=1)

    def cross_track(self, traj: np.ndarray) -> np.ndarray:
        return np.abs(traj[..., 1] - self.centre(traj[..., 0]))


def evaluate_lane(task: LaneTask, policy: Policy, n: int, seed: int) -> tuple[float, np.ndarray]:
    """Success rate (never left the lane) and the mean |cross-track error| at every step."""
    errs = task.cross_track(task.rollout(policy, n, np.random.default_rng(seed)))
    return float(np.mean(errs.max(axis=1) < task.half_width_m)), errs.mean(axis=0)


def fmt_errs(err: np.ndarray) -> str:
    return "  ".join(f"{err[i]:.3f}" for i in (20, 40, 60, 80))


def bc_vs_dagger(n_demos: int = 10, dagger_iters: int = 4, rollouts_per_iter: int = 10,
                 n_eval: int = 200, epochs: int = 500, seed: int = 0, verbose: bool = True) -> dict:
    task = LaneTask()
    rng = np.random.default_rng(seed)

    # Demonstrations from a perfect teleoperator: no noise, so every state lies on the centre line.
    xs = task.rollout(task.expert, n_demos, rng, noise=False)[:, :-1].reshape(-1, 2)
    ys = task.expert(xs)

    expert_success, expert_err = evaluate_lane(task, task.expert, n_eval, seed + 100)
    policy = as_policy(fit_mse(xs, ys, seed, epochs))
    bc_success, bc_err = evaluate_lane(task, policy, n_eval, seed + 100)
    result: dict = {"expert_success": expert_success, "bc_success": bc_success, "bc_err": bc_err,
                    "dagger_success": [], "dagger_err": [], "dataset_sizes": [len(xs)]}
    if verbose:
        print("1) Compounding error - lane following, actuation noise 0.3 m/s, lane +/-0.2 m, 80 steps")
        print("                                          mean |cross-track error| [m] at t = 2 s  4 s  6 s  8 s")
        print(f"   expert (acting with the same noise) : success {expert_success:5.0%}   {fmt_errs(expert_err)}")
        print(f"   BC on {len(xs)} clean expert samples  : success {bc_success:5.0%}   {fmt_errs(bc_err)}")

    # DAgger: run the LEARNER, ask the expert what it would have done in the visited states,
    # aggregate everything, retrain.
    for it in range(1, dagger_iters + 1):
        visited = task.rollout(policy, rollouts_per_iter, rng)[:, :-1].reshape(-1, 2)
        xs = np.concatenate([xs, visited])
        ys = np.concatenate([ys, task.expert(visited)])
        policy = as_policy(fit_mse(xs, ys, seed + it, epochs))
        succ, err = evaluate_lane(task, policy, n_eval, seed + 100)
        result["dagger_success"].append(succ)
        result["dagger_err"].append(err)
        result["dataset_sizes"].append(len(xs))
        if verbose:
            print(f"   DAgger iteration {it} ({len(xs):4d} samples)  : success {succ:5.0%}   {fmt_errs(err)}")
    return result


# ----------------------------------------------------------------------------------------------
# Experiment 2 - multimodal demonstrations: MSE regression vs a diffusion policy over waypoint chunks
# ----------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class ObstacleTask:
    """Drive from (0, 0) to (2, 0). A round obstacle of radius 0.3 m sits at (1, 0).

    Half of the demonstrations pass left of it (y > 0), half pass right. Both are correct.
    Policies here act like position-controlled arms: they predict a CHUNK of future positions
    (a waypoint every `stride` steps, relative to the current position), and a low-level
    controller moves along them at constant velocity.
    """

    obstacle: tuple[float, float] = (1.0, 0.0)
    radius_m: float = 0.3
    goal: tuple[float, float] = (2.0, 0.0)
    speed_m_s: float = 0.5
    detour_m: float = 0.5           # lateral offset of the path beside the obstacle
    start_spread_m: float = 0.05    # start y ~ U(-spread, +spread); the side chosen does NOT depend on it
    steps: int = 70                 # 7 s at 10 Hz
    stride: int = 5                 # one waypoint every 0.5 s
    n_waypoints: int = 6            # chunk = 3 s of motion = 12 numbers

    def starts(self, n: int, rng: np.random.Generator) -> np.ndarray:
        return np.stack([np.zeros(n), rng.uniform(-self.start_spread_m, self.start_spread_m, n)], axis=1)

    def expert_demo(self, rng: np.random.Generator, side: float) -> np.ndarray:
        """A teleoperated path: beside the obstacle's near edge, its far edge, then the goal. -> states (T, 2)"""
        s = self.starts(1, rng)[0]
        ox = self.obstacle[0]
        waypoints = [np.array([ox - 0.35, side * self.detour_m]), np.array([ox + 0.35, side * self.detour_m]),
                     np.array(self.goal)]
        states = []
        for _ in range(self.steps):
            while len(waypoints) > 1 and np.linalg.norm(waypoints[0] - s) < 0.08:
                waypoints.pop(0)
            d = waypoints[0] - s
            dist = float(np.linalg.norm(d))
            v = self.speed_m_s * d / max(dist, 1e-9) * min(1.0, dist / (self.speed_m_s * DT))
            v = v + rng.normal(0.0, 0.02, 2)        # a human hand is never perfectly steady
            states.append(s.copy())
            s = s + DT * v
        return np.array(states)

    def collided(self, traj: np.ndarray) -> np.ndarray:
        return np.any(np.linalg.norm(traj - np.array(self.obstacle), axis=-1) < self.radius_m, axis=-1)

    def reached(self, traj: np.ndarray) -> np.ndarray:
        return np.linalg.norm(traj[..., -1, :] - np.array(self.goal), axis=-1) < 0.15


def waypoint_chunks(task: ObstacleTask, demos: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Training pairs (s_t, [s_(t+5) - s_t, s_(t+10) - s_t, ...]); positions past the end repeat the last one."""
    obs, chunks = [], []
    for ep in demos:
        padded = np.concatenate([ep, np.repeat(ep[-1:], task.stride * task.n_waypoints, axis=0)])
        for t in range(len(ep)):
            future = padded[t + task.stride: t + task.stride * task.n_waypoints + 1: task.stride]
            obs.append(ep[t])
            chunks.append((future - ep[t]).reshape(-1))
    return np.array(obs), np.array(chunks)


class DiffusionPolicy:
    """A minimal DDPM over action chunks, conditioned on the state (the idea of Chi et al. 2023).

    `chunks` is (N, D): any flat action chunk (here 6 waypoints x 2 = 12 numbers).

    Training: take a demo chunk a0, pick a noise level k, build a_k = sqrt(abar_k) a0 + sqrt(1-abar_k) eps,
    and teach eps_theta(a_k, k, s) to predict eps (MSE). Sampling: start from pure noise and denoise.
    """

    def __init__(self, obs: np.ndarray, chunks: np.ndarray, k_steps: int = 50, iters: int = 3000,
                 batch: int = 256, hidden: int = 128, lr: float = 2e-3, seed: int = 0) -> None:
        torch.manual_seed(seed)
        self.k_steps = k_steps
        self.dim = chunks.shape[1]
        # squared-cosine noise schedule with betas capped at 0.999 (diffusers "squaredcos_cap_v2",
        # the LeRobot Diffusion Policy default)
        f = lambda t: np.cos((t + 0.008) / 1.008 * np.pi / 2) ** 2
        t = np.arange(k_steps + 1) / k_steps
        betas = np.minimum(1.0 - f(t[1:]) / f(t[:-1]), 0.999)
        self.betas = torch.as_tensor(betas, dtype=torch.float32)
        self.alphas = 1.0 - self.betas
        self.abar = torch.cumprod(self.alphas, dim=0)
        self.a_scale = float(np.abs(chunks).max())          # normalise actions to [-1, 1]
        self.freqs = torch.tensor([1.0, 2.0, 4.0, 8.0]) * np.pi   # sinusoidal embedding of the noise level k
        self.net = mlp(self.dim + 2 + 2 * len(self.freqs), self.dim, hidden, layers=3)
        opt = torch.optim.Adam(self.net.parameters(), lr=lr)
        sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, iters)
        o = torch.as_tensor(obs, dtype=torch.float32)
        a0 = torch.as_tensor(chunks / self.a_scale, dtype=torch.float32)
        g = torch.Generator().manual_seed(seed)
        for _ in range(iters):
            idx = torch.randint(0, len(o), (batch,), generator=g)
            k = torch.randint(0, k_steps, (batch,), generator=g)
            eps = torch.randn(batch, self.dim, generator=g)
            ab = self.abar[k][:, None]
            noisy = ab.sqrt() * a0[idx] + (1 - ab).sqrt() * eps
            loss = ((self.eps(noisy, k, o[idx]) - eps) ** 2).mean()
            opt.zero_grad()
            loss.backward()
            opt.step()
            sched.step()

    def eps(self, a_k: torch.Tensor, k: torch.Tensor, s: torch.Tensor) -> torch.Tensor:
        x = (k.float() / self.k_steps)[:, None] * self.freqs
        return self.net(torch.cat([a_k, s, torch.sin(x), torch.cos(x)], dim=1))

    @torch.no_grad()
    def sample(self, states: np.ndarray, g: torch.Generator, ddim_steps: int | None = None) -> np.ndarray:
        """DDPM (all K small stochastic steps) or DDIM (a few large deterministic steps). -> (n, D)"""
        n = len(states)
        st = torch.as_tensor(states, dtype=torch.float32)
        a = torch.randn(n, self.dim, generator=g)
        if ddim_steps is None:
            for k in reversed(range(self.k_steps)):
                e = self.eps(a, torch.full((n,), k), st)
                ab = self.abar[k]
                ab_prev = self.abar[k - 1] if k > 0 else torch.tensor(1.0)
                a0 = ((a - (1 - ab).sqrt() * e) / ab.sqrt()).clamp(-1.0, 1.0)   # predicted clean chunk
                # mean of q(a_{k-1} | a_k, a0): a weighted blend of the clean guess and the current sample
                a = (ab_prev.sqrt() * self.betas[k] * a0 + self.alphas[k].sqrt() * (1 - ab_prev) * a) / (1 - ab)
                if k > 0:
                    var = self.betas[k] * (1 - ab_prev) / (1 - ab)
                    a = a + var.sqrt() * torch.randn(n, self.dim, generator=g)
        else:
            ks = np.linspace(self.k_steps - 1, 0, ddim_steps).round().astype(int)
            for i, k in enumerate(ks):
                e = self.eps(a, torch.full((n,), int(k)), st)
                ab = self.abar[k]
                a0 = ((a - (1 - ab).sqrt() * e) / ab.sqrt()).clamp(-1.0, 1.0)   # predicted clean chunk
                e = (a - ab.sqrt() * a0) / (1 - ab).sqrt()
                ab_prev = self.abar[ks[i + 1]] if i + 1 < len(ks) else torch.tensor(1.0)
                a = ab_prev.sqrt() * a0 + (1 - ab_prev).sqrt() * e
        return a.numpy() * self.a_scale


def run_obstacle(task: ObstacleTask, chunk_policy: ChunkPolicy, starts: np.ndarray,
                 execute: int = 2) -> tuple[np.ndarray, int]:
    """Receding horizon: predict 6 waypoints, drive through the first `execute`, re-plan. -> (n, T+1, 2)"""
    s = starts
    traj = [s]
    calls = 0
    while len(traj) <= task.steps:
        origin = s
        chunk = chunk_policy(s)                       # (n, n_waypoints, 2), relative to origin
        calls += 1
        for j in range(execute):
            v = (origin + chunk[:, j] - s) / (task.stride * DT)   # constant velocity to the next waypoint
            for _ in range(task.stride):
                s = s + DT * v
                traj.append(s)
    return np.stack(traj[:task.steps + 1], axis=1), calls


def multimodal(n_demos: int = 40, n_eval: int = 100, diffusion_iters: int = 3000, execute: int = 2,
               seed: int = 0, verbose: bool = True, plots: bool = False) -> dict:
    task = ObstacleTask()
    rng = np.random.default_rng(seed)
    demos = [task.expert_demo(rng, side=1.0 if i % 2 == 0 else -1.0) for i in range(n_demos)]
    obs, chunks = waypoint_chunks(task, demos)
    starts = task.starts(n_eval, np.random.default_rng(seed + 100))       # same starts for every policy
    shape = (task.n_waypoints, 2)
    result: dict = {}

    # (a) behavior cloning with MSE: the network predicts the MEAN chunk for each state.
    t0 = time.perf_counter()
    mse = as_policy(fit_mse(obs, chunks, seed, epochs=800, hidden=128), out_shape=shape)
    result["mse_train_s"] = time.perf_counter() - t0
    traj_mse, _ = run_obstacle(task, mse, starts, execute)
    result["mse_collision_rate"] = float(task.collided(traj_mse).mean())
    result["mse_success"] = float((task.reached(traj_mse) & ~task.collided(traj_mse)).mean())
    result["mse_first_chunk"] = mse(np.zeros((1, 2)))[0]

    # (b) diffusion policy: learn p(chunk | state) and SAMPLE from it.
    t0 = time.perf_counter()
    dp = DiffusionPolicy(obs, chunks, iters=diffusion_iters, seed=seed)
    result["train_s"] = time.perf_counter() - t0
    trajs = {}
    rows = []
    for name, steps in (("ddpm", None), ("ddim10", 10), ("ddim5", 5)):
        g = torch.Generator().manual_seed(seed + 1)
        t0 = time.perf_counter()
        traj, calls = run_obstacle(task, lambda s: dp.sample(s, g, ddim_steps=steps).reshape(len(s), *shape),
                                   starts, execute)
        ms = 1000 * (time.perf_counter() - t0) / calls           # one call = a whole batch of n_eval
        collided = task.collided(traj)
        ok = task.reached(traj) & ~collided
        left = traj[:, task.steps // 2, 1] > 0
        result[f"{name}_success"] = float(ok.mean())
        result[f"{name}_collision_rate"] = float(collided.mean())
        result[f"{name}_left_fraction"] = float(left.mean())
        result[f"{name}_ms_per_call"] = ms
        trajs[name] = traj
        rows.append((steps or dp.k_steps, "DDPM" if steps is None else "DDIM", ok.mean(), collided.mean(),
                     left.mean(), ms))

    if verbose:
        c = result["mse_first_chunk"]
        print(f"2) Multimodal demos - obstacle radius 0.3 m at (1, 0), {n_demos} demos: half pass left, half right")
        print("   MSE chunk at (0, 0), waypoints 1-3 [m]: " + "  ".join(f"({x:+.2f}, {y:+.2f})" for x, y in c[:3]))
        print(f"   MSE behavior cloning    : success {result['mse_success']:4.0%}   "
              f"collisions {result['mse_collision_rate']:4.0%}")
        print(f"   diffusion policy trained in {result['train_s']:.1f} s ({diffusion_iters} iterations), {n_eval} episodes:")
        for k, label, succ, coll, left, ms in rows:
            print(f"   {label} {k:2d} denoising steps: success {succ:4.0%}   collisions {coll:4.0%}   "
                  f"went left {left:4.0%}   {ms:5.2f} ms per call (batch of {n_eval})")
    if plots:
        plot_multimodal(task, demos, traj_mse[:30], trajs["ddpm"][:30])
    return result


# ----------------------------------------------------------------------------------------------
# Experiment 3 - action chunking and temporal ensembling (ACT) with a noisy sensor
# ----------------------------------------------------------------------------------------------
def chunk_rollout(task: LaneTask, predict: ChunkPolicy, k: int, mode: str, n: int, rng: np.random.Generator,
                  obs_noise_m: float, m: float = 0.1) -> tuple[np.ndarray, np.ndarray]:
    """mode: 'single' (k=1), 'open-loop' (query every k steps), 'ensemble' (query every step, ACT).

    Returns states (n, T+1, 2) and executed actions (n, T, 2).
    """
    s = np.zeros((n, 2))
    traj, acts = [s], []
    history: list[np.ndarray] = []                   # chunks predicted at t-len+1 ... t
    plan = np.zeros((n, k, 2))
    for t in range(task.steps):
        observed = s + rng.normal(0.0, obs_noise_m, (n, 2))
        if mode == "single":
            a = predict(observed)[:, 0]
        elif mode == "open-loop":
            if t % k == 0:
                plan = predict(observed)
            a = plan[:, t % k]
        else:
            history = (history + [predict(observed)])[-k:]
            # the chunk predicted i steps ago contributes its action number i; oldest first
            candidates = np.stack([c[:, len(history) - 1 - j] for j, c in enumerate(history)])
            w = np.exp(-m * np.arange(len(candidates)))          # w_0 = oldest prediction, as in ACT
            a = np.tensordot(w / w.sum(), candidates, axes=1)
        acts.append(a)
        s = s + DT * (a + lateral(n, task.act_noise_m_s, rng))
        traj.append(s)
    return np.stack(traj, axis=1), np.stack(acts, axis=1)


def chunking(k: int = 10, n_demos: int = 30, n_eval: int = 100, epochs: int = 600, seed: int = 0,
             obs_noise_m: float = 0.04, m: float = 0.1, verbose: bool = True, plots: bool = False) -> dict:
    """m is the temporal-ensembling coefficient: w_i = exp(-m i), w_0 = oldest prediction."""
    task = LaneTask(act_noise_m_s=0.05)
    rng = np.random.default_rng(seed)
    # Noisy demos: the expert is pushed around and corrects itself, so the data covers the lane.
    states, actions = [], []
    s = np.stack([np.zeros(n_demos), rng.uniform(-0.1, 0.1, n_demos)], axis=1)
    for _ in range(task.steps):
        a = task.expert(s)
        states.append(s)
        actions.append(a)
        s = s + DT * (a + lateral(n_demos, 0.3, rng))
    states_ep = list(np.stack(states, axis=1))
    actions_ep = list(np.stack(actions, axis=1))

    models: dict[int, ChunkPolicy] = {}
    for horizon in (1, k):
        obs, ch = demos_to_chunks(states_ep, actions_ep, horizon)
        models[horizon] = as_policy(fit_mse(obs, ch, seed, epochs, hidden=128), out_shape=(horizon, 2))

    result: dict = {}
    for mode, horizon in (("single", 1), ("open-loop", k), ("ensemble", k)):
        traj, acts = chunk_rollout(task, models[horizon], horizon, mode, n_eval,
                                   np.random.default_rng(seed + 7), obs_noise_m, m)
        dvy = np.diff(acts[:, :, 1], axis=1)
        result[mode] = {
            "jitter": float(np.sqrt(np.mean(dvy ** 2))),              # RMS step-to-step change of vy
            "max_jump": float(np.mean(np.abs(dvy).max(axis=1))),     # biggest single change, per episode
            "rms_error": float(np.sqrt(np.mean(task.cross_track(traj) ** 2))),
            "example_actions": acts[0],
        }
    if verbose:
        print(f"3) Action chunking - position sensor noise {obs_noise_m * 1000:.0f} mm, chunk k={k}, "
              f"ACT weights exp(-{m:g} i)")
        for mode in ("single", "open-loop", "ensemble"):
            r = result[mode]
            print(f"   {mode:9s}: jitter {r['jitter']:.3f} m/s   largest jump {r['max_jump']:.3f} m/s   "
                  f"RMS cross-track error {r['rms_error'] * 1000:5.1f} mm")
    if plots:
        plot_chunking(result)
    return result


# ----------------------------------------------------------------------------------------------
# Plots (optional)
# ----------------------------------------------------------------------------------------------
def plot_multimodal(task: ObstacleTask, demo_states: list[np.ndarray], traj_mse: np.ndarray,
                    trajs_dp: np.ndarray) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    OUT.mkdir(exist_ok=True)
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6), sharey=True)
    titles = ("demonstrations", "MSE behavior cloning", "diffusion policy (DDPM, 50 steps)")
    for ax, title in zip(axes, titles):
        ax.add_patch(plt.Circle(task.obstacle, task.radius_m, color="0.6"))
        ax.plot(*task.goal, "g*", ms=12)
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.set_xlim(-0.2, 2.3)
        ax.set_ylim(-0.9, 0.9)
        ax.set_xlabel("x [m]")
    for s in demo_states:
        axes[0].plot(s[:, 0], s[:, 1], lw=0.8)
    for tr in traj_mse:
        axes[1].plot(tr[:, 0], tr[:, 1], "r", lw=0.8)
    for tr in trajs_dp:
        axes[2].plot(tr[:, 0], tr[:, 1], lw=0.8)
    axes[0].set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(OUT / "18_multimodal.png", dpi=120)
    plt.close(fig)
    print(f"   wrote {OUT / '18_multimodal.png'}")


def plot_chunking(result: dict) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    OUT.mkdir(exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 3.2))
    for mode in ("single", "open-loop", "ensemble"):
        acts = result[mode]["example_actions"]
        ax.plot(np.arange(len(acts)) * DT, acts[:, 1], label=mode)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("commanded vy [m/s]")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "18_chunking.png", dpi=120)
    plt.close(fig)
    print(f"   wrote {OUT / '18_chunking.png'}")


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Tiny imitation-learning lab (module 18)")
    ap.add_argument("experiment", choices=["all", "bc-dagger", "multimodal", "chunking"])
    ap.add_argument("--plots", action="store_true", help="write PNGs to ./out (needs matplotlib)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args(argv)
    torch.set_num_threads(1)          # tiny networks: one thread is fast enough and reproducible
    experiments = {
        "bc-dagger": lambda: bc_vs_dagger(seed=args.seed),
        "multimodal": lambda: multimodal(seed=args.seed, plots=args.plots),
        "chunking": lambda: chunking(seed=args.seed, plots=args.plots),
    }
    t_all = time.perf_counter()
    for name, run in experiments.items():
        if args.experiment in ("all", name):
            t0 = time.perf_counter()
            run()
            print(f"   ({name}: {time.perf_counter() - t0:.1f} s)")
    print(f"done in {time.perf_counter() - t_all:.1f} s")


if __name__ == "__main__":
    main()
