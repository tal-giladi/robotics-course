"""cross_embodiment.py - two unglamorous problems behind every robot foundation model (lesson 18.09).

  1. Action spaces differ: an SO-101 sends 6 joint positions in degrees, a Franka 7 joints in radians
     plus a gripper width in metres, a mobile base 2 velocities. One network needs one tensor shape
     and comparable numbers -> pad to a fixed width with a mask, normalize per dataset.
  2. Datasets differ in size by orders of magnitude -> how often do you sample each one?

numpy only. Run: py cross_embodiment.py
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

MAX_ACTION_DIM = 32   # SmolVLA's default max_action_dim / max_state_dim in LeRobot v0.6.1


@dataclass(frozen=True)
class Embodiment:
    name: str
    dims: int
    unit: str
    typical_scale: float     # standard deviation of each action dimension, in its own unit


def pad_with_mask(actions: np.ndarray, max_dim: int = MAX_ACTION_DIM) -> tuple[np.ndarray, np.ndarray]:
    """(T, d) -> (T, max_dim) zero-padded, plus a (max_dim,) boolean mask of real dimensions."""
    t, d = actions.shape
    if d > max_dim:
        raise ValueError(f"{d} action dims > max_dim {max_dim}")
    out = np.zeros((t, max_dim))
    out[:, :d] = actions
    mask = np.zeros(max_dim, dtype=bool)
    mask[:d] = True
    return out, mask


def normalize(actions: np.ndarray, mean: np.ndarray, std: np.ndarray) -> np.ndarray:
    return (actions - mean) / np.maximum(std, 1e-8)


def masked_l1(pred: np.ndarray, target: np.ndarray, mask: np.ndarray) -> float:
    """Loss over real dimensions only; padded zeros must not count as 'easy correct answers'."""
    return float(np.abs(pred - target)[:, mask].mean())


def sampling_weights(sizes: dict[str, int], alpha: float) -> dict[str, float]:
    """p_i proportional to n_i ** alpha. alpha=1: proportional to size; alpha=0: uniform."""
    raw = {k: n**alpha for k, n in sizes.items()}
    total = sum(raw.values())
    return {k: v / total for k, v in raw.items()}


def main() -> None:
    rng = np.random.default_rng(0)
    robots = [
        Embodiment("SO-101 (deg, 6)", 6, "deg", 20.0),
        Embodiment("Franka (rad, 7+1)", 8, "rad / m", 0.3),
        Embodiment("mobile base (m/s, rad/s)", 2, "m/s, rad/s", 0.2),
    ]
    print("1. One loss across embodiments")
    raw_losses, norm_losses = [], []
    for r in robots:
        a = rng.standard_normal((1000, r.dims)) * r.typical_scale
        pred = a + rng.standard_normal(a.shape) * 0.1 * r.typical_scale   # equally good (10% error) for every robot
        pa, mask = pad_with_mask(a)
        pp, _ = pad_with_mask(pred)
        raw = masked_l1(pp, pa, mask)
        mean, std = a.mean(axis=0), a.std(axis=0)
        n = masked_l1(pad_with_mask(normalize(pred, mean, std))[0], pad_with_mask(normalize(a, mean, std))[0], mask)
        unmasked = float(np.abs(pp - pa).mean())
        raw_losses.append(raw)
        norm_losses.append(n)
        print(f"   {r.name:<26} raw L1 {raw:7.3f}   normalized L1 {n:5.3f}   (unmasked raw L1 {unmasked:6.3f})")
    share = 100 * raw_losses[0] / sum(raw_losses)
    print(f"   -> without normalization the SO-101 is {share:.0f}% of the summed loss although all three "
          f"predictions are equally good; normalized: {100 * norm_losses[0] / sum(norm_losses):.0f}%")

    print("\n2. How often to sample each dataset (episodes)")
    sizes = {"DROID": 76_000, "community SO-100/101": 22_900, "your 150 demos": 150}
    for alpha in (1.0, 0.5, 0.3, 0.0):
        w = sampling_weights(sizes, alpha)
        print(f"   alpha={alpha:.1f}: " + ", ".join(f"{k} {100 * v:5.2f}%" for k, v in w.items()))
    w = sampling_weights(sizes, 0.5)
    batches = 1000
    print(f"   at alpha=0.5 and batch 64, {batches} steps show your demos "
          f"{w['your 150 demos'] * 64 * batches:.0f} times (vs {150 / sum(sizes.values()) * 64 * batches:.0f} at alpha=1)")


if __name__ == "__main__":
    main()
