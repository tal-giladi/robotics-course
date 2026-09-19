"""action_representations.py - three ways a VLA can output actions, on toy data (lesson 18.07).

  1. binned tokens (RT-2, OpenVLA): 256 bins per dimension, one token per dimension per step
  2. compressed tokens (FAST idea): DCT of the chunk, keep and quantize the big coefficients
  3. flow matching (pi0, SmolVLA, GR00T): integrate a velocity field from noise to an action,
     shown on two valid demonstrations (pass an obstacle left or right)

numpy only. Run: py action_representations.py
"""
from __future__ import annotations

import numpy as np

H = 50                      # chunk length (actions)
LOW, HIGH = -0.05, 0.05     # per-step end-effector delta range, metres
BINS = 256


def demo_chunk() -> np.ndarray:
    """A smooth 50-step, 1-D chunk of x-deltas (m): accelerate, cruise, slow down."""
    t = np.linspace(0, 1, H)
    return 0.004 * np.sin(np.pi * t) ** 2


def to_bins(a: np.ndarray) -> np.ndarray:
    return np.clip(((a - LOW) / (HIGH - LOW) * BINS).astype(int), 0, BINS - 1)


def from_bins(b: np.ndarray) -> np.ndarray:
    return LOW + (b + 0.5) * (HIGH - LOW) / BINS


def dct_ii(x: np.ndarray) -> np.ndarray:
    n = len(x)
    k = np.arange(n)[:, None]
    basis = np.cos(np.pi / n * (np.arange(n)[None, :] + 0.5) * k)
    scale = np.full(n, np.sqrt(2 / n)); scale[0] = np.sqrt(1 / n)
    return scale * (basis @ x)


def idct_ii(c: np.ndarray) -> np.ndarray:
    n = len(c)
    k = np.arange(n)[None, :]
    basis = np.cos(np.pi / n * (np.arange(n)[:, None] + 0.5) * k)
    scale = np.full(n, np.sqrt(2 / n)); scale[0] = np.sqrt(1 / n)
    return basis @ (scale * c)


def main() -> None:
    a = demo_chunk()

    # 1. binning
    b = to_bins(a)
    err = np.abs(from_bins(b) - a).max()
    print(f"1. binning: bin width {1000 * (HIGH - LOW) / BINS:.3f} mm, max error {1000 * err:.3f} mm, "
          f"{H} tokens for a 1-D chunk, {7 * H} tokens for 7-DoF")

    # 2. DCT compression (the FAST idea, without the BPE step)
    c = dct_ii(a)
    q = np.round(c / 1e-4)                     # quantize coefficients to 0.1 mm steps
    nonzero = int(np.count_nonzero(q))
    rec = idct_ii(q * 1e-4)
    print(f"2. DCT: {nonzero} non-zero quantized coefficients out of {H}, "
          f"max reconstruction error {1000 * np.abs(rec - a).max():.3f} mm")

    # 3. flow matching with TWO valid demonstrations: pass the obstacle 3 cm left or 3 cm right.
    #    Path: x_tau = (1 - tau) * noise + tau * action. We use the exact velocity field for this
    #    two-mode data (what a perfectly trained network would output) and integrate with Euler.
    modes = np.array([-0.03, 0.03])            # lateral offset, metres
    sigma = 0.03                               # noise scale
    rng = np.random.default_rng(0)
    x0 = rng.standard_normal(2000) * sigma

    def velocity(x: np.ndarray, tau: float) -> np.ndarray:
        s = max(1 - tau, 1e-6) * sigma         # std of x_tau around tau * mode
        logw = -((x[:, None] - tau * modes[None, :]) ** 2) / (2 * s * s)
        w = np.exp(logw - logw.max(axis=1, keepdims=True))
        w /= w.sum(axis=1, keepdims=True)
        return (w * (modes[None, :] - x[:, None])).sum(axis=1) / max(1 - tau, 1e-6)

    for steps in (1, 2, 4, 10):
        x = x0.copy()
        for i in range(steps):
            x = x + velocity(x, i / steps) / steps
        near = np.abs(x[:, None] - modes[None, :]).min(axis=1) < 0.005
        middle = np.abs(x - modes.mean()) < 0.01
        print(f"3. flow matching, {steps:2d} Euler steps: {100 * near.mean():5.1f}% of samples within 5 mm of a "
              f"demonstrated path, {100 * middle.mean():5.1f}% within 1 cm of the middle (collision)")

    tau = 0.5
    print(f"   one training example at tau={tau}: noise {1000 * x0[0]:+.1f} mm, action +30.0 mm -> "
          f"x_tau {1000 * ((1 - tau) * x0[0] + tau * 0.03):+.1f} mm, target velocity {1000 * (0.03 - x0[0]):+.1f} mm")


if __name__ == "__main__":
    main()
