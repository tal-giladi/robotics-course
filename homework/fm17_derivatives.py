"""fm17_derivatives.py — derivatives for robots: finite differences, encoder velocity, gradients.

Run:  python fm17_derivatives.py      (writes fm17_encoder_velocity.png)
Needs: numpy, matplotlib
"""
from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

OUT = Path(__file__).resolve().parent
TICKS_PER_REV = 2464          # karmel: 11 PPR x 56:1 gearbox x 4 (quadrature)
WHEEL_RADIUS_M = 0.045


def position(t: float) -> float:
    """A robot oscillating on a rail: x(t) = 0.5 sin(0.8 t) meters."""
    return 0.5 * math.sin(0.8 * t)


def finite_difference_errors() -> None:
    t = 1.0
    v_exact = 0.5 * 0.8 * math.cos(0.8 * t)
    print(f"1) Derivative of x(t)=0.5 sin(0.8t) at t=1: exact v={v_exact:.6f} m/s")
    print("        h       forward error    central error")
    for h in (1e-1, 1e-2, 1e-3, 1e-5, 1e-8, 1e-11):
        fwd = (position(t + h) - position(t)) / h
        ctr = (position(t + h) - position(t - h)) / (2 * h)
        print(f"   {h:8.0e}    {abs(fwd - v_exact):12.3e}    {abs(ctr - v_exact):12.3e}")


def encoder_velocity(rng: np.random.Generator) -> None:
    """Differentiate quantized encoder counts at different sample rates."""
    print("2) Wheel speed from encoder ticks (true speed 10 rad/s + 1 rad/s wobble at 2 Hz)")
    t_fine = np.arange(0.0, 2.0, 1e-4)
    omega_true = 10.0 + np.sin(2 * math.pi * 2.0 * t_fine)
    angle = np.cumsum(omega_true) * 1e-4 + rng.uniform(0, 2 * math.pi / TICKS_PER_REV)
    ticks = np.floor(angle * TICKS_PER_REV / (2 * math.pi)).astype(np.int64)
    fig, ax = plt.subplots(figsize=(7, 3.5))
    for rate_hz in (1000, 100, 20):
        step = int(round(1e4 / rate_hz))
        tk, tt = ticks[::step], t_fine[::step]
        dt = 1.0 / rate_hz
        omega_est = np.diff(tk) * 2 * math.pi / TICKS_PER_REV / dt
        t_mid = tt[:-1] + dt / 2
        omega_mid = 10.0 + np.sin(2 * math.pi * 2.0 * t_mid)
        err = omega_est - omega_mid
        res = 2 * math.pi / TICKS_PER_REV / dt
        print(f"   {rate_hz:>4} Hz: resolution={res:6.3f} rad/s ({res * WHEEL_RADIUS_M * 1000:6.1f} mm/s)  "
              f"error std={err.std():.3f} rad/s")
        ax.step(t_mid, omega_est, where="mid", lw=1, label=f"{rate_hz} Hz difference")
    ax.plot(t_fine, omega_true, "k", lw=2, label="true")
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 20)
    ax.set_xlabel("time [s]")
    ax.set_ylabel("wheel speed [rad/s]")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(OUT / "fm17_encoder_velocity.png", dpi=120)


def range_bearing(pose: np.ndarray, landmark: np.ndarray) -> np.ndarray:
    dx, dy = landmark[0] - pose[0], landmark[1] - pose[1]
    return np.array([math.hypot(dx, dy), math.atan2(dy, dx) - pose[2]])


def range_bearing_jacobian(pose: np.ndarray, landmark: np.ndarray) -> np.ndarray:
    dx, dy = landmark[0] - pose[0], landmark[1] - pose[1]
    q = dx * dx + dy * dy
    d = math.sqrt(q)
    return np.array([[-dx / d, -dy / d, 0.0],
                     [dy / q, -dx / q, -1.0]])


def numerical_jacobian(f: Callable[[np.ndarray], np.ndarray], x: np.ndarray, h: float = 1e-6) -> np.ndarray:
    fx = f(x)
    J = np.zeros((fx.size, x.size))
    for i in range(x.size):
        e = np.zeros_like(x)
        e[i] = h
        J[:, i] = (f(x + e) - f(x - e)) / (2 * h)
    return J


def gradients() -> None:
    pose = np.array([0.5, 0.2, 0.0])
    landmark = np.array([2.0, 1.0])
    print("3) Range and bearing to a landmark at (2.0, 1.0) from pose (0.5, 0.2, 0.0)")
    print("   z =", range_bearing(pose, landmark).round(5).tolist())
    J = range_bearing_jacobian(pose, landmark)
    Jn = numerical_jacobian(lambda p: range_bearing(p, landmark), pose)
    print("   analytic Jacobian :", J.round(5).tolist())
    print("   numerical Jacobian:", Jn.round(5).tolist())
    print(f"   max abs difference: {np.abs(J - Jn).max():.2e}")
    g = J[0, :2]
    print(f"   gradient of range wrt (x,y) = {g.round(5).tolist()}, norm = {np.linalg.norm(g):.5f}")


def arm_chain_rule() -> None:
    l1, l2 = 0.12, 0.10
    q = np.array([math.radians(30), math.radians(60)])

    def fk(q: np.ndarray) -> np.ndarray:
        return np.array([l1 * math.cos(q[0]) + l2 * math.cos(q[0] + q[1]),
                         l1 * math.sin(q[0]) + l2 * math.sin(q[0] + q[1])])

    s1, c1 = math.sin(q[0]), math.cos(q[0])
    s12, c12 = math.sin(q[0] + q[1]), math.cos(q[0] + q[1])
    J = np.array([[-l1 * s1 - l2 * s12, -l2 * s12],
                  [l1 * c1 + l2 * c12, l2 * c12]])
    print("4) 2-link arm (0.12 m, 0.10 m) at q=(30 deg, 60 deg): chain rule Jacobian")
    print("   end effector:", fk(q).round(5).tolist())
    print("   analytic J :", J.round(5).tolist())
    print("   numerical J:", numerical_jacobian(fk, q).round(5).tolist())
    qdot = np.array([0.5, -0.5])
    print("   joint speeds (0.5, -0.5) rad/s -> tip velocity", (J @ qdot).round(5).tolist(), "m/s")


def main() -> None:
    rng = np.random.default_rng(seed=17)
    finite_difference_errors()
    encoder_velocity(rng)
    gradients()
    arm_chain_rule()
    print("wrote fm17_encoder_velocity.png")


if __name__ == "__main__":
    main()
