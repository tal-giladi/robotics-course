"""arm_viz — draw arms, workspaces, Jacobian ellipses and trajectory profiles to PNG (no hardware).

Headless (Agg backend), writes PNGs you open afterwards:

    py arm_viz.py list
    py arm_viz.py two-link                          # both IK branches of a 2-link arm   (14.04, 14.05)
    py arm_viz.py so101 --q 0 -30 40 60 0          # SO-101 stick figure, degrees        (14.01, 14.04)
    py arm_viz.py workspace                         # SO-101 reachable points             (14.01)
    py arm_viz.py ellipses                          # manipulability ellipses             (14.06)
    py arm_viz.py resolved-rate                     # pseudo-inverse vs damped LS         (14.06)
    py arm_viz.py profiles                          # trapezoid / cubic / quintic         (14.07)
    py arm_viz.py joint-vs-cartesian                # tool paths of the two move types    (14.07)
    py arm_viz.py all --out arm_out

Frame colors follow REP-103 / RViz: x red, y green, z blue.
"""

from __future__ import annotations

import argparse
import math
import os
from collections.abc import Callable, Sequence
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.axes import Axes  # noqa: E402
from matplotlib.patches import Circle, Ellipse  # noqa: E402

import arm_kinematics as ak  # noqa: E402
import trajectories as tr  # noqa: E402

DEFAULT_OUT = Path(os.environ.get("ARM_VIZ_OUT", "arm_out"))
AXIS_COLORS = ("tab:red", "tab:green", "tab:blue")
L1, L2 = 0.116, 0.135          # SO-101 upper arm and forearm (so101_link_lengths), planar model


def _save(fig: plt.Figure, out: Path, name: str) -> Path:
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{name}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def draw_planar_arm(ax: Axes, lengths: Sequence[float], q: Sequence[float], color: str = "tab:gray",
                    label: str | None = None, alpha: float = 1.0) -> None:
    pts = ak.planar_joint_points(lengths, q)
    ax.plot(pts[:, 0], pts[:, 1], "-", color=color, lw=4, alpha=alpha, solid_capstyle="round", label=label)
    ax.plot(pts[:-1, 0], pts[:-1, 1], "o", color="black", ms=6, alpha=alpha)
    ax.plot(pts[-1, 0], pts[-1, 1], "s", color=color, ms=7, alpha=alpha)


# ------------------------------------------------------------------------------------------
# Scenes
# ------------------------------------------------------------------------------------------
def scene_two_link(out: Path) -> Path:
    target = (0.18, 0.10)
    sols = ak.two_link_ik(L1, L2, *target)
    fig, ax = plt.subplots(figsize=(6.5, 6.0))
    ax.add_patch(Circle((0, 0), L1 + L2, fill=False, ls="--", color="tab:gray", label="outer reach l1+l2"))
    ax.add_patch(Circle((0, 0), abs(L2 - L1), fill=False, ls=":", color="tab:gray", label="inner reach |l2-l1|"))
    for sol, color in zip(sols, ("tab:blue", "tab:orange")):
        draw_planar_arm(ax, (L1, L2), (sol.q1, sol.q2), color=color,
                        label=f"elbow {sol.elbow}: q1={math.degrees(sol.q1):.1f}°, q2={math.degrees(sol.q2):.1f}°")
    ax.plot(*target, "x", color="black", ms=12, mew=2, label=f"target {target}")
    ax.set_aspect("equal")
    ax.set_xlim(-0.28, 0.28)
    ax.set_ylim(-0.28, 0.28)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]  (vertical plane of the arm)")
    ax.set_title("2-link arm (SO-101 upper arm + forearm): two IK solutions")
    ax.legend(loc="lower left", fontsize=8)
    return _save(fig, out, "two-link")


def draw_so101(ax: Axes, chain: ak.SerialChain, q: Sequence[float], frame_len: float = 0.03) -> None:
    frames = chain.frames(q)
    pts = np.array([np.zeros(3)] + [T[:3, 3] for _, T in frames])
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color="tab:gray", lw=3, ms=4)
    for name, T in [("base_link", np.eye(4))] + frames:
        for k in range(3):
            end = T[:3, 3] + frame_len * T[:3, k]
            ax.plot(*zip(T[:3, 3], end), color=AXIS_COLORS[k], lw=1.5)
    tool = frames[-1][1][:3, 3]
    ax.text(*tool, f"  tool ({tool[0]:.3f}, {tool[1]:.3f}, {tool[2]:.3f})", fontsize=8)


def scene_so101(out: Path, q_deg: Sequence[float] = (0, 0, 0, 0, 0)) -> Path:
    chain = ak.load_so101()
    q = np.radians(q_deg)
    fig = plt.figure(figsize=(7, 6.5))
    ax = fig.add_subplot(projection="3d")
    draw_so101(ax, chain, q)
    ax.set_xlim(-0.1, 0.4)
    ax.set_ylim(-0.25, 0.25)
    ax.set_zlim(0.0, 0.5)
    ax.set_box_aspect((0.5, 0.5, 0.5))
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    pitch = math.degrees(ak.approach_pitch(chain.fk(q)))
    ax.set_title(f"SO-101 at q = {tuple(round(v, 1) for v in q_deg)} deg\napproach pitch {pitch:.1f} deg (URDF zero = L pose)")
    ax.view_init(elev=20, azim=-60)
    return _save(fig, out, "so101")


def scene_workspace(out: Path) -> Path:
    chain = ak.load_so101()
    rng = np.random.default_rng(0)
    n = 6000
    qs = rng.uniform(chain.lower, chain.upper, size=(n, chain.n_dof))
    side_q = qs.copy()
    side_q[:, 0] = 0.0                       # pan fixed: the vertical plane of the arm
    side = np.array([chain.fk(q)[:3, 3] for q in side_q])
    top = np.array([chain.fk(q)[:3, 3] for q in qs])
    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 5.5))
    a.scatter(side[:, 0], side[:, 2], s=2, color="tab:blue", alpha=0.35)
    a.axhline(0.0, color="black", lw=1)
    a.text(0.30, 0.005, "table (z = 0): points below are unreachable in practice", fontsize=8, va="bottom")
    a.set_title("Side view, shoulder_pan = 0: tool positions inside the URDF joint limits")
    a.set_xlabel("x [m]")
    a.set_ylabel("z [m]")
    b.scatter(top[:, 0], top[:, 1], s=2, color="tab:blue", alpha=0.25)
    b.set_title("Top view, all joints random within limits (pan ±110°)")
    b.set_xlabel("x [m]")
    b.set_ylabel("y [m]")
    for ax in (a, b):
        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3)
    return _save(fig, out, "workspace")


def scene_ellipses(out: Path) -> Path:
    fig, ax = plt.subplots(figsize=(7, 6))
    q1 = math.radians(30)
    for q2_deg, color in ((120, "tab:blue"), (90, "tab:green"), (45, "tab:orange"), (10, "tab:red")):
        q = (q1, math.radians(q2_deg))
        draw_planar_arm(ax, (L1, L2), q, color=color, alpha=0.7)
        J = ak.planar_jacobian((L1, L2), q)
        S, U = ak.velocity_ellipse(J)
        tip = ak.planar_joint_points((L1, L2), q)[-1]
        scale = 0.25                         # ellipse of tip velocity for |qd| = 0.25 rad/s
        angle = math.degrees(math.atan2(U[1, 0], U[0, 0]))
        ax.add_patch(Ellipse(tip, 2 * scale * S[0], 2 * scale * S[1], angle=angle, fill=False, color=color, lw=2,
                             label=f"q2={q2_deg}°  w={ak.manipulability(J) * 1e3:.2f}e-3  σ_min={S[1]:.3f}"))
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("Tip-velocity ellipses for |q̇| = 0.25 rad/s: they flatten as the elbow straightens")
    ax.legend(fontsize=8, loc="upper left")
    return _save(fig, out, "ellipses")


def resolved_rate_run(damping: float, steps: int = 400, dt: float = 0.01) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """2-link tip moves along +x at 5 cm/s through the fully stretched pose. Returns t, qd, tip x."""
    q = np.array([ak.two_link_ik(L1, L2, 0.20, 0.03)[1].q1, ak.two_link_ik(L1, L2, 0.20, 0.03)[1].q2])
    v = np.array([0.05, 0.0])
    ts, qds, xs = [], [], []
    for k in range(steps):
        J = ak.planar_jacobian((L1, L2), q)
        qd = ak.dls_solve(J, v, damping)
        qd = np.clip(qd, -20.0, 20.0)               # a real servo saturates; keep the plot readable
        q = q + qd * dt
        ts.append(k * dt)
        qds.append(qd)
        xs.append(ak.planar_fk((L1, L2), q)[0])
    return np.array(ts), np.array(qds), np.array(xs)


def scene_resolved_rate(out: Path) -> Path:
    fig, (a, b) = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    for damping, color, label in ((0.0, "tab:red", "pseudo-inverse (λ = 0)"), (0.02, "tab:blue", "damped LS (λ = 0.02)")):
        t, qd, x = resolved_rate_run(damping)
        a.plot(t, np.abs(qd).max(axis=1), color=color, lw=2, label=label)
        b.plot(t, x, color=color, lw=2, label=label)
    a.set_ylabel("max |joint speed| [rad/s]")
    a.set_title("Asking the tip for 5 cm/s straight out through the reach limit (0.251 m)")
    a.legend()
    a.grid(True, alpha=0.3)
    b.axhline(L1 + L2, color="tab:gray", ls="--", label="l1 + l2")
    b.set_ylabel("tip x [m]")
    b.set_xlabel("t [s]")
    b.legend()
    b.grid(True, alpha=0.3)
    return _save(fig, out, "resolved-rate")


def scene_profiles(out: Path) -> Path:
    D, vmax, amax = math.radians(90), math.radians(60), math.radians(120)
    trap = tr.Trapezoid.fastest(0.0, D, vmax, amax)
    Tc = tr.min_duration_cubic(D, vmax, amax)
    Tq = tr.min_duration_quintic(D, vmax, amax)
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    curves = [
        (f"trapezoid, T = {trap.duration:.2f} s", "tab:blue", np.linspace(0, trap.duration, 400), trap.sample),
        (f"cubic, T = {Tc:.2f} s", "tab:orange", np.linspace(0, Tc, 400),
         lambda t: tr.polynomial_sample(tr.cubic_coefficients(0.0, D, Tc), t)),
        (f"quintic, T = {Tq:.2f} s", "tab:green", np.linspace(0, Tq, 400),
         lambda t: tr.polynomial_sample(tr.quintic_coefficients(0.0, D, Tq), t)),
    ]
    for label, color, t, fn in curves:
        p, v, a = fn(t)
        for ax, y in zip(axes, (p, v, a)):
            ax.plot(t, np.degrees(y), color=color, lw=2, label=label)
    for ax, lim, name in zip(axes, (None, vmax, amax), ("position [deg]", "velocity [deg/s]", "acceleration [deg/s²]")):
        if lim is not None:
            for s in (1, -1):
                ax.axhline(s * math.degrees(lim), color="tab:gray", ls="--", lw=1)
        ax.set_ylabel(name)
        ax.grid(True, alpha=0.3)
    axes[0].set_title("90° move, limits 60°/s and 120°/s² (dashed): each profile at its shortest legal duration")
    axes[0].legend(fontsize=8)
    axes[-1].set_xlabel("t [s]")
    return _save(fig, out, "profiles")


def scene_joint_vs_cartesian(out: Path) -> Path:
    chain = ak.load_so101()
    q_a = np.radians([0, 20, 30, 40, 0])
    q_b = np.radians([0, -40, 60, 60, 0])
    p_a, p_b = chain.fk(q_a)[:3, 3], chain.fk(q_b)[:3, 3]
    _, qj, _, _ = tr.joint_space_trajectory(q_a, q_b, math.radians(60), math.radians(120), dt=0.02)
    joint_path = np.array([chain.fk(q)[:3, 3] for q in qj])

    def ik(p: np.ndarray, q_prev: np.ndarray) -> np.ndarray:
        res = ak.ik_dls(chain, p, q_prev, target_pitch=ak.approach_pitch(chain.fk(q_prev)), pitch_weight=0.02)
        return res.q

    _, cart_points, qc = tr.straight_line_trajectory(p_a, p_b, v_max=0.10, a_max=0.2, dt=0.02, ik=ik, q_start=q_a)
    cart_path = np.array([chain.fk(q)[:3, 3] for q in qc])
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(joint_path[:, 0], joint_path[:, 2], color="tab:orange", lw=2, label="joint-space move (synchronized trapezoids)")
    ax.plot(cart_path[:, 0], cart_path[:, 2], color="tab:blue", lw=2, label="Cartesian straight line (IK every 20 ms)")
    ax.plot(*p_a[[0, 2]], "o", color="black")
    ax.plot(*p_b[[0, 2]], "s", color="black")
    ax.annotate("start", p_a[[0, 2]], textcoords="offset points", xytext=(6, 6))
    ax.annotate("goal", p_b[[0, 2]], textcoords="offset points", xytext=(6, 6))
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("SO-101 tool path, side view")
    ax.legend(fontsize=8)
    return _save(fig, out, "joint-vs-cartesian")


SCENES: dict[str, tuple[str, Callable[[Path], Path]]] = {
    "two-link": ("2-link arm, both IK branches and reach circles", scene_two_link),
    "so101": ("SO-101 stick figure with joint frames at q = 0", scene_so101),
    "workspace": ("SO-101 reachable tool positions (side and top)", scene_workspace),
    "ellipses": ("manipulability ellipses of the 2-link arm", scene_ellipses),
    "resolved-rate": ("pseudo-inverse vs damped least squares near a singularity", scene_resolved_rate),
    "profiles": ("trapezoid, cubic and quintic profiles vs limits", scene_profiles),
    "joint-vs-cartesian": ("SO-101 tool path: joint-space vs Cartesian line", scene_joint_vs_cartesian),
}


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Draw module-14 arm figures to PNG files.")
    parser.add_argument("scene", choices=["list", "all", *SCENES])
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--q", type=float, nargs=5, metavar="DEG", help="SO-101 joint angles for 'so101'")
    args = parser.parse_args(argv)
    if args.scene == "list":
        for name, (desc, _) in SCENES.items():
            print(f"{name:20s} {desc}")
        return
    names = list(SCENES) if args.scene == "all" else [args.scene]
    for name in names:
        if name == "so101" and args.q:
            path = scene_so101(args.out, args.q)
        else:
            path = SCENES[name][1](args.out)
        print("wrote", path)


if __name__ == "__main__":
    main()
