"""11.05 — A 2D pose-graph SLAM back end (Gauss-Newton on SE(2)) plus a loop-closure front end.

Library:
    Edge(i, j, measurement, information, kind)    "node j seen from node i is measurement, this sure"
    edge_error(xi, xj, z) / edge_jacobians(...)   the residual of one edge and its derivatives
    optimize(poses, edges)                        Gauss-Newton, first node held fixed
    optimize_with_scipy(poses, edges)             the same problem through scipy.optimize.least_squares
    build_graph(log, ...)                         keyframes -> odometry edges (ICP) -> loop closures (ICP)

Demo (plots go to 11-slam/code/out/):
    python 11-slam/code/pose_graph.py                        ICP edges + loop closures (full pipeline)
    python 11-slam/code/pose_graph.py --odometry wheel       wheel-odometry edges instead of ICP
    python 11-slam/code/pose_graph.py --no-loops             the same graph without loop closures
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np

from icp import icp_coarse_to_fine, match_is_reliable
from slam_common import OUT, SE2, TourLog, angle_diff, headless_pyplot, load_tour, position_rmse, trajectory_array


@dataclass(frozen=True)
class Edge:
    i: int
    j: int
    measurement: SE2  # pose of node j expressed in the frame of node i
    information: np.ndarray  # 3x3 inverse covariance of the measurement (x, y, theta)
    kind: str = "odometry"  # or "loop"


def information_from_sigmas(sigma_xy: float, sigma_theta: float) -> np.ndarray:
    """Ω = diag(1/σ²): a 2 cm / 1° measurement pulls far harder than a 20 cm / 10° one."""
    return np.diag([1.0 / sigma_xy**2, 1.0 / sigma_xy**2, 1.0 / sigma_theta**2])


def rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def edge_error(xi: np.ndarray, xj: np.ndarray, z: SE2) -> np.ndarray:
    """e = t2v(Z⁻¹ · (Xi⁻¹ · Xj)): how far the *predicted* relative pose is from the *measured* one,
    expressed in the measurement's frame. Zero when the two poses agree with the edge."""
    Ri, Rz = rot(xi[2]), rot(z.theta)
    t = Rz.T @ (Ri.T @ (xj[:2] - xi[:2]) - np.array([z.x, z.y]))
    return np.array([t[0], t[1], angle_diff(xj[2] - xi[2], z.theta)])


def edge_jacobians(xi: np.ndarray, xj: np.ndarray, z: SE2) -> tuple[np.ndarray, np.ndarray]:
    """A = ∂e/∂xi and B = ∂e/∂xj (3x3 each), in closed form (Grisetti et al. 2010, eq. 30-31)."""
    Ri, Rz = rot(xi[2]), rot(z.theta)
    c, s = math.cos(xi[2]), math.sin(xi[2])
    dRi_T = np.array([[-s, c], [-c, -s]])  # d(Riᵀ)/dθi
    A = np.zeros((3, 3))
    A[:2, :2] = -Rz.T @ Ri.T
    A[:2, 2] = Rz.T @ dRi_T @ (xj[:2] - xi[:2])
    A[2, 2] = -1.0
    B = np.zeros((3, 3))
    B[:2, :2] = Rz.T @ Ri.T
    B[2, 2] = 1.0
    return A, B


def total_error(poses: np.ndarray, edges: list[Edge]) -> float:
    """χ² = Σ eᵀ Ω e: the quantity Gauss-Newton minimizes."""
    return float(sum(e @ edge.information @ e for edge in edges for e in [edge_error(poses[edge.i], poses[edge.j], edge.measurement)]))


def optimize(
    poses: np.ndarray, edges: list[Edge], iterations: int = 20, tolerance: float = 1e-6, verbose: bool = False
) -> tuple[np.ndarray, list[float], np.ndarray]:
    """Gauss-Newton: linearize every edge, accumulate H Δx = -b, solve, update, repeat.

    Node 0 is anchored (a huge prior), because moving the whole map rigidly costs nothing: without
    an anchor H is singular. Returns ``(poses, chi2 per iteration, last H)``.
    """
    x = np.array(poses, dtype=float).reshape(-1, 3).copy()
    n = len(x)
    history = [total_error(x, edges)]
    H = np.zeros((3 * n, 3 * n))
    for it in range(iterations):
        H = np.zeros((3 * n, 3 * n))
        b = np.zeros(3 * n)
        for edge in edges:
            i, j, omega = edge.i, edge.j, edge.information
            e = edge_error(x[i], x[j], edge.measurement)
            A, B = edge_jacobians(x[i], x[j], edge.measurement)
            si, sj = slice(3 * i, 3 * i + 3), slice(3 * j, 3 * j + 3)
            H[si, si] += A.T @ omega @ A
            H[si, sj] += A.T @ omega @ B
            H[sj, si] += B.T @ omega @ A
            H[sj, sj] += B.T @ omega @ B
            b[si] += A.T @ omega @ e
            b[sj] += B.T @ omega @ e
        H[:3, :3] += np.eye(3) * 1e9  # anchor node 0
        dx = np.linalg.solve(H, -b)  # real systems: sparse Cholesky (H is block-sparse)
        x += dx.reshape(-1, 3)
        x[:, 2] = np.arctan2(np.sin(x[:, 2]), np.cos(x[:, 2]))
        history.append(total_error(x, edges))
        if verbose:
            print(f"  iteration {it + 1}: chi2 = {history[-1]:.4g}, |dx| = {np.linalg.norm(dx):.2e}")
        if np.linalg.norm(dx) < tolerance:
            break
    return x, history, H


def optimize_with_scipy(poses: np.ndarray, edges: list[Edge]) -> np.ndarray:
    """The same least-squares problem through ``scipy.optimize.least_squares`` (numeric Jacobian).
    Residual per edge: L e, with Ω = Lᵀ L, so Σ |L e|² = Σ eᵀ Ω e. Node 0 is kept fixed."""
    from scipy.optimize import least_squares

    x0 = np.array(poses, dtype=float).reshape(-1, 3)
    roots = [np.linalg.cholesky(edge.information).T for edge in edges]

    def residuals(free: np.ndarray) -> np.ndarray:
        x = np.vstack([x0[:1], free.reshape(-1, 3)])
        return np.concatenate([L @ edge_error(x[e.i], x[e.j], e.measurement) for L, e in zip(roots, edges)])

    result = least_squares(residuals, x0[1:].ravel(), method="trf", x_scale="jac")
    x = np.vstack([x0[:1], result.x.reshape(-1, 3)])
    x[:, 2] = np.arctan2(np.sin(x[:, 2]), np.cos(x[:, 2]))
    return x


# --- front end: keyframes, odometry edges, loop closures ------------------------------------------------
@dataclass
class Graph:
    keyframes: list[int]  # indices into the tour log
    initial: np.ndarray  # (N, 3) initial guess: the chained odometry
    edges: list[Edge]


def build_graph(
    log: TourLog,
    odometry: str = "icp",
    loops: bool = True,
    min_separation: int = 10,
    search_radius: float = 1.5,
) -> Graph:
    """Turn a tour into a pose graph, the way slam_toolbox does it (in miniature).

    1. Keyframes: a new node after 0.3 m or 20° of motion.
    2. Odometry edges between consecutive nodes: ICP seeded with wheel odometry (``"icp"``), or
       raw wheel odometry (``"wheel"``).
    3. Loop closures: for every node, look for an *old* node (at least ``min_separation`` nodes
       back) within ``search_radius`` of the current estimate; verify with ICP; keep the edge only
       if the match is reliable.
    """
    keys = log.keyframes()
    points = [log.scans[k].points() for k in keys]
    edges: list[Edge] = []
    initial = [log.odom[keys[0]]]
    for n in range(1, len(keys)):
        guess = log.odom[keys[n - 1]].between(log.odom[keys[n]])
        z, info = guess, information_from_sigmas(0.05, math.radians(3))
        if odometry == "icp":
            res = icp_coarse_to_fine(points[n], points[n - 1], guess, gates=(0.5, 0.25, 0.1))
            if match_is_reliable(res):
                z, info = res.transform, information_from_sigmas(0.02, math.radians(1))
        edges.append(Edge(n - 1, n, z, info))
        initial.append(initial[-1] @ z)
    if loops:
        for j in range(len(keys)):
            candidates = [i for i in range(j - min_separation + 1) if initial[i].distance_to(initial[j]) < search_radius]
            if not candidates:
                continue
            i = min(candidates, key=lambda c: initial[c].distance_to(initial[j]))
            res = icp_coarse_to_fine(points[j], points[i], initial[i].between(initial[j]))
            if match_is_reliable(res, min_inlier_fraction=0.8, max_rmse=0.03):
                edges.append(Edge(i, j, res.transform, information_from_sigmas(0.02, math.radians(1)), "loop"))
    return Graph(keys, trajectory_array(initial), edges)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--odometry", choices=["icp", "wheel"], default="icp")
    parser.add_argument("--no-loops", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    log = load_tour(realistic=True, seed=args.seed)
    graph = build_graph(log, args.odometry, loops=not args.no_loops)
    truth = trajectory_array([log.truth[k] for k in graph.keyframes])
    loops = [e for e in graph.edges if e.kind == "loop"]
    print(f"{len(graph.keyframes)} nodes, {len(graph.edges) - len(loops)} odometry edges ({args.odometry}), {len(loops)} loop closures")
    for e in loops:
        true_rel = SE2(*truth[e.i]).between(SE2(*truth[e.j]))
        print(f"  loop {e.i:2d} <- {e.j:2d}: measured ({e.measurement.x:+.3f}, {e.measurement.y:+.3f}, "
              f"{math.degrees(e.measurement.theta):+.1f} deg), true ({true_rel.x:+.3f}, {true_rel.y:+.3f}, "
              f"{math.degrees(true_rel.theta):+.1f} deg)")

    optimized, history, H = optimize(graph.initial, graph.edges, verbose=True)
    via_scipy = optimize_with_scipy(graph.initial, graph.edges)
    for name, est in (("before (odometry chain)", graph.initial), ("after Gauss-Newton", optimized), ("after scipy least_squares", via_scipy)):
        end_err = math.hypot(*(est[-1, :2] - truth[-1, :2]))
        head = math.degrees(abs(angle_diff(est[-1, 2], truth[-1, 2])))
        print(f"{name:26s} RMSE {position_rmse(est, truth):.3f} m, end error {end_err:.3f} m / {head:.1f} deg")
    print(f"chi2 {history[0]:.1f} -> {history[-1]:.1f}; Gauss-Newton vs scipy max difference "
          f"{np.abs(optimized[:, :2] - via_scipy[:, :2]).max() * 1000:.2f} mm")

    from occupancy_grid_mapping import build_map

    plt = headless_pyplot()
    from robotlab.sim import viz

    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    for ax, est, title in ((axes[0], graph.initial, "before optimization"), (axes[1], optimized, "after optimization")):
        poses = [SE2(*p) for p in est]
        grid, _ = build_map(poses, [log.scans[k] for k in graph.keyframes], log.world, log.lidar_offset)
        viz.draw_occupancy_grid(ax, grid.to_occupancy_grid())
        viz.draw_trajectory(ax, truth, color="tab:green", linewidth=1, label="truth")
        viz.draw_trajectory(ax, est, color="tab:blue", marker=".", linewidth=1, label="estimate")
        for e in loops:
            ax.plot(est[[e.i, e.j], 0], est[[e.i, e.j], 1], color="tab:red", linewidth=2)
        ax.set_title(f"{title}: RMSE {position_rmse(est, truth):.3f} m")
        ax.set_aspect("equal")
        ax.legend(loc="upper left")
    axes[2].spy(np.abs(H) > 1e-9, markersize=1)
    axes[2].set_title(f"non-zeros of H ({len(graph.keyframes)} nodes -> {H.shape[0]}x{H.shape[1]})")
    OUT.mkdir(exist_ok=True)
    name = f"pose_graph_{args.odometry}{'_noloops' if args.no_loops else ''}.png"
    fig.savefig(OUT / name, dpi=110, bbox_inches="tight")
    print("wrote", OUT / name)


if __name__ == "__main__":
    main()
