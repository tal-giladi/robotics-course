"""11.04 — Point-to-point ICP for 2D LiDAR scans (numpy + scipy.spatial.KDTree).

Library:
    best_fit_transform(src, dst) -> SE2      Kabsch/SVD: the rigid motion that best maps src onto dst
    icp(source, target, initial) -> IcpResult  iterate: nearest neighbours -> reject far pairs -> Kabsch
    match_is_reliable(result)                   accept or reject a match before trusting it

Demos (plots go to 11-slam/code/out/):
    python 11-slam/code/icp.py example     the 3-point Kabsch example worked in the lesson
    python 11-slam/code/icp.py pair        align two apartment scans, print every iteration
    python 11-slam/code/icp.py variants    point-to-point vs coarse-to-fine vs point-to-line accuracy
    python 11-slam/code/icp.py basin       success map over initial rotation/translation errors
    python 11-slam/code/icp.py corridor    the failure case: a long featureless corridor
    python 11-slam/code/icp.py odometry    scan-to-scan ICP odometry vs wheel odometry on the tour
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np
from scipy.spatial import KDTree

from slam_common import OUT, SE2, World, angle_diff, headless_pyplot, load_tour, trajectory_array


@dataclass(frozen=True)
class IcpResult:
    transform: SE2  # maps SOURCE points into the TARGET frame: target ≈ transform.apply(source)
    rmse: float  # root-mean-square distance of the inlier pairs at the end, m
    inlier_fraction: float  # share of source points with a partner closer than the gate
    iterations: int
    converged: bool


def best_fit_transform(src: np.ndarray, dst: np.ndarray) -> SE2:
    """Least-squares rigid transform (rotation + translation, no scale) with dst ≈ R @ src + t.

    Arun/Kabsch: remove the centroids, build the 2x2 cross-covariance H = Σ (s - s̄)(d - d̄)ᵀ,
    take its SVD H = U S Vᵀ, then R = V Uᵀ (flip the last column of V if det < 0: a reflection
    fits mirrored point sets better than any rotation, but a robot cannot mirror itself).
    """
    src = np.asarray(src, dtype=float).reshape(-1, 2)
    dst = np.asarray(dst, dtype=float).reshape(-1, 2)
    if len(src) != len(dst) or len(src) < 2:
        raise ValueError("need at least 2 corresponding point pairs")
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, np.sign(np.linalg.det(Vt.T @ U.T)) or 1.0])
    R = Vt.T @ D @ U.T
    t = mu_d - R @ mu_s
    return SE2(t[0], t[1], math.atan2(R[1, 0], R[0, 0]))


def icp(
    source: np.ndarray,
    target: np.ndarray,
    initial: SE2 = SE2(),
    max_iterations: int = 50,
    tolerance: float = 1e-5,
    max_correspondence_distance: float = 0.3,
    tree: KDTree | None = None,
    history: list[SE2] | None = None,
) -> IcpResult:
    """Point-to-point ICP. ``initial`` is the guess (e.g. from wheel odometry) for the transform.

    Each iteration: move the source with the current estimate, pair every moved point with its
    nearest target point (KD-tree), drop pairs farther than ``max_correspondence_distance``
    (outliers: things only one scan sees), solve Kabsch on the rest, compose the small correction
    onto the estimate. Stop when the correction is below ``tolerance`` (m and rad).
    """
    source = np.asarray(source, dtype=float).reshape(-1, 2)
    target = np.asarray(target, dtype=float).reshape(-1, 2)
    tree = tree if tree is not None else KDTree(target)
    T = initial
    converged = False
    iterations = 0
    for iterations in range(1, max_iterations + 1):
        moved = T.apply(source)
        dist, idx = tree.query(moved)
        inliers = dist < max_correspondence_distance
        if inliers.sum() < 3:
            break
        step = best_fit_transform(moved[inliers], target[idx[inliers]])
        T = step @ T
        if history is not None:
            history.append(T)
        if math.hypot(step.x, step.y) < tolerance and abs(step.theta) < tolerance:
            converged = True
            break
    dist, _ = tree.query(T.apply(source))
    inliers = dist < max_correspondence_distance
    rmse = float(np.sqrt(np.mean(dist[inliers] ** 2))) if inliers.any() else math.inf
    return IcpResult(T, rmse, float(inliers.mean()) if len(source) else 0.0, iterations, converged)


def match_is_reliable(result: IcpResult, min_inlier_fraction: float = 0.7, max_rmse: float = 0.05) -> bool:
    """Reject matches that converged to the wrong place. A wrong match that you trust is worse
    than no match at all: in a pose graph (11.05) it bends the whole map."""
    return result.converged and result.inlier_fraction >= min_inlier_fraction and result.rmse <= max_rmse


def icp_coarse_to_fine(
    source: np.ndarray,
    target: np.ndarray,
    initial: SE2 = SE2(),
    gates: tuple[float, ...] = (1.0, 0.5, 0.25, 0.1),
    max_iterations: int = 30,
) -> IcpResult:
    """Run :func:`icp` several times with a shrinking correspondence gate.

    A wide gate lets far-off points pull the estimate into the right valley (big basin); a narrow
    gate at the end ignores the pairs that only exist in one scan (small bias).
    """
    tree = KDTree(np.asarray(target, dtype=float).reshape(-1, 2))
    T, total, res = initial, 0, None
    for gate in gates:
        res = icp(source, target, T, max_iterations, max_correspondence_distance=gate, tree=tree)
        T, total = res.transform, total + res.iterations
    assert res is not None
    return IcpResult(T, res.rmse, res.inlier_fraction, total, res.converged)


def estimate_normals(points: np.ndarray, k: int = 5) -> np.ndarray:
    """Unit normal of the local line through each point's ``k`` nearest neighbours (PCA)."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    _, idx = KDTree(points).query(points, k=min(k, len(points)))
    centered = points[idx] - points[idx].mean(axis=1, keepdims=True)
    cov = np.einsum("nki,nkj->nij", centered, centered)
    _, vecs = np.linalg.eigh(cov)
    return vecs[:, :, 0]  # eigenvector of the smallest eigenvalue = across the line


def icp_point_to_line(
    source: np.ndarray,
    target: np.ndarray,
    initial: SE2 = SE2(),
    max_iterations: int = 50,
    tolerance: float = 1e-6,
    max_correspondence_distance: float = 0.3,
) -> IcpResult:
    """Point-to-line ICP (Censi 2008 idea): minimize the distance to the target's *surface*,
    n · (p - q), not to the target *point*. Sliding along a wall costs nothing, so the different
    sampling of the two scans stops biasing the result. One Gauss-Newton step per iteration with
    the small-angle model p' ≈ p + dθ·(-p_y, p_x) + t."""
    source = np.asarray(source, dtype=float).reshape(-1, 2)
    target = np.asarray(target, dtype=float).reshape(-1, 2)
    tree, normals = KDTree(target), estimate_normals(target)
    T, converged, iterations = initial, False, 0
    for iterations in range(1, max_iterations + 1):
        moved = T.apply(source)
        dist, idx = tree.query(moved)
        ok = dist < max_correspondence_distance
        if ok.sum() < 3:
            break
        p, q, n = moved[ok], target[idx[ok]], normals[idx[ok]]
        J = np.column_stack([n[:, 0], n[:, 1], n[:, 1] * p[:, 0] - n[:, 0] * p[:, 1]])
        r = np.sum(n * (p - q), axis=1)
        delta = np.linalg.lstsq(J, -r, rcond=None)[0]
        T = SE2(*delta) @ T
        if np.linalg.norm(delta) < tolerance:
            converged = True
            break
    dist, idx = tree.query(T.apply(source))
    ok = dist < max_correspondence_distance
    line_dist = np.abs(np.sum(normals[idx[ok]] * (T.apply(source)[ok] - target[idx[ok]]), axis=1))
    rmse = float(np.sqrt(np.mean(line_dist**2))) if ok.any() else math.inf
    return IcpResult(T, rmse, float(ok.mean()), iterations, converged)


def degeneracy_ratio(points: np.ndarray) -> float:
    """How well a scan pins down translation: smallest / largest eigenvalue of Σ n nᵀ over the
    surface normals. ≈0 means some direction has no wall facing it (a corridor), 1 is ideal."""
    n = estimate_normals(points)
    eig = np.linalg.eigvalsh(n.T @ n)
    return float(eig[0] / eig[-1])


# --- demos ------------------------------------------------------------------------------------------
def demo_example() -> None:
    src = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 1.0]])
    true = SE2(1.0, 0.5, math.radians(30))
    dst = true.apply(src)
    print("source points:\n", src)
    print("target points (source rotated 30 deg, moved by (1.0, 0.5)):\n", dst.round(4))
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    print("centroids:", mu_s.round(4), mu_d.round(4))
    H = (src - mu_s).T @ (dst - mu_d)
    print("H =\n", H.round(4))
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    print("singular values:", S.round(4), " det(V U^T) =", round(float(np.linalg.det(R)), 4))
    print("R =\n", R.round(4), f"\n-> theta = {math.degrees(math.atan2(R[1, 0], R[0, 0])):.2f} deg")
    print("t = mu_d - R mu_s =", (mu_d - R @ mu_s).round(4))
    print("best_fit_transform:", best_fit_transform(src, dst))


def demo_pair() -> None:
    world = World.apartment()
    a, b = SE2(1.0, 1.3, 0.0), SE2(1.35, 1.45, math.radians(12))
    angles = -math.pi + np.arange(360) * (2 * math.pi / 360)
    pts = []
    for pose in (a, b):
        r = world.raycast((pose.x, pose.y), angles + pose.theta, 12.0)
        ok = np.isfinite(r)
        pts.append(np.column_stack([r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok])]))
    truth = a.between(b)
    history: list[SE2] = []
    res = icp(pts[1], pts[0], SE2(), history=history)
    print(f"true transform  x={truth.x:.4f} y={truth.y:.4f} theta={math.degrees(truth.theta):.3f} deg")
    for i, T in enumerate(history, 1):
        if i <= 5 or i == len(history):
            print(f"iter {i:2d}        x={T.x:.4f} y={T.y:.4f} theta={math.degrees(T.theta):.3f} deg")
    print(f"converged={res.converged} after {res.iterations} iterations, rmse={res.rmse * 1000:.1f} mm, "
          f"inliers={res.inlier_fraction:.0%}, reliable={match_is_reliable(res)}")
    plt = headless_pyplot()
    fig, axes = plt.subplots(1, 2, figsize=(11, 5))
    for ax, T, title in ((axes[0], SE2(), "before: initial guess = identity"), (axes[1], res.transform, "after ICP")):
        ax.scatter(pts[0][:, 0], pts[0][:, 1], s=4, label="target (scan A)")
        moved = T.apply(pts[1])
        ax.scatter(moved[:, 0], moved[:, 1], s=4, label="source (scan B) moved")
        ax.set_aspect("equal")
        ax.set_title(title)
        ax.legend(loc="upper right")
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / "icp_pair.png", dpi=110, bbox_inches="tight")
    print("wrote", OUT / "icp_pair.png")


def simulated_scan(world: World, pose: SE2, noise_std: float = 0.0, seed: int = 0, samples: int = 360) -> np.ndarray:
    """Valid LiDAR points (sensor frame) seen from ``pose``: the same geometry as ``lidar_scan()``,
    without driving the robot there."""
    angles = -math.pi + np.arange(samples) * (2 * math.pi / samples)
    r = world.raycast((pose.x, pose.y), angles + pose.theta, 12.0)
    if noise_std > 0:
        r = r + np.random.default_rng(seed).normal(0.0, noise_std, samples)
    ok = np.isfinite(r)
    return np.column_stack([r[ok] * np.cos(angles[ok]), r[ok] * np.sin(angles[ok])])


def demo_variants() -> None:
    world = World.apartment()
    a, b = SE2(1.0, 1.3, 0.0), SE2(1.35, 1.45, math.radians(12))
    truth = a.between(b)
    src, dst = simulated_scan(world, b, 0.01, seed=1), simulated_scan(world, a, 0.01, seed=2)
    print(f"true transform x={truth.x:.3f} y={truth.y:.3f} theta={math.degrees(truth.theta):.2f} deg; 1 cm range noise")
    for name, run in (
        ("point-to-point, gate 0.3 m", lambda: icp(src, dst, SE2())),
        ("coarse-to-fine, gates 1.0->0.1", lambda: icp_coarse_to_fine(src, dst, SE2())),
        ("point-to-line, gate 0.3 m", lambda: icp_point_to_line(src, dst, SE2())),
    ):
        res = run()
        T = res.transform
        print(f"{name:31s} error {1000 * math.hypot(T.x - truth.x, T.y - truth.y):5.1f} mm "
              f"{abs(math.degrees(angle_diff(T.theta, truth.theta))):5.2f} deg, {res.iterations:3d} iterations")


def demo_basin() -> None:
    world = World.apartment()
    a, b = SE2(1.0, 1.3, 0.0), SE2(1.2, 1.4, math.radians(5))
    src, dst = simulated_scan(world, b), simulated_scan(world, a)
    truth = a.between(b)
    tree = KDTree(dst)
    rot_errors = np.arange(0, 91, 15)
    trans_errors = np.arange(0.0, 1.01, 0.25)
    trials = 6
    methods = {
        "plain ICP (gate 0.5 m)": lambda g: icp(src, dst, g, tree=tree, max_correspondence_distance=0.5),
        "coarse-to-fine (1.0 -> 0.1 m)": lambda g: icp_coarse_to_fine(src, dst, g),
    }
    plt = headless_pyplot()
    fig, axes = plt.subplots(1, 2, figsize=(12, 4))
    im = None
    for ax, (name, run) in zip(axes, methods.items()):
        rng = np.random.default_rng(0)
        success = np.zeros((len(trans_errors), len(rot_errors)))
        for i, te in enumerate(trans_errors):
            for j, re in enumerate(rot_errors):
                ok = 0
                for _ in range(trials):
                    direction = rng.uniform(-math.pi, math.pi)
                    sign = rng.choice([-1.0, 1.0])
                    guess = SE2(truth.x + te * math.cos(direction), truth.y + te * math.sin(direction),
                                truth.theta + sign * math.radians(re))
                    T = run(guess).transform
                    ok += math.hypot(T.x - truth.x, T.y - truth.y) < 0.05 and abs(angle_diff(T.theta, truth.theta)) < math.radians(2)
                success[i, j] = ok / trials
        print(f"{name}: success rate (rows: initial translation error, columns: rotation error deg)")
        print("        " + " ".join(f"{r:5d}" for r in rot_errors))
        for te, row in zip(trans_errors, success):
            print(f"{te:5.2f} m " + " ".join(f"{v:5.2f}" for v in row))
        im = ax.imshow(success, origin="lower", aspect="auto", cmap="viridis", vmin=0, vmax=1,
                       extent=(-7.5, rot_errors[-1] + 7.5, -0.125, trans_errors[-1] + 0.125))
        ax.set_xlabel("initial rotation error [deg]")
        ax.set_ylabel("initial translation error [m]")
        ax.set_title(name)
    fig.colorbar(im, ax=axes, label="success rate (< 5 cm, < 2 deg)")
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / "icp_basin.png", dpi=110, bbox_inches="tight")
    print("wrote", OUT / "icp_basin.png")


def corridor_case(true_dx: float = 0.5) -> dict[str, tuple[SE2, IcpResult, float]]:
    """The same 0.5 m step forward in a 60 m corridor (1.2 m wide) and in the living room."""
    corridor = World.from_segments([[-30.0, 0.0, 30.0, 0.0], [-30.0, 1.2, 30.0, 1.2]])
    cases = {"corridor": (corridor, SE2(0.0, 0.6, 0.0)), "living room": (World.apartment(), SE2(1.0, 1.3, 0.0))}
    out = {}
    for name, (world, a) in cases.items():
        b = a @ SE2(true_dx, 0.0, 0.0)
        src, dst = simulated_scan(world, b, 0.01, seed=1), simulated_scan(world, a, 0.01, seed=2)
        out[name] = (a.between(b), icp_coarse_to_fine(src, dst, SE2()), degeneracy_ratio(dst))
    return out


def demo_corridor() -> None:
    for name, (truth, res, ratio) in corridor_case().items():
        T = res.transform
        print(f"{name:12s} true dx={truth.x:.3f} m  ICP dx={T.x:.3f} dy={T.y:.3f} theta={math.degrees(T.theta):.2f} deg  "
              f"rmse={res.rmse * 1000:.1f} mm inliers={res.inlier_fraction:.0%} reliable={match_is_reliable(res)} "
              f"degeneracy ratio={ratio:.3f}")


def scan_to_scan_odometry(log, keyframes: list[int]) -> list[SE2]:
    """Chain ICP between consecutive keyframes, seeded with the wheel-odometry motion.
    An unreliable match falls back to the wheel-odometry step."""
    poses = [log.odom[keyframes[0]]]
    for k0, k1 in zip(keyframes[:-1], keyframes[1:]):
        guess = log.odom[k0].between(log.odom[k1])
        res = icp_coarse_to_fine(log.scans[k1].points(), log.scans[k0].points(), guess, gates=(0.5, 0.25, 0.1))
        step = res.transform if match_is_reliable(res) else guess
        poses.append(poses[-1] @ step)
    return poses


def demo_odometry(seed: int = 0) -> None:
    from slam_common import position_rmse

    log = load_tour(realistic=True, seed=seed)
    keys = log.keyframes()
    icp_poses = scan_to_scan_odometry(log, keys)
    truth = [log.truth[k] for k in keys]
    wheel = [log.odom[k] for k in keys]
    for name, est in (("wheel odometry", wheel), ("ICP odometry", icp_poses)):
        end = est[-1]
        print(f"{name:15s} RMSE {position_rmse(est, truth):.3f} m, end error {end.distance_to(truth[-1]):.3f} m, "
              f"heading {math.degrees(abs(angle_diff(end.theta, truth[-1].theta))):.1f} deg")
    headless_pyplot()
    from robotlab.sim import viz

    fig, ax = viz.new_axes(log.world, title=f"Scan-to-scan ICP vs wheel odometry ({len(keys)} keyframes)")
    for poses, style, label in ((truth, "-", "truth"), (wheel, "--", "wheel odometry"), (icp_poses, "-.", "ICP odometry")):
        viz.draw_trajectory(ax, trajectory_array(poses), linestyle=style, label=label)
    ax.legend(loc="lower right")
    OUT.mkdir(exist_ok=True)
    fig.savefig(OUT / "icp_odometry.png", dpi=110, bbox_inches="tight")
    print("wrote", OUT / "icp_odometry.png")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("demo", choices=["example", "pair", "variants", "basin", "corridor", "odometry"])
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    {"example": demo_example, "pair": demo_pair, "variants": demo_variants, "basin": demo_basin,
     "corridor": demo_corridor, "odometry": lambda: demo_odometry(args.seed)}[args.demo]()
