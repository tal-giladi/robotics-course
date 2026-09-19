"""12.03 — Sampling-based planning: RRT and RRT* in any configuration space (reference implementation).

    python 12-navigation/code/sampling_planning.py            # all demos, PNGs in nav_out/
    python 12-navigation/code/sampling_planning.py --quick    # fewer seeds

The planners only need three things, so the same code plans for a disk robot in the apartment
(configuration q = (x, y)) and for a two-link arm (q = (theta1, theta2)):

* ``bounds``: ``(d, 2)`` array of [low, high] per dimension,
* ``is_free(q) -> bool``: is this configuration collision-free?
* ``motion_free(q_a, q_b) -> bool``: is the straight segment between them collision-free?
"""

from __future__ import annotations

import argparse
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

import nav_common
from robotlab.sim import World

Config = NDArray[np.floating]
IsFree = Callable[[Config], bool]
MotionFree = Callable[[Config, Config], bool]


# --- collision checking for a disk robot: the configuration-space view ----------------------------------
@dataclass
class DiskRobotChecker:
    """A round robot of ``radius`` in a :class:`World`. Its C-space obstacles are the real obstacles
    grown by ``radius``: q = (x, y) is free iff the clearance at (x, y) is at least ``radius``."""

    world: World
    radius: float
    step: float = 0.03  # check motions every 3 cm (well below the ~16 cm radius)
    checks: int = field(default=0, init=False)  # how many clearance queries were made

    def is_free(self, q: Config) -> bool:
        self.checks += 1
        return bool(self.world.distance_to_obstacles(np.asarray(q, dtype=float)[None, :2])[0] >= self.radius)

    def motion_free(self, a: Config, b: Config) -> bool:
        n = max(2, int(math.ceil(float(np.linalg.norm(b - a)) / self.step)) + 1)
        pts = a[None, :] + np.linspace(0.0, 1.0, n)[:, None] * (b - a)[None, :]
        self.checks += n
        return bool(np.all(self.world.distance_to_obstacles(pts) >= self.radius))


# --- RRT and RRT* ---------------------------------------------------------------------------------
@dataclass
class Tree:
    nodes: NDArray[np.floating]  # (capacity, d); only the first `size` rows are used
    parent: NDArray[np.int64]
    cost: NDArray[np.floating]  # cost-to-come from the root
    size: int = 1
    children: list[list[int]] = field(default_factory=lambda: [[]])

    @classmethod
    def with_root(cls, root: Config, capacity: int) -> Tree:
        nodes = np.zeros((capacity, len(root)))
        nodes[0] = root
        return cls(nodes, np.full(capacity, -1, dtype=np.int64), np.zeros(capacity))

    def add(self, q: Config, parent: int, cost: float) -> int:
        i = self.size
        self.nodes[i], self.parent[i], self.cost[i] = q, parent, cost
        self.children.append([])
        self.children[parent].append(i)
        self.size += 1
        return i

    def path_to(self, i: int) -> NDArray[np.floating]:
        idx = []
        while i >= 0:
            idx.append(i)
            i = int(self.parent[i])
        return self.nodes[idx[::-1]].copy()


@dataclass
class RRTResult:
    path: NDArray[np.floating] | None
    tree: Tree
    iterations: int  # samples drawn
    first_solution_iteration: int | None
    cost_history: list[tuple[int, float]]  # (iteration, best path cost) each time it improved
    seconds: float


def steer(q_near: Config, q_rand: Config, step_size: float) -> Config:
    """Move from ``q_near`` toward ``q_rand`` by at most ``step_size``."""
    d = q_rand - q_near
    dist = float(np.linalg.norm(d))
    return q_rand.copy() if dist <= step_size else q_near + d * (step_size / dist)


def rrt(
    start: Config,
    goal: Config,
    bounds: NDArray[np.floating],
    is_free: IsFree,
    motion_free: MotionFree,
    rng: np.random.Generator,
    *,
    step_size: float = 0.3,
    goal_bias: float = 0.05,
    goal_tolerance: float = 0.15,
    max_iterations: int = 5000,
    star: bool = False,
    rewire_radius: float = 0.6,
    keep_improving: bool = False,
) -> RRTResult:
    """RRT (``star=False``) or RRT* (``star=True``: choose the cheapest parent nearby and rewire).

    ``keep_improving``: keep sampling after the first solution until ``max_iterations`` (the point of
    RRT*: the best cost keeps dropping). Plain RRT returns at the first solution.
    """
    t0 = time.perf_counter()
    start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
    if not is_free(start) or not is_free(goal):
        return RRTResult(None, Tree.with_root(start, 2), 0, None, [], time.perf_counter() - t0)
    tree = Tree.with_root(start, max_iterations + 2)
    low, high = bounds[:, 0], bounds[:, 1]
    goal_nodes: list[int] = []
    best_cost, first, history = math.inf, None, []
    for it in range(1, max_iterations + 1):
        q_rand = goal.copy() if rng.random() < goal_bias else rng.uniform(low, high)
        nodes = tree.nodes[: tree.size]
        d2 = np.sum((nodes - q_rand) ** 2, axis=1)
        nearest = int(np.argmin(d2))
        q_new = steer(nodes[nearest], q_rand, step_size)
        if not is_free(q_new) or not motion_free(nodes[nearest], q_new):
            continue
        parent, cost = nearest, float(tree.cost[nearest] + np.linalg.norm(q_new - nodes[nearest]))
        near: NDArray[np.int64] = np.zeros(0, dtype=np.int64)
        if star:
            dist_new = np.linalg.norm(nodes - q_new, axis=1)
            near = np.flatnonzero(dist_new <= rewire_radius)
            for j in near[np.argsort(tree.cost[near] + dist_new[near])]:  # cheapest candidates first
                c = float(tree.cost[j] + dist_new[j])
                if c >= cost:
                    break
                if motion_free(nodes[j], q_new):
                    parent, cost = int(j), c
                    break
        new = tree.add(q_new, parent, cost)
        if star:  # rewire: would going through q_new be cheaper for the neighbours?
            for j in near:
                c = cost + float(np.linalg.norm(tree.nodes[j] - q_new))
                if c + 1e-9 < tree.cost[j] and motion_free(q_new, tree.nodes[j]):
                    _reparent(tree, int(j), new, c)
        if np.linalg.norm(q_new - goal) <= goal_tolerance:
            goal_nodes.append(new)
            first = it if first is None else first
        if goal_nodes:
            best = min(goal_nodes, key=lambda i: tree.cost[i])
            if tree.cost[best] + 1e-9 < best_cost:
                best_cost = float(tree.cost[best])
                history.append((it, best_cost))
            if not keep_improving:
                break
    path = None
    if goal_nodes:
        best = min(goal_nodes, key=lambda i: tree.cost[i])
        path = tree.path_to(best)
    return RRTResult(path, tree, it, first, history, time.perf_counter() - t0)


def _reparent(tree: Tree, j: int, new_parent: int, new_cost: float) -> None:
    """Give node j a cheaper parent and push the saving down to all its descendants."""
    delta = tree.cost[j] - new_cost
    tree.children[int(tree.parent[j])].remove(j)
    tree.children[new_parent].append(j)
    tree.parent[j] = new_parent
    stack = [j]
    while stack:
        k = stack.pop()
        tree.cost[k] -= delta
        stack.extend(tree.children[k])


def path_length(path: NDArray[np.floating]) -> float:
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def random_shortcut(path: NDArray[np.floating], motion_free: MotionFree, rng: np.random.Generator, attempts: int = 100) -> NDArray[np.floating]:
    """Classic post-processing for sampling planners: try to replace a random stretch by a straight line."""
    pts = [p for p in path]
    for _ in range(attempts):
        if len(pts) < 3:
            break
        i, j = sorted(rng.choice(len(pts), 2, replace=False))
        if j - i < 2:
            continue
        if motion_free(pts[i], pts[j]):
            pts = pts[: i + 1] + pts[j:]
    return np.array(pts)


# --- a two-link planar arm: where C-space stops looking like the workspace -----------------------------
@dataclass
class TwoLinkArm:
    """Planar arm at the origin, links l1 and l2, joint angles (theta1, theta2); obstacles are circles."""

    l1: float = 0.30
    l2: float = 0.25
    obstacles: tuple[tuple[float, float, float], ...] = ((0.30, 0.25, 0.08), (-0.20, 0.35, 0.10), (0.05, -0.38, 0.08))

    def points(self, q: Config, per_link: int = 12) -> NDArray[np.floating]:
        t1, t2 = float(q[0]), float(q[1])
        elbow = np.array([self.l1 * math.cos(t1), self.l1 * math.sin(t1)])
        tip = elbow + np.array([self.l2 * math.cos(t1 + t2), self.l2 * math.sin(t1 + t2)])
        s = np.linspace(0.0, 1.0, per_link)[:, None]
        return np.vstack([s * elbow, elbow + s * (tip - elbow)])

    def is_free(self, q: Config) -> bool:
        pts = self.points(q)
        for cx, cy, r in self.obstacles:
            if np.any(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) <= r):
                return False
        return True

    def motion_free(self, a: Config, b: Config) -> bool:
        n = max(2, int(math.ceil(float(np.max(np.abs(b - a))) / math.radians(2.0))) + 1)
        return all(self.is_free(a + (b - a) * t) for t in np.linspace(0.0, 1.0, n))


def grid_cells_needed(dof: int, step_deg: float) -> float:
    """How many cells a grid over ``dof`` joints (each -180..180 deg) needs at ``step_deg`` resolution."""
    return (360.0 / step_deg) ** dof


# --- demos -----------------------------------------------------------------------------------------
def narrow_passage_world(gap: float) -> World:
    """A 4 x 3 m room split by a 1 m thick wall (x 1.5..2.5) with one corridor of width ``gap`` at y = 1.5."""
    lo, hi = 1.5 - gap / 2, 1.5 + gap / 2
    walls = [[0, 0, 4, 0], [4, 0, 4, 3], [4, 3, 0, 3], [0, 3, 0, 0],
             [1.5, 0, 1.5, lo], [2.5, 0, 2.5, lo], [1.5, lo, 2.5, lo],  # a 1 m thick wall with a corridor
             [1.5, 3, 1.5, hi], [2.5, 3, 2.5, hi], [1.5, hi, 2.5, hi]]
    return World.from_segments(walls)


def main() -> None:
    import matplotlib.pyplot as plt

    from robotlab.config import load_config
    from robotlab.sim import viz

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--quick", action="store_true", help="fewer seeds (faster)")
    args = ap.parse_args()
    seeds = 5 if args.quick else 20

    cfg = load_config()
    radius = cfg.chassis.footprint_radius_m
    world = World.apartment()
    checker = DiskRobotChecker(world, radius)
    bounds = np.array([[0.0, 6.0], [0.0, 5.0]])
    start, goal = np.array(nav_common.START_XY), np.array(nav_common.KITCHEN_XY)

    # 1. RRT: random, fast, ugly, different every time.
    lengths, iters, times, shortcut_lengths = [], [], [], []
    runs = []
    for seed in range(seeds):
        res = rrt(start, goal, bounds, checker.is_free, checker.motion_free, np.random.default_rng(seed), max_iterations=5000)
        runs.append(res)
        if res.path is not None:
            lengths.append(path_length(res.path))
            iters.append(res.iterations)
            times.append(res.seconds)
            short = random_shortcut(res.path, checker.motion_free, np.random.default_rng(seed))
            shortcut_lengths.append(path_length(short))
    print(f"RRT, living room -> kitchen, {seeds} seeds: success {len(lengths)}/{seeds}, "
          f"length {np.mean(lengths):.2f} m (min {np.min(lengths):.2f}, max {np.max(lengths):.2f}), "
          f"{np.mean(iters):.0f} samples, {np.mean(times) * 1e3:.0f} ms")
    print(f"  after random shortcutting: {np.mean(shortcut_lengths):.2f} m on average (A* on the 5 cm grid: 4.39 m)")

    # 2. RRT*: keeps improving with more samples.
    star = rrt(start, goal, bounds, checker.is_free, checker.motion_free, np.random.default_rng(0),
               star=True, keep_improving=True, max_iterations=3000, rewire_radius=0.6)
    print(f"RRT* seed 0: first solution at sample {star.first_solution_iteration} "
          f"({star.cost_history[0][1]:.2f} m), after 3000 samples {star.cost_history[-1][1]:.2f} m, {star.seconds:.1f} s")
    for budget in (500, 1000, 2000, 3000):
        best = [c for i, c in star.cost_history if i <= budget]
        print(f"  best cost within {budget:4d} samples: {best[-1]:.2f} m" if best else f"  {budget}: no solution yet")

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    for ax, res, title in ((axes[0], runs[0], "RRT (seed 0), stops at first solution"),
                           (axes[1], star, "RRT* (seed 0), 3000 samples")):
        ax.set_aspect("equal")
        ax.set_title(title)
        viz.draw_world(ax, world, show_landmarks=False)
        t = res.tree
        for i in range(1, t.size):
            p = t.nodes[t.parent[i]]
            ax.plot([p[0], t.nodes[i][0]], [p[1], t.nodes[i][1]], color="0.7", lw=0.5)
        if res.path is not None:
            ax.plot(res.path[:, 0], res.path[:, 1], color="tab:red", lw=2.5, label=f"path {path_length(res.path):.2f} m")
        ax.plot(*start, "go", ms=9)
        ax.plot(*goal, "r*", ms=14)
        ax.legend(loc="upper right")
    nav_common.save(fig, "12.03_rrt_vs_rrtstar")

    # 3. Configuration space of the disk robot: the obstacles grown by the radius.
    fig, ax = viz.new_axes(world, title=f"C-space of a disk robot (r = {radius:.2f} m): shaded = centre may not go here")
    xs, ys = np.meshgrid(np.arange(-0.2, 6.2, 0.025), np.arange(-0.2, 5.2, 0.025))
    clearance = world.distance_to_obstacles(np.column_stack([xs.ravel(), ys.ravel()])).reshape(xs.shape)
    ax.contourf(xs, ys, clearance < radius, levels=[0.5, 1.5], colors=["tab:red"], alpha=0.3)
    viz.draw_robot(ax, (2.05, 3.2, math.pi / 2), radius=radius, color="tab:green")
    nav_common.save(fig, "12.03_cspace_disk")

    # 4. Probabilistic completeness: the narrower the passage, the more samples it takes.
    print("\nNarrow passage (4 x 3 m room, one corridor through a 1 m thick wall), samples until the first solution:")
    for gap in (1.0, 0.5, 0.4):
        w = narrow_passage_world(gap)
        ch = DiskRobotChecker(w, radius)
        needed = []
        for seed in range(seeds):
            res = rrt(np.array([0.5, 0.5]), np.array([3.5, 2.5]), np.array([[0, 4], [0, 3]]), ch.is_free, ch.motion_free,
                      np.random.default_rng(seed), max_iterations=4000)
            needed.append(res.first_solution_iteration or math.inf)
        needed_arr = np.array(needed)
        ok = needed_arr[np.isfinite(needed_arr)]
        print(f"  corridor {gap:.2f} m (free width for the centre {gap - 2 * radius:.2f} m): "
              f"solved {len(ok)}/{seeds} within 4000, median {np.median(ok) if len(ok) else math.inf:.0f} samples; "
              + ", ".join(f"P(solved by {b}) = {np.mean(needed_arr <= b):.2f}" for b in (250, 1000, 4000)))

    # 5. A two-link arm: C-space obstacles are weird shapes; RRT does not care.
    arm = TwoLinkArm()
    angles = np.radians(np.arange(-180, 180, 2.0))
    free = np.array([[arm.is_free(np.array([t1, t2])) for t1 in angles] for t2 in angles])
    q_start, q_goal = np.radians([-100.0, 20.0]), np.radians([150.0, -20.0])
    res = rrt(q_start, q_goal, np.radians([[-180.0, 180.0], [-180.0, 180.0]]), arm.is_free, arm.motion_free,
              np.random.default_rng(1), step_size=math.radians(15), goal_tolerance=math.radians(8), max_iterations=5000)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))
    ax = axes[0]
    ax.set_aspect("equal")
    ax.set_title("workspace: arm at start (green) and goal (red)")
    for cx, cy, r in arm.obstacles:
        ax.add_patch(plt.Circle((cx, cy), r, color="0.5"))
    for q, color in ((q_start, "tab:green"), (q_goal, "tab:red")):
        pts = arm.points(q, per_link=2)
        ax.plot(pts[[0, 1, 3], 0], pts[[0, 1, 3], 1], "o-", color=color, lw=3)
    if res.path is not None:
        for q in res.path[:: max(1, len(res.path) // 8)]:
            pts = arm.points(q, per_link=2)
            ax.plot(pts[[0, 1, 3], 0], pts[[0, 1, 3], 1], "-", color="tab:blue", lw=1, alpha=0.5)
    ax.set_xlim(-0.6, 0.6)
    ax.set_ylim(-0.6, 0.6)
    ax = axes[1]
    ax.set_title("C-space (theta1, theta2): black = collision")
    ax.imshow(~free, origin="lower", extent=[-180, 180, -180, 180], cmap="gray_r", alpha=0.8)
    if res.path is not None:
        p = np.degrees(res.path)
        ax.plot(p[:, 0], p[:, 1], "-", color="tab:blue", lw=2)
    ax.plot(*np.degrees(q_start), "go", ms=9)
    ax.plot(*np.degrees(q_goal), "r*", ms=14)
    ax.set_xlabel("theta1 [deg]")
    ax.set_ylabel("theta2 [deg]")
    nav_common.save(fig, "12.03_two_link_arm_cspace")
    print(f"\nTwo-link arm: {np.mean(~free):.0%} of joint space in collision; RRT found a path with "
          f"{len(res.path) if res.path is not None else 0} nodes after {res.iterations} samples")

    print("\nGrid size to cover every joint from -180 to 180 deg:")
    for dof in (2, 3, 6, 7):
        print(f"  {dof} joints: at 5 deg {grid_cells_needed(dof, 5):.2e} cells, at 1 deg {grid_cells_needed(dof, 1):.2e} cells")


if __name__ == "__main__":
    main()
