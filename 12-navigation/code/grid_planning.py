"""12.02 — Grid path planning: BFS, Dijkstra and A* on an occupancy grid (reference implementation).

    python 12-navigation/code/grid_planning.py          # tiny ASCII example + planner race in the apartment

Conventions: a grid is a boolean array ``blocked[row, col]`` (row 0 = smallest y, like
``OccupancyGrid.data``); a cell is ``(row, col)``; costs are in *cells* (multiply by the resolution
for meters). Moves are 8-connected: straight steps cost 1, diagonal steps cost sqrt(2), and a
diagonal step may not squeeze between two blocked orthogonal neighbours ("no corner cutting").
"""

from __future__ import annotations

import heapq
import math
import time
from collections import deque
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import distance_transform_edt

import nav_common
from robotlab.sim import OccupancyGrid

Cell = tuple[int, int]
Heuristic = Callable[[Cell, Cell], float]
SQRT2 = math.sqrt(2.0)
MOVES_8 = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
           (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2))


# --- the graph ---------------------------------------------------------------------------------
def neighbors(blocked: NDArray[np.bool_], cell: Cell, connectivity: int = 8) -> Iterator[tuple[Cell, float]]:
    """Free neighbours of ``cell`` and the length of the step to each (1 or sqrt 2)."""
    r, c = cell
    rows, cols = blocked.shape
    for dr, dc, step in MOVES_8 if connectivity == 8 else MOVES_8[:4]:
        nr, nc = r + dr, c + dc
        if not (0 <= nr < rows and 0 <= nc < cols) or blocked[nr, nc]:
            continue
        if dr and dc and (blocked[r, nc] or blocked[nr, c]):
            continue  # no corner cutting
        yield (nr, nc), step


# --- heuristics (all in cells) ---------------------------------------------------------------------
def zero(a: Cell, b: Cell) -> float:
    """h = 0 turns A* into Dijkstra."""
    return 0.0


def octile(a: Cell, b: Cell) -> float:
    """Exact shortest distance on an EMPTY 8-connected grid: admissible and consistent."""
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc)


def euclidean(a: Cell, b: Cell) -> float:
    """Straight-line distance: admissible, but smaller than octile, so A* expands more."""
    return math.hypot(a[0] - b[0], a[1] - b[1])


def manhattan(a: Cell, b: Cell) -> float:
    """|dr| + |dc|: NOT admissible on an 8-connected grid (a diagonal step costs 1.41, not 2)."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


# --- search --------------------------------------------------------------------------------------
@dataclass
class PlanResult:
    path: list[Cell] | None  # start ... goal, or None when no path exists
    cost: float  # path length in cells (inf when no path)
    expanded: int  # cells taken off the open list and finalized
    expansion_order: list[Cell] = field(default_factory=list, repr=False)
    seconds: float = 0.0


def path_cost(path: list[Cell]) -> float:
    """Geometric length of a cell path, in cells."""
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:]))


def _reconstruct(parent: dict[Cell, Cell | None], goal: Cell) -> list[Cell]:
    path, cell = [], goal
    while cell is not None:
        path.append(cell)
        cell = parent[cell]
    return path[::-1]


def bfs(blocked: NDArray[np.bool_], start: Cell, goal: Cell, connectivity: int = 8) -> PlanResult:
    """Breadth-first search: fewest MOVES. Only shortest when every move costs the same."""
    t0 = time.perf_counter()
    if blocked[start] or blocked[goal]:
        return PlanResult(None, math.inf, 0, [], time.perf_counter() - t0)
    parent: dict[Cell, Cell | None] = {start: None}
    queue, order = deque([start]), []
    while queue:
        cell = queue.popleft()
        order.append(cell)
        if cell == goal:
            path = _reconstruct(parent, goal)
            return PlanResult(path, path_cost(path), len(order), order, time.perf_counter() - t0)
        for nxt, _step in neighbors(blocked, cell, connectivity):
            if nxt not in parent:
                parent[nxt] = cell
                queue.append(nxt)
    return PlanResult(None, math.inf, len(order), order, time.perf_counter() - t0)


def astar(
    blocked: NDArray[np.bool_],
    start: Cell,
    goal: Cell,
    heuristic: Heuristic = octile,
    *,
    weight: float = 1.0,
    tie_break: bool = True,
    connectivity: int = 8,
    cell_cost: NDArray[np.floating] | None = None,
) -> PlanResult:
    """A* search. ``heuristic=zero`` is Dijkstra; ``weight > 1`` is weighted A* (faster, not optimal).

    ``tie_break``: among equal f = g + h prefer the smaller h (the node closer to the goal). Without
    it, a wide-open room has thousands of cells with the same f and A* explores all of them.
    ``cell_cost``: optional extra cost per cell (>= 0); a step into cell n costs
    ``step * (1 + cell_cost[n])``. Costmap-aware planning (12.04) uses this.
    """
    t0 = time.perf_counter()
    if blocked[start] or blocked[goal]:
        return PlanResult(None, math.inf, 0, [], time.perf_counter() - t0)
    g: dict[Cell, float] = {start: 0.0}
    parent: dict[Cell, Cell | None] = {start: None}
    closed: set[Cell] = set()
    order: list[Cell] = []
    counter = 0
    h0 = heuristic(start, goal)
    open_heap: list[tuple[float, float, int, Cell]] = [(weight * h0, h0 if tie_break else 0.0, counter, start)]
    while open_heap:
        _f, _h, _n, cell = heapq.heappop(open_heap)
        if cell in closed:
            continue  # stale entry: we already found a cheaper way here ("lazy deletion")
        closed.add(cell)
        order.append(cell)
        if cell == goal:
            path = _reconstruct(parent, goal)
            return PlanResult(path, g[goal], len(order), order, time.perf_counter() - t0)
        for nxt, step in neighbors(blocked, cell, connectivity):
            if nxt in closed:
                continue
            new_g = g[cell] + step * (1.0 + (float(cell_cost[nxt]) if cell_cost is not None else 0.0))
            if new_g < g.get(nxt, math.inf):
                g[nxt] = new_g
                parent[nxt] = cell
                h = heuristic(nxt, goal)
                counter += 1
                heapq.heappush(open_heap, (new_g + weight * h, h if tie_break else 0.0, counter, nxt))
    return PlanResult(None, math.inf, len(order), order, time.perf_counter() - t0)


def dijkstra(blocked: NDArray[np.bool_], start: Cell, goal: Cell, **kwargs: object) -> PlanResult:
    return astar(blocked, start, goal, zero, **kwargs)  # type: ignore[arg-type]


# --- configuration space and post-processing -------------------------------------------------------
def configuration_space(grid: OccupancyGrid, robot_radius_m: float, unknown_is_blocked: bool = True) -> NDArray[np.bool_]:
    """Grow every occupied cell by the robot radius, so the robot can be planned as a point.

    A free cell is blocked when the distance between its center and the nearest occupied cell's
    center is <= ``robot_radius_m`` (the same cell-center rule Nav2's inflation layer uses).
    """
    occupied = grid.data >= 65  # map_server's occupied_thresh 0.65 -> 65 in OccupancyGrid units
    distance_m = distance_transform_edt(~occupied) * grid.resolution
    blocked = distance_m <= robot_radius_m
    if unknown_is_blocked:
        blocked |= grid.data < 0
    return blocked


def line_of_sight(blocked: NDArray[np.bool_], a: Cell, b: Cell) -> bool:
    """True if the straight segment between the two cell centers touches only free cells."""
    n = int(math.ceil(max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 4)) + 1
    t = np.linspace(0.0, 1.0, n + 1)
    r = a[0] + 0.5 + t * (b[0] - a[0])
    c = a[1] + 0.5 + t * (b[1] - a[1])
    for eps_r, eps_c in ((-0.02, -0.02), (-0.02, 0.02), (0.02, -0.02), (0.02, 0.02)):  # grazing a corner counts
        rr = np.clip(np.floor(r + eps_r).astype(int), 0, blocked.shape[0] - 1)
        cc = np.clip(np.floor(c + eps_c).astype(int), 0, blocked.shape[1] - 1)
        if blocked[rr, cc].any():
            return False
    return True


def shortcut(blocked: NDArray[np.bool_], path: list[Cell]) -> list[Cell]:
    """Greedy any-angle smoothing: from each kept cell, jump to the farthest cell still in view."""
    if len(path) < 3:
        return list(path)
    out, i = [path[0]], 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_of_sight(blocked, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out


def cells_to_world(grid: OccupancyGrid, path: list[Cell]) -> NDArray[np.floating]:
    """``(N, 2)`` world coordinates of the cell centers."""
    rows, cols = np.array(path).T
    x, y = grid.cell_to_world(rows, cols)
    return np.column_stack([x, y])


def densify(points: NDArray[np.floating], spacing: float = 0.05) -> NDArray[np.floating]:
    """Resample a polyline so consecutive points are at most ``spacing`` apart (controllers like that)."""
    out = [points[0]]
    for p, q in zip(points[:-1], points[1:]):
        n = max(1, int(math.ceil(np.linalg.norm(q - p) / spacing)))
        out.extend(p + (q - p) * k / n for k in range(1, n + 1))
    return np.array(out)


# --- demos ---------------------------------------------------------------------------------------
TINY_MAP = [
    "..........",
    "..........",
    "....###...",
    "......#...",
    "......#...",
    "..........",
]


def ascii_demo() -> None:
    """The 6x10 example of the lesson: S at row 0, col 2 and G at row 4, col 8 (row 0 printed first)."""
    blocked = np.array([[ch == "#" for ch in row] for row in TINY_MAP])
    start, goal = (0, 2), (4, 8)
    results = {"BFS": bfs(blocked, start, goal), "Dijkstra": dijkstra(blocked, start, goal),
               "A* octile": astar(blocked, start, goal)}
    for name, result in results.items():
        print(f"{name:10s} moves={len(result.path) - 1:2d}  length={result.cost:.3f} cells  expanded={result.expanded}")
    for name in ("BFS", "A* octile"):
        path = set(results[name].path or [])
        print(f"  {name} path:")
        for r, row in enumerate(TINY_MAP):
            print("    " + "".join("S" if (r, c) == start else "G" if (r, c) == goal else "*" if (r, c) in path else ch
                                   for c, ch in enumerate(row)))


def main() -> None:
    from robotlab.config import load_config
    import matplotlib.pyplot as plt

    from robotlab.sim import World, viz

    ascii_demo()
    cfg = load_config()
    world = World.apartment()
    grid = world.to_occupancy_grid(nav_common.GRID_RESOLUTION, margin=0.5)
    blocked = configuration_space(grid, cfg.chassis.footprint_radius_m)
    start = grid.world_to_cell(*nav_common.START_XY)
    goal = grid.world_to_cell(*nav_common.KITCHEN_XY)
    print(f"\ngrid {grid.width}x{grid.height} cells at {grid.resolution} m, robot radius "
          f"{cfg.chassis.footprint_radius_m:.3f} m -> {blocked.mean():.0%} of cells blocked in C-space")
    print(f"start cell {start}, goal cell {goal}")

    runs = {
        "BFS (fewest moves)": bfs(blocked, start, goal),
        "Dijkstra": dijkstra(blocked, start, goal),
        "A* octile": astar(blocked, start, goal, octile),
        "A* octile, no tie-break": astar(blocked, start, goal, octile, tie_break=False),
        "A* euclidean": astar(blocked, start, goal, euclidean),
        "A* manhattan (inadmissible)": astar(blocked, start, goal, manhattan),
        "weighted A* (w=2)": astar(blocked, start, goal, octile, weight=2.0),
    }
    print(f"\n{'planner':30s} {'length [m]':>10s} {'expanded':>9s} {'time [ms]':>9s}")
    for name, res in runs.items():
        print(f"{name:30s} {res.cost * grid.resolution:10.3f} {res.expanded:9d} {res.seconds * 1e3:9.1f}")

    best = runs["A* octile"]
    assert best.path is not None
    smooth = shortcut(blocked, best.path)
    print(f"\nshortcut smoothing: {len(best.path)} cells -> {len(smooth)} waypoints, "
          f"length {path_cost(best.path) * grid.resolution:.3f} m -> {path_cost(smooth) * grid.resolution:.3f} m")

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    panels = [("Dijkstra", "Dijkstra: expanded cells"), ("A* octile", "A* (octile): expanded cells"),
              ("weighted A* (w=2)", "weighted A* (w=2) vs optimal"), (None, "shortcut-smoothed path")]
    for ax, (key, title) in zip(axes.ravel(), panels):
        ax.set_aspect("equal")
        ax.set_title(title)
        viz.draw_occupancy_grid(ax, grid)
        ax.imshow(np.ma.masked_where(~blocked, blocked), origin="lower", extent=grid.extent, cmap="Reds", alpha=0.25,
                  vmin=0, vmax=1)
        if key is not None:
            res = runs[key]
            if key != "weighted A* (w=2)":
                pts = cells_to_world(grid, res.expansion_order)
                ax.scatter(pts[:, 0], pts[:, 1], s=2, c=np.arange(len(pts)), cmap="viridis")
            else:
                opt = cells_to_world(grid, best.path)
                ax.plot(opt[:, 0], opt[:, 1], color="tab:green", lw=2, label=f"optimal {best.cost * grid.resolution:.2f} m")
            p = cells_to_world(grid, res.path)
            ax.plot(p[:, 0], p[:, 1], color="tab:blue", lw=2, label=f"{res.cost * grid.resolution:.2f} m, {res.expanded} expanded")
        else:
            p = cells_to_world(grid, best.path)
            s = cells_to_world(grid, smooth)
            ax.plot(p[:, 0], p[:, 1], color="0.5", lw=1.5, label="A* cells")
            ax.plot(s[:, 0], s[:, 1], "o-", color="tab:purple", lw=2, label=f"shortcut, {path_cost(smooth) * grid.resolution:.2f} m")
        ax.plot(*nav_common.START_XY, "go", ms=9)
        ax.plot(*nav_common.KITCHEN_XY, "r*", ms=14)
        ax.legend(loc="upper right", fontsize=8)
    nav_common.save(fig, "12.02_planners")


if __name__ == "__main__":
    main()
