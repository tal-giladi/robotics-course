"""Checker for 12.02 — A* on an occupancy grid.

Run: ``python course.py check 12.02`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import heapq
import math

import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from robotlab.config import load_config
from robotlab.sim import World

SQRT2 = math.sqrt(2.0)
approx = pytest.approx

TINY_MAP = [  # row 0 printed first
    "..........",
    "..........",
    "....###...",
    "......#...",
    "......#...",
    "..........",
]


def parse(rows: list[str]) -> np.ndarray:
    return np.array([[ch == "#" for ch in row] for row in rows])


def assert_valid_path(blocked: np.ndarray, path: list[tuple[int, int]], start, goal, cost: float) -> None:
    """Every step is one 8-connected move into a free cell, without cutting a blocked corner."""
    assert path[0] == tuple(start) and path[-1] == tuple(goal), "the path must start at start and end at goal"
    length = 0.0
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        dr, dc = r1 - r0, c1 - c0
        assert max(abs(dr), abs(dc)) == 1, f"step {(r0, c0)} -> {(r1, c1)} is not a single 8-connected move"
        assert not blocked[r1, c1], f"the path enters blocked cell {(r1, c1)}"
        if dr and dc:
            assert not blocked[r0, c1] and not blocked[r1, c0], f"diagonal step {(r0, c0)} -> {(r1, c1)} cuts a corner"
        length += SQRT2 if dr and dc else 1.0
    assert cost == approx(length), "PlanResult.cost must equal the length of the returned path"


# --- octile_distance ---------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("a", "b", "expected"),
    [
        ((0, 0), (0, 0), 0.0),
        ((0, 0), (0, 7), 7.0),
        ((0, 0), (3, 5), 3 * SQRT2 + 2),  # 3 diagonal + 2 straight = 6.2426
        ((10, 2), (4, 8), 6 * SQRT2),  # pure diagonal
        ((0, 2), (4, 8), 4 * SQRT2 + 2),  # the lesson's tiny map: 7.657
    ],
)
def test_octile_distance(impl, a, b, expected):
    assert impl.octile_distance(a, b) == approx(expected)
    assert impl.octile_distance(b, a) == approx(expected), "distance must be symmetric"


# --- neighbors ---------------------------------------------------------------------------------
def as_dict(result):
    return {tuple(cell): step for cell, step in result}


def test_neighbors_in_the_open(impl):
    got = as_dict(impl.neighbors(np.zeros((3, 3), bool), (1, 1)))
    assert len(got) == 8
    assert sorted(got.values()) == approx([1.0] * 4 + [SQRT2] * 4)
    assert got[(0, 0)] == approx(SQRT2) and got[(0, 1)] == approx(1.0)


def test_neighbors_stay_inside_the_grid(impl):
    assert as_dict(impl.neighbors(np.zeros((3, 3), bool), (0, 0))) == approx({(0, 1): 1.0, (1, 0): 1.0, (1, 1): SQRT2})


def test_neighbors_no_corner_cutting(impl):
    blocked = parse([".#.", "...", "..."])
    # from (0, 0): (0, 1) is blocked, and the diagonal to (1, 1) would squeeze past it
    assert as_dict(impl.neighbors(blocked, (0, 0))) == approx({(1, 0): 1.0})
    # from (1, 0): the diagonal to (2, 1) passes (1, 1) and (2, 0), both free -> allowed; (0, 1) is blocked
    assert as_dict(impl.neighbors(blocked, (1, 0))) == approx({(0, 0): 1.0, (2, 0): 1.0, (1, 1): 1.0, (2, 1): SQRT2})


# --- astar / dijkstra on hand-made grids ---------------------------------------------------------
def test_tiny_map_optimal_cost(impl):
    blocked = parse(TINY_MAP)
    a = impl.astar(blocked, (0, 2), (4, 8))
    d = impl.dijkstra(blocked, (0, 2), (4, 8))
    optimal = 2 * SQRT2 + 6  # over the top of the wall: 8.828 cells
    assert a.cost == approx(optimal) and d.cost == approx(optimal)
    assert_valid_path(blocked, a.path, (0, 2), (4, 8), a.cost)
    assert_valid_path(blocked, d.path, (0, 2), (4, 8), d.cost)
    assert a.expanded < d.expanded, "the heuristic should let A* close fewer cells than Dijkstra"


def test_start_equals_goal(impl):
    res = impl.astar(np.zeros((4, 4), bool), (2, 2), (2, 2))
    assert res.path == [(2, 2)] and res.cost == 0.0


def test_blocked_start_or_goal(impl):
    blocked = parse(["#...", "....", "...#"])
    for start, goal in (((0, 0), (1, 1)), ((1, 1), (2, 3))):
        res = impl.astar(blocked, start, goal)
        assert res.path is None and res.cost == math.inf


def test_no_path_when_goal_is_walled_in(impl):
    blocked = parse([
        "........",
        "...###..",
        "...#.#..",
        "...###..",
        "........",
    ])
    for fn in (impl.astar, impl.dijkstra):
        res = fn(blocked, (0, 0), (2, 4))
        assert res.path is None, "the goal is enclosed: there is no path"
        assert res.cost == math.inf
        assert res.expanded == int((~blocked).sum()) - 1, "with no path, every reachable cell gets closed"


def test_no_squeezing_through_a_diagonal_gap(impl):
    blocked = parse([".#", "#."])
    assert impl.astar(blocked, (0, 0), (1, 1)).path is None, "diagonal corner cutting must not be allowed"


def test_tie_breaking_on_an_open_floor(impl):
    blocked = np.zeros((40, 60), bool)
    res = impl.astar(blocked, (0, 0), (20, 59))
    assert res.cost == approx(20 * SQRT2 + 39)
    assert res.expanded <= 2 * len(res.path), (
        f"A* closed {res.expanded} cells for a {len(res.path)}-cell path on an empty grid: "
        "break ties on f toward the smaller h"
    )


def test_inadmissible_heuristic_still_returns_a_valid_path(impl):
    blocked = parse(["#.........", "#.##......", "..#..#....", ".##.....#.", "..#.......", "......#..."])
    optimal = impl.dijkstra(blocked, (5, 0), (0, 9))
    greedy = impl.astar(blocked, (5, 0), (0, 9), lambda a, b: 10.0 * impl.octile_distance(a, b))
    assert_valid_path(blocked, greedy.path, (5, 0), (0, 9), greedy.cost)
    assert greedy.cost >= optimal.cost - 1e-9


# --- the apartment ---------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def apartment():
    """C-space of the course apartment at 5 cm for karmel's footprint radius (from karmel.yaml)."""
    grid = World.apartment().to_occupancy_grid(0.05)
    radius = load_config().chassis.footprint_radius_m
    blocked = distance_transform_edt(grid.data != 100) * grid.resolution <= radius
    return grid, blocked


GOALS = {"kitchen": (5.0, 2.3), "study": (0.8, 3.8), "bedroom": (5.0, 3.2)}


@pytest.mark.parametrize("room", sorted(GOALS))
def test_apartment_astar_matches_dijkstra(impl, apartment, room):
    grid, blocked = apartment
    start, goal = grid.world_to_cell(1.0, 1.3), grid.world_to_cell(*GOALS[room])
    a = impl.astar(blocked, start, goal)
    d = impl.dijkstra(blocked, start, goal)
    assert a.path is not None and d.path is not None, f"the {room} is reachable from the living room"
    assert a.cost == approx(d.cost, abs=1e-6), "an admissible heuristic must give the optimal (Dijkstra) cost"
    assert_valid_path(blocked, a.path, start, goal, a.cost)
    assert a.expanded <= 0.5 * d.expanded, f"A* closed {a.expanded} cells, Dijkstra {d.expanded}: the heuristic is not helping"


def cost_to_go(blocked: np.ndarray, goal) -> np.ndarray:
    """Exact 8-connected, no-corner-cutting distance from every cell to ``goal`` (independent reference)."""
    dist = np.full(blocked.shape, np.inf)
    dist[goal] = 0.0
    heap = [(0.0, goal)]
    rows, cols = blocked.shape
    while heap:
        d, (r, c) = heapq.heappop(heap)
        if d > dist[r, c]:
            continue
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nr, nc = r + dr, c + dc
                if (dr or dc) and 0 <= nr < rows and 0 <= nc < cols and not blocked[nr, nc]:
                    if dr and dc and (blocked[r, nc] or blocked[nr, c]):
                        continue
                    nd = d + (SQRT2 if dr and dc else 1.0)
                    if nd < dist[nr, nc]:
                        dist[nr, nc] = nd
                        heapq.heappush(heap, (nd, (nr, nc)))
    return dist


def test_octile_is_admissible_and_consistent_in_the_apartment(impl, apartment):
    grid, blocked = apartment
    goal = grid.world_to_cell(5.0, 2.3)
    true_cost = cost_to_go(blocked, goal)
    rows, cols = np.nonzero(np.isfinite(true_cost))
    for r, c in zip(rows[::7], cols[::7]):
        h = impl.octile_distance((int(r), int(c)), goal)
        assert h <= true_cost[r, c] + 1e-9, f"h{(r, c)} = {h:.3f} overestimates the true cost {true_cost[r, c]:.3f}"
        for (nr, nc), step in impl.neighbors(blocked, (int(r), int(c))):
            assert h <= step + impl.octile_distance((nr, nc), goal) + 1e-9, "the heuristic must be consistent"
