# 12.02 — A* on an occupancy grid

Lesson: [12.02 Grid path planning — BFS, Dijkstra and A*](../../../12-navigation/12.02-grid-path-planning.md)

Plan the shortest collision-free path for karmel across the course apartment. The grid you get is
already the configuration space (obstacles grown by the robot radius), so the robot is a point.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `octile_distance(a, b)` | shortest 8-connected distance on an empty grid: `max(dr, dc) + (√2 − 1)·min(dr, dc)` |
| `neighbors(blocked, cell)` | free 8-connected neighbours with step length 1 or √2; no leaving the grid, no corner cutting |
| `astar(blocked, start, goal, heuristic)` | A* with a binary heap, lazy deletion, ties on f broken toward smaller h; returns `PlanResult(path, cost, expanded)` |
| `dijkstra(blocked, start, goal)` | A* with `h = 0` |

Cells are `(row, col)`; costs are in cells (multiply by the grid resolution for meters). Each stub
raises `NotImplementedError`: replace the body, keep the signature.

## Check

```bash
python course.py check 12.02              # your code
python course.py check 12.02 --solution   # the reference, to see what passing looks like
```

The tests start with hand-computed cases (octile distances, neighbours next to a blocked corner, the
lesson's 6 × 10 map with its 8.828-cell optimum), then the special cases (start = goal, blocked start,
an enclosed goal, a diagonal gap you may not squeeze through) and finally the apartment at 5 cm,
C-space for `chassis.footprint_radius_m` from `labs/config/karmel.yaml`:

* A* must return **exactly Dijkstra's cost** to the kitchen, the study and the bedroom, with a valid
  path, while closing **at most half** as many cells;
* octile distance must never exceed the true cost-to-go (admissible) and must satisfy
  `h(a) ≤ step + h(b)` for every move (consistent);
* on an empty 40 × 60 grid A* may close at most **2 × the path length** cells — that only works with
  the tie-breaking rule.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* Heap entries `(f, h, counter, cell)`: the counter keeps `heapq` from ever comparing two cells and
  makes the order deterministic.
* Don't try to "decrease key" in the heap. Push the cell again with the better f and skip stale
  entries when they come out (`if cell in closed: continue`).
* Rebuild the path by walking `parent` back from the goal, then reverse it.
