# 11.02 — Occupancy grid mapping with known poses

Lesson: [11.02 Occupancy grid mapping with known poses](../../../11-slam/11.02-occupancy-grid-mapping.md)

Turn LiDAR scans taken at known poses into an occupancy grid: trace every beam through the grid
with Bresenham's line algorithm, update each cell's log-odds with the inverse sensor model, and
threshold the result into the free / occupied / unknown map that ROS `map_server` stores.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `bresenham(x0, y0, x1, y1)` | integer cells on a line, both ends included, all eight octants |
| `logodds(p)`, `probability(l)` | probability ↔ log-odds (`probability` also works on numpy arrays) |
| `LogOddsMap.update_ray(start, end, hit)` | miss update for the cells a beam passed, hit (or miss) for its end cell; clamp; skip cells outside the grid |
| `LogOddsMap.integrate_scan(pose, ranges, angles, max_range)` | one scan from a sensor at `(x, y, theta)`: hits, `+inf` = free to max range, `nan` = ignore |
| `LogOddsMap.to_trinary(free_thresh, occupied_thresh)` | `int8` map: 100 occupied, 0 free, −1 unknown |

`LogOddsMap.world_to_cell` is given. Each stub raises `NotImplementedError`: replace the body,
keep the signature.

## Check

```bash
python course.py check 11.02              # your code
python course.py check 11.02 --solution   # the reference, to see what passing looks like
```

The tests start with hand-computed cases (lines in every direction, log-odds 0.8473 / −0.6190,
a hit-hit-miss sequence that ends at p = 0.746, clamping at ±4, single beams with `inf` and
`nan`). Then they map a known 3 m × 2 m room with a dividing wall and a round obstacle from five
noise-free scans and compare it with the true geometry:

* more than 200 occupied cells, over 97% of them within 7.5 cm of a real wall;
* over 97% of free cells contain no wall;
* over 90% of the open floor is mapped as free.

A skipped test means that part is not implemented yet. The check passes only when nothing is
skipped.

## Hints

* `logodds[row, col]` is `logodds[y, x]`. Swapping them is the most common bug: the test map
  comes out mirrored along the diagonal.
* Clamp after *every* update, not once at the end.
* A beam with range `+inf` still tells you something: the space in front of it is free.
