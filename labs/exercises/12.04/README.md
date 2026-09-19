# 12.04 — The inflation layer, exactly like Nav2

Lesson: [12.04 Costmaps, inflation and the robot footprint](../../../12-navigation/12.04-costmaps.md)

Reimplement the part of `nav2_costmap_2d` that decides how close to a wall karmel is willing to drive:
the footprint radii and the inflation layer. When this passes, your numbers are Nav2's numbers, so you
can tune `inflation_radius` and `cost_scaling_factor` in `nav2_params.yaml` with a calculator.

## What to implement (`student.py`)

| Function | Does |
|---|---|
| `footprint_radii(footprint)` | (inscribed, circumscribed) radius of a polygon: min / max distance from base_link to any vertex or edge |
| `inflation_cost(distance_m, inscribed_radius_m, cost_scaling_factor)` | 254 at distance 0, 253 inside the inscribed radius, else `int(252·exp(−k·(d − r)))` |
| `distance_for_cost(cost, inscribed_radius_m, cost_scaling_factor)` | inverse of the decay: where does the cost fall to `cost`? |
| `inflate(master, resolution, inscribed_radius_m, cost_scaling_factor, inflation_radius_m)` | new costmap: distance from every cell to the nearest lethal cell, cost, Nav2's combination rule with unknown cells |

Each stub raises `NotImplementedError`: replace the body, keep the signature. numpy is enough.

## Check

```bash
python course.py check 12.04              # your code
python course.py check 12.04 --solution   # the reference, to see what passing looks like
```

The tests compare against a line-by-line transcription of `InflationLayer::computeCost` from Nav2
Jazzy: hand-computed cases (164 at 0.20 m with k = 5 and r = 0.115 m, because C++ truncates),
thousands of distances at three resolutions, karmel's footprint from `labs/config/karmel.yaml` and
from `nav2_params.yaml` (with and without `footprint_padding`), a single obstacle cell, a wall next
to a post, the round-up of `inflation_radius` to whole cells, unknown cells and existing costs,
and finally the apartment map.

A skipped test means that part is not implemented yet; the check passes only when nothing is skipped.

## Hints

* Distances are between cell **centers**, in cells: an offset of (3, 4) cells is 5 cells = 0.25 m at 5 cm.
* `ceil(0.33 / 0.05) = 7`: Nav2 inflates whole cells, so the cell 0.35 m away still gets a cost.
* An unknown cell (255) keeps being unknown unless the inflation says 253 or 254: "maybe risky" must
  not turn unexplored space into "known and fairly safe".
