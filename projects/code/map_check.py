"""Is the map you just saved the right size, and is it sharp?

    py projects/code/map_check.py projects/evidence/P09/home.yaml \\
        --measure "0.10,0.35; 4.28,0.35 = 4.20" \\
        --measure "0.10,0.35; 0.10,3.05 = 2.75" \\
        --max-scale-error 2 --max-unknown 12 --max-wall-thickness 15 \\
        --title "P09 - home map, first run"

A SLAM map looks convincing long before it is correct. Two faults are invisible by eye and fatal
downstream:

* **Scale.** A wheel radius that is 2 % wrong makes every distance in the map 2 % wrong, and no
  sensor can notice, because every part of the robot agrees with itself. You notice with a tape
  measure: pick two features you can find in both the map and the room, give this tool the map
  coordinates and the measured truth, and read the error.
* **Smear.** A wall drawn 25 cm thick is the same wall entered several times at slightly different
  poses. The map still "looks like the flat", and Nav2 will refuse to plan through your doorway.
  Wall thickness is measured here as the run length of consecutive occupied cells, which is a
  proxy you can compute from the saved map alone.

Reads the standard ROS map pair: the ``.yaml`` (``image``, ``resolution``, ``origin``, ``negate``,
``occupied_thresh``, ``free_thresh``) and its PGM, binary (P5) or ASCII (P2). No ROS needed - this
runs on your laptop against the files ``nav2_map_server``'s ``map_saver_cli`` wrote.

One trap worth knowing before you read the unknown count: map_saver paints unknown cells the grey
value **205**, whose occupancy works out to 50/255 = 0.19608. That is *just above* the
``free_thresh: 0.196`` map_saver writes, which is the only reason those cells come back as unknown
rather than free. A map YAML you edited to ``free_thresh: 0.25`` - a value that looks rounder and
more sensible - silently reclassifies every unknown cell as open floor, and your robot will plan
paths through the walls of rooms it has never seen. This tool warns when it sees that.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass
from pathlib import Path

import yaml

from report import percentile_abs

OCCUPIED, FREE, UNKNOWN = 1, 0, -1
UNKNOWN_GREY = 205        # the value map_saver paints cells the robot never saw

Point = tuple[float, float]


@dataclass(frozen=True)
class MapMeta:
    """The fields of a ROS map YAML that decide what the pixels mean."""

    image: Path
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    origin_yaw_rad: float
    negate: int
    occupied_thresh: float
    free_thresh: float

    @classmethod
    def load(cls, yaml_path: Path) -> "MapMeta":
        data = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
        for key in ("image", "resolution", "origin"):
            if key not in data:
                raise KeyError(f"{yaml_path} has no '{key}' key - is it a ROS map YAML?")
        origin = list(data["origin"]) + [0.0, 0.0, 0.0]
        image = Path(data["image"])
        if not image.is_absolute():
            image = yaml_path.parent / image
        resolution = float(data["resolution"])
        if resolution <= 0.0:
            raise ValueError("resolution must be > 0")
        return cls(
            image=image,
            resolution_m=resolution,
            origin_x_m=float(origin[0]),
            origin_y_m=float(origin[1]),
            origin_yaw_rad=float(origin[2]),
            negate=int(data.get("negate", 0)),
            occupied_thresh=float(data.get("occupied_thresh", 0.65)),
            free_thresh=float(data.get("free_thresh", 0.25)),
        )


def read_pgm(path: Path) -> tuple[int, int, int, list[int]]:
    """``(width, height, maxval, values)`` from a P5 or P2 PGM. Row 0 is the top row."""
    raw = path.read_bytes()
    if raw[:2] not in (b"P5", b"P2"):
        raise ValueError(f"{path} is not a PGM (magic {raw[:2]!r}); map_saver writes P5")
    binary = raw[:2] == b"P5"
    # The header is magic + three integers, with '#' comments allowed between any of them.
    fields: list[int] = []
    pos = 2
    while len(fields) < 3:
        while pos < len(raw) and raw[pos : pos + 1].isspace():
            pos += 1
        if raw[pos : pos + 1] == b"#":
            while pos < len(raw) and raw[pos : pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while pos < len(raw) and not raw[pos : pos + 1].isspace():
            pos += 1
        fields.append(int(raw[start:pos]))
    width, height, maxval = fields
    if maxval > 255:
        raise ValueError(f"{path}: 16-bit PGMs are not supported (maxval {maxval})")
    pos += 1  # exactly one whitespace byte separates the header from binary data
    if binary:
        values = list(raw[pos : pos + width * height])
    else:
        values = [int(tok) for tok in re.findall(rb"\d+", raw[pos:])]
    if len(values) != width * height:
        raise ValueError(f"{path}: expected {width * height} pixels, found {len(values)}")
    return width, height, maxval, values


@dataclass(frozen=True)
class OccupancyMap:
    """A saved map, classified into occupied / free / unknown exactly as map_server does."""

    meta: MapMeta
    width: int
    height: int
    cells: tuple[int, ...]
    values: tuple[int, ...]

    @classmethod
    def load(cls, yaml_path: Path) -> "OccupancyMap":
        meta = MapMeta.load(yaml_path)
        width, height, maxval, values = read_pgm(meta.image)
        cells = tuple(classify(v, maxval, meta) for v in values)
        return cls(meta=meta, width=width, height=height, cells=cells, values=tuple(values))

    def misread_unknown_cells(self) -> int:
        """Cells painted the unknown grey that this YAML's thresholds classify as something else.

        Non-zero means the YAML's ``free_thresh`` was edited above 0.196; see the module docstring.
        """
        return sum(1 for value, cell in zip(self.values, self.cells)
                   if value == UNKNOWN_GREY and cell != UNKNOWN)

    def at(self, col: int, row: int) -> int:
        """Cell state by image coordinates; row 0 is the top of the image."""
        if not (0 <= col < self.width and 0 <= row < self.height):
            raise IndexError(f"({col}, {row}) is outside a {self.width}x{self.height} map")
        return self.cells[row * self.width + col]

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        """Map coordinates -> (col, row). Row 0 is the *top*, i.e. the largest y."""
        col = int((x_m - self.meta.origin_x_m) / self.meta.resolution_m)
        row = self.height - 1 - int((y_m - self.meta.origin_y_m) / self.meta.resolution_m)
        return col, row

    def counts(self) -> dict[str, int]:
        return {
            "occupied": sum(1 for c in self.cells if c == OCCUPIED),
            "free": sum(1 for c in self.cells if c == FREE),
            "unknown": sum(1 for c in self.cells if c == UNKNOWN),
        }

    def area_m2(self, state: int) -> float:
        cell = self.meta.resolution_m ** 2
        return sum(1 for c in self.cells if c == state) * cell

    def unknown_percent(self) -> float:
        return 100.0 * self.counts()["unknown"] / len(self.cells)

    def rows(self) -> list[list[int]]:
        return [list(self.cells[r * self.width : (r + 1) * self.width]) for r in range(self.height)]

    def columns(self) -> list[list[int]]:
        return [[self.at(c, r) for r in range(self.height)] for c in range(self.width)]


def classify(value: int, maxval: int, meta: MapMeta) -> int:
    """map_server's rule: p = (maxval - value)/maxval (or value/maxval when negate)."""
    p = value / maxval if meta.negate else (maxval - value) / maxval
    if p > meta.occupied_thresh:
        return OCCUPIED
    if p < meta.free_thresh:
        return FREE
    return UNKNOWN


def distance_to_open(grid: OccupancyMap) -> list[int]:
    """Manhattan distance from every occupied cell to the nearest cell that is not occupied.

    ``0`` at every free or unknown cell, ``1`` at an occupied cell that touches one, and upward
    from there. Two chamfer passes over the grid (forward, then backward), which is the cheap way
    to get an exact Manhattan distance transform without numpy or scipy.
    """
    width, height = grid.width, grid.height
    far = width + height          # larger than any real Manhattan distance in this grid
    d = [0 if cell != OCCUPIED else far for cell in grid.cells]
    for row in range(height):
        for col in range(width):
            i = row * width + col
            if d[i] == 0:
                continue
            if col > 0:
                d[i] = min(d[i], d[i - 1] + 1)
            if row > 0:
                d[i] = min(d[i], d[i - width] + 1)
    for row in reversed(range(height)):
        for col in reversed(range(width)):
            i = row * width + col
            if d[i] == 0:
                continue
            if col < width - 1:
                d[i] = min(d[i], d[i + 1] + 1)
            if row < height - 1:
                d[i] = min(d[i], d[i + width] + 1)
    return d


def cell_thickness(grid: OccupancyMap) -> list[int]:
    """Local wall thickness in cells at every occupied cell: ``2 * distance_to_open - 1``.

    A cell that touches open space is one cell thick. A cell one step inside a slab is three.
    This is the usual morphological reading of "thickness" - twice the largest circle that fits,
    plus the cell itself - and unlike a run length it is not fooled by corners, where both the
    horizontal and the vertical run are long but the wall is still one cell thick. It resolves
    only to odd multiples of the cell size, so a four-cell slab reports as three.
    """
    return [2 * d - 1 for d, cell in zip(distance_to_open(grid), grid.cells) if cell == OCCUPIED]


def wall_thickness_cm(grid: OccupancyMap) -> tuple[float, float, int]:
    """``(median, p95, n)`` wall thickness in centimetres, over every occupied cell.

    A crisp map of 5 cm cells gives a median and a p95 of one cell (5 cm): that is the LiDAR's
    view of a surface, drawn once. A smeared map keeps the median (the outside of the wall is
    always one cell) and pushes the **p95** up, which is why the p95 is the criterion the
    projects check: smearing starts in one corridor rather than everywhere.
    """
    thickness = cell_thickness(grid)
    if not thickness:
        return 0.0, 0.0, 0
    scale = grid.meta.resolution_m * 100.0
    return (percentile_abs(thickness, 50.0) * scale,
            percentile_abs(thickness, 95.0) * scale,
            len(thickness))


@dataclass(frozen=True)
class Measurement:
    """One tape-measure check: two points in the map, and the truth from your floor."""

    a: Point
    b: Point
    truth_m: float

    @property
    def map_m(self) -> float:
        return math.hypot(self.b[0] - self.a[0], self.b[1] - self.a[1])

    @property
    def error_m(self) -> float:
        return self.map_m - self.truth_m

    @property
    def scale_error_percent(self) -> float:
        if self.truth_m <= 0.0:
            raise ValueError("the measured truth must be > 0")
        return 100.0 * self.error_m / self.truth_m

    @classmethod
    def parse(cls, text: str) -> "Measurement":
        """``"0.1,0.35; 4.28,0.35 = 4.20"`` -> two map points and the tape-measured 4.20 m."""
        points_text, sep, truth_text = text.partition("=")
        if not sep:
            raise ValueError(f"measurement {text!r} needs '= <metres you measured>'")
        points = []
        for chunk in points_text.split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            parts = chunk.split(",")
            if len(parts) != 2:
                raise ValueError(f"point {chunk!r} in {text!r} is not 'x,y'")
            points.append((float(parts[0]), float(parts[1])))
        if len(points) != 2:
            raise ValueError(f"measurement {text!r} needs exactly two points")
        return cls(points[0], points[1], float(truth_text))


def summary_table(grid: OccupancyMap) -> str:
    counts = grid.counts()
    total = len(grid.cells)
    res = grid.meta.resolution_m
    median_cm, p95_cm, n_runs = wall_thickness_cm(grid)
    return "\n".join([
        "| Property | Value |",
        "|---|---|",
        f"| size | {grid.width} x {grid.height} cells = "
        f"{grid.width * res:.2f} x {grid.height * res:.2f} m |",
        f"| resolution | {res * 100:.1f} cm/cell |",
        f"| origin | ({grid.meta.origin_x_m:.3f}, {grid.meta.origin_y_m:.3f}) m |",
        f"| free | {counts['free']} cells = {grid.area_m2(FREE):.2f} m2 "
        f"({100.0 * counts['free'] / total:.1f} %) |",
        f"| occupied | {counts['occupied']} cells ({100.0 * counts['occupied'] / total:.1f} %) |",
        f"| unknown | {counts['unknown']} cells ({grid.unknown_percent():.1f} %) |",
        f"| wall thickness | median {median_cm:.1f} cm, p95 {p95_cm:.1f} cm "
        f"(over {n_runs} occupied cells) |",
    ])


def threshold_warning(grid: OccupancyMap) -> str:
    """A line to print when the YAML's thresholds disagree with what map_saver painted."""
    misread = grid.misread_unknown_cells()
    if not misread:
        return ""
    return (f"warning: {misread} cells are painted the unknown grey ({UNKNOWN_GREY}) but this "
            f"YAML's free_thresh ({grid.meta.free_thresh:g}) classifies them as free. "
            f"map_saver writes free_thresh: 0.196 for exactly this reason.")


def measurement_table(measurements: list[Measurement]) -> str:
    lines = ["| In the map | Tape measure | Error | Scale error |", "|---|---|---|---|"]
    for m in measurements:
        lines.append(f"| {m.map_m:.3f} m | {m.truth_m:.3f} m | {m.error_m * 100:+.1f} cm "
                     f"| {m.scale_error_percent:+.2f} % |")
    return "\n".join(lines)


def check(grid: OccupancyMap, measurements: list[Measurement],
          max_scale_error_percent: float | None = None,
          max_unknown_percent: float | None = None,
          max_wall_thickness_cm: float | None = None) -> list[tuple[str, str, bool]]:
    """Every limit you declared, the measured value, and whether it holds."""
    rows: list[tuple[str, str, bool]] = []
    if max_scale_error_percent is not None:
        if not measurements:
            rows.append(("scale error", "no --measure given", False))
        else:
            worst = max(abs(m.scale_error_percent) for m in measurements)
            n = len(measurements)
            rows.append((f"worst absolute scale error <= {max_scale_error_percent:g} %",
                         f"{worst:.2f} % over {n} measurement{'' if n == 1 else 's'}",
                         worst <= max_scale_error_percent))
    if max_unknown_percent is not None:
        unknown = grid.unknown_percent()
        rows.append((f"unknown cells <= {max_unknown_percent:g} %",
                     f"{unknown:.1f} %", unknown <= max_unknown_percent))
    if max_wall_thickness_cm is not None:
        _, p95_cm, _ = wall_thickness_cm(grid)
        rows.append((f"p95 wall thickness <= {max_wall_thickness_cm:g} cm",
                     f"{p95_cm:.1f} cm", p95_cm <= max_wall_thickness_cm))
    return rows


def verdict_table(rows: list[tuple[str, str, bool]]) -> str:
    lines = ["| Criterion | Measured | Verdict |", "|---|---|---|"]
    for detail, measured, passed in rows:
        lines.append(f"| {detail} | {measured} | {'pass' if passed else '**FAIL**'} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map_yaml", type=Path, help="the .yaml written by map_saver_cli")
    ap.add_argument("--measure", action="append", default=[], metavar="EXPR",
                    help='"x1,y1; x2,y2 = metres_you_measured" (repeat for each check)')
    ap.add_argument("--max-scale-error", type=float, metavar="PCT",
                    help="limit on the worst |scale error| across --measure checks")
    ap.add_argument("--max-unknown", type=float, metavar="PCT",
                    help="limit on the percentage of unknown cells")
    ap.add_argument("--max-wall-thickness", type=float, metavar="CM",
                    help="limit on the p95 occupied-run length (smear)")
    ap.add_argument("--title", default="", help="heading printed above the tables")
    args = ap.parse_args(argv)

    grid = OccupancyMap.load(args.map_yaml)
    measurements = [Measurement.parse(text) for text in args.measure]

    if args.title:
        print(f"### {args.title}\n")
    print(summary_table(grid))
    warning = threshold_warning(grid)
    if warning:
        print("\n" + warning)
    if measurements:
        print("\n" + measurement_table(measurements))
    rows = check(grid, measurements, args.max_scale_error, args.max_unknown,
                 args.max_wall_thickness)
    if rows:
        print("\n" + verdict_table(rows))
    return 0 if all(passed for _, _, passed in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
