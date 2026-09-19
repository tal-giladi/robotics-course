#!/usr/bin/env python3
"""Check an SDF world against the checklist for a good SLAM / navigation test world (lesson 06.08).

    py 06-simulation/code/gazebo/world_check.py labs/ros2_ws/src/karmel_gazebo/worlds/apartment.sdf
    py 06-simulation/code/gazebo/world_check.py 06-simulation/code/gazebo/*.sdf --verbose

It is deliberately a *static* checker: pure Python, no Gazebo, no ROS, so it runs on any machine
in under a second and can sit in CI next to `gz sdf -k` (which checks the XML, not the robotics).

What it checks, and why each one is on the list:

  1. one <world>, well-formed XML                 anything else confuses `gz sim` and the launch files
  2. max_step_size <= 1 ms                        wheel contacts jitter at 10 ms (lesson 06.05)
  3. the five systems karmel needs                Physics, UserCommands, SceneBroadcaster, Sensors, Imu
  4. a ground plane with friction                 mu = 0 means the wheels spin and nothing moves
  5. at least one light                           the camera renders black otherwise
  6. geometry at the LiDAR scan plane             a 2D LiDAR at 16.5 cm cannot see a 72 cm table top
  7. the free space is enclosed                   escaping rays -> unbounded costmap, unstable SLAM
  8. the free space is not translation-ambiguous  a featureless corridor slides under scan matching

Checks 7 and 8 raycast a 2D model of the world built from every axis-aligned-plus-yaw <box>
collision whose vertical span crosses the scan plane. Cylinders and spheres are approximated by
their bounding box; meshes are reported as "not modelled" rather than silently ignored.
"""

from __future__ import annotations

import argparse
import math
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

SCAN_Z = 0.165          # karmel's laser frame: 0.045 wheel radius + 0.120 (labs/config/karmel.yaml)
MAX_RANGE = 12.0        # sensors.lidar.max_range_m
REQUIRED_SYSTEMS = (
    "gz::sim::systems::Physics",
    "gz::sim::systems::UserCommands",
    "gz::sim::systems::SceneBroadcaster",
    "gz::sim::systems::Sensors",
    "gz::sim::systems::Imu",
)


@dataclass
class Rect:
    """A box collision, flattened to the scan plane: centre, half-extents, yaw."""

    name: str
    x: float
    y: float
    hx: float
    hy: float
    yaw: float

    def contains(self, px: float, py: float, margin: float = 0.0) -> bool:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = px - self.x, py - self.y
        u, v = c * dx + s * dy, -s * dx + c * dy
        return abs(u) <= self.hx + margin and abs(v) <= self.hy + margin

    def ray_distance(self, ox: float, oy: float, angle: float) -> float:
        """Distance from (ox, oy) along `angle` to this rectangle, or inf (slab method, box frame)."""
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = ox - self.x, oy - self.y
        ux, uy = c * dx + s * dy, -s * dx + c * dy
        a = angle - self.yaw
        dxr, dyr = math.cos(a), math.sin(a)
        t_min, t_max = 0.0, math.inf
        for origin, direction, half in ((ux, dxr, self.hx), (uy, dyr, self.hy)):
            if abs(direction) < 1e-12:
                if abs(origin) > half:
                    return math.inf
                continue
            t1, t2 = (-half - origin) / direction, (half - origin) / direction
            t_min, t_max = max(t_min, min(t1, t2)), min(t_max, max(t1, t2))
            if t_min > t_max:
                return math.inf
        return t_min if t_max >= t_min else math.inf


@dataclass
class Report:
    path: str
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    rects: list[Rect] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def _pose(element: ET.Element | None) -> tuple[float, float, float, float]:
    """<pose>x y z r p y</pose> -> (x, y, z, yaw). Missing pose = identity."""
    if element is None or element.find("pose") is None or not (element.find("pose").text or "").strip():
        return 0.0, 0.0, 0.0, 0.0
    values = [float(v) for v in element.find("pose").text.split()]
    values += [0.0] * (6 - len(values))
    return values[0], values[1], values[2], values[5]


def _compose(parent: tuple[float, float, float, float],
             child: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    c, s = math.cos(pyaw), math.sin(pyaw)
    return px + c * cx - s * cy, py + s * cx + c * cy, pz + cz, pyaw + cyaw


def collect_rects(world: ET.Element, report: Report, scan_z: float = SCAN_Z) -> list[Rect]:
    """Every collision box whose vertical span crosses the scan plane, in world coordinates."""
    rects: list[Rect] = []
    below = above = 0
    for model in world.iter("model"):
        model_pose = _pose(model)
        model_name = model.get("name", "?")
        for link in model.iter("link"):
            link_pose = _compose(model_pose, _pose(link))
            for collision in link.iter("collision"):
                x, y, z, yaw = _compose(link_pose, _pose(collision))
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                name = f"{model_name}/{collision.get('name', '?')}"
                if geometry.find("plane") is not None:
                    continue                                    # the floor: never a scan return
                if geometry.find("mesh") is not None:
                    report.notes.append(f"{name}: mesh collision — not modelled by this checker")
                    continue
                if (box := geometry.find("box")) is not None:
                    sx, sy, sz = (float(v) for v in box.find("size").text.split())
                elif (cylinder := geometry.find("cylinder")) is not None:
                    r = float(cylinder.find("radius").text)
                    sx = sy = 2 * r
                    sz = float(cylinder.find("length").text)
                elif (sphere := geometry.find("sphere")) is not None:
                    r = float(sphere.find("radius").text)
                    sx = sy = sz = 2 * r
                else:
                    continue
                if z + sz / 2 < scan_z:
                    below += 1
                    continue
                if z - sz / 2 > scan_z:
                    above += 1
                    continue
                rects.append(Rect(name, x, y, sx / 2, sy / 2, yaw))
    if below or above:
        report.notes.append(
            f"{below} collision(s) entirely below and {above} entirely above the {scan_z:.3f} m scan "
            f"plane — invisible to a 2D LiDAR (tabletops, thresholds, ceiling lamps)")
    return rects


def raycast(rects: list[Rect], x: float, y: float, angle: float, max_range: float = MAX_RANGE) -> float:
    best = max_range
    for rect in rects:
        distance = rect.ray_distance(x, y, angle)
        if distance < best:
            best = distance
    return best


def scan(rects: list[Rect], x: float, y: float, rays: int = 72, max_range: float = MAX_RANGE) -> list[float]:
    return [raycast(rects, x, y, 2 * math.pi * i / rays, max_range) for i in range(rays)]


def free_points(rects: list[Rect], step: float, clearance: float = 0.25) -> list[tuple[float, float]]:
    """Free grid points *enclosed* by the world's geometry — the part a robot can be driven in.

    The robot never drives on the outside of the apartment's walls, so sampling there would make
    every world look "not enclosed". A flood fill from the bounding box border marks the outside;
    whatever free space it cannot reach is the interior.
    """
    if not rects:
        return []
    xs = [r.x for r in rects]
    ys = [r.y for r in rects]
    x0, y0 = min(xs) - 1.0, min(ys) - 1.0
    n_x = max(2, int((max(xs) + 1.0 - x0) / step)) + 1
    n_y = max(2, int((max(ys) + 1.0 - y0) / step)) + 1

    def at(i: int, j: int) -> tuple[float, float]:
        return x0 + i * step, y0 + j * step

    blocked = [[any(r.contains(*at(i, j), clearance) for r in rects) for j in range(n_y)]
               for i in range(n_x)]
    outside = [[False] * n_y for _ in range(n_x)]
    stack = [(i, j) for i in range(n_x) for j in (0, n_y - 1)]
    stack += [(i, j) for j in range(n_y) for i in (0, n_x - 1)]
    while stack:
        i, j = stack.pop()
        if not (0 <= i < n_x and 0 <= j < n_y) or outside[i][j] or blocked[i][j]:
            continue
        outside[i][j] = True
        stack += [(i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)]
    return [at(i, j) for i in range(n_x) for j in range(n_y)
            if not blocked[i][j] and not outside[i][j]]


def enclosure(rects: list[Rect], points: list[tuple[float, float]], rays: int = 72) -> tuple[float, int]:
    """Fraction of (point, ray) pairs that reach max_range without hitting anything, and the count."""
    escaped = total = 0
    for x, y in points:
        for value in scan(rects, x, y, rays):
            total += 1
            escaped += value >= MAX_RANGE - 1e-6
    return (escaped / total if total else 1.0), total


def ambiguity(rects: list[Rect], points: list[tuple[float, float]], shift: float = 0.10,
              rays: int = 72, threshold: float = 0.02) -> tuple[float, list[tuple[float, float]]]:
    """Fraction of free points where moving `shift` in the *least* detectable direction is invisible.

    For each point, the scan is compared with the scan taken `shift` metres away in each of eight
    directions; the score is the smallest mean absolute range change. A corridor gives a tiny score
    along its axis: the scan matcher cannot tell where along the corridor you are, so SLAM slides
    and AMCL's particles spread. Only rays that return a finite range in both scans are compared.
    """
    ambiguous = []
    for x, y in points:
        base = scan(rects, x, y, rays)
        worst = math.inf
        for k in range(8):
            a = 2 * math.pi * k / 8
            nx, ny = x + shift * math.cos(a), y + shift * math.sin(a)
            if any(r.contains(nx, ny, 0.20) for r in rects):
                continue
            other = scan(rects, nx, ny, rays)
            pairs = [(u, v) for u, v in zip(base, other) if u < MAX_RANGE and v < MAX_RANGE]
            if not pairs:
                continue
            worst = min(worst, sum(abs(u - v) for u, v in pairs) / len(pairs))
        if worst < threshold:
            ambiguous.append((x, y))
    return (len(ambiguous) / len(points) if points else 0.0), ambiguous


def check(path: Path, grid: float = 0.5, verbose: bool = False) -> Report:
    report = Report(str(path))
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError as exc:
        report.errors.append(f"not well-formed XML: {exc}")
        return report

    worlds = root.findall("world")
    if len(worlds) != 1:
        report.errors.append(f"expected exactly one <world>, found {len(worlds)}")
        return report
    world = worlds[0]
    report.notes.append(f"world name: {world.get('name')!r}")

    # 2. physics
    step = world.find("physics/max_step_size")
    if step is None:
        report.warnings.append("no <physics><max_step_size> — Gazebo's default is 1 ms, but say so")
    elif float(step.text) > 0.001:
        report.errors.append(f"max_step_size {step.text} s > 0.001 s: wheel contacts will jitter")

    # 3. systems
    plugins = {p.get("name") for p in world.findall("plugin")}
    for system in REQUIRED_SYSTEMS:
        if system not in plugins:
            report.errors.append(f"missing system plugin {system}")
    sensors = world.find("plugin[@name='gz::sim::systems::Sensors']")
    if sensors is not None and sensors.find("render_engine") is None:
        report.warnings.append("Sensors system has no <render_engine>; karmel's worlds set ogre2")

    # 4. ground + friction
    ground = [m for m in world.iter("model") if m.find(".//plane") is not None]
    if not ground:
        report.errors.append("no ground plane: the robot will fall for ever")
    else:
        mus = [float(e.text) for e in ground[0].iter() if e.tag in ("mu", "mu2")]
        if not mus:
            report.warnings.append("ground plane has no <mu>: the engine default may not be what you think")
        elif min(mus) < 0.2:
            report.errors.append(f"ground friction mu = {min(mus)} — the wheels will spin in place")

    # 5. light
    if not world.findall("light") and not any(m.find("light") is not None for m in world.iter("model")):
        report.warnings.append("no <light>: the simulated camera renders a black image")

    # 6-8. the 2D world the LiDAR actually sees
    rects = collect_rects(world, report)
    report.rects = rects
    if not rects:
        report.warnings.append(f"nothing at the {SCAN_Z:.3f} m scan plane: every ray returns inf")
        return report
    report.notes.append(f"{len(rects)} collision box(es) cross the scan plane")

    points = free_points(rects, grid)
    if not points:
        report.warnings.append(
            "no enclosed free space: every free grid point is reachable from outside the world. "
            "Fine for a physics demo, wrong for a SLAM or navigation lab")
        return report

    escaped, total = enclosure(rects, points)
    report.notes.append(f"{len(points)} enclosed free sample points ({grid} m grid), {total} rays")
    if escaped > 0.02:
        report.warnings.append(
            f"{escaped:.1%} of rays escape to {MAX_RANGE} m: the world is not enclosed. "
            f"SLAM and the Nav2 global costmap both behave better inside a closed room")

    fraction, ambiguous = ambiguity(rects, points)
    if fraction > 0.10:
        report.warnings.append(
            f"{fraction:.1%} of the free area is translation-ambiguous at 10 cm "
            f"(a 0.10 m move changes the scan by < 2 cm on average): scan matching will slide here")
    else:
        report.notes.append(f"translation-ambiguous area: {fraction:.1%} (good: features everywhere)")
    if verbose and ambiguous:
        report.notes.append("ambiguous points: " + ", ".join(f"({x:.1f},{y:.1f})" for x, y in ambiguous[:12]))
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("worlds", nargs="+", type=Path)
    parser.add_argument("--grid", type=float, default=0.5, help="sample spacing in metres (default 0.5)")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    failed = False
    for path in args.worlds:
        report = check(path, args.grid, args.verbose)
        print(f"== {path}")
        for note in report.notes:
            print(f"   .  {note}")
        for warning in report.warnings:
            print(f"   ?  WARN  {warning}")
        for error in report.errors:
            print(f"   !  ERROR {error}")
        print(f"   => {'PASS' if report.ok else 'FAIL'}"
              f"{'' if not report.warnings else f' with {len(report.warnings)} warning(s)'}")
        failed |= not report.ok
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
