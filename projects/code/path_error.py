"""How far was the robot from the path it was supposed to drive?

    py projects/code/path_error.py projects/evidence/P05/square_ccw_pose.csv \\
        --waypoints "0,0; 2,0; 2,2; 0,2; 0,0"

Reads a pose log (``t,x,y,theta`` in metres and radians, REP-103: x forward, y left, theta
counter-clockwise) and reports the three numbers the projects ask for:

* **closing error** - how far the last pose is from the first. The square's headline number.
* **cross-track error** - the signed perpendicular distance from the straight segment the robot
  was on. Positive means the robot is to the *left* of the segment direction. A heading loop
  (08.10) drives heading to zero and leaves a constant cross-track offset; only a cross-track
  layer removes it.
* **waypoint miss** - the closest the robot ever came to each waypoint.

Nothing here knows about hardware: feed it odometry (09.04), a ground-truth log from the
simulator, or a handful of tape-measure readings you typed in yourself.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

Point = tuple[float, float]


@dataclass(frozen=True)
class Pose:
    """A planar pose from a log. ``theta`` is radians, counter-clockwise from +x."""

    t: float
    x: float
    y: float
    theta: float = 0.0


@dataclass(frozen=True)
class SegmentReport:
    """Cross-track statistics for one straight segment of the plan."""

    index: int
    start: Point
    end: Point
    samples: int
    mean_m: float
    abs_max_m: float


def wrap_angle(a: float) -> float:
    """Wrap to (-pi, pi]. The bug that makes a robot turn 260 degrees the wrong way (08.11)."""
    return math.pi - (math.pi - a) % (2.0 * math.pi)


def heading_error_deg(measured_rad: float, reference_rad: float) -> float:
    """Signed shortest-way heading error, in degrees."""
    return math.degrees(wrap_angle(measured_rad - reference_rad))


def closing_error_m(poses: list[Pose]) -> float:
    """Distance from the first pose to the last. The number you measure with a tape."""
    if len(poses) < 2:
        raise ValueError("need at least two poses")
    first, last = poses[0], poses[-1]
    return math.hypot(last.x - first.x, last.y - first.y)


def path_length_m(poses: list[Pose]) -> float:
    """Total distance travelled along the logged poses."""
    return sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(poses, poses[1:]))


def cross_track_m(point: Point, start: Point, end: Point) -> float:
    """Signed perpendicular distance from ``point`` to the infinite line ``start``->``end``.

    Positive = left of the direction of travel. A zero-length segment raises.
    """
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        raise ValueError("segment has zero length")
    # The unit normal pointing left of the direction of travel is (-dy, dx) / length.
    return ((point[1] - start[1]) * dx - (point[0] - start[0]) * dy) / length


def projection_fraction(point: Point, start: Point, end: Point) -> float:
    """Where along the segment the point projects: 0 at ``start``, 1 at ``end``, unclamped."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq == 0.0:
        raise ValueError("segment has zero length")
    return ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq


def segment_reports(poses: list[Pose], waypoints: list[Point]) -> list[SegmentReport]:
    """Cross-track statistics per segment, assigning each pose to its nearest segment.

    Each pose is assigned to the segment it is physically closest to (distance to the segment,
    not to the infinite line), so a pose sitting in a corner belongs to exactly one segment and
    no sample is silently dropped.
    """
    if len(waypoints) < 2:
        raise ValueError("need at least two waypoints")
    segments = list(zip(waypoints, waypoints[1:]))
    buckets: list[list[float]] = [[] for _ in segments]
    for pose in poses:
        point = (pose.x, pose.y)
        best = min(range(len(segments)), key=lambda i: distance_to_segment_m(point, *segments[i]))
        buckets[best].append(cross_track_m(point, *segments[best]))

    reports = []
    for i, ((a, b), errors) in enumerate(zip(segments, buckets)):
        reports.append(SegmentReport(
            index=i,
            start=a,
            end=b,
            samples=len(errors),
            mean_m=statistics.fmean(errors) if errors else 0.0,
            abs_max_m=max((abs(e) for e in errors), default=0.0),
        ))
    return reports


def distance_to_segment_m(point: Point, start: Point, end: Point) -> float:
    """Unsigned distance to the *segment* (not the infinite line): the projection is clamped."""
    fraction = min(1.0, max(0.0, projection_fraction(point, start, end)))
    nearest_x = start[0] + fraction * (end[0] - start[0])
    nearest_y = start[1] + fraction * (end[1] - start[1])
    return math.hypot(point[0] - nearest_x, point[1] - nearest_y)


def waypoint_misses_m(poses: list[Pose], waypoints: list[Point]) -> list[float]:
    """Closest approach to each waypoint over the whole run."""
    if not poses:
        raise ValueError("no poses")
    return [min(math.hypot(p.x - wx, p.y - wy) for p in poses) for wx, wy in waypoints]


def parse_waypoints(text: str) -> list[Point]:
    """``"0,0; 2,0; 2,2"`` -> ``[(0.0, 0.0), (2.0, 0.0), (2.0, 2.0)]``."""
    points = []
    for chunk in text.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = chunk.split(",")
        if len(parts) != 2:
            raise ValueError(f"waypoint {chunk!r} is not 'x,y'")
        points.append((float(parts[0]), float(parts[1])))
    if len(points) < 2:
        raise ValueError("need at least two waypoints")
    return points


def read_poses(path: Path) -> list[Pose]:
    """Read a ``t,x,y,theta`` CSV. ``theta`` may be missing; extra columns are ignored."""
    poses = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            poses.append(Pose(
                t=float(row.get("t") or 0.0),
                x=float(row["x"]),
                y=float(row["y"]),
                theta=float(row.get("theta") or 0.0),
            ))
    if len(poses) < 2:
        raise ValueError(f"{path} has fewer than two poses")
    return poses


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="pose log with columns t,x,y[,theta]")
    ap.add_argument("--waypoints", help='the planned path, e.g. "0,0; 2,0; 2,2; 0,2; 0,0"')
    ap.add_argument("--final-heading-deg", type=float,
                    help="the heading the robot was supposed to finish with, degrees")
    args = ap.parse_args(argv)

    poses = read_poses(args.csv)
    print(f"samples        {len(poses)}")
    print(f"path length    {path_length_m(poses):.3f} m")
    print(f"closing error  {closing_error_m(poses) * 100:.1f} cm")
    if args.final_heading_deg is not None:
        error = heading_error_deg(poses[-1].theta, math.radians(args.final_heading_deg))
        print(f"heading error  {error:+.1f} deg")
    if args.waypoints:
        waypoints = parse_waypoints(args.waypoints)
        print("\nsegment  from        to          samples  mean XTE  worst XTE")
        for r in segment_reports(poses, waypoints):
            print(f"{r.index:>7}  {r.start[0]:+.2f},{r.start[1]:+.2f}  {r.end[0]:+.2f},{r.end[1]:+.2f}  "
                  f"{r.samples:>7}  {r.mean_m * 100:+7.1f} cm  {r.abs_max_m * 100:7.1f} cm")
        print("\nwaypoint  closest approach")
        for i, miss in enumerate(waypoint_misses_m(poses, waypoints)):
            print(f"{i:>8}  {miss * 100:.1f} cm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
