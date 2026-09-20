"""Two logs of the same manoeuvre - how far apart did they drift, and when?

    py projects/code/traj_compare.py sim_square.csv real_square.csv --hz 10 \\
        --max-final 0.30 --max-heading 15 --title "P07 - 2 m square, sim vs real"

``path_error.py`` answers "how far was the robot from the path it was *told* to drive".
This module answers a different question that every project from P07 on asks: **two pose logs
of the same run, how far apart are they?**

* P07 compares Gazebo's odometry with the real robot's on the same commands.
* P08 compares raw wheel odometry with the EKF, or with a calibrated reference run.
* P10 compares AMCL's pose with the odometry-only pose, which is how you see the correction.

The two logs never share a clock or a starting pose - the simulation starts at second 0 in its
own world, the robot at 1789521600 on your floor - so ``--align pose`` (the default) rebases both
to *start at the origin, heading +x, at t = 0*. What is left is the difference in what the robot
did, which is the only comparable thing.

Divergence is reported as a distribution, not a single number: a pair of runs that agree for 30 s
and then split has the same mean as a pair that drifts apart steadily, and the two mean completely
different bugs. Read ``max`` and ``final`` together with ``rms``.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from path_error import Pose, path_length_m, read_poses, wrap_angle
from report import percentile_abs

ALIGNMENTS = ("none", "time", "pose")


@dataclass(frozen=True)
class Divergence:
    """How far two trajectories were apart, over the window they share."""

    samples: int
    window_s: float
    mean_m: float
    rms_m: float
    p95_m: float
    max_m: float
    final_m: float
    heading_rms_deg: float
    heading_max_deg: float
    final_heading_deg: float
    path_a_m: float
    path_b_m: float

    @property
    def final_percent_of_path(self) -> float:
        """The final gap as a percentage of the distance driven - the comparable number.

        A 12 cm divergence over 8 m of driving (1.5 %) and a 12 cm divergence over 0.5 m
        (24 %) are not the same result.
        """
        reference = max(self.path_a_m, self.path_b_m)
        return 100.0 * self.final_m / reference if reference > 0.0 else 0.0


def rebase(poses: list[Pose], alignment: str = "pose") -> list[Pose]:
    """Move a log into its own frame: ``time`` zeroes t, ``pose`` also zeroes the start pose.

    ``pose`` applies the inverse of the first pose to every sample (an SE(2) transform), so the
    run starts at (0, 0) pointing along +x. Two runs recorded in different worlds become
    comparable; two runs recorded in the *same* world lose the information that they started in
    different places, so use ``time`` when the absolute frame is the point (P10).
    """
    if alignment not in ALIGNMENTS:
        raise ValueError(f"unknown alignment {alignment!r}; use one of {', '.join(ALIGNMENTS)}")
    if not poses:
        raise ValueError("no poses")
    if alignment == "none":
        return list(poses)
    first = poses[0]
    if alignment == "time":
        return [Pose(p.t - first.t, p.x, p.y, p.theta) for p in poses]
    cos_t, sin_t = math.cos(first.theta), math.sin(first.theta)
    out = []
    for p in poses:
        dx, dy = p.x - first.x, p.y - first.y
        out.append(Pose(
            t=p.t - first.t,
            x=dx * cos_t + dy * sin_t,
            y=-dx * sin_t + dy * cos_t,
            theta=wrap_angle(p.theta - first.theta),
        ))
    return out


def interpolate(poses: list[Pose], t: float) -> Pose:
    """The pose at time ``t``, linearly between the two samples that bracket it.

    Heading is interpolated the short way around, so a log that crosses +-pi between two samples
    does not produce a 359-degree excursion.
    """
    if len(poses) < 2:
        raise ValueError("need at least two poses to interpolate")
    if t < poses[0].t or t > poses[-1].t:
        raise ValueError(f"t={t} is outside the log [{poses[0].t}, {poses[-1].t}]")
    low, high = 0, len(poses) - 1
    while high - low > 1:
        mid = (low + high) // 2
        if poses[mid].t <= t:
            low = mid
        else:
            high = mid
    a, b = poses[low], poses[high]
    span = b.t - a.t
    f = 0.0 if span <= 0.0 else (t - a.t) / span
    return Pose(
        t=t,
        x=a.x + f * (b.x - a.x),
        y=a.y + f * (b.y - a.y),
        theta=wrap_angle(a.theta + f * wrap_angle(b.theta - a.theta)),
    )


def common_grid(a: list[Pose], b: list[Pose], hz: float) -> list[float]:
    """Sample times inside the window both logs cover. Raises if they barely overlap."""
    if hz <= 0.0:
        raise ValueError("sample rate must be > 0")
    start = max(a[0].t, b[0].t)
    end = min(a[-1].t, b[-1].t)
    if end - start <= 0.0:
        raise ValueError("the two logs do not overlap in time (did you forget --align pose?)")
    n = int((end - start) * hz)
    if n < 1:
        raise ValueError(f"overlap is only {end - start:.3f} s; lower --hz")
    return [start + i / hz for i in range(n + 1)]


def compare(a: list[Pose], b: list[Pose], hz: float = 10.0,
            alignment: str = "pose") -> Divergence:
    """Resample both logs onto one grid and summarize the gap between them."""
    ra, rb = rebase(a, alignment), rebase(b, alignment)
    times = common_grid(ra, rb, hz)
    separations, headings = [], []
    for t in times:
        pa, pb = interpolate(ra, t), interpolate(rb, t)
        separations.append(math.hypot(pa.x - pb.x, pa.y - pb.y))
        headings.append(math.degrees(wrap_angle(pa.theta - pb.theta)))
    return Divergence(
        samples=len(times),
        window_s=times[-1] - times[0],
        mean_m=sum(separations) / len(separations),
        rms_m=math.sqrt(sum(s * s for s in separations) / len(separations)),
        p95_m=percentile_abs(separations, 95.0),
        max_m=max(separations),
        final_m=separations[-1],
        heading_rms_deg=math.sqrt(sum(h * h for h in headings) / len(headings)),
        heading_max_deg=max(abs(h) for h in headings),
        final_heading_deg=headings[-1],
        path_a_m=path_length_m(ra),
        path_b_m=path_length_m(rb),
    )


def markdown_table(d: Divergence, label_a: str = "A", label_b: str = "B") -> str:
    """The table a project asks you to paste into ``projects/evidence/<Pxx>/README.md``."""
    return "\n".join([
        f"{d.samples} samples over {d.window_s:.1f} s, "
        f"path {label_a} {d.path_a_m:.2f} m, path {label_b} {d.path_b_m:.2f} m",
        "",
        "| Quantity | Value |",
        "|---|---|",
        f"| mean separation | {d.mean_m * 100:.1f} cm |",
        f"| rms separation | {d.rms_m * 100:.1f} cm |",
        f"| p95 separation | {d.p95_m * 100:.1f} cm |",
        f"| worst separation | {d.max_m * 100:.1f} cm |",
        f"| final separation | {d.final_m * 100:.1f} cm ({d.final_percent_of_path:.1f} % of path) |",
        f"| rms heading difference | {d.heading_rms_deg:.1f} deg |",
        f"| worst heading difference | {d.heading_max_deg:.1f} deg |",
        f"| final heading difference | {d.final_heading_deg:+.1f} deg |",
    ])


def check(d: Divergence, max_rms_m: float | None = None, max_separation_m: float | None = None,
          max_final_m: float | None = None, max_heading_deg: float | None = None) -> list[tuple[str, str, bool]]:
    """Each limit you declared, the measured value, and whether it holds."""
    rows: list[tuple[str, str, bool]] = []
    if max_rms_m is not None:
        rows.append((f"rms separation <= {max_rms_m * 100:.0f} cm",
                     f"{d.rms_m * 100:.1f} cm", d.rms_m <= max_rms_m))
    if max_separation_m is not None:
        rows.append((f"worst separation <= {max_separation_m * 100:.0f} cm",
                     f"{d.max_m * 100:.1f} cm", d.max_m <= max_separation_m))
    if max_final_m is not None:
        rows.append((f"final separation <= {max_final_m * 100:.0f} cm",
                     f"{d.final_m * 100:.1f} cm", d.final_m <= max_final_m))
    if max_heading_deg is not None:
        rows.append((f"worst heading difference <= {max_heading_deg:g} deg",
                     f"{d.heading_max_deg:.1f} deg", d.heading_max_deg <= max_heading_deg))
    return rows


def verdict_table(rows: list[tuple[str, str, bool]]) -> str:
    lines = ["| Criterion | Measured | Verdict |", "|---|---|---|"]
    for detail, measured, passed in rows:
        lines.append(f"| {detail} | {measured} | {'pass' if passed else '**FAIL**'} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a", type=Path, help="first pose log, columns t,x,y[,theta]")
    ap.add_argument("b", type=Path, help="second pose log, same columns")
    ap.add_argument("--hz", type=float, default=10.0, help="comparison sample rate (default 10)")
    ap.add_argument("--align", default="pose", choices=ALIGNMENTS,
                    help="pose: rebase both to start at the origin (default); "
                         "time: only zero the clocks; none: compare raw coordinates")
    ap.add_argument("--max-rms", type=float, metavar="M", help="limit on the rms separation")
    ap.add_argument("--max-divergence", type=float, metavar="M", help="limit on the worst separation")
    ap.add_argument("--max-final", type=float, metavar="M", help="limit on the final separation")
    ap.add_argument("--max-heading", type=float, metavar="DEG",
                    help="limit on the worst heading difference")
    ap.add_argument("--title", default="", help="heading printed above the tables")
    args = ap.parse_args(argv)

    d = compare(read_poses(args.a), read_poses(args.b), args.hz, args.align)
    if args.title:
        print(f"### {args.title}\n")
    print(markdown_table(d, args.a.stem, args.b.stem))
    rows = check(d, args.max_rms, args.max_divergence, args.max_final, args.max_heading)
    if rows:
        print("\n" + verdict_table(rows))
    return 0 if all(passed for _, _, passed in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
