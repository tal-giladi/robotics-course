"""The robot says there are objects at these map coordinates. Were they really there?

    py projects/code/object_map_eval.py --truth projects/evidence/P13/truth.csv \\
        --objects projects/evidence/P13/object_map.csv \\
        --gate 0.6 --max-abs-mean 0.25 --max-worst 0.40 --min-recall 0.9 \\
        --max-phantoms 1 --min-hits 3 --max-sigma-ratio 3 \\
        --title "P13 - 6 objects, 3 rooms, 5 patrols"

Two CSVs of objects in the **map** frame, in metres::

    label,x,y,z                      (ground truth, one row per object, tape-measured)
    bottle,2.050,1.700,0.120

    label,x,y,z,hits,sigma_m         (what the robot's object map holds at the end of a run)
    bottle,1.871,1.546,0.117,17,0.081

Estimates are matched to truths **greedily, closest pair first, within one label**, and a match
counts when the two are within ``--gate`` metres. Each truth can be claimed once, so a robot that
puts three "bottle" entries around one real bottle scores one match and two **phantoms** - which is
the behaviour you want, because the duplicate-object failure of 13.16 is the one that quietly
destroys a semantic map and it must not be reported as three quarters of a success.

Four numbers, and they fail for different reasons:

* **position error** - how far each confirmed object is from where it really is. This is the
  number a Nav2 goal or an arm target inherits, and 13.16 says to expect 1-3 cm with a depth
  image and 20-30 cm from a known-size estimate. Read the *sign pattern* as well as the size: a
  consistent pull toward the robot is box bias, not noise, and averaging will never remove it.
* **recall** - how many real objects the map knows about at all. Recall is what patrolling buys.
* **phantoms** - entries with no object under them. Phantoms come from stale transforms
  (13.16-E4), from a confirmation rule that is too eager, and from a gate that is too tight.
* **sigma honesty** - the ratio of the actual error to the sigma the robot published. A map that
  says "bottle, +/- 2 cm" and is 24 cm out is not imprecise, it is *lying*, and every consumer
  downstream - the association gate, the planner, the agent - believes it. A ratio above about 3
  means the covariance is wrong, and 13.16's advice applies: fix the covariance, not the gate.

Nothing here knows about ROS. Export your object map to a CSV from whatever holds it
(13.16's ``object_map_node``, 19.08's belief store) and measure it against a tape.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

from report import percentile_abs


@dataclass(frozen=True)
class MapObject:
    """One object, true or believed, in the map frame. Metres."""

    label: str
    x: float
    y: float
    z: float = 0.0
    hits: int = 0
    sigma_m: float = 0.0

    def distance_to(self, other: "MapObject") -> float:
        return math.dist((self.x, self.y, self.z), (other.x, other.y, other.z))


@dataclass(frozen=True)
class Match:
    """A believed object paired with the real one it is closest to."""

    truth: MapObject
    estimate: MapObject
    error_m: float

    @property
    def sigma_ratio(self) -> float | None:
        """error / sigma. ``None`` when no sigma was published (which is itself a finding)."""
        if self.estimate.sigma_m <= 0.0:
            return None
        return self.error_m / self.estimate.sigma_m


@dataclass(frozen=True)
class MapReport:
    """What the object map got right, what it missed, and what it invented."""

    matches: tuple[Match, ...]
    missed: tuple[MapObject, ...]
    phantoms: tuple[MapObject, ...]
    gate_m: float

    @property
    def recall(self) -> float:
        total = len(self.matches) + len(self.missed)
        return len(self.matches) / total if total else 0.0

    @property
    def errors_m(self) -> list[float]:
        return [m.error_m for m in self.matches]

    def stat(self, name: str) -> float:
        """``abs_mean``, ``worst``, ``p95``, ``recall``, ``phantoms``, ``min_hits``, ``max_sigma_ratio``."""
        errors = self.errors_m
        if name == "recall":
            return self.recall
        if name == "phantoms":
            return float(len(self.phantoms))
        if name == "min_hits":
            return float(min((m.estimate.hits for m in self.matches), default=0))
        if name == "max_sigma_ratio":
            ratios = [r for m in self.matches if (r := m.sigma_ratio) is not None]
            return max(ratios) if ratios else 0.0
        if not errors:
            return 0.0
        if name == "abs_mean":
            return statistics.fmean(errors)
        if name == "worst":
            return max(errors)
        if name == "p95":
            return percentile_abs(errors, 95.0)
        raise ValueError(f"unknown statistic {name!r}")


def read_objects(path: Path) -> list[MapObject]:
    """Read ``label,x,y[,z][,hits][,sigma_m]``. Extra columns are ignored."""
    objects: list[MapObject] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path} is empty")
        missing = {"label", "x", "y"} - {f.strip() for f in reader.fieldnames}
        if missing:
            raise ValueError(f"{path} is missing column(s): {', '.join(sorted(missing))}")
        for row in reader:
            clean = {(k or "").strip(): (v or "").strip() for k, v in row.items()}
            if not clean.get("label"):
                continue
            objects.append(MapObject(
                label=clean["label"],
                x=float(clean["x"]),
                y=float(clean["y"]),
                z=float(clean.get("z") or 0.0),
                hits=int(float(clean.get("hits") or 0)),
                sigma_m=float(clean.get("sigma_m") or 0.0),
            ))
    if not objects:
        raise ValueError(f"{path} has no rows")
    return objects


def evaluate(truth: list[MapObject], estimates: list[MapObject], gate_m: float) -> MapReport:
    """Greedy closest-pair matching inside one label, each truth claimed at most once."""
    if gate_m <= 0.0:
        raise ValueError("gate must be > 0 m")
    pairs = sorted(
        ((t.distance_to(e), ti, ei) for ti, t in enumerate(truth) for ei, e in enumerate(estimates)
         if t.label == e.label and t.distance_to(e) <= gate_m),
        key=lambda p: (p[0], p[1], p[2]),
    )
    taken_truth: set[int] = set()
    taken_est: set[int] = set()
    matches: list[Match] = []
    for distance, ti, ei in pairs:
        if ti in taken_truth or ei in taken_est:
            continue
        taken_truth.add(ti)
        taken_est.add(ei)
        matches.append(Match(truth[ti], estimates[ei], distance))
    matches.sort(key=lambda m: (m.truth.label, m.truth.x, m.truth.y))
    return MapReport(
        matches=tuple(matches),
        missed=tuple(t for i, t in enumerate(truth) if i not in taken_truth),
        phantoms=tuple(e for i, e in enumerate(estimates) if i not in taken_est),
        gate_m=gate_m,
    )


def markdown_table(report: MapReport) -> str:
    """The per-object table you paste into the evidence write-up."""
    lines = ["| object | true (x, y) | believed (x, y) | error cm | hits | sigma cm | err/sigma |",
             "|---|---|---|---|---|---|---|"]
    for m in report.matches:
        ratio = m.sigma_ratio
        lines.append(
            f"| {m.truth.label} | ({m.truth.x:.2f}, {m.truth.y:.2f}) | "
            f"({m.estimate.x:.2f}, {m.estimate.y:.2f}) | {m.error_m * 100:.1f} | "
            f"{m.estimate.hits} | "
            f"{m.estimate.sigma_m * 100:.1f} | "
            f"{'-' if ratio is None else f'{ratio:.1f}'} |")
    for t in report.missed:
        lines.append(f"| {t.label} | ({t.x:.2f}, {t.y:.2f}) | **NOT IN THE MAP** | - | - | - | - |")
    for e in report.phantoms:
        lines.append(f"| {e.label} | **nothing there** | ({e.x:.2f}, {e.y:.2f}) | - | "
                     f"{e.hits} | {e.sigma_m * 100:.1f} | - |")
    return "\n".join(lines)


CRITERIA = (
    ("max_abs_mean", "abs_mean", "<=", "mean position error", "m"),
    ("max_worst", "worst", "<=", "worst position error", "m"),
    ("max_p95", "p95", "<=", "p95 position error", "m"),
    ("min_recall", "recall", ">=", "recall", ""),
    ("max_phantoms", "phantoms", "<=", "phantom objects", ""),
    ("min_hits", "min_hits", ">=", "hits on the least-seen object", ""),
    ("max_sigma_ratio", "max_sigma_ratio", "<=", "worst error/sigma", ""),
)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--truth", type=Path, required=True, help="CSV of the real objects")
    ap.add_argument("--objects", type=Path, required=True, help="CSV of what the robot believes")
    ap.add_argument("--gate", type=float, default=0.6, help="match radius in m (default 0.6)")
    ap.add_argument("--title", default="")
    for flag, _stat, _op, _label, _unit in CRITERIA:
        ap.add_argument(f"--{flag.replace('_', '-')}", type=float, default=None)
    args = ap.parse_args(argv)

    try:
        report = evaluate(read_objects(args.truth), read_objects(args.objects), args.gate)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.title:
        print(f"# {args.title}\n")
    print(markdown_table(report))
    print(f"\nrecall {report.recall * 100:.0f}% "
          f"({len(report.matches)} of {len(report.matches) + len(report.missed)}), "
          f"{len(report.phantoms)} phantom(s), gate {report.gate_m:.2f} m")

    failures = 0
    checked = 0
    for flag, stat, op, label, unit in CRITERIA:
        limit = getattr(args, flag)
        if limit is None:
            continue
        checked += 1
        value = report.stat(stat)
        ok = value <= limit if op == "<=" else value >= limit
        failures += not ok
        if unit:
            shown = f"{value:.3f} {unit}"
        elif stat in ("phantoms", "min_hits"):
            shown = f"{value:.0f}"
        else:
            shown = f"{value:.2f}"
        print(f"{'PASS' if ok else 'FAIL'}  {label} {op} {limit:g}: {shown}")
    if checked and failures:
        print(f"\n{failures} of {checked} criteria failed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
