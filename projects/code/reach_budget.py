"""Does the error budget of your reach fit inside the slack your gripper actually has?

    py projects/code/reach_budget.py --source "hand-eye=3.2" --source "object pose=4.6" \\
        --source "FK + backlash=2.6" --source "execution=1.5" \\
        --measured projects/evidence/P15/reaches.csv --column lateral_mm \\
        --stroke 45 --object-width 30 \\
        --max-median 8 --max-p90 12 --min-graspable-width 30 \\
        --title "P15 - 20 reaches on a 30 mm block"

Two halves of one comparison, from 15.07.

**The budget.** Error sources add in **quadrature**, not by addition:

    total = sqrt( sum of each source squared )

which has one consequence worth internalising before you spend an evening: attacking anything but
the largest term is nearly free of benefit. The ``halving`` column says exactly what you would get
for halving each source, so you can decide with a number instead of a hunch. 15.07 measured the
object pose as the biggest contributor on this hardware - bigger than the hand-eye calibration
that everyone re-runs first.

**The tolerance.** The jaws arrive fully open, so the lateral slack before one pad touches first is

    free stroke = (jaw stroke - object width) / 2

A 45 mm jaw on a 30 mm block gives +/- 7.5 mm. Put the two halves together and the verdict is
arithmetic: with a p90 lateral error of 9 mm, an open-loop reach onto that block works about four
times in five - which is the most expensive kind of working, because it convinces you the system
is fine and then fails in front of someone.

``--measured`` turns the same arithmetic on your own trials: give it a CSV column of lateral
errors in millimetres and it reports the median, the p90, and the **largest object you can grasp
open loop**, defined as ``stroke - 2 * p90``. If that number is smaller than the object you
actually want to pick, the answer is not a better reach - it is a bigger jaw, a smaller object, or
closing the loop with visual servoing (15.06).
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

from report import percentile_abs, read_column


@dataclass(frozen=True)
class Source:
    """One contribution to the budget, as a 1-sigma value in millimetres."""

    name: str
    sigma_mm: float

    def __post_init__(self) -> None:
        if self.sigma_mm < 0.0:
            raise ValueError(f"{self.name}: a sigma cannot be negative")

    @classmethod
    def parse(cls, text: str) -> "Source":
        """``"hand-eye=3.2"`` -> Source("hand-eye", 3.2)."""
        if "=" not in text:
            raise ValueError(f"expected 'name=mm', got {text!r}")
        name, _, value = text.partition("=")
        name = name.strip()
        if not name:
            raise ValueError(f"empty source name in {text!r}")
        try:
            return cls(name, float(value))
        except ValueError:
            raise ValueError(f"{text!r}: {value.strip()!r} is not a number") from None


@dataclass(frozen=True)
class Measured:
    """What your own trials did, in millimetres."""

    n: int
    median_mm: float
    p90_mm: float
    max_mm: float


def quadrature_mm(sources: list[Source]) -> float:
    """The combined 1-sigma. Independent errors add as squares, which is why the total is
    always less than the sum and always more than the biggest term."""
    return math.sqrt(sum(s.sigma_mm ** 2 for s in sources))


def share(sources: list[Source], name: str) -> float:
    """The fraction of the *variance* one source owns - the honest measure of "how much of it"."""
    total = sum(s.sigma_mm ** 2 for s in sources)
    if total <= 0.0:
        return 0.0
    return sum(s.sigma_mm ** 2 for s in sources if s.name == name) / total


def total_if_halved(sources: list[Source], name: str) -> float:
    """What the total becomes if you halve exactly this source and change nothing else."""
    halved = [Source(s.name, s.sigma_mm / 2.0) if s.name == name else s for s in sources]
    return quadrature_mm(halved)


def free_stroke_mm(stroke_mm: float, object_width_mm: float) -> float:
    """Total slack across the jaws. Half of it is the lateral error you may make."""
    if stroke_mm <= 0.0 or object_width_mm <= 0.0:
        raise ValueError("stroke and object width must both be > 0 mm")
    return stroke_mm - object_width_mm


def lateral_tolerance_mm(stroke_mm: float, object_width_mm: float) -> float:
    """``+/-`` this much before a pad touches the object instead of closing on it."""
    return free_stroke_mm(stroke_mm, object_width_mm) / 2.0


def max_graspable_width_mm(stroke_mm: float, p90_mm: float) -> float:
    """The widest object whose tolerance still covers your p90 error. Can come out negative,
    which means "this gripper cannot reliably grasp anything open loop at this error"."""
    if stroke_mm <= 0.0:
        raise ValueError("stroke must be > 0 mm")
    return stroke_mm - 2.0 * p90_mm


def measure(values_mm: list[float]) -> Measured:
    """Median, p90 and worst case of a list of lateral errors."""
    if not values_mm:
        raise ValueError("no measurements")
    magnitudes = [abs(v) for v in values_mm]
    return Measured(n=len(magnitudes),
                    median_mm=statistics.median(magnitudes),
                    p90_mm=percentile_abs(magnitudes, 90.0),
                    max_mm=max(magnitudes))


def budget_table(sources: list[Source]) -> str:
    total = quadrature_mm(sources)
    lines = ["| source | 1 sigma mm | share of variance | total if halved mm | improvement |",
             "|---|---|---|---|---|"]
    for s in sorted(sources, key=lambda s: -s.sigma_mm):
        halved = total_if_halved(sources, s.name)
        gain = (1.0 - halved / total) * 100.0 if total > 0 else 0.0
        lines.append(f"| {s.name} | {s.sigma_mm:.2f} | {share(sources, s.name) * 100:.0f}% | "
                     f"{halved:.2f} | {gain:.0f}% |")
    lines.append(f"| **all of it (quadrature)** | **{total:.2f}** | 100% | - | - |")
    lines.append(f"| (their plain sum, for contrast) | {sum(s.sigma_mm for s in sources):.2f} | - | - | - |")
    return "\n".join(lines)


def tolerance_table(stroke_mm: float, widths_mm: list[float], p90_mm: float | None) -> str:
    lines = ["| object width mm | free stroke mm | lateral tolerance mm | verdict |", "|---|---|---|---|"]
    for width in widths_mm:
        tolerance = lateral_tolerance_mm(stroke_mm, width)
        if p90_mm is None:
            verdict = "-"
        elif tolerance >= p90_mm:
            verdict = "works, with margin"
        elif tolerance >= p90_mm / 2.0:
            verdict = "works most of the time"
        else:
            verdict = "does not work open loop"
        lines.append(f"| {width:.0f} | {free_stroke_mm(stroke_mm, width):.1f} | "
                     f"+/- {tolerance:.1f} | {verdict} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", action="append", default=[], metavar="NAME=MM",
                    help="one error source, 1 sigma in mm; repeatable")
    ap.add_argument("--measured", type=Path, help="CSV of measured lateral errors")
    ap.add_argument("--column", default="lateral_mm")
    ap.add_argument("--stroke", type=float, default=45.0, help="jaw stroke in mm (SO-101: 45)")
    ap.add_argument("--object-width", type=float, action="append", default=[],
                    help="object width in mm; repeatable (default 20, 30, 40)")
    ap.add_argument("--title", default="")
    ap.add_argument("--max-median", type=float, default=None, help="mm")
    ap.add_argument("--max-p90", type=float, default=None, help="mm")
    ap.add_argument("--min-graspable-width", type=float, default=None, help="mm")
    ap.add_argument("--min-trials", type=int, default=None)
    args = ap.parse_args(argv)

    try:
        sources = [Source.parse(s) for s in args.source]
        measured = measure(read_column(args.measured, args.column)) if args.measured else None
    except (OSError, ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.title:
        print(f"# {args.title}\n")
    if sources:
        print(budget_table(sources))
        print()
    widths = args.object_width or [20.0, 30.0, 40.0]
    print(tolerance_table(args.stroke, widths, measured.p90_mm if measured else None))
    if measured:
        widest = max_graspable_width_mm(args.stroke, measured.p90_mm)
        print(f"\nmeasured over {measured.n} reach(es): median {measured.median_mm:.2f} mm, "
              f"p90 {measured.p90_mm:.2f} mm, worst {measured.max_mm:.2f} mm")
        print(f"widest object a {args.stroke:.0f} mm jaw grasps reliably open loop: "
              f"{widest:.1f} mm")
        if sources:
            predicted = quadrature_mm(sources)
            print(f"predicted 1 sigma from the budget: {predicted:.2f} mm "
                  f"({'consistent with' if abs(predicted - measured.median_mm) <= 0.5 * predicted else 'NOT consistent with'}"
                  f" the measured median - a large gap means an assumption about the robot is wrong)")

    checks: list[tuple[str, bool, str]] = []
    if measured is not None:
        if args.max_median is not None:
            checks.append(("median lateral error", measured.median_mm <= args.max_median,
                           f"{measured.median_mm:.2f} mm <= {args.max_median:g}"))
        if args.max_p90 is not None:
            checks.append(("p90 lateral error", measured.p90_mm <= args.max_p90,
                           f"{measured.p90_mm:.2f} mm <= {args.max_p90:g}"))
        if args.min_graspable_width is not None:
            widest = max_graspable_width_mm(args.stroke, measured.p90_mm)
            checks.append(("widest reliable object", widest >= args.min_graspable_width,
                           f"{widest:.1f} mm >= {args.min_graspable_width:g}"))
        if args.min_trials is not None:
            checks.append(("trials", measured.n >= args.min_trials,
                           f"{measured.n} >= {args.min_trials}"))
    elif any(v is not None for v in (args.max_median, args.max_p90, args.min_graspable_width)):
        print("error: those criteria need --measured", file=sys.stderr)
        return 2

    failures = 0
    for label, ok, detail in checks:
        failures += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {label}: {detail}")
    if failures:
        print(f"\n{failures} of {len(checks)} criteria failed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
