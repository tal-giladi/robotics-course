"""Two different numbers, both called "the arm is accurate to N mm". Separate them, then fix one.

    py projects/code/arm_accuracy.py projects/evidence/P14/grid.csv --units mm \\
        --max-repeatability 3 --max-accuracy 15 --max-holdout 4 \\
        --title "P14 - 24 targets x 10 repeats, SO-101"

Reads one row per **measurement** (not per target)::

    target_x,target_y,target_z,x,y,z,trial
    150,0,20,141.5,-2.0,23.0,1
    150,0,20,141.0,-1.5,23.5,2

``target_*`` is what you commanded, ``x,y,z`` is where the gripper actually went, measured with a
rule or calipers in ``base_link``. Units are millimetres unless you pass ``--units m``.

The two numbers, from 14.10, and the reason they must never be averaged together:

* **repeatability** = max |p_i - mean(p)| over the repeats at one target. Scatter about the arm's
  own mean. It is *mechanical* - backlash, friction, servo dead-band - and no amount of software
  improves it. If yours is over 5 mm, stop reading and go tighten a servo horn.
* **accuracy** = |mean(p) - commanded|. The offset of that mean from the truth. It is a property
  of **your software**: a calibration error of one degree is about 7 mm at the tool, and it is
  the same 7 mm every time, so it can be measured and removed.

Then the removal, in the three levels of ambition 14.10 describes, each reported with mean and
worst-case residual:

* **raw** - what you measured.
* **offset** - one constant vector added to every command. Two lines of code, and it usually takes
  about two thirds of the error away.
* **affine** - ``p_corrected = A p + b``, twelve parameters fitted by least squares. It absorbs
  scale and skew as well as translation, which is what a per-joint angle bias actually looks like
  in task space.
* **affine, held out** - fitted on half the targets, scored on the other half. This is the only
  honest number in the table. A twelve-parameter fit scored on the points it was fitted to always
  looks good; if the held-out residual is much worse than the fitted one, you fitted noise.

No hardware, no ROS, no numpy: feed it a CSV you typed off a ruler, or one your ``--simulate`` run
produced with an injected joint bias.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

Point = tuple[float, float, float]
Pair = tuple[Point, Point]      # (measured, commanded)

UNITS = {"mm": 1.0, "m": 1000.0, "cm": 10.0}


@dataclass(frozen=True)
class TargetReport:
    """One commanded point, and what the arm did when asked for it ``n`` times."""

    target: Point
    n: int
    mean: Point
    accuracy_mm: float
    repeatability_mm: float

    @property
    def dominant(self) -> str:
        """Which of the two numbers you should be working on for this target."""
        return "accuracy" if self.accuracy_mm >= self.repeatability_mm else "repeatability"


@dataclass(frozen=True)
class Correction:
    """A fitted correction and how well it did. ``residuals_mm`` is one distance per point."""

    name: str
    residuals_mm: tuple[float, ...]

    @property
    def mean_mm(self) -> float:
        return statistics.fmean(self.residuals_mm) if self.residuals_mm else 0.0

    @property
    def max_mm(self) -> float:
        return max(self.residuals_mm, default=0.0)


def _sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def read_measurements(path: Path, units: str = "mm") -> list[tuple[Point, Point]]:
    """Read ``target_x,target_y,target_z,x,y,z`` into (target, measured) pairs, in **mm**."""
    if units not in UNITS:
        raise ValueError(f"unknown units {units!r}; use one of {', '.join(UNITS)}")
    scale = UNITS[units]
    needed = ["target_x", "target_y", "target_z", "x", "y", "z"]
    rows: list[tuple[Point, Point]] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path} is empty")
        present = {(f or "").strip() for f in reader.fieldnames}
        missing = [c for c in needed if c not in present]
        if missing:
            raise ValueError(f"{path} is missing column(s): {', '.join(missing)}")
        for row in reader:
            clean = {(k or "").strip(): (v or "").strip() for k, v in row.items()}
            if not clean.get("x"):
                continue
            target = tuple(float(clean[f"target_{a}"]) * scale for a in "xyz")
            measured = tuple(float(clean[a]) * scale for a in "xyz")
            rows.append((target, measured))    # type: ignore[arg-type]
    if not rows:
        raise ValueError(f"{path} has no measurement rows")
    return rows


def accuracy_and_repeatability(measured: list[Point], target: Point) -> tuple[float, float, Point]:
    """The two numbers of 14.10, plus the mean point they are computed from. All in mm."""
    if not measured:
        raise ValueError("no measurements for this target")
    mean = tuple(statistics.fmean(p[i] for p in measured) for i in range(3))
    accuracy = math.dist(mean, target)
    repeatability = max(math.dist(p, mean) for p in measured)
    return accuracy, repeatability, mean    # type: ignore[return-value]


def per_target(rows: list[tuple[Point, Point]]) -> list[TargetReport]:
    """Group the measurements by commanded point, in the order the targets first appear."""
    order: list[Point] = []
    groups: dict[Point, list[Point]] = {}
    for target, measured in rows:
        key = tuple(round(v, 6) for v in target)    # type: ignore[assignment]
        if key not in groups:
            groups[key] = []
            order.append(key)                       # type: ignore[arg-type]
        groups[key].append(measured)
    reports = []
    for target in order:
        accuracy, repeatability, mean = accuracy_and_repeatability(groups[target], target)
        reports.append(TargetReport(target, len(groups[target]), mean, accuracy, repeatability))
    return reports


def _solve(matrix: list[list[float]], rhs: list[float]) -> list[float]:
    """Gaussian elimination with partial pivoting. Small, dense, and numpy-free on purpose."""
    n = len(rhs)
    aug = [row[:] + [rhs[i]] for i, row in enumerate(matrix)]
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(aug[r][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise ValueError("the fit is under-determined: use more, better-spread targets")
        aug[col], aug[pivot] = aug[pivot], aug[col]
        for r in range(col + 1, n):
            factor = aug[r][col] / aug[col][col]
            for c in range(col, n + 1):
                aug[r][c] -= factor * aug[col][c]
    out = [0.0] * n
    for r in reversed(range(n)):
        total = aug[r][n] - sum(aug[r][c] * out[c] for c in range(r + 1, n))
        out[r] = total / aug[r][r]
    return out


def fit_offset(pairs: list[Pair]) -> Point:
    """The constant vector that, added to a measured point, lands on the commanded one."""
    if not pairs:
        raise ValueError("fit_offset needs at least one pair")
    return tuple(statistics.fmean(c[i] - m[i] for m, c in pairs) for i in range(3))   # type: ignore[return-value]


def apply_offset(point: Point, offset: Point) -> Point:
    return (point[0] + offset[0], point[1] + offset[1], point[2] + offset[2])


def fit_affine(pairs: list[Pair]) -> tuple[tuple[float, ...], ...]:
    """Least-squares ``commanded ~ A measured + b``, returned as three rows of (a, b, c, d).

    Twelve parameters, so it needs at least four well-spread targets - and 14.10's warning
    applies: fit it on some points and score it on others.
    """
    if len(pairs) < 4:
        raise ValueError("fit_affine needs at least 4 pairs (12 parameters)")
    design = [[m[0], m[1], m[2], 1.0] for m, _ in pairs]
    normal = [[sum(row[i] * row[j] for row in design) for j in range(4)] for i in range(4)]
    rows = []
    for axis in range(3):
        rhs = [sum(row[i] * c[axis] for row, (_, c) in zip(design, pairs)) for i in range(4)]
        rows.append(tuple(_solve([r[:] for r in normal], rhs)))
    return tuple(rows)


def apply_affine(point: Point, affine: tuple[tuple[float, ...], ...]) -> Point:
    return tuple(r[0] * point[0] + r[1] * point[1] + r[2] * point[2] + r[3] for r in affine)  # type: ignore[return-value]


def residuals(pairs: list[Pair], correct) -> list[float]:
    """Distance, per pair, between the corrected measurement and the commanded point."""
    return [math.dist(correct(m), c) for m, c in pairs]


def holdout_split(pairs: list[Pair], seed: int = 0) -> tuple[list[Pair], list[Pair]]:
    """Split the targets into a fit half and a test half, with a fixed pseudo-random permutation.

    Deterministic, so your number is reproducible - but *not* "every other row", which looks
    tidier and is a trap: on a grid whose fastest-varying axis has two values, taking every
    other target gives you a fit set with a constant z, twelve parameters and no information
    about one of them. The fit then fails outright (which is the good case) or is wildly
    extrapolating (which is not).
    """
    order = list(range(len(pairs)))
    random.Random(seed).shuffle(order)
    half = len(pairs) // 2
    fit = [pairs[i] for i in sorted(order[:half])]
    test = [pairs[i] for i in sorted(order[half:])]
    return fit, test


def corrections(reports: list[TargetReport]) -> list[Correction]:
    """The four rows of the correction table, from one (measured mean -> commanded) pair per target."""
    pairs: list[Pair] = [(r.mean, r.target) for r in reports]
    out = [Correction("raw", tuple(residuals(pairs, lambda p: p)))]
    offset = fit_offset(pairs)
    out.append(Correction("constant offset", tuple(residuals(pairs, lambda p: apply_offset(p, offset)))))
    if len(pairs) >= 4:
        affine = fit_affine(pairs)
        out.append(Correction("affine fit", tuple(residuals(pairs, lambda p: apply_affine(p, affine)))))
        fit, test = holdout_split(pairs)
        if len(fit) >= 4 and test:
            held = fit_affine(fit)
            out.append(Correction("affine, held out",
                                  tuple(residuals(test, lambda p: apply_affine(p, held)))))
    return out


def markdown_tables(reports: list[TargetReport], fits: list[Correction]) -> str:
    lines = ["| target (x, y, z) mm | n | accuracy mm | repeatability mm | dominant |",
             "|---|---|---|---|---|"]
    for r in reports:
        lines.append(f"| ({r.target[0]:.0f}, {r.target[1]:.0f}, {r.target[2]:.0f}) | {r.n} | "
                     f"{r.accuracy_mm:.2f} | {r.repeatability_mm:.2f} | {r.dominant} |")
    lines += ["", "| correction | points | mean residual mm | worst mm |", "|---|---|---|---|"]
    for f in fits:
        lines.append(f"| {f.name} | {len(f.residuals_mm)} | {f.mean_mm:.2f} | {f.max_mm:.2f} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--units", default="mm", choices=sorted(UNITS))
    ap.add_argument("--title", default="")
    ap.add_argument("--max-repeatability", type=float, default=None, help="mm, worst target")
    ap.add_argument("--max-accuracy", type=float, default=None, help="mm, worst target, uncorrected")
    ap.add_argument("--max-holdout", type=float, default=None, help="mm, mean held-out residual")
    ap.add_argument("--min-repeats", type=int, default=None, help="minimum repeats per target")
    args = ap.parse_args(argv)

    try:
        rows = read_measurements(args.csv, args.units)
        reports = per_target(rows)
        fits = corrections(reports)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    if args.title:
        print(f"# {args.title}\n")
    print(markdown_tables(reports, fits))
    worst_rep = max(r.repeatability_mm for r in reports)
    worst_acc = max(r.accuracy_mm for r in reports)
    held = next((f for f in fits if f.name == "affine, held out"), None)
    print(f"\n{len(reports)} target(s), {len(rows)} measurement(s); "
          f"worst repeatability {worst_rep:.2f} mm, worst accuracy {worst_acc:.2f} mm")

    checks: list[tuple[str, bool, str]] = []
    if args.max_repeatability is not None:
        checks.append(("worst repeatability", worst_rep <= args.max_repeatability,
                       f"{worst_rep:.2f} mm <= {args.max_repeatability:g}"))
    if args.max_accuracy is not None:
        checks.append(("worst accuracy (uncorrected)", worst_acc <= args.max_accuracy,
                       f"{worst_acc:.2f} mm <= {args.max_accuracy:g}"))
    if args.max_holdout is not None:
        value = held.mean_mm if held else float("inf")
        checks.append(("held-out mean residual", held is not None and value <= args.max_holdout,
                       f"{value:.2f} mm <= {args.max_holdout:g}" if held else "no held-out fit (need >= 8 targets)"))
    if args.min_repeats is not None:
        fewest = min(r.n for r in reports)
        checks.append(("repeats per target", fewest >= args.min_repeats,
                       f"{fewest} >= {args.min_repeats}"))
    failures = 0
    for label, ok, detail in checks:
        failures += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {label}: {detail}")
    if failures:
        print(f"\n{failures} of {len(checks)} criteria failed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
