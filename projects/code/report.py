"""Turn a column of measured trial results into the acceptance table a project asks for.

    py projects/code/report.py projects/evidence/P05/square_ccw.csv --column closing_cm \\
        --criterion "abs_mean<=15" --criterion "sd<=7" --criterion "abs_max<=25" \\
        --title "P05 - 2 m square, counter-clockwise"

Every project in this course asks for numbers with tolerances over a stated number of trials.
Five runs are a measurement; one run is an anecdote. This module does the boring half of that:
it reads the column, computes the statistics the criteria name, and prints a markdown table you
can paste into ``projects/evidence/<Pxx>/README.md``. Exit code 1 if any criterion fails, so it
can also be the last line of a test script.

Statistics available to ``--criterion``:

===========  ==========================================================
``n``        how many trials
``mean``     signed mean (use it to see *bias*)
``abs_mean`` mean of the absolute values (use it for "within X" limits)
``sd``       sample standard deviation (use it to see *spread*); 0 when n < 2
``min``      smallest signed value
``max``      largest signed value
``abs_max``  worst case, ignoring sign
``p95``      95th percentile of the absolute values, linear interpolation
``range``    max - min
===========  ==========================================================

Bias and spread are different problems with different fixes: a non-zero ``mean`` is a wrong
constant and calibration removes it (09.05); a large ``sd`` is slip and surface and calibration
does nothing for it.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

OPERATORS = ("<=", ">=")
STATS = ("n", "mean", "abs_mean", "sd", "min", "max", "abs_max", "p95", "range")


@dataclass(frozen=True)
class Criterion:
    """One acceptance criterion: a statistic, a comparison and a limit."""

    stat: str
    op: str
    limit: float

    def __post_init__(self) -> None:
        if self.stat not in STATS:
            raise ValueError(f"unknown statistic {self.stat!r}; known: {', '.join(STATS)}")
        if self.op not in OPERATORS:
            raise ValueError(f"unknown operator {self.op!r}; use <= or >=")

    @classmethod
    def parse(cls, text: str) -> "Criterion":
        """``Criterion.parse("abs_mean<=15")`` -> ``Criterion("abs_mean", "<=", 15.0)``."""
        for op in OPERATORS:
            if op in text:
                stat, _, limit = text.partition(op)
                return cls(stat.strip(), op, float(limit.strip()))
        raise ValueError(f"cannot parse criterion {text!r}; expected e.g. 'abs_mean<=15'")

    def holds(self, value: float) -> bool:
        return value <= self.limit if self.op == "<=" else value >= self.limit

    def __str__(self) -> str:
        return f"{self.stat} {self.op} {self.limit:g}"


@dataclass(frozen=True)
class Result:
    """The outcome of checking one criterion against one set of trials."""

    criterion: Criterion
    value: float
    passed: bool


def percentile_abs(values: list[float], pct: float) -> float:
    """Linear-interpolated percentile of the absolute values (pct in 0..100)."""
    if not values:
        raise ValueError("no values")
    ordered = sorted(abs(v) for v in values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * pct / 100.0
    low = math.floor(position)
    high = math.ceil(position)
    if low == high:
        return ordered[low]
    return ordered[low] + (ordered[high] - ordered[low]) * (position - low)


def summarize(values: list[float]) -> dict[str, float]:
    """Every statistic ``Criterion`` can name, for one column of trial results."""
    if not values:
        raise ValueError("no values: a criterion needs at least one trial")
    absolute = [abs(v) for v in values]
    return {
        "n": float(len(values)),
        "mean": statistics.fmean(values),
        "abs_mean": statistics.fmean(absolute),
        "sd": statistics.stdev(values) if len(values) > 1 else 0.0,
        "min": min(values),
        "max": max(values),
        "abs_max": max(absolute),
        "p95": percentile_abs(values, 95.0),
        "range": max(values) - min(values),
    }


def evaluate(values: list[float], criteria: list[Criterion]) -> list[Result]:
    """Check every criterion against the trials. Order is preserved."""
    stats = summarize(values)
    return [Result(c, stats[c.stat], c.holds(stats[c.stat])) for c in criteria]


def read_column(path: Path, column: str) -> list[float]:
    """Read one named column of a CSV as floats, skipping blank cells."""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None or column not in reader.fieldnames:
            available = ", ".join(reader.fieldnames or [])
            raise KeyError(f"no column {column!r} in {path} (columns: {available})")
        out = []
        for row in reader:
            cell = (row[column] or "").strip()
            if cell:
                out.append(float(cell))
    return out


def markdown_table(values: list[float], results: list[Result], unit: str = "") -> str:
    """The acceptance table: every criterion, the measured value, pass or fail."""
    suffix = f" {unit}" if unit else ""
    stats = summarize(values)
    lines = [
        f"n = {int(stats['n'])} trials, mean {stats['mean']:+.3g}{suffix}, "
        f"sd {stats['sd']:.3g}{suffix}, worst |x| {stats['abs_max']:.3g}{suffix}",
        "",
        "| Criterion | Measured | Verdict |",
        "|---|---|---|",
    ]
    for r in results:
        verdict = "pass" if r.passed else "**FAIL**"
        # "n" counts trials; it is not measured in the column's unit.
        shown = f"{int(r.value)}" if r.criterion.stat == "n" else f"{r.value:.3g}{suffix}"
        lines.append(f"| {r.criterion} | {shown} | {verdict} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="CSV file with one row per trial")
    ap.add_argument("--column", required=True, help="which column holds the measured error")
    ap.add_argument("--criterion", action="append", default=[], metavar="EXPR",
                    help="e.g. 'abs_mean<=15' (repeat for each criterion)")
    ap.add_argument("--unit", default="", help="unit printed next to every number, e.g. cm")
    ap.add_argument("--title", default="", help="heading printed above the table")
    args = ap.parse_args(argv)

    values = read_column(args.csv, args.column)
    criteria = [Criterion.parse(c) for c in args.criterion]
    results = evaluate(values, criteria) if criteria else []
    if args.title:
        print(f"### {args.title}\n")
    print(markdown_table(values, results, args.unit))
    return 0 if all(r.passed for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
