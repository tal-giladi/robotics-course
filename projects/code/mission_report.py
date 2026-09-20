"""Twenty navigation goals, how many arrived - and what the failures have in common.

    py projects/code/mission_report.py projects/evidence/P11/goals.csv \\
        --group goal --min-success-rate 0.9 --min-clearance 0.10 \\
        --max-duration 120 --forbid collision,intervention --min-trials 20 \\
        --title "P11 - go to the kitchen, 20 attempts"

Reads one row per attempt::

    trial,goal,outcome,duration_s,path_m,min_clearance_m,error_code
    1,kitchen,success,48.2,6.31,0.21,
    2,kitchen,aborted,92.0,3.10,0.08,105

``outcome`` is free text; ``success`` (case-insensitive) is the only value that counts as one.
Everything else is a failure and is listed by name, because "17 of 20" without a breakdown tells
you nothing about what to fix: eleven ``105 FAILED_TO_MAKE_PROGRESS`` on the same doorway is one
bug, whereas nine different codes is an unreliable robot.

The success rate is reported with a **Wilson 95 % interval**, which is the honest way to read a
small sample: 18/20 is a point estimate of 90 %, and the interval is 70-97 %. That is why the
projects ask for twenty attempts rather than five, and why a criterion of "90 % over 20 trials"
means the point estimate, not a claim about the robot's true reliability.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

SUCCESS = "success"


@dataclass(frozen=True)
class Attempt:
    """One goal the robot was sent to, and what happened."""

    trial: str
    group: str
    outcome: str
    duration_s: float | None = None
    path_m: float | None = None
    min_clearance_m: float | None = None
    error_code: str = ""

    @property
    def succeeded(self) -> bool:
        return self.outcome.strip().lower() == SUCCESS


@dataclass(frozen=True)
class GroupReport:
    """The numbers for one goal (or for everything, when ``group`` is ``all``)."""

    group: str
    n: int
    successes: int
    failures: Counter
    median_duration_s: float | None
    p95_duration_s: float | None
    median_path_m: float | None
    worst_clearance_m: float | None

    @property
    def success_rate(self) -> float:
        return self.successes / self.n if self.n else 0.0

    @property
    def interval(self) -> tuple[float, float]:
        return wilson_interval(self.successes, self.n)


def wilson_interval(successes: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """Wilson score interval for a proportion - sane at 20/20 and at 0/20, unlike mean +- z*se.

    ``wilson_interval(20, 20)`` is (0.839, 1.0), not (1.0, 1.0): twenty out of twenty is good
    evidence of 84 % or better, and no evidence at all of perfection.
    """
    if n <= 0:
        raise ValueError("need at least one trial")
    if not 0 <= successes <= n:
        raise ValueError("successes must be between 0 and n")
    p = successes / n
    denominator = 1.0 + z * z / n
    centre = (p + z * z / (2.0 * n)) / denominator
    margin = z * math.sqrt(p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denominator
    return max(0.0, centre - margin), min(1.0, centre + margin)


def _stat(values: list[float], fn) -> float | None:
    return fn(values) if values else None


def summarize(attempts: list[Attempt], group: str = "all") -> GroupReport:
    """Success rate, durations over the *successful* attempts, and the failure breakdown."""
    if not attempts:
        raise ValueError("no attempts")
    successes = [a for a in attempts if a.succeeded]
    failures: Counter = Counter()
    for a in attempts:
        if not a.succeeded:
            label = a.outcome.strip().lower() or "unknown"
            if a.error_code.strip():
                label += f" ({a.error_code.strip()})"
            failures[label] += 1
    # Durations are only meaningful for attempts that finished the job; a 92 s abort is not a
    # "slow success", it is a different event, and averaging the two hides both.
    durations = sorted(a.duration_s for a in successes if a.duration_s is not None)
    paths = [a.path_m for a in successes if a.path_m is not None]
    clearances = [a.min_clearance_m for a in attempts if a.min_clearance_m is not None]
    return GroupReport(
        group=group,
        n=len(attempts),
        successes=len(successes),
        failures=failures,
        median_duration_s=_stat(durations, statistics.median),
        p95_duration_s=_stat(durations, lambda v: percentile(v, 95.0)),
        median_path_m=_stat(paths, statistics.median),
        worst_clearance_m=_stat(clearances, min),
    )


def percentile(sorted_values: list[float], pct: float) -> float:
    """Linear-interpolated percentile of an already sorted list of signed values."""
    if not sorted_values:
        raise ValueError("no values")
    if len(sorted_values) == 1:
        return sorted_values[0]
    position = (len(sorted_values) - 1) * pct / 100.0
    low, high = math.floor(position), math.ceil(position)
    if low == high:
        return sorted_values[low]
    return sorted_values[low] + (sorted_values[high] - sorted_values[low]) * (position - low)


def _optional_float(row: dict[str, str], key: str) -> float | None:
    cell = (row.get(key) or "").strip()
    return float(cell) if cell else None


def read_attempts(path: Path, group_column: str | None = None) -> list[Attempt]:
    """Read the attempts CSV. Only ``outcome`` is required; everything else is optional."""
    attempts = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        if "outcome" not in fields:
            raise KeyError(f"no column 'outcome' in {path} (columns: {', '.join(fields)})")
        if group_column and group_column not in fields:
            raise KeyError(f"no column {group_column!r} in {path} (columns: {', '.join(fields)})")
        for i, row in enumerate(reader, start=1):
            attempts.append(Attempt(
                trial=(row.get("trial") or str(i)).strip(),
                group=(row.get(group_column) or "all").strip() if group_column else "all",
                outcome=(row.get("outcome") or "").strip(),
                duration_s=_optional_float(row, "duration_s"),
                path_m=_optional_float(row, "path_m"),
                min_clearance_m=_optional_float(row, "min_clearance_m"),
                error_code=(row.get("error_code") or "").strip(),
            ))
    if not attempts:
        raise ValueError(f"{path} has no rows")
    return attempts


def by_group(attempts: list[Attempt]) -> list[GroupReport]:
    """One report per group, in first-seen order, then one for everything."""
    order: list[str] = []
    buckets: dict[str, list[Attempt]] = {}
    for a in attempts:
        if a.group not in buckets:
            buckets[a.group] = []
            order.append(a.group)
        buckets[a.group].append(a)
    reports = [summarize(buckets[g], g) for g in order]
    if len(order) > 1:
        reports.append(summarize(attempts, "ALL"))
    return reports


def markdown_table(reports: list[GroupReport]) -> str:
    lines = ["| Goal | n | Success | Wilson 95 % | Median | p95 | Worst clearance |",
             "|---|---|---|---|---|---|---|"]
    for r in reports:
        low, high = r.interval
        median = f"{r.median_duration_s:.0f} s" if r.median_duration_s is not None else "-"
        p95 = f"{r.p95_duration_s:.0f} s" if r.p95_duration_s is not None else "-"
        clearance = (f"{r.worst_clearance_m * 100:.0f} cm"
                     if r.worst_clearance_m is not None else "-")
        lines.append(f"| {r.group} | {r.n} | {r.successes}/{r.n} ({100 * r.success_rate:.0f} %) "
                     f"| {100 * low:.0f}-{100 * high:.0f} % | {median} | {p95} | {clearance} |")
    return "\n".join(lines)


def failure_table(report: GroupReport) -> str:
    if not report.failures:
        return "No failures."
    lines = ["| Failure | Count |", "|---|---|"]
    for label, count in report.failures.most_common():
        lines.append(f"| {label} | {count} |")
    return "\n".join(lines)


def check(report: GroupReport, min_success_rate: float | None = None,
          min_clearance_m: float | None = None, max_duration_s: float | None = None,
          forbid: list[str] | None = None, min_trials: int | None = None) -> list[tuple[str, str, bool]]:
    """Every limit you declared, against the overall report."""
    rows: list[tuple[str, str, bool]] = []
    if min_trials is not None:
        rows.append((f"attempts >= {min_trials}", f"{report.n}", report.n >= min_trials))
    if min_success_rate is not None:
        rows.append((f"success rate >= {100 * min_success_rate:.0f} %",
                     f"{report.successes}/{report.n} = {100 * report.success_rate:.0f} %",
                     report.success_rate >= min_success_rate))
    if min_clearance_m is not None:
        worst = report.worst_clearance_m
        rows.append((f"min clearance >= {min_clearance_m * 100:.0f} cm",
                     f"{worst * 100:.0f} cm" if worst is not None else "not logged",
                     worst is not None and worst >= min_clearance_m))
    if max_duration_s is not None:
        p95 = report.p95_duration_s
        rows.append((f"p95 duration <= {max_duration_s:g} s",
                     f"{p95:.0f} s" if p95 is not None else "not logged",
                     p95 is not None and p95 <= max_duration_s))
    for banned in forbid or []:
        banned = banned.strip().lower()
        if not banned:
            continue
        hits = sum(count for label, count in report.failures.items() if label.startswith(banned))
        rows.append((f"zero '{banned}' outcomes", f"{hits}", hits == 0))
    return rows


def verdict_table(rows: list[tuple[str, str, bool]]) -> str:
    lines = ["| Criterion | Measured | Verdict |", "|---|---|---|"]
    for detail, measured, passed in rows:
        lines.append(f"| {detail} | {measured} | {'pass' if passed else '**FAIL**'} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="one row per goal attempt")
    ap.add_argument("--group", default=None, metavar="COLUMN",
                    help="split the table by this column, e.g. 'goal'")
    ap.add_argument("--min-success-rate", type=float, metavar="FRACTION",
                    help="e.g. 0.9 for 90 %%")
    ap.add_argument("--min-clearance", type=float, metavar="M",
                    help="the smallest min_clearance_m any attempt may report")
    ap.add_argument("--max-duration", type=float, metavar="S", help="limit on the p95 duration")
    ap.add_argument("--forbid", default="", metavar="A,B",
                    help="outcomes that must never occur, e.g. collision,intervention")
    ap.add_argument("--min-trials", type=int, metavar="N", help="how many attempts are required")
    ap.add_argument("--title", default="", help="heading printed above the tables")
    args = ap.parse_args(argv)

    attempts = read_attempts(args.csv, args.group)
    reports = by_group(attempts)
    overall = summarize(attempts, "ALL")

    if args.title:
        print(f"### {args.title}\n")
    print(markdown_table(reports))
    print("\n" + failure_table(overall))
    rows = check(overall, args.min_success_rate, args.min_clearance, args.max_duration,
                 args.forbid.split(","), args.min_trials)
    if rows:
        print("\n" + verdict_table(rows))
    return 0 if all(passed for _, _, passed in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
