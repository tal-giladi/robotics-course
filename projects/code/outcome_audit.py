"""The robot says it succeeded. The world disagrees. How often, and in which direction?

    py projects/code/outcome_audit.py projects/evidence/P16/attempts.csv \\
        --group object --min-verified-success 0.8 --max-false-success 0 \\
        --min-trials 20 --max-mean-duration 90 \\
        --title "P16 - 20 picks, 4 objects, verified by hand"

Reads one row per attempt, with **two** outcome columns::

    trial,object,reported,verified,phase,duration_s,retries
    1,block,success,success,,52.4,0
    2,block,success,failure,transport,61.0,1
    3,can,failure,failure,grasp,44.8,2

* ``reported`` is what the pipeline (or the agent) says happened - its own return code.
* ``verified`` is what you or an independent check found in the world afterwards: the object is
  on the destination surface, or it is not.

``mission_report.py`` answers "how many of the twenty arrived". This module answers a different
and more uncomfortable question that every project from P16 on has to ask, because 15.08 and
19.07 both end at the same place: **a component grading its own homework will pass.** The four
cells of the agreement table are not equally bad:

* **true success** - it worked and it knew.
* **false success** - it failed and reported success. The worst cell on the table by a distance:
  the pipeline goes back for the next object, the agent reports "done", a dock-and-charge runs
  with something still in the gripper, and your evaluation harness scores the run as a win. One
  of these in twenty is a broken verifier, not bad luck.
* **false failure** - it worked and reported failure. Cheap, annoying, and it burns the retry
  budget on a world that was already correct.
* **true failure** - it failed and said so. This is a *working* system, and the number you should
  be willing to trade for: a pipeline that succeeds 80 % of the time and is right about which
  80 % is far more useful than one that succeeds 90 % and cannot tell you which.

The verified success rate carries a **Wilson 95 % interval** (from ``mission_report.py``), which
is why these projects ask for twenty attempts and not five: 16/20 is a point estimate of 80 % and
an interval of 58-92 %.

``--group`` splits by any column (the object, the surface, the task, the seed), because "17 of 20"
hides "the can failed five times out of five and everything else was perfect", and those are
completely different bugs.
"""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

from mission_report import wilson_interval

SUCCESS_WORDS = {"success", "succeeded", "ok", "pass", "passed", "true", "yes", "1"}


@dataclass(frozen=True)
class Attempt:
    """One attempt, as the system saw it and as the world saw it."""

    trial: str
    group: str
    reported: str
    verified: str
    phase: str = ""
    duration_s: float | None = None
    retries: int | None = None

    @property
    def reported_success(self) -> bool:
        return self.reported.strip().lower() in SUCCESS_WORDS

    @property
    def verified_success(self) -> bool:
        return self.verified.strip().lower() in SUCCESS_WORDS

    @property
    def cell(self) -> str:
        """Which of the four cells of the agreement table this attempt lands in."""
        if self.verified_success:
            return "true success" if self.reported_success else "false failure"
        return "false success" if self.reported_success else "true failure"


@dataclass(frozen=True)
class GroupAudit:
    """The agreement table and the timings for one group (or for everything)."""

    group: str
    attempts: tuple[Attempt, ...]

    @property
    def n(self) -> int:
        return len(self.attempts)

    def count(self, cell: str) -> int:
        return sum(1 for a in self.attempts if a.cell == cell)

    @property
    def verified_successes(self) -> int:
        return sum(1 for a in self.attempts if a.verified_success)

    @property
    def verified_rate(self) -> float:
        return self.verified_successes / self.n if self.n else 0.0

    @property
    def reported_rate(self) -> float:
        return sum(1 for a in self.attempts if a.reported_success) / self.n if self.n else 0.0

    @property
    def interval(self) -> tuple[float, float]:
        return wilson_interval(self.verified_successes, self.n)

    @property
    def mean_duration_s(self) -> float | None:
        values = [a.duration_s for a in self.attempts if a.duration_s is not None]
        return statistics.fmean(values) if values else None

    @property
    def mean_retries(self) -> float | None:
        values = [a.retries for a in self.attempts if a.retries is not None]
        return statistics.fmean(values) if values else None

    @property
    def failure_phases(self) -> Counter:
        """Where the *genuine* failures happened. Unlabelled ones are counted as 'unrecorded',
        because a failure with no phase is a failure you cannot fix."""
        return Counter(a.phase.strip() or "unrecorded"
                       for a in self.attempts if not a.verified_success)


def read_attempts(path: Path, group_column: str | None = None) -> list[Attempt]:
    """Read ``trial,reported,verified`` plus whatever optional columns are present."""
    attempts: list[Attempt] = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"{path} is empty")
        present = {(f or "").strip() for f in reader.fieldnames}
        missing = {"reported", "verified"} - present
        if missing:
            raise ValueError(f"{path} is missing column(s): {', '.join(sorted(missing))}")
        if group_column and group_column not in present:
            raise ValueError(f"{path} has no column {group_column!r}; it has: {', '.join(sorted(present))}")
        for index, row in enumerate(reader, start=1):
            clean = {(k or "").strip(): (v or "").strip() for k, v in row.items()}
            if not clean.get("reported") and not clean.get("verified"):
                continue
            attempts.append(Attempt(
                trial=clean.get("trial") or str(index),
                group=clean.get(group_column, "") if group_column else "all",
                reported=clean.get("reported", ""),
                verified=clean.get("verified", ""),
                phase=clean.get("phase", ""),
                duration_s=float(clean["duration_s"]) if clean.get("duration_s") else None,
                retries=int(float(clean["retries"])) if clean.get("retries") else None,
            ))
    if not attempts:
        raise ValueError(f"{path} has no attempt rows")
    return attempts


def audit(attempts: list[Attempt], by_group: bool = False) -> list[GroupAudit]:
    """Overall first, then one row per group in the order the groups first appear."""
    out = [GroupAudit("all", tuple(attempts))]
    if by_group:
        order: list[str] = []
        for a in attempts:
            if a.group not in order:
                order.append(a.group)
        out += [GroupAudit(g, tuple(a for a in attempts if a.group == g)) for g in order]
    return out


CELLS = ("true success", "false success", "false failure", "true failure")


def markdown_table(audits: list[GroupAudit]) -> str:
    lines = ["| group | n | verified | reported | 95% interval | true succ | **false succ** | false fail | true fail |",
             "|---|---|---|---|---|---|---|---|---|"]
    for a in audits:
        low, high = a.interval
        lines.append(
            f"| {a.group} | {a.n} | {a.verified_rate * 100:.0f}% | {a.reported_rate * 100:.0f}% | "
            f"{low * 100:.0f}-{high * 100:.0f}% | " +
            " | ".join(str(a.count(c)) for c in CELLS) + " |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path)
    ap.add_argument("--group", default=None, help="column to break the table down by")
    ap.add_argument("--title", default="")
    ap.add_argument("--min-verified-success", type=float, default=None, help="0-1, overall")
    ap.add_argument("--max-false-success", type=int, default=None, help="count, overall")
    ap.add_argument("--min-trials", type=int, default=None)
    ap.add_argument("--max-mean-duration", type=float, default=None, help="seconds")
    ap.add_argument("--max-mean-retries", type=float, default=None)
    args = ap.parse_args(argv)

    try:
        attempts = read_attempts(args.csv, args.group)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    audits = audit(attempts, by_group=bool(args.group))
    overall = audits[0]
    if args.title:
        print(f"# {args.title}\n")
    print(markdown_table(audits))

    phases = overall.failure_phases
    if phases:
        print("\nfailures by phase: " +
              ", ".join(f"{name} x{count}" for name, count in phases.most_common()))
    if overall.mean_duration_s is not None:
        print(f"mean duration {overall.mean_duration_s:.1f} s")
    if overall.mean_retries is not None:
        print(f"mean retries {overall.mean_retries:.2f}")

    checks: list[tuple[str, bool, str]] = []
    if args.min_trials is not None:
        checks.append(("trials", overall.n >= args.min_trials, f"{overall.n} >= {args.min_trials}"))
    if args.min_verified_success is not None:
        checks.append(("verified success rate", overall.verified_rate >= args.min_verified_success,
                       f"{overall.verified_rate:.2f} >= {args.min_verified_success:g}"))
    if args.max_false_success is not None:
        value = overall.count("false success")
        checks.append(("false successes", value <= args.max_false_success,
                       f"{value} <= {args.max_false_success}"))
    if args.max_mean_duration is not None:
        value = overall.mean_duration_s
        checks.append(("mean duration", value is not None and value <= args.max_mean_duration,
                       f"{value:.1f} s <= {args.max_mean_duration:g}" if value is not None
                       else "no duration_s column"))
    if args.max_mean_retries is not None:
        value = overall.mean_retries
        checks.append(("mean retries", value is not None and value <= args.max_mean_retries,
                       f"{value:.2f} <= {args.max_mean_retries:g}" if value is not None
                       else "no retries column"))

    failures = 0
    for label, ok, detail in checks:
        failures += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {label}: {detail}")
    if failures:
        print(f"\n{failures} of {len(checks)} criteria failed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
