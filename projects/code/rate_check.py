"""Did every topic publish as often as its contract promised - and did it ever stop?

    python projects/code/rate_check.py projects/evidence/P06/stamps.csv \\
        --expect /odom=20 --expect /joint_states=20 --expect /battery_state=1:2000 \\
        --max-gap 150 --title "P06 - 5 minute bring-up"

Reads a CSV of message arrival times::

    topic,t
    /odom,0.0000
    /odom,0.0502
    /battery_state,0.0510

and reports, per topic, the mean rate and the **gap distribution** - because "20.1 Hz average"
is exactly what you see when a node publishes at 40 Hz for half the run and stalls for the other
half. A mean rate cannot fail; a p99 gap can.

``--expect <topic>=<hz>[:<max_gap_ms>]`` turns that into a pass/fail: the mean rate must be within
``--tolerance`` of the promised rate, and no gap may exceed the limit (``--max-gap`` if the
expectation does not give its own). Exit code 1 if anything fails, so this can be the last line of
a bring-up script.

Producing the CSV is deliberately not this module's job: any recorder that can write two columns
will do. P06 shows a nine-line rclpy node that subscribes with a generic callback and appends a
row per message; ``ros2 bag`` exports and a serial telemetry log work just as well.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

from report import percentile_abs


@dataclass(frozen=True)
class TopicRate:
    """What one topic actually did, as opposed to what its documentation says."""

    topic: str
    n: int
    duration_s: float
    mean_hz: float
    p50_gap_ms: float
    p95_gap_ms: float
    p99_gap_ms: float
    max_gap_ms: float

    def gaps_over(self, limit_ms: float, gaps: list[float]) -> int:
        """How many gaps exceeded ``limit_ms``. Pass the same list you summarized."""
        return sum(1 for g in gaps if g > limit_ms)


@dataclass(frozen=True)
class Expectation:
    """One topic's contract: a nominal rate and, optionally, its own gap limit."""

    topic: str
    hz: float
    max_gap_ms: float | None = None

    @classmethod
    def parse(cls, text: str) -> "Expectation":
        """``Expectation.parse("/odom=20:150")`` -> rate 20 Hz, no gap above 150 ms."""
        topic, sep, rest = text.partition("=")
        if not sep or not topic.strip():
            raise ValueError(f"cannot parse expectation {text!r}; expected e.g. '/odom=20'")
        hz_text, _, gap_text = rest.partition(":")
        hz = float(hz_text)
        if hz <= 0.0:
            raise ValueError(f"expected rate must be > 0 in {text!r}")
        gap = float(gap_text) if gap_text.strip() else None
        if gap is not None and gap <= 0.0:
            raise ValueError(f"gap limit must be > 0 in {text!r}")
        return cls(topic.strip(), hz, gap)


@dataclass(frozen=True)
class Verdict:
    """One line of the acceptance table."""

    topic: str
    detail: str
    measured: str
    passed: bool


def gaps_ms(times: list[float]) -> list[float]:
    """Milliseconds between consecutive messages. Timestamps are sorted first.

    Out-of-order arrivals are a real thing on a lossy link; sorting means a reordered pair shows
    up as one normal gap rather than one negative and one double gap.
    """
    if len(times) < 2:
        raise ValueError("need at least two timestamps to measure a gap")
    ordered = sorted(times)
    return [(b - a) * 1000.0 for a, b in zip(ordered, ordered[1:])]


def summarize(topic: str, times: list[float]) -> tuple[TopicRate, list[float]]:
    """Rate statistics for one topic, plus the raw gaps so you can ask further questions."""
    gaps = gaps_ms(times)
    duration = (max(times) - min(times))
    return TopicRate(
        topic=topic,
        n=len(times),
        duration_s=duration,
        # n-1 intervals over the observed window: the rate you would have measured with a stopwatch.
        mean_hz=(len(times) - 1) / duration if duration > 0.0 else 0.0,
        p50_gap_ms=percentile_abs(gaps, 50.0),
        p95_gap_ms=percentile_abs(gaps, 95.0),
        p99_gap_ms=percentile_abs(gaps, 99.0),
        max_gap_ms=max(gaps),
    ), gaps


def read_stamps(path: Path) -> dict[str, list[float]]:
    """Read ``topic,t`` rows into ``{topic: [t, ...]}``. Blank cells are skipped."""
    per_topic: dict[str, list[float]] = {}
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        for needed in ("topic", "t"):
            if needed not in fields:
                raise KeyError(f"no column {needed!r} in {path} (columns: {', '.join(fields)})")
        for row in reader:
            topic = (row["topic"] or "").strip()
            cell = (row["t"] or "").strip()
            if topic and cell:
                per_topic.setdefault(topic, []).append(float(cell))
    if not per_topic:
        raise ValueError(f"{path} has no rows")
    return per_topic


def evaluate(rate: TopicRate, gaps: list[float], expectation: Expectation,
             tolerance: float = 0.10, default_max_gap_ms: float | None = None) -> list[Verdict]:
    """Check one topic against its contract: the rate, then the worst gap."""
    if not 0.0 <= tolerance < 1.0:
        raise ValueError("tolerance is a fraction in [0, 1)")
    floor_hz = expectation.hz * (1.0 - tolerance)
    verdicts = [Verdict(
        topic=rate.topic,
        detail=f"mean rate >= {floor_hz:.3g} Hz ({expectation.hz:g} Hz -{tolerance * 100:.0f}%)",
        measured=f"{rate.mean_hz:.3g} Hz over {rate.duration_s:.1f} s",
        passed=rate.mean_hz >= floor_hz,
    )]
    limit = expectation.max_gap_ms if expectation.max_gap_ms is not None else default_max_gap_ms
    if limit is not None:
        over = rate.gaps_over(limit, gaps)
        verdicts.append(Verdict(
            topic=rate.topic,
            detail=f"no gap > {limit:g} ms",
            measured=f"worst {rate.max_gap_ms:.0f} ms, p99 {rate.p99_gap_ms:.0f} ms, {over} over limit",
            passed=over == 0,
        ))
    return verdicts


def markdown_table(rates: list[TopicRate]) -> str:
    """What every topic did, whether or not you declared an expectation for it."""
    lines = ["| Topic | n | Window | Mean rate | p50 gap | p99 gap | Worst gap |", "|---|---|---|---|---|---|---|"]
    for r in rates:
        lines.append(f"| `{r.topic}` | {r.n} | {r.duration_s:.1f} s | {r.mean_hz:.3g} Hz "
                     f"| {r.p50_gap_ms:.0f} ms | {r.p99_gap_ms:.0f} ms | {r.max_gap_ms:.0f} ms |")
    return "\n".join(lines)


def verdict_table(verdicts: list[Verdict]) -> str:
    """The acceptance table for the topics you declared an expectation for."""
    lines = ["| Topic | Criterion | Measured | Verdict |", "|---|---|---|---|"]
    for v in verdicts:
        lines.append(f"| `{v.topic}` | {v.detail} | {v.measured} | "
                     f"{'pass' if v.passed else '**FAIL**'} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="CSV with columns topic,t (t in seconds)")
    ap.add_argument("--expect", action="append", default=[], metavar="TOPIC=HZ[:MAXGAP_MS]",
                    help="a topic's contract; repeat for each topic")
    ap.add_argument("--tolerance", type=float, default=0.10,
                    help="allowed shortfall on the mean rate, as a fraction (default 0.10)")
    ap.add_argument("--max-gap", type=float, default=None, metavar="MS",
                    help="gap limit for expectations that do not carry their own")
    ap.add_argument("--title", default="", help="heading printed above the tables")
    args = ap.parse_args(argv)

    per_topic = read_stamps(args.csv)
    summaries = {topic: summarize(topic, times) for topic, times in sorted(per_topic.items())
                 if len(times) >= 2}
    thin = sorted(t for t, times in per_topic.items() if len(times) < 2)

    if args.title:
        print(f"### {args.title}\n")
    print(markdown_table([rate for rate, _ in summaries.values()]))
    for topic in thin:
        print(f"\nwarning: `{topic}` has fewer than two messages - no rate to measure")

    verdicts: list[Verdict] = []
    for text in args.expect:
        expectation = Expectation.parse(text)
        entry = summaries.get(expectation.topic)
        if entry is None:
            verdicts.append(Verdict(expectation.topic, f"mean rate >= {expectation.hz:g} Hz",
                                    "not published (or a single message)", False))
            continue
        rate, gaps = entry
        verdicts += evaluate(rate, gaps, expectation, args.tolerance, args.max_gap)

    if verdicts:
        print("\n" + verdict_table(verdicts))
    return 0 if all(v.passed for v in verdicts) else 1


if __name__ == "__main__":
    sys.exit(main())
