"""Your detector found things. Were they the right things, and was it fast enough?

    py projects/code/detect_eval.py --truth labels.csv --detections runs/yolo_pi5.csv \\
        --iou 0.5 --score 0.35 --latency runs/yolo_pi5_latency.csv \\
        --min-precision 0.9 --min-recall 0.75 --max-p95-latency 200 \\
        --title "P12 - bottle+cup+chair, 200 frames, Pi 5"

Two CSVs of boxes, in pixels, top-left corner plus size::

    image,label,x,y,w,h              (ground truth, one row per object)
    frame_0007.jpg,bottle,412,118,64,190

    image,label,x,y,w,h,score        (detections, one row per box the model emitted)
    frame_0007.jpg,bottle,409,121,66,186,0.87

Detections are matched to truths **greedily, highest score first, inside one image and one
label**, and a match counts when the IoU is at least ``--iou``. Each truth can be claimed once:
two boxes on one bottle are one true positive and one false positive, which is the behaviour you
want, because a detector that emits five boxes per object is not five times better.

The three numbers that matter on a robot, and why they are separate:

* **precision** - of the things you acted on, how many were real. A robot that drives to
  hallucinated bottles wastes its battery and its credibility.
* **recall** - of the things that were there, how many you saw. Recall is what
  ``--score`` buys and sells: raise the threshold and precision goes up, recall goes down, and
  the right trade-off is a property of the *task*, not of the model.
* **latency** - the p95, not the mean. A detector whose mean is 80 ms and whose p95 is 400 ms
  makes a robot that mostly reacts in time.

Nothing here is model-specific or framework-specific: export boxes from whatever you ran
(13.10, 13.15) into these two files.
"""

from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

from report import percentile_abs, read_column


@dataclass(frozen=True)
class Box:
    """One annotated or detected object. ``x, y`` is the top-left corner, in pixels."""

    image: str
    label: str
    x: float
    y: float
    w: float
    h: float
    score: float = 1.0

    def __post_init__(self) -> None:
        if self.w <= 0.0 or self.h <= 0.0:
            raise ValueError(f"box in {self.image} has non-positive size ({self.w} x {self.h})")

    @property
    def area(self) -> float:
        return self.w * self.h

    @property
    def key(self) -> tuple[str, str]:
        """Detections are only ever compared inside one image and one class."""
        return self.image, self.label


def iou(a: Box, b: Box) -> float:
    """Intersection over union. 0.0 when the boxes do not overlap at all."""
    left = max(a.x, b.x)
    top = max(a.y, b.y)
    right = min(a.x + a.w, b.x + b.w)
    bottom = min(a.y + a.h, b.y + b.h)
    if right <= left or bottom <= top:
        return 0.0
    intersection = (right - left) * (bottom - top)
    return intersection / (a.area + b.area - intersection)


@dataclass(frozen=True)
class Evaluation:
    """Counts and the three rates, for one label or for everything."""

    label: str
    tp: int
    fp: int
    fn: int

    @property
    def precision(self) -> float:
        predicted = self.tp + self.fp
        return self.tp / predicted if predicted else 0.0

    @property
    def recall(self) -> float:
        actual = self.tp + self.fn
        return self.tp / actual if actual else 0.0

    @property
    def f1(self) -> float:
        p, r = self.precision, self.recall
        return 2.0 * p * r / (p + r) if p + r > 0.0 else 0.0


def group(boxes: list[Box]) -> dict[tuple[str, str], list[Box]]:
    buckets: dict[tuple[str, str], list[Box]] = {}
    for box in boxes:
        buckets.setdefault(box.key, []).append(box)
    return buckets


def match(truths: list[Box], detections: list[Box], iou_threshold: float = 0.5
          ) -> tuple[list[tuple[Box, Box, float]], list[Box], list[Box]]:
    """``(matches, false_positives, missed_truths)`` for ONE image and ONE label.

    Greedy, highest score first: the confident boxes get first pick of the truths, which is what
    a downstream consumer that sorts by score would see.
    """
    if not 0.0 < iou_threshold <= 1.0:
        raise ValueError("--iou must be in (0, 1]")
    unclaimed = list(truths)
    matches: list[tuple[Box, Box, float]] = []
    false_positives: list[Box] = []
    for detection in sorted(detections, key=lambda b: -b.score):
        best, best_iou = None, 0.0
        for truth in unclaimed:
            overlap = iou(detection, truth)
            if overlap > best_iou:
                best, best_iou = truth, overlap
        if best is not None and best_iou >= iou_threshold:
            unclaimed.remove(best)
            matches.append((detection, best, best_iou))
        else:
            false_positives.append(detection)
    return matches, false_positives, unclaimed


def evaluate(truths: list[Box], detections: list[Box], iou_threshold: float = 0.5,
             min_score: float = 0.0) -> list[Evaluation]:
    """One ``Evaluation`` per label, in alphabetical order, then one labelled ``ALL``."""
    kept = [d for d in detections if d.score >= min_score]
    truth_groups, detection_groups = group(truths), group(kept)
    labels = sorted({label for _, label in truth_groups} | {label for _, label in detection_groups})
    keys = set(truth_groups) | set(detection_groups)

    per_label: dict[str, list[int]] = {label: [0, 0, 0] for label in labels}
    for key in keys:
        matches, false_positives, missed = match(
            truth_groups.get(key, []), detection_groups.get(key, []), iou_threshold)
        counts = per_label[key[1]]
        counts[0] += len(matches)
        counts[1] += len(false_positives)
        counts[2] += len(missed)

    results = [Evaluation(label, *per_label[label]) for label in labels]
    if len(results) != 1:
        results.append(Evaluation("ALL",
                                  sum(r.tp for r in results),
                                  sum(r.fp for r in results),
                                  sum(r.fn for r in results)))
    return results


def read_boxes(path: Path) -> list[Box]:
    """Read ``image,label,x,y,w,h[,score]``. A missing score is treated as 1.0."""
    required = ("image", "label", "x", "y", "w", "h")
    boxes = []
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = reader.fieldnames or []
        for column in required:
            if column not in fields:
                raise KeyError(f"no column {column!r} in {path} (columns: {', '.join(fields)})")
        for row in reader:
            score_cell = (row.get("score") or "").strip()
            boxes.append(Box(
                image=(row["image"] or "").strip(),
                label=(row["label"] or "").strip(),
                x=float(row["x"]), y=float(row["y"]),
                w=float(row["w"]), h=float(row["h"]),
                score=float(score_cell) if score_cell else 1.0,
            ))
    return boxes


@dataclass(frozen=True)
class Latency:
    """Per-frame inference times, as the robot experiences them."""

    n: int
    mean_ms: float
    p50_ms: float
    p95_ms: float
    p99_ms: float
    max_ms: float

    @property
    def sustained_hz(self) -> float:
        """The rate you can actually promise: one frame per p95, not per mean."""
        return 1000.0 / self.p95_ms if self.p95_ms > 0.0 else 0.0


def summarize_latency(values_ms: list[float]) -> Latency:
    if not values_ms:
        raise ValueError("no latency samples")
    return Latency(
        n=len(values_ms),
        mean_ms=sum(values_ms) / len(values_ms),
        p50_ms=percentile_abs(values_ms, 50.0),
        p95_ms=percentile_abs(values_ms, 95.0),
        p99_ms=percentile_abs(values_ms, 99.0),
        max_ms=max(values_ms),
    )


def markdown_table(results: list[Evaluation]) -> str:
    lines = ["| Label | TP | FP | FN | Precision | Recall | F1 |", "|---|---|---|---|---|---|---|"]
    for r in results:
        lines.append(f"| {r.label} | {r.tp} | {r.fp} | {r.fn} | {r.precision:.3f} "
                     f"| {r.recall:.3f} | {r.f1:.3f} |")
    return "\n".join(lines)


def latency_table(latency: Latency) -> str:
    return "\n".join([
        "| Latency | Value |",
        "|---|---|",
        f"| frames | {latency.n} |",
        f"| mean | {latency.mean_ms:.0f} ms |",
        f"| p50 | {latency.p50_ms:.0f} ms |",
        f"| p95 | {latency.p95_ms:.0f} ms |",
        f"| p99 | {latency.p99_ms:.0f} ms |",
        f"| worst | {latency.max_ms:.0f} ms |",
        f"| sustained rate (1 / p95) | {latency.sustained_hz:.1f} Hz |",
    ])


def check(overall: Evaluation, latency: Latency | None = None,
          min_precision: float | None = None, min_recall: float | None = None,
          min_f1: float | None = None,
          max_p95_latency_ms: float | None = None) -> list[tuple[str, str, bool]]:
    rows: list[tuple[str, str, bool]] = []
    if min_precision is not None:
        rows.append((f"precision >= {min_precision:.2f}", f"{overall.precision:.3f}",
                     overall.precision >= min_precision))
    if min_recall is not None:
        rows.append((f"recall >= {min_recall:.2f}", f"{overall.recall:.3f}",
                     overall.recall >= min_recall))
    if min_f1 is not None:
        rows.append((f"F1 >= {min_f1:.2f}", f"{overall.f1:.3f}", overall.f1 >= min_f1))
    if max_p95_latency_ms is not None:
        measured = f"{latency.p95_ms:.0f} ms" if latency else "no --latency given"
        rows.append((f"p95 latency <= {max_p95_latency_ms:g} ms", measured,
                     latency is not None and latency.p95_ms <= max_p95_latency_ms))
    return rows


def verdict_table(rows: list[tuple[str, str, bool]]) -> str:
    lines = ["| Criterion | Measured | Verdict |", "|---|---|---|"]
    for detail, measured, passed in rows:
        lines.append(f"| {detail} | {measured} | {'pass' if passed else '**FAIL**'} |")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--truth", type=Path, required=True, help="image,label,x,y,w,h")
    ap.add_argument("--detections", type=Path, required=True, help="image,label,x,y,w,h,score")
    ap.add_argument("--iou", type=float, default=0.5, help="IoU that counts as a match (default 0.5)")
    ap.add_argument("--score", type=float, default=0.0, help="drop detections below this score")
    ap.add_argument("--latency", type=Path, help="CSV of per-frame inference times")
    ap.add_argument("--latency-column", default="ms", help="which column holds the milliseconds")
    ap.add_argument("--min-precision", type=float)
    ap.add_argument("--min-recall", type=float)
    ap.add_argument("--min-f1", type=float)
    ap.add_argument("--max-p95-latency", type=float, metavar="MS")
    ap.add_argument("--title", default="", help="heading printed above the tables")
    args = ap.parse_args(argv)

    results = evaluate(read_boxes(args.truth), read_boxes(args.detections), args.iou, args.score)
    overall = results[-1]
    latency = (summarize_latency(read_column(args.latency, args.latency_column))
               if args.latency else None)

    if args.title:
        print(f"### {args.title}\n")
    print(f"IoU >= {args.iou:g}, detections scored >= {args.score:g}\n")
    print(markdown_table(results))
    if latency:
        print("\n" + latency_table(latency))
    rows = check(overall, latency, args.min_precision, args.min_recall, args.min_f1,
                 args.max_p95_latency)
    if rows:
        print("\n" + verdict_table(rows))
    return 0 if all(passed for _, _, passed in rows) else 1


if __name__ == "__main__":
    sys.exit(main())
