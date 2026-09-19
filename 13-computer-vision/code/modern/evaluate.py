"""Lesson 13.10 — evaluate a detector on YOUR labelled images: precision/recall per threshold, AP50.

  py evaluate.py                                   # the two sample photos, ssdlite vs rtdetr_v2_r18
  py evaluate.py --models ssdlite frcnn_mobile --labels my_labels.json

my_labels.json (pixel corners, one entry per photo; classes = COCO names):
  [
    {"image": "photos/desk_01.jpg", "objects": [{"label": "cup", "box": [312, 140, 398, 260]}]},
    {"image": "photos/desk_02.jpg", "objects": []}
  ]
An image with an empty "objects" list is useful: every detection on it is a false positive.
"""
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from boxes import average_precision, match, pr_curve, precision_recall_at
from common import ROBOT_TARGETS, SAMPLES, load_image
from detect import build_detector


@dataclass(frozen=True)
class LabelledImage:
    source: str
    objects: list[tuple[str, tuple[float, float, float, float]]]


def sample_labels() -> list[LabelledImage]:
    return [LabelledImage(f"sample:{name}", [(l, b) for l, b in s.truth]) for name, s in SAMPLES.items()]


def load_labels(path: Path) -> list[LabelledImage]:
    base = path.parent
    items = []
    for entry in json.loads(path.read_text(encoding="utf-8")):
        src = entry["image"]
        if not src.startswith(("sample:", "http")) and not Path(src).is_absolute():
            src = str(base / src)
        items.append(LabelledImage(src, [(o["label"], tuple(o["box"])) for o in entry["objects"]]))
    return items


def evaluate(model: str, data: list[LabelledImage], classes: tuple[str, ...],
             iou: float = 0.5, thresholds: tuple[float, ...] = (0.3, 0.5, 0.7)) -> dict:
    detector = build_detector(model)
    pooled: dict[str, dict[str, list]] = {c: {"scores": [], "tp": [], "n": [0]} for c in classes}
    for item in data:
        dets = detector(load_image(item.source))
        for c in classes:
            preds = [d for d in dets if d.label == c]
            truths = [b for l, b in item.objects if l == c]
            m = match(np.array([d.box for d in preds]), np.array([d.score for d in preds]),
                      np.array(truths), iou)
            pooled[c]["scores"] += [d.score for d in preds]
            pooled[c]["tp"] += m.is_tp.tolist()
            pooled[c]["n"][0] += m.n_truth
    report = {}
    for c in classes:
        scores, tp, n = np.array(pooled[c]["scores"]), np.array(pooled[c]["tp"], dtype=bool), pooled[c]["n"][0]
        report[c] = {
            "n_truth": n,
            "ap50": average_precision(pr_curve(scores, tp, n)) if n else float("nan"),
            "at": {t: precision_recall_at(scores, tp, n, t) for t in thresholds},
        }
    return report


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models", nargs="+", default=["ssdlite", "rtdetr_v2_r18"])
    ap.add_argument("--labels", type=Path)
    ap.add_argument("--iou", type=float, default=0.5)
    args = ap.parse_args()

    data = load_labels(args.labels) if args.labels else sample_labels()
    classes = tuple(c for c in ROBOT_TARGETS if any(l == c for item in data for l, _ in item.objects))
    print(f"{len(data)} images, classes with ground truth: {classes}, match IoU >= {args.iou}")
    for model in args.models:
        report = evaluate(model, data, classes, args.iou)
        print(f"\n{model}")
        print(f"  {'class':<8} {'truth':>5} {'AP50':>6}   threshold: precision / recall (TP, FP)")
        for c, r in report.items():
            cells = "   ".join(f"{t:.1f}: {p:.2f}/{rc:.2f} ({tp},{fp})" for t, (p, rc, tp, fp) in r["at"].items())
            print(f"  {c:<8} {r['n_truth']:>5} {r['ap50']:6.2f}   {cells}")


if __name__ == "__main__":
    main()
