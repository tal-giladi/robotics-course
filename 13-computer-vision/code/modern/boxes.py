"""Box geometry and detector evaluation in plain numpy (lessons 13.10 and 13.12).

  iou_matrix(a, b)            pairwise IoU between two sets of xyxy boxes
  nms(boxes, scores, thr)     greedy non-maximum suppression, the algorithm inside every classic detector
  match(preds, truths, thr)   greedy one-to-one matching -> true/false positives
  pr_curve(...)               precision/recall at every score threshold
  average_precision(...)      area under the PR curve (all-point interpolation, as in COCO/VOC)

Deliberately no torch: you can read, test and reuse these on the robot.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """IoU for every pair. a: (N, 4), b: (M, 4), boxes as x1, y1, x2, y2. Returns (N, M)."""
    a = np.asarray(a, dtype=float).reshape(-1, 4)
    b = np.asarray(b, dtype=float).reshape(-1, 4)
    ix1 = np.maximum(a[:, None, 0], b[None, :, 0])
    iy1 = np.maximum(a[:, None, 1], b[None, :, 1])
    ix2 = np.minimum(a[:, None, 2], b[None, :, 2])
    iy2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(ix2 - ix1, 0, None) * np.clip(iy2 - iy1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-12), 0.0)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float = 0.5) -> list[int]:
    """Greedy NMS: keep the best box, delete every box overlapping it by > iou_threshold, repeat.

    Returns the indices of kept boxes, highest score first. Run it per class.
    """
    boxes = np.asarray(boxes, dtype=float).reshape(-1, 4)
    order = list(np.argsort(-np.asarray(scores, dtype=float), kind="stable"))
    keep: list[int] = []
    while order:
        best = order.pop(0)
        keep.append(int(best))
        if not order:
            break
        ious = iou_matrix(boxes[best], boxes[order])[0]
        order = [i for i, iou in zip(order, ious) if iou <= iou_threshold]
    return keep


@dataclass(frozen=True)
class MatchResult:
    is_tp: np.ndarray        # (N_pred,) bool, in the order of the predictions passed in
    matched_truth: np.ndarray  # (N_pred,) index of the truth box, -1 for false positives
    n_truth: int


def match(pred_boxes: np.ndarray, pred_scores: np.ndarray, truth_boxes: np.ndarray,
          iou_threshold: float = 0.5) -> MatchResult:
    """Greedy matching in descending score order. Each truth box can be claimed once.

    A second box on an already-matched object is a false positive: that is exactly what NMS
    is supposed to prevent, and why evaluation punishes duplicates.
    """
    pred_boxes = np.asarray(pred_boxes, dtype=float).reshape(-1, 4)
    truth_boxes = np.asarray(truth_boxes, dtype=float).reshape(-1, 4)
    n = len(pred_boxes)
    is_tp = np.zeros(n, dtype=bool)
    matched = np.full(n, -1)
    if n == 0 or len(truth_boxes) == 0:
        return MatchResult(is_tp, matched, len(truth_boxes))
    ious = iou_matrix(pred_boxes, truth_boxes)
    taken = np.zeros(len(truth_boxes), dtype=bool)
    for i in np.argsort(-np.asarray(pred_scores, dtype=float), kind="stable"):
        candidates = np.where(~taken, ious[i], -1.0)
        j = int(np.argmax(candidates))
        if candidates[j] >= iou_threshold:
            is_tp[i], matched[i], taken[j] = True, j, True
    return MatchResult(is_tp, matched, len(truth_boxes))


@dataclass(frozen=True)
class PRCurve:
    thresholds: np.ndarray   # score of each prediction, descending
    precision: np.ndarray
    recall: np.ndarray


def pr_curve(scores: np.ndarray, is_tp: np.ndarray, n_truth: int) -> PRCurve:
    """Precision and recall if you kept every prediction with score >= each threshold.

    scores/is_tp may be pooled over many images (match each image separately first).
    """
    scores = np.asarray(scores, dtype=float)
    order = np.argsort(-scores, kind="stable")
    tp = np.cumsum(np.asarray(is_tp, dtype=bool)[order])
    fp = np.cumsum(~np.asarray(is_tp, dtype=bool)[order])
    precision = tp / np.maximum(tp + fp, 1)
    recall = tp / max(n_truth, 1)
    return PRCurve(scores[order], precision, recall)


def average_precision(curve: PRCurve) -> float:
    """All-point interpolated AP: make precision monotone, integrate over recall."""
    if len(curve.recall) == 0:
        return 0.0
    r = np.concatenate([[0.0], curve.recall, [curve.recall[-1]]])
    p = np.concatenate([[1.0], curve.precision, [0.0]])
    p = np.maximum.accumulate(p[::-1])[::-1]
    steps = np.where(r[1:] != r[:-1])[0]
    return float(np.sum((r[steps + 1] - r[steps]) * p[steps + 1]))


def precision_recall_at(scores: np.ndarray, is_tp: np.ndarray, n_truth: int,
                        threshold: float) -> tuple[float, float, int, int]:
    """(precision, recall, TP, FP) keeping predictions with score >= threshold."""
    keep = np.asarray(scores) >= threshold
    tp = int(np.sum(np.asarray(is_tp)[keep]))
    fp = int(np.sum(keep)) - tp
    precision = tp / (tp + fp) if tp + fp else 1.0
    recall = tp / n_truth if n_truth else 1.0
    return precision, recall, tp, fp


if __name__ == "__main__":
    # The worked example from lesson 13.10: three boxes on one cup, one on a bottle.
    boxes = np.array([[100, 100, 200, 220], [105, 98, 204, 225], [300, 80, 360, 240],
                      [98, 110, 190, 215]], dtype=float)
    scores = np.array([0.92, 0.85, 0.77, 0.60])
    print("IoU box0 vs box1:", round(float(iou_matrix(boxes[0], boxes[1])[0, 0]), 3))
    print("IoU box0 vs box3:", round(float(iou_matrix(boxes[0], boxes[3])[0, 0]), 3))
    print("NMS keep @0.5:", nms(boxes, scores, 0.5))
    truth = np.array([[102, 100, 202, 222], [298, 82, 362, 236]], dtype=float)
    m = match(boxes, scores, truth, 0.5)
    print("TP flags:", m.is_tp.tolist())
    curve = pr_curve(scores, m.is_tp, m.n_truth)
    print("precision:", np.round(curve.precision, 3).tolist(), "recall:", np.round(curve.recall, 3).tolist())
    print("AP50:", round(average_precision(curve), 3))
