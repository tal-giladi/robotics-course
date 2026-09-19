"""Detection metrics used in 16.03, 16.04 and 16.06: IoU, greedy matching, AP@0.5 per class.

Deliberately small and dependency-free (numpy only) so you can read every line.
FCV.06 explains IoU, precision/recall and mAP from zero; this is the same maths in code.

    >>> ap = average_precision(preds, gts, class_id=1, iou_thr=0.5)
    preds: list per image of (boxes (N,4) x1y1x2y2, scores (N,), labels (N,))
    gts:   list per image of (boxes (M,4), labels (M,))
"""
from __future__ import annotations

import numpy as np


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Pairwise IoU between boxes a (N,4) and b (M,4), x1y1x2y2."""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)))
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    return inter / (area_a[:, None] + area_b[None, :] - inter + 1e-9)


def match(preds, gts, class_id: int, iou_thr: float = 0.5) -> tuple[np.ndarray, np.ndarray, int]:
    """Greedy matching by descending score. Returns (scores, is_true_positive, number_of_gt)."""
    scores, tps, n_gt = [], [], 0
    for (pb, ps, pl), (gb, gl) in zip(preds, gts):
        pb, ps = pb[pl == class_id], ps[pl == class_id]
        gb = gb[gl == class_id]
        n_gt += len(gb)
        order = np.argsort(-ps)
        ious = iou_matrix(pb[order], gb)
        used = np.zeros(len(gb), bool)
        for i, score in enumerate(ps[order]):
            j = int(np.argmax(ious[i])) if len(gb) else -1
            ok = j >= 0 and ious[i, j] >= iou_thr and not used[j]
            if ok:
                used[j] = True
            scores.append(float(score))
            tps.append(ok)
    return np.array(scores), np.array(tps, bool), n_gt


def average_precision(preds, gts, class_id: int, iou_thr: float = 0.5) -> float:
    """Area under the precision-recall curve (all-point interpolation, as in PASCAL VOC 2010+)."""
    scores, tps, n_gt = match(preds, gts, class_id, iou_thr)
    if n_gt == 0:
        return float("nan")
    order = np.argsort(-scores)
    tp = np.cumsum(tps[order])
    fp = np.cumsum(~tps[order])
    recall = tp / n_gt
    precision = tp / np.maximum(tp + fp, 1e-9)
    r = np.concatenate([[0.0], recall, [1.0]])
    p = np.concatenate([[1.0], precision, [0.0]])
    p = np.maximum.accumulate(p[::-1])[::-1]              # make precision monotonically decreasing
    return float(np.sum((r[1:] - r[:-1]) * p[1:]))


def precision_recall_at(preds, gts, class_id: int, score_thr: float, iou_thr: float = 0.5) -> tuple[float, float]:
    """Precision and recall if the robot only acts on detections with score >= score_thr."""
    scores, tps, n_gt = match(preds, gts, class_id, iou_thr)
    keep = scores >= score_thr
    tp = int(tps[keep].sum())
    return tp / max(int(keep.sum()), 1), tp / max(n_gt, 1)


if __name__ == "__main__":
    gts = [(np.array([[10, 10, 50, 90]], float), np.array([1]))]
    preds = [(np.array([[12, 8, 52, 88], [100, 100, 120, 130]], float), np.array([0.9, 0.6]), np.array([1, 1]))]
    print("IoU", iou_matrix(preds[0][0], gts[0][0]).round(3).tolist())
    print("AP50", average_precision(preds, gts, 1), "P/R@0.5", precision_recall_at(preds, gts, 1, 0.5))
