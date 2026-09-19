"""Tests for boxes.py — run with: py -m pytest 13-computer-vision/code/modern/tests"""
import numpy as np
import pytest

from boxes import average_precision, iou_matrix, match, nms, pr_curve, precision_recall_at


def test_iou_known_values():
    a = np.array([[0, 0, 10, 10]])
    assert iou_matrix(a, a)[0, 0] == pytest.approx(1.0)
    assert iou_matrix(a, np.array([[5, 0, 15, 10]]))[0, 0] == pytest.approx(50 / 150)
    assert iou_matrix(a, np.array([[20, 20, 30, 30]]))[0, 0] == 0.0


def test_iou_is_symmetric_and_shaped():
    rng = np.random.default_rng(0)
    xy = rng.uniform(0, 100, (5, 2))
    a = np.hstack([xy, xy + rng.uniform(1, 50, (5, 2))])
    m = iou_matrix(a, a[:3])
    assert m.shape == (5, 3)
    assert np.allclose(iou_matrix(a[:3], a), m.T)


def test_nms_removes_duplicates_keeps_distinct_objects():
    boxes = np.array([[100, 100, 200, 220], [105, 98, 204, 225], [300, 80, 360, 240], [98, 110, 190, 215]])
    scores = np.array([0.92, 0.85, 0.77, 0.60])
    assert nms(boxes, scores, 0.5) == [0, 2]
    assert nms(boxes, scores, 0.9) == [0, 1, 2, 3]   # threshold too high: duplicates survive


def test_match_counts_duplicates_as_false_positives():
    truth = np.array([[0, 0, 10, 10]])
    preds = np.array([[0, 0, 10, 10], [0, 0, 10, 11]])
    m = match(preds, np.array([0.9, 0.8]), truth, 0.5)
    assert m.is_tp.tolist() == [True, False]


def test_ap_perfect_and_worked_example():
    perfect = pr_curve(np.array([0.9, 0.8]), np.array([True, True]), 2)
    assert average_precision(perfect) == pytest.approx(1.0)
    worked = pr_curve(np.array([0.92, 0.85, 0.77, 0.60]), np.array([True, False, True, False]), 2)
    assert average_precision(worked) == pytest.approx(0.5 * 1.0 + 0.5 * 2 / 3)
    p, r, tp, fp = precision_recall_at(np.array([0.92, 0.85, 0.77, 0.60]), np.array([True, False, True, False]), 2, 0.8)
    assert (tp, fp) == (1, 1) and p == 0.5 and r == 0.5
