"""Tests for tracking.py (no detector, no torch)."""
import numpy as np

from tracking import ByteTracker, IoUTracker, KalmanBox, SortTracker, associate, evaluate, simulate


def test_associate_is_one_to_one():
    tracks = np.array([[0, 0, 10, 10], [100, 100, 110, 110]])
    dets = np.array([[101, 101, 111, 111], [1, 0, 11, 10], [500, 500, 510, 510]])
    matches, lost, new = associate(tracks, dets, 0.3)
    assert sorted(matches) == [(0, 1), (1, 0)] and lost == [] and new == [2]


def test_kalman_learns_velocity():
    kf = KalmanBox(np.array([0.0, 0.0, 20.0, 20.0]))
    for f in range(1, 15):
        kf.predict()
        kf.update(np.array([10.0 * f, 0.0, 10.0 * f + 20, 20.0]))
    assert abs(kf.x[4] - 10.0) < 1.0                  # vx ≈ 10 px/frame
    predicted = kf.predict()
    assert abs(predicted[0] - 150.0) < 3.0


def test_fast_object_breaks_iou_tracker_but_not_sort():
    seq = simulate(seed=0, ball_speed=18.0)
    iou = evaluate(IoUTracker(), seq)
    sort = evaluate(SortTracker(), simulate(seed=0, ball_speed=18.0))
    assert iou.ids_per_object[3] > 5
    assert sort.ids_per_object[3] <= 2


def test_bytetrack_keeps_id_through_long_occlusion_on_average():
    switches_sort = sum(evaluate(SortTracker(), simulate(s, ball_speed=8.0)).id_switches for s in range(10))
    switches_byte = sum(evaluate(ByteTracker(), simulate(s, ball_speed=8.0)).id_switches for s in range(10))
    assert switches_byte < switches_sort
