"""Lesson 13.12 — multi-object tracking: IoU tracker, SORT (Kalman + Hungarian) and ByteTrack.

  py tracking.py                       # compare the three trackers on a synthetic tabletop sequence
  py tracking.py --seeds 20            # average over 20 random sequences
  py tracking.py --ball-speed 8        # slow ball: the plain IoU tracker copes again

Pure numpy + scipy, no detector needed: the "detector" is simulated so the ground truth is known.
To track real detections, call tracker.update(boxes, scores) once per frame with the output of detect.py.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import linear_sum_assignment

from boxes import iou_matrix


@dataclass(frozen=True)
class TrackOutput:
    track_id: int
    box: tuple[float, float, float, float]
    det_index: int | None          # which input detection updated it this frame (None = coasting)


def associate(track_boxes: np.ndarray, det_boxes: np.ndarray, iou_threshold: float
              ) -> tuple[list[tuple[int, int]], list[int], list[int]]:
    """Optimal one-to-one assignment maximising total IoU (Hungarian algorithm on cost = 1 - IoU).

    Returns (matches [(track, det)], unmatched track indices, unmatched detection indices).
    """
    n_t, n_d = len(track_boxes), len(det_boxes)
    if n_t == 0 or n_d == 0:
        return [], list(range(n_t)), list(range(n_d))
    iou = iou_matrix(track_boxes, det_boxes)
    rows, cols = linear_sum_assignment(1.0 - iou)
    matches = [(int(r), int(c)) for r, c in zip(rows, cols) if iou[r, c] >= iou_threshold]
    mt = {r for r, _ in matches}
    md = {c for _, c in matches}
    return matches, [i for i in range(n_t) if i not in mt], [j for j in range(n_d) if j not in md]


# ----------------------------------------------------------------------------- 1. IoU tracker
class IoUTracker:
    """Match each detection to last frame's boxes by IoU. No motion model, no memory."""

    def __init__(self, iou_threshold: float = 0.3, min_score: float = 0.5, max_missed: int = 1) -> None:
        self.iou_threshold, self.min_score, self.max_missed = iou_threshold, min_score, max_missed
        self.tracks: dict[int, tuple[np.ndarray, int]] = {}   # id -> (last box, frames missed)
        self._next_id = 1

    def update(self, boxes: np.ndarray, scores: np.ndarray) -> list[TrackOutput]:
        keep = np.where(np.asarray(scores) >= self.min_score)[0]
        det = np.asarray(boxes, dtype=float).reshape(-1, 4)[keep]
        ids = list(self.tracks)
        matches, lost, new = associate(np.array([self.tracks[i][0] for i in ids]).reshape(-1, 4), det,
                                       self.iou_threshold)
        out = []
        for t, d in matches:
            self.tracks[ids[t]] = (det[d], 0)
            out.append(TrackOutput(ids[t], tuple(det[d]), int(keep[d])))
        for t in lost:
            box, missed = self.tracks[ids[t]]
            if missed + 1 > self.max_missed:
                del self.tracks[ids[t]]
            else:
                self.tracks[ids[t]] = (box, missed + 1)
        for d in new:
            self.tracks[self._next_id] = (det[d], 0)
            out.append(TrackOutput(self._next_id, tuple(det[d]), int(keep[d])))
            self._next_id += 1
        return out


# ----------------------------------------------------------------------------- 2. Kalman box + SORT
def xyxy_to_z(b: np.ndarray) -> np.ndarray:
    return np.array([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2, b[2] - b[0], b[3] - b[1]])


def z_to_xyxy(z: np.ndarray) -> np.ndarray:
    cx, cy, w, h = z[:4]
    return np.array([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])


class KalmanBox:
    """Constant-velocity Kalman filter on the box. State x = [cx, cy, w, h, vcx, vcy, vw, vh] (px, px/frame).

    Same predict/update equations as the Kalman filter in 10.04-10.05, applied to pixels.
    """

    def __init__(self, box: np.ndarray, pos_std: float = 2.0, vel_std: float = 1.0, meas_std: float = 3.0) -> None:
        self.x = np.concatenate([xyxy_to_z(box), np.zeros(4)])
        self.P = np.diag([meas_std**2] * 4 + [30.0**2] * 4)      # unknown initial velocity: large variance
        self.F = np.eye(8)
        self.F[:4, 4:] = np.eye(4)                              # position += velocity * 1 frame
        self.H = np.eye(4, 8)                                   # we measure position and size only
        self.Q = np.diag([pos_std**2] * 4 + [vel_std**2] * 4)   # how much the motion may change per frame
        self.R = np.eye(4) * meas_std**2                        # detector box noise

    def predict(self) -> np.ndarray:
        self.x = self.F @ self.x
        self.x[2:4] = np.maximum(self.x[2:4], 1.0)
        self.P = self.F @ self.P @ self.F.T + self.Q
        return z_to_xyxy(self.x)

    def update(self, box: np.ndarray) -> None:
        y = xyxy_to_z(box) - self.H @ self.x                    # innovation
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)                # Kalman gain
        self.x = self.x + K @ y
        self.P = (np.eye(8) - K @ self.H) @ self.P


@dataclass
class _Track:
    track_id: int
    kf: KalmanBox
    hits: int = 1
    missed: int = 0
    box: np.ndarray = field(default_factory=lambda: np.zeros(4))


class SortTracker:
    """SORT (Bewley et al. 2016): Kalman predict -> Hungarian on IoU -> update; tracks coast while missed."""

    def __init__(self, iou_threshold: float = 0.3, min_score: float = 0.5, max_age: int = 5, min_hits: int = 2) -> None:
        self.iou_threshold, self.min_score, self.max_age, self.min_hits = iou_threshold, min_score, max_age, min_hits
        self.tracks: list[_Track] = []
        self._next_id = 1

    def _predict(self) -> np.ndarray:
        for t in self.tracks:
            t.box = t.kf.predict()
        return np.array([t.box for t in self.tracks]).reshape(-1, 4)

    def _new_track(self, box: np.ndarray) -> _Track:
        t = _Track(self._next_id, KalmanBox(box), box=box.copy())
        self._next_id += 1
        self.tracks.append(t)
        return t

    def _outputs(self, updated: dict[int, int]) -> list[TrackOutput]:
        self.tracks = [t for t in self.tracks if t.missed <= self.max_age]
        return [TrackOutput(t.track_id, tuple(float(v) for v in t.box), updated.get(t.track_id))
                for t in self.tracks if t.hits >= self.min_hits]

    def update(self, boxes: np.ndarray, scores: np.ndarray) -> list[TrackOutput]:
        boxes = np.asarray(boxes, dtype=float).reshape(-1, 4)
        keep = np.where(np.asarray(scores) >= self.min_score)[0]
        predicted = self._predict()
        matches, lost, new = associate(predicted, boxes[keep], self.iou_threshold)
        updated: dict[int, int] = {}
        for ti, di in matches:
            t = self.tracks[ti]
            t.kf.update(boxes[keep[di]])
            t.box, t.hits, t.missed = z_to_xyxy(t.kf.x), t.hits + 1, 0
            updated[t.track_id] = int(keep[di])
        for ti in lost:
            self.tracks[ti].missed += 1
        for di in new:
            updated[self._new_track(boxes[keep[di]]).track_id] = int(keep[di])
        return self._outputs(updated)


class ByteTracker(SortTracker):
    """ByteTrack (Zhang et al. 2022): SORT, plus a second association round with LOW-confidence boxes.

    A partly hidden object still produces a weak detection; ByteTrack uses it to keep the track
    alive instead of throwing it away. Low boxes never start new tracks (they are mostly clutter).
    """

    def __init__(self, high: float = 0.5, low: float = 0.1, iou_threshold: float = 0.3,
                 low_iou_threshold: float = 0.5, max_age: int = 5, min_hits: int = 2) -> None:
        super().__init__(iou_threshold, high, max_age, min_hits)
        self.low, self.low_iou_threshold = low, low_iou_threshold

    def update(self, boxes: np.ndarray, scores: np.ndarray) -> list[TrackOutput]:
        boxes = np.asarray(boxes, dtype=float).reshape(-1, 4)
        scores = np.asarray(scores, dtype=float)
        high = np.where(scores >= self.min_score)[0]
        low = np.where((scores >= self.low) & (scores < self.min_score))[0]
        predicted = self._predict()
        updated: dict[int, int] = {}

        def apply(ti: int, det_idx: int) -> None:
            t = self.tracks[ti]
            t.kf.update(boxes[det_idx])
            t.box, t.hits, t.missed = z_to_xyxy(t.kf.x), t.hits + 1, 0
            updated[t.track_id] = int(det_idx)

        # round 1: every track vs high-confidence detections
        m1, lost1, new_high = associate(predicted, boxes[high], self.iou_threshold)
        for ti, di in m1:
            apply(ti, high[di])
        # round 2: tracks still unmatched vs low-confidence detections (stricter IoU)
        m2, lost2, _ = associate(predicted[lost1], boxes[low], self.low_iou_threshold)
        for k, di in m2:
            apply(lost1[k], low[di])
        for k in lost2:
            self.tracks[lost1[k]].missed += 1
        for di in new_high:
            updated[self._new_track(boxes[high[di]]).track_id] = int(high[di])
        return self._outputs(updated)


# ----------------------------------------------------------------------------- simulation + metrics
@dataclass(frozen=True)
class Frame:
    truth: dict[int, np.ndarray]     # object id -> true box
    boxes: np.ndarray                # detector output (N, 4)
    scores: np.ndarray               # (N,)


def simulate(seed: int = 0, frames: int = 60, ball_speed: float = 18.0) -> list[Frame]:
    """A 640x480 tabletop: a static cup, a bottle sliding slowly, a ball rolling right and passing
    BEHIND the bottle (occlusion). The detector is noisy, sometimes misses, sometimes hallucinates,
    and reports the half-hidden ball with low confidence — exactly what real detectors do."""
    rng = np.random.default_rng(seed)
    out = []
    for f in range(frames):
        truth = {
            1: np.array([60.0, 300.0, 130.0, 400.0]),                                          # cup
            2: np.array([330.0 + 0.5 * f, 180.0, 390.0 + 0.5 * f, 360.0]),                     # bottle
            3: np.array([20.0 + ball_speed * f, 290.0, 50.0 + ball_speed * f, 320.0]),         # ball
        }
        truth = {k: b for k, b in truth.items() if b[0] < 640}
        boxes, scores = [], []
        for oid, b in truth.items():
            # the ball is 'behind' the bottle while their x-ranges overlap
            occluded = oid == 3 and 2 in truth and b[2] > truth[2][0] and b[0] < truth[2][2]
            if rng.random() < 0.08:
                continue                                          # plain miss
            score = rng.uniform(0.15, 0.4) if occluded else rng.uniform(0.6, 0.95)
            boxes.append(b + rng.normal(0, 3.0, 4))
            scores.append(score)
        if rng.random() < 0.15:                                  # clutter: a random weak box
            x, y = rng.uniform(0, 560), rng.uniform(0, 400)
            boxes.append(np.array([x, y, x + 40, y + 40]))
            scores.append(rng.uniform(0.1, 0.55))
        out.append(Frame(truth, np.array(boxes).reshape(-1, 4), np.array(scores)))
    return out


@dataclass(frozen=True)
class MotMetrics:
    mota: float
    id_switches: int
    misses: int
    false_positives: int
    n_truth: int
    ids_per_object: dict[int, int]

    def __str__(self) -> str:
        per = ", ".join(f"obj{k}:{v}" for k, v in sorted(self.ids_per_object.items()))
        return (f"MOTA {self.mota:5.2f}  ID switches {self.id_switches:3d}  misses {self.misses:3d}  "
                f"false pos {self.false_positives:3d}  track IDs used per object [{per}]")


def evaluate(tracker, sequence: list[Frame], iou_threshold: float = 0.3) -> MotMetrics:
    """CLEAR-MOT style: match tracker output to truth each frame; MOTA = 1 - (FN + FP + IDSW) / GT."""
    last_id: dict[int, int] = {}
    ids_seen: dict[int, set[int]] = {}
    fn = fp = idsw = n_gt = 0
    for frame in sequence:
        outs = tracker.update(frame.boxes, frame.scores)
        gt_ids = list(frame.truth)
        gt_boxes = np.array([frame.truth[i] for i in gt_ids]).reshape(-1, 4)
        matches, unmatched_tr, unmatched_gt = associate(np.array([o.box for o in outs]).reshape(-1, 4),
                                                        gt_boxes, iou_threshold)
        n_gt += len(gt_ids)
        fn += len(unmatched_gt)
        fp += len(unmatched_tr)
        for ti, gi in matches:
            oid, tid = gt_ids[gi], outs[ti].track_id
            if oid in last_id and last_id[oid] != tid:
                idsw += 1
            last_id[oid] = tid
            ids_seen.setdefault(oid, set()).add(tid)
    return MotMetrics(1 - (fn + fp + idsw) / max(n_gt, 1), idsw, fn, fp, n_gt,
                      {k: len(v) for k, v in ids_seen.items()})


def make_trackers() -> dict[str, object]:
    return {"iou": IoUTracker(), "sort": SortTracker(), "bytetrack": ByteTracker()}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--frames", type=int, default=60)
    ap.add_argument("--ball-speed", type=float, default=18.0)
    args = ap.parse_args()

    totals: dict[str, list[MotMetrics]] = {k: [] for k in make_trackers()}
    for seed in range(args.seeds):
        seq = simulate(seed, args.frames, args.ball_speed)
        for name, tracker in make_trackers().items():
            totals[name].append(evaluate(tracker, seq))
    for name, results in totals.items():
        if args.seeds == 1:
            print(f"{name:<10} {results[0]}")
        else:
            mota = np.mean([r.mota for r in results])
            sw = np.mean([r.id_switches for r in results])
            print(f"{name:<10} mean MOTA {mota:5.2f}   mean ID switches {sw:5.1f}   over {args.seeds} sequences")


if __name__ == "__main__":
    main()
