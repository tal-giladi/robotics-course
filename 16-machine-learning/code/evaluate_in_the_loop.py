"""16.06 — evaluate a detector the way the robot experiences it, not the way a leaderboard does.

    py evaluate_in_the_loop.py task          # offline AP vs closed-loop "find the bottle" success (Monte Carlo)
    py evaluate_in_the_loop.py calibration   # are scores probabilities? reliability table, ECE, temperature scaling
    py evaluate_in_the_loop.py shift         # field monitor: does today's camera data look like the training data?

All three use a *parametric* detector (recall vs distance, false alarms, latency, over-confidence)
so you can change one property at a time and see what the robot feels. Swap in logs from your
real detector (score, correct?) once you have them; the analysis code does not change.
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from detection_metrics import average_precision

HERE = Path(__file__).resolve().parent


# ------------------------------------------------------------------------------ a parametric detector
@dataclass(frozen=True)
class DetectorModel:
    name: str
    latency_s: float           # image capture -> detections available, measured on the robot's computer
    recall_near: float         # probability of detecting a visible bottle at 0.5 m
    range_m: float             # distance at which recall has halved
    false_alarms_per_frame: float
    overconfidence: float      # 1.0 = calibrated scores; >1 = scores pushed towards 0/1

    def recall(self, dist_m: float) -> float:
        return self.recall_near / (1.0 + (max(dist_m - 0.5, 0.0) / (self.range_m - 0.5)) ** 2)

    def score(self, correct: bool, rng: np.random.Generator) -> float:
        """Draw a score. The *true* probability of being correct is p; the reported score is
        p pushed through a logit scaled by `overconfidence` (what an un-calibrated net does)."""
        p = float(np.clip(rng.beta(6, 2) if correct else rng.beta(2, 3), 1e-4, 1 - 1e-4))
        logit = math.log(p / (1 - p)) * self.overconfidence
        return 1.0 / (1.0 + math.exp(-logit))


BIG = DetectorModel("big (ResNet-50 class)", latency_s=1.60, recall_near=0.97, range_m=3.5,
                    false_alarms_per_frame=0.05, overconfidence=2.0)
SMALL = DetectorModel("small (MobileNet class)", latency_s=0.12, recall_near=0.90, range_m=2.2,
                      false_alarms_per_frame=0.08, overconfidence=2.0)


def offline_ap(model: DetectorModel, rng: np.random.Generator, n_images: int = 3000) -> float:
    """AP@0.5 on a held-out image set with bottles at 0.5-4 m: what a benchmark table would report."""
    preds, gts = [], []
    gt_box = np.array([[100, 50, 130, 150]], float)
    for _ in range(n_images):
        d = rng.uniform(0.5, 4.0)
        boxes, scores = [], []
        if rng.random() < model.recall(d):
            boxes.append(gt_box[0]); scores.append(model.score(True, rng))
        for _ in range(rng.poisson(model.false_alarms_per_frame)):
            boxes.append([10, 10, 40, 60]); scores.append(model.score(False, rng))
        preds.append((np.array(boxes, float).reshape(-1, 4), np.array(scores), np.ones(len(scores), int)))
        gts.append((gt_box, np.array([1])))
    return average_precision(preds, gts, 1)


# ------------------------------------------------------------------------------ 1. closed-loop task
def run_episode(model: DetectorModel, rng: np.random.Generator, score_thr: float, confirm: tuple[int, int],
                use_capture_pose: bool, max_time_s: float = 45.0) -> tuple[str, float]:
    """karmel spins in place to search, then drives to the first thing it believes is a bottle.

    World: one bottle 1.0-3.5 m away at a random bearing. Behaviour: rotate at 0.5 rad/s; when
    `confirm` = (k, n) means k of the last n detector outputs contain a detection >= score_thr,
    stop spinning and drive at 0.25 m/s to the detected position. The detection's bearing is
    relative to the camera AT CAPTURE TIME; `use_capture_pose=False` is the classic bug of
    combining it with the robot's CURRENT heading. Success = within 0.35 m of the bottle.
    Returns (outcome, time).
    """
    dt, fov = 0.05, 1.20
    rng_b, brg = rng.uniform(1.0, 3.5), rng.uniform(-math.pi, math.pi)
    bx, by = rng_b * math.cos(brg), rng_b * math.sin(brg)
    x = y = yaw = t = 0.0
    mode, history, pending, goal = "search", [], None, None
    while t < max_time_s:
        if mode == "search":
            yaw += 0.5 * dt
        else:
            heading = math.atan2(goal[1] - y, goal[0] - x)
            x += 0.25 * dt * math.cos(heading)
            y += 0.25 * dt * math.sin(heading)
            if math.hypot(bx - x, by - y) < 0.35:
                return "success", t
            if math.hypot(goal[0] - x, goal[1] - y) < 0.1:
                return "drove to a wrong place", t
        t += dt
        if mode != "search":
            continue
        if pending is None:                                   # grab a frame, start inference
            pending = (t + model.latency_s, x, y, yaw)
        if t >= pending[0]:                                   # inference finished
            _, cx, cy, cyaw = pending
            pending = None
            rel = (math.atan2(by - cy, bx - cx) - cyaw + math.pi) % (2 * math.pi) - math.pi
            dist = math.hypot(bx - cx, by - cy)
            hits = []                                         # (score, bearing relative to camera, range)
            if abs(rel) < fov / 2 and rng.random() < model.recall(dist):
                hits.append((model.score(True, rng), rel + rng.normal(0, 0.02), dist * rng.uniform(0.9, 1.1)))
            for _ in range(rng.poisson(model.false_alarms_per_frame)):
                hits.append((model.score(False, rng), rng.uniform(-fov / 2, fov / 2), rng.uniform(1.0, 3.0)))
            best = max((h for h in hits if h[0] >= score_thr), default=None)
            history = (history + [best])[-confirm[1]:]
            if best is not None and sum(h is not None for h in history) >= confirm[0]:
                ox, oy, oyaw = (cx, cy, cyaw) if use_capture_pose else (x, y, yaw)
                goal = (ox + best[2] * math.cos(oyaw + best[1]), oy + best[2] * math.sin(oyaw + best[1]))
                mode = "approach"
    return "timeout", t


def cmd_task(episodes: int) -> None:
    rng = np.random.default_rng(0)
    configs = [  # (model, score threshold, (k, n) confirmation, use capture pose)
        (BIG, 0.5, (1, 1), False), (BIG, 0.5, (1, 1), True), (BIG, 0.8, (2, 3), True),
        (SMALL, 0.5, (1, 1), False), (SMALL, 0.5, (1, 1), True), (SMALL, 0.8, (2, 3), True),
    ]
    aps = {m.name: offline_ap(m, rng) for m in (BIG, SMALL)}
    print(f"{'detector':24s} {'AP50':>5s} {'latency':>7s} {'thr':>4s} {'k/n':>4s} {'pose used':>9s} "
          f"{'success':>7s} {'wrong':>6s} {'timeout':>7s} {'median time':>11s}")
    for model, thr, confirm, capture in configs:
        runs = [run_episode(model, rng, thr, confirm, capture) for _ in range(episodes)]
        outcome = [r[0] for r in runs]
        times = [r[1] for r in runs if r[0] == "success"]
        print(f"{model.name:24s} {aps[model.name]:5.2f} {model.latency_s:6.2f}s {thr:4.1f} {confirm[0]}/{confirm[1]:<2d} "
              f"{'capture' if capture else 'current':>9s} {outcome.count('success') / episodes:7.0%} "
              f"{outcome.count('drove to a wrong place') / episodes:6.0%} {outcome.count('timeout') / episodes:7.0%} "
              + (f"{np.median(times):10.1f}s" if times else f"{'-':>11s}"))


# ------------------------------------------------------------------------------ 2. calibration
def reliability(scores: np.ndarray, correct: np.ndarray, bins: int = 10) -> tuple[float, list[tuple]]:
    """Expected calibration error: weighted mean |accuracy - confidence| over score bins."""
    edges, rows, ece = np.linspace(0, 1, bins + 1), [], 0.0
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (scores >= lo) & (scores < hi if hi < 1 else scores <= hi)
        if m.sum() == 0:
            continue
        conf, acc = float(scores[m].mean()), float(correct[m].mean())
        ece += m.mean() * abs(acc - conf)
        rows.append((lo, hi, int(m.sum()), conf, acc))
    return ece, rows


def fit_temperature(scores: np.ndarray, correct: np.ndarray) -> float:
    """Temperature scaling (Guo et al. 2017): one scalar T, calibrated = sigmoid(logit(score) / T),
    chosen to minimise negative log-likelihood on a VALIDATION set. Ranking (and so AP) is unchanged."""
    logits = np.log(np.clip(scores, 1e-6, 1 - 1e-6) / np.clip(1 - scores, 1e-6, 1))
    best = min(np.linspace(0.25, 5.0, 96), key=lambda T: -np.mean(
        correct * np.log(1 / (1 + np.exp(-logits / T)) + 1e-9) + (1 - correct) * np.log(1 - 1 / (1 + np.exp(-logits / T)) + 1e-9)))
    return float(best)


def apply_temperature(scores: np.ndarray, T: float) -> np.ndarray:
    logits = np.log(np.clip(scores, 1e-6, 1 - 1e-6) / np.clip(1 - scores, 1e-6, 1))
    return 1 / (1 + np.exp(-logits / T))


def sample_scores(model: DetectorModel, n: int, rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """Each detection has a true chance p of being right; the network reports an over-confident score."""
    p = np.clip(rng.beta(2.0, 1.2, n), 1e-4, 1 - 1e-4)
    correct = (rng.random(n) < p).astype(float)
    scores = 1 / (1 + np.exp(-np.log(p / (1 - p)) * model.overconfidence))
    return scores, correct


def cmd_calibration() -> None:
    rng = np.random.default_rng(1)
    val_s, val_c = sample_scores(SMALL, 4000, rng)             # tune T here ...
    test_s, test_c = sample_scores(SMALL, 4000, rng)           # ... measure here
    T = fit_temperature(val_s, val_c)
    for label, s in (("raw scores", test_s), (f"temperature-scaled, T={T:.2f}", apply_temperature(test_s, T))):
        ece, rows = reliability(s, test_c)
        print(f"\n{label}: ECE = {ece:.3f}")
        print(f"  {'score bin':>11s} {'n':>5s} {'mean score':>10s} {'fraction correct':>16s}")
        for lo, hi, n, conf, acc in rows:
            print(f"  {lo:4.1f}-{hi:3.1f}   {n:5d} {conf:10.2f} {acc:16.2f}")
    thr_raw = 0.9
    thr_cal = float(apply_temperature(np.array([thr_raw]), T)[0])
    keep = test_s >= thr_raw
    print(f"\n'act only when score >= 0.9': {keep.mean():.0%} of detections pass, "
          f"{test_c[keep].mean():.0%} of those are correct (the raw score promised >= 90 %). "
          f"Calibrated, the same detections read >= {thr_cal:.2f}.")


# ------------------------------------------------------------------------------ 3. distribution shift monitor
def psi(reference: np.ndarray, current: np.ndarray, bins: int = 10) -> float:
    """Population stability index over reference-quantile bins. Rule of thumb from credit-risk
    monitoring: < 0.1 stable, 0.1-0.25 drifting, > 0.25 shifted. A starting point, not a law."""
    inner = np.quantile(reference, np.linspace(0, 1, bins + 1)[1:-1])
    r = np.bincount(np.searchsorted(inner, reference), minlength=bins) / len(reference) + 1e-3
    c = np.bincount(np.searchsorted(inner, current), minlength=bins) / len(current) + 1e-3
    return float(np.sum((c - r) * np.log(c / r)))


def field_day(model: DetectorModel, rng: np.random.Generator, frames: int, shifted: bool) -> dict[str, np.ndarray]:
    """One day of detector outputs as the robot logs them (no labels in the field!).
    A shifted day = the robot works in a dim room it never saw: the net finds fewer bottles,
    is less sure about the ones it finds, and hallucinates more."""
    m = replace(model, recall_near=model.recall_near * (0.55 if shifted else 1.0),
                false_alarms_per_frame=model.false_alarms_per_frame * (2.5 if shifted else 1.0))
    scores, per_frame = [], []
    for _ in range(frames):
        n = 0
        if rng.random() < 0.4 and rng.random() < m.recall(rng.uniform(0.5, 3.0)):   # 40 % of frames show a bottle
            p = rng.beta(4, 3) if shifted else rng.beta(6, 2)
            scores.append(1 / (1 + math.exp(-math.log(p / (1 - p)) * m.overconfidence))); n += 1
        for _ in range(rng.poisson(m.false_alarms_per_frame)):
            scores.append(m.score(False, rng)); n += 1
        per_frame.append(n)
    return {"scores": np.array(scores), "per_frame": np.array(per_frame)}


def cmd_shift() -> None:
    rng = np.random.default_rng(3)
    reference = field_day(SMALL, rng, 5000, shifted=False)          # logged during the acceptance test
    ref_rate = reference["per_frame"].mean()
    print("daily field monitor (no labels needed): detector outputs vs the acceptance-test reference")
    print(f"{'day':>3s} {'condition':24s} {'dets/100 frames':>15s} {'uncertain 0.3-0.7':>18s} {'PSI(scores)':>12s}   alarm")
    for day in range(1, 8):
        shifted = day >= 6                                            # day 6: karmel moved to a dim new room
        today = field_day(SMALL, rng, 1500, shifted)
        s = today["scores"]
        uncertain = float(np.mean((s > 0.3) & (s < 0.7)))
        p = psi(reference["scores"], s)
        rate = 100 * today["per_frame"].mean()
        alarm = p > 0.25 or abs(rate - 100 * ref_rate) > 0.3 * 100 * ref_rate
        cond = "dim new room" if shifted else "usual rooms"
        print(f"{day:3d} {cond:24s} {rate:15.1f} {uncertain:18.0%} {p:12.2f}   {'ALARM' if alarm else '-'}")
    ref_unc = float(np.mean((reference['scores'] > 0.3) & (reference['scores'] < 0.7)))
    print(f"reference: {100 * ref_rate:.1f} detections/100 frames, {ref_unc:.0%} uncertain")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["task", "calibration", "shift"])
    ap.add_argument("--episodes", type=int, default=300)
    a = ap.parse_args()
    if a.cmd == "task":
        cmd_task(a.episodes)
    elif a.cmd == "calibration":
        cmd_calibration()
    else:
        cmd_shift()


if __name__ == "__main__":
    main()
