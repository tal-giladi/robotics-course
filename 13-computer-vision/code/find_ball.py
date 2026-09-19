"""Lesson 13.03 — find the orange ball: HSV segmentation + contours + shape checks.

  py find_ball.py                       evaluate on synthetic scenes (lighting x distance sweep)
  py find_ball.py --naive               same sweep with the naive "largest orange blob" detector
  py find_ball.py --v-min 25 --min-area 30   same sweep with tuned thresholds
  py find_ball.py --camera 0            live: prints detections and FPS, saves annotated frames
  py find_ball.py --camera 0 --sample   print HSV statistics of the image center (tune thresholds)
"""
from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np

from synthetic import BALL_RADIUS_M, ball_scene
from webcam import open_camera, out_path


@dataclass(frozen=True)
class BallDetection:
    u: float              # center, pixels
    v: float
    radius_px: float      # radius of the minimum enclosing circle
    area_px: float
    circularity: float    # 4*pi*A / P^2: 1.0 for a perfect disc, 0.785 for a square
    fill: float           # contour area / enclosing-circle area: ~1 for a disc, 0.64 for a square


@dataclass(frozen=True)
class BallDetectorConfig:
    hsv_lo: tuple[int, int, int] = (5, 160, 50)      # H 0-179, S, V
    hsv_hi: tuple[int, int, int] = (22, 255, 255)
    min_area_px: float = 60.0                        # ~ a 9 px diameter disc: ball at about 3.5 m
    min_circularity: float = 0.70
    min_fill: float = 0.70
    blur_ksize: int = 5
    open_ksize: int = 5
    close_ksize: int = 7


def segment(bgr: np.ndarray, cfg: BallDetectorConfig) -> np.ndarray:
    blurred = cv2.GaussianBlur(bgr, (cfg.blur_ksize, cfg.blur_ksize), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, cfg.hsv_lo, cfg.hsv_hi)
    ko = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.open_ksize, cfg.open_ksize))
    kc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.close_ksize, cfg.close_ksize))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, ko)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kc)


def describe(contour: np.ndarray) -> BallDetection:
    area = cv2.contourArea(contour)
    perimeter = cv2.arcLength(contour, True)
    (u, v), r = cv2.minEnclosingCircle(contour)
    circ = 4 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0.0
    fill = area / (math.pi * r * r) if r > 0 else 0.0
    return BallDetection(u, v, r, area, circ, fill)


def find_ball(bgr: np.ndarray, cfg: BallDetectorConfig = BallDetectorConfig()) -> BallDetection | None:
    """The most ball-like blob that passes every check, or None."""
    mask = segment(bgr, cfg)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    best, best_score = None, 0.0
    for c in contours:
        d = describe(c)
        if d.area_px < cfg.min_area_px or d.circularity < cfg.min_circularity or d.fill < cfg.min_fill:
            continue
        score = d.area_px * d.fill                   # prefer big, round blobs
        if score > best_score:
            best, best_score = d, score
    return best


def find_ball_naive(bgr: np.ndarray) -> BallDetection | None:
    """What everyone writes first: hue range only, largest contour wins."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (5, 50, 50), (22, 255, 255))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    return describe(max(contours, key=cv2.contourArea))


def annotate(bgr: np.ndarray, det: BallDetection | None) -> np.ndarray:
    out = bgr.copy()
    if det is not None:
        cv2.circle(out, (int(round(det.u)), int(round(det.v))), int(round(det.radius_px)), (0, 255, 0), 2)
        cv2.putText(out, f"r={det.radius_px:.1f}px circ={det.circularity:.2f}",
                    (int(det.u) + 10, int(det.v) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return out


def evaluate(detector, seed: int = 3) -> dict:
    """Sweep lighting and distance; a detection is correct if its center is within
    max(3 px, 30% of the true radius) of the truth."""
    rng = np.random.default_rng(seed)
    lights = [(0.25, 0.45), (0.45, 0.8), (0.55, 1.15), (0.9, 1.4), (1.2, 1.6)]
    distances = [0.5, 0.8, 1.2, 1.8, 2.5, 3.2]
    rows = []
    for light in lights:
        for x in distances:
            y = float(rng.uniform(-0.25, 0.25)) * x
            img, truth = ball_scene(rng, ball_xy=(x, y), light=light)
            det = detector(img)
            err = math.hypot(det.u - truth.u, det.v - truth.v) if det else float("inf")
            ok = err <= max(3.0, 0.3 * truth.radius_px)
            rows.append((light, x, truth, det, err, ok))
    return {"rows": rows, "rate": sum(r[5] for r in rows) / len(rows)}


def run_camera(index: int, sample: bool, frames: int, cfg: BallDetectorConfig) -> None:
    cap = open_camera(index)
    try:
        t0, n = time.perf_counter(), 0
        for i in range(frames):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("no frame")
            if sample:
                h, w = frame.shape[:2]
                patch = cv2.cvtColor(frame[h // 2 - 10:h // 2 + 10, w // 2 - 10:w // 2 + 10], cv2.COLOR_BGR2HSV)
                p = patch.reshape(-1, 3)
                print(f"center HSV  min {p.min(0)}  median {np.median(p, 0).astype(int)}  max {p.max(0)}")
                continue
            det = find_ball(frame, cfg)
            n += 1
            fps = n / (time.perf_counter() - t0)
            if det:
                print(f"ball at u={det.u:6.1f} v={det.v:6.1f} r={det.radius_px:5.1f}px "
                      f"circ={det.circularity:.2f}  {fps:4.1f} fps")
            else:
                print(f"no ball  {fps:4.1f} fps")
            if i % 30 == 0:
                cv2.imwrite(str(out_path(f"13.03-camera-{i:04d}.png")), annotate(frame, det))
    finally:
        cap.release()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--sample", action="store_true", help="print HSV stats of the image center")
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--naive", action="store_true")
    ap.add_argument("--s-min", type=int, default=BallDetectorConfig.hsv_lo[1])
    ap.add_argument("--v-min", type=int, default=BallDetectorConfig.hsv_lo[2])
    ap.add_argument("--min-area", type=float, default=BallDetectorConfig.min_area_px)
    args = ap.parse_args()
    lo = BallDetectorConfig.hsv_lo
    cfg = BallDetectorConfig(hsv_lo=(lo[0], args.s_min, args.v_min), min_area_px=args.min_area)

    if args.camera is not None:
        run_camera(args.camera, args.sample, args.frames, cfg)
        return

    detector = find_ball_naive if args.naive else (lambda im: find_ball(im, cfg))
    result = evaluate(detector)
    print(f"{'light (L,R)':<13}{'dist m':>7}{'true r px':>10}{'found r px':>11}{'center err px':>14}  ok")
    for light, x, truth, det, err, ok in result["rows"]:
        found = f"{det.radius_px:.1f}" if det else "-"
        e = f"{err:.1f}" if math.isfinite(err) else "miss"
        print(f"{str(light):<13}{x:>7.1f}{truth.radius_px:>10.1f}{found:>11}{e:>14}  {'yes' if ok else 'NO'}")
    print(f"detection rate: {100 * result['rate']:.0f}%  (ball radius {BALL_RADIUS_M * 100:.1f} cm)")

    img, truth = ball_scene(np.random.default_rng(0), ball_xy=(1.1, 0.1))
    cv2.imwrite(str(out_path("13.03-mask.png")), segment(img, cfg))
    cv2.imwrite(str(out_path("13.03-detection.png")), annotate(img, detector(img)))


if __name__ == "__main__":
    main()
