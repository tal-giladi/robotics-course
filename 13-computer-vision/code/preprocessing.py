"""Lesson 13.02 — OpenCV preprocessing: blur, threshold, morphology, and what each costs.

  py preprocessing.py               synthetic scenes
  py preprocessing.py --camera 0    also time the pipeline on a real webcam frame

Prints measurements and writes images to ./out/.
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from synthetic import ball_scene
from webcam import grab_frame, out_path


def time_ms(fn, *args, repeat: int = 30) -> float:
    """Median wall time of fn(*args) in milliseconds (median ignores the odd slow run)."""
    fn(*args)                                   # warm-up
    samples = []
    for _ in range(repeat):
        t0 = time.perf_counter()
        fn(*args)
        samples.append((time.perf_counter() - t0) * 1000)
    return float(np.median(samples))


def noise_std(gray: np.ndarray, roi=(slice(20, 120), slice(20, 200))) -> float:
    """Noise estimate: std of a flat region after removing its smooth trend (a 15x15 box blur)."""
    patch = gray[roi].astype(np.float32)
    trend = cv2.blur(patch, (15, 15))
    return float((patch - trend)[10:-10, 10:-10].std())


def blur_comparison(gray: np.ndarray) -> list[tuple[str, float, float, np.ndarray]]:
    """(name, noise std after, time ms, result) for the standard smoothing filters."""
    filters = [
        ("none", lambda g: g),
        ("box 5x5", lambda g: cv2.blur(g, (5, 5))),
        ("gaussian 5x5 s=1.1", lambda g: cv2.GaussianBlur(g, (5, 5), 1.1)),
        ("median 5", lambda g: cv2.medianBlur(g, 5)),
        ("bilateral d=9", lambda g: cv2.bilateralFilter(g, 9, 40, 5)),
    ]
    return [(name, noise_std(f(gray)), time_ms(f, gray), f(gray)) for name, f in filters]


def add_salt_pepper(gray: np.ndarray, fraction: float, rng: np.random.Generator) -> np.ndarray:
    out = gray.copy()
    r = rng.random(gray.shape)
    out[r < fraction / 2] = 0
    out[r > 1 - fraction / 2] = 255
    return out


def naive_orange_mask(bgr: np.ndarray) -> np.ndarray:
    """Hue-only mask (deliberately naive — 13.03 builds the robust version)."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, (5, 150, 60), (22, 255, 255))


def count_blobs(mask: np.ndarray) -> int:
    n, _ = cv2.connectedComponents(mask)
    return n - 1                                   # label 0 is the background


def clean_mask(mask: np.ndarray, open_k: int = 5, close_k: int = 7) -> np.ndarray:
    """Opening removes specks smaller than the kernel; closing fills holes and gaps."""
    k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_k, open_k))
    k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_k, close_k))
    m = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k_open)
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, k_close)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=None)
    args = ap.parse_args()
    rng = np.random.default_rng(1)

    # 1. Blur: noise vs. time vs. edge preservation
    img, truth = ball_scene(rng, ball_xy=(1.1, 0.1), noise_sigma=8.0)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print("1) smoothing a noisy frame (sigma = 8 gray levels added)")
    for name, sd, ms, res in blur_comparison(gray):
        print(f"   {name:<20} noise std {sd:5.2f}   {ms:6.2f} ms")
        cv2.imwrite(str(out_path(f"13.02-blur-{name.split()[0]}.png")), res)

    clean, _ = ball_scene(np.random.default_rng(1), ball_xy=(1.1, 0.1), noise_sigma=0.0)
    clean_gray = cv2.cvtColor(clean, cv2.COLOR_BGR2GRAY)
    sp = add_salt_pepper(clean_gray, 0.02, rng)
    for name, f in (("gaussian", lambda g: cv2.GaussianBlur(g, (5, 5), 1.1)),
                    ("median", lambda g: cv2.medianBlur(g, 5))):
        err = np.abs(f(sp).astype(int) - f(clean_gray).astype(int))
        print(f"   salt-and-pepper 2% -> {name:<8} pixels off by > 20 levels: {np.count_nonzero(err > 20)}")

    # 2. Threshold under a lighting gradient: global vs Otsu vs adaptive
    print("2) finding the dark grout lines under a strong lighting gradient")
    dark, _ = ball_scene(rng, ball_xy=(1.1, 0.1), light=(0.35, 1.3), noise_sigma=4.0, distractors=False)
    g = cv2.GaussianBlur(cv2.cvtColor(dark, cv2.COLOR_BGR2GRAY), (5, 5), 1.1)
    floor = g[300:, :]                                            # bottom of the image = floor
    _, fixed = cv2.threshold(floor, 90, 255, cv2.THRESH_BINARY_INV)
    otsu_t, otsu = cv2.threshold(floor, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    adaptive = cv2.adaptiveThreshold(floor, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                     cv2.THRESH_BINARY_INV, 31, 12)
    thirds = np.array_split(np.arange(floor.shape[1]), 3)
    for name, m in (("fixed 90", fixed), (f"otsu ({otsu_t:.0f})", otsu), ("adaptive 31/12", adaptive)):
        cover = [100.0 * np.count_nonzero(m[:, c]) / m[:, c].size for c in thirds]
        print(f"   {name:<16} % of floor marked 'dark' in left/middle/right third: "
              f"{cover[0]:5.1f} {cover[1]:5.1f} {cover[2]:5.1f}")
        cv2.imwrite(str(out_path(f"13.02-threshold-{name.split()[0]}.png")), m)

    # 3. Morphology on a noisy color mask
    print("3) cleaning a color mask with morphology")
    noisy_img, _ = ball_scene(rng, ball_xy=(0.9, -0.05), noise_sigma=14.0)
    mask = naive_orange_mask(noisy_img)
    cleaned = clean_mask(mask)
    print(f"   blobs before: {count_blobs(mask)}, after open(5)+close(7): {count_blobs(cleaned)}")
    cv2.imwrite(str(out_path("13.02-mask-raw.png")), mask)
    cv2.imwrite(str(out_path("13.02-mask-clean.png")), cleaned)

    # 4. Resolution is the cheapest speed-up
    print("4) cost of a small pipeline (blur -> HSV -> inRange -> open/close)")

    def pipeline(bgr):
        blurred = cv2.GaussianBlur(bgr, (5, 5), 1.1)
        return clean_mask(naive_orange_mask(blurred))

    frame = img if args.camera is None else grab_frame(args.camera)
    half = cv2.resize(frame, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    print(f"   {frame.shape[1]}x{frame.shape[0]}: {time_ms(pipeline, frame):.2f} ms   "
          f"{half.shape[1]}x{half.shape[0]}: {time_ms(pipeline, half):.2f} ms")


if __name__ == "__main__":
    main()
