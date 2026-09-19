"""Lesson 13.01 — images as numpy arrays: shape, dtype, channels, BGR vs RGB, grayscale, HSV.

  py images_as_data.py              synthetic ball scene (no camera needed)
  py images_as_data.py --camera 0   one frame from a USB webcam

Writes PNGs to ./out/ so it also works over SSH on the robot (no window needed).
"""
from __future__ import annotations

import argparse

import cv2
import numpy as np

from synthetic import ball_scene
from webcam import grab_frame, out_path


def gray_by_hand(bgr: np.ndarray) -> np.ndarray:
    """ITU-R BT.601 luma, the weights cv2.COLOR_BGR2GRAY uses: Y = 0.299 R + 0.587 G + 0.114 B."""
    b, g, r = bgr[..., 0].astype(np.float32), bgr[..., 1].astype(np.float32), bgr[..., 2].astype(np.float32)
    return np.clip(np.round(0.299 * r + 0.587 * g + 0.114 * b), 0, 255).astype(np.uint8)


def hsv_of(bgr_pixel) -> tuple[int, int, int]:
    """HSV of one BGR pixel in OpenCV's 8-bit ranges: H 0-179 (degrees / 2), S 0-255, V 0-255."""
    h, s, v = cv2.cvtColor(np.uint8([[bgr_pixel]]), cv2.COLOR_BGR2HSV)[0, 0]
    return int(h), int(s), int(v)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=None, help="webcam index; omit for synthetic")
    args = ap.parse_args()

    if args.camera is None:
        img, truth = ball_scene(np.random.default_rng(0), ball_xy=(1.1, 0.1))
        u, v = int(round(truth.u)), int(round(truth.v))
    else:
        img = grab_frame(args.camera)
        v, u = img.shape[0] // 2, img.shape[1] // 2          # hold the ball in the image center

    print(f"type={type(img).__name__} shape={img.shape} dtype={img.dtype} "
          f"bytes={img.nbytes} ({img.nbytes / 1e6:.2f} MB)")
    h, w = img.shape[:2]
    print(f"width={w} height={h}  -> index as img[row=y, col=x]")

    px = img[v, u]
    print(f"pixel at (x={u}, y={v}) = {px.tolist()}  <- order is B, G, R")
    rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    print(f"same pixel after BGR->RGB = {rgb[v, u].tolist()}")
    print(f"HSV of that pixel (H 0-179, S, V) = {hsv_of(px)}")

    gray_cv = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_np = gray_by_hand(img)
    diff = np.abs(gray_cv.astype(int) - gray_np.astype(int))
    print(f"grayscale: shape={gray_cv.shape}, max |cv2 - by hand| = {diff.max()} gray level(s)")
    wrong = np.clip(np.round(0.299 * img[..., 0] + 0.587 * img[..., 1] + 0.114 * img[..., 2]), 0, 255)
    print(f"ball pixel gray: correct={gray_cv[v, u]}, with R and B swapped={int(wrong[v, u])}")

    # uint8 arithmetic wraps; OpenCV saturates
    brighter_np = img + np.uint8(100)
    brighter_cv = cv2.add(img, np.full_like(img, 100))
    print(f"wall pixel {img[40, 620].tolist()} +100: numpy={brighter_np[40, 620].tolist()} "
          f"cv2.add={brighter_cv[40, 620].tolist()}")

    # slices are views: writing into a crop writes into the image
    crop = img[v - 20:v + 20, u - 20:u + 20]
    print(f"crop shape={crop.shape}, shares memory with img: {np.shares_memory(crop, img)}")

    cv2.imwrite(str(out_path("13.01-original.png")), img)
    cv2.imwrite(str(out_path("13.01-swapped-channels.png")), rgb)   # what 'forgot BGR' looks like
    cv2.imwrite(str(out_path("13.01-gray.png")), gray_cv)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imwrite(str(out_path("13.01-hsv-channels.png")), np.hstack([hsv[..., 0] * 255 // 179, hsv[..., 1], hsv[..., 2]]).astype(np.uint8))
    b, g, r = cv2.split(img)
    cv2.imwrite(str(out_path("13.01-bgr-channels.png")), np.hstack([b, g, r]))
    print(f"wrote PNGs to {out_path('')}")


if __name__ == "__main__":
    main()
