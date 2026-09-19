"""Lesson 13.04 — edges, corners, ORB features, matching with the ratio test, RANSAC homography.

  py features.py                     synthetic poster seen before/after the robot turns
  py features.py --camera 0          grab two webcam frames (turn the camera a little between them)

The second view is made with a known homography, so every match can be graded against the truth.
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

from camera_model import karmel_camera, rot_x, rot_y, rot_z
from synthetic import checkerboard_texture
from webcam import grab_frame, out_path


def poster(rng: np.random.Generator, width: int = 640, height: int = 480) -> np.ndarray:
    """A cluttered grayscale scene: shapes, lines and text on a smooth background."""
    yy, xx = np.mgrid[0:height, 0:width]
    img = np.clip(110 + 40 * np.sin(xx / 90.0) + 25 * np.cos(yy / 70.0), 0, 255).astype(np.uint8)
    for _ in range(40):
        kind = rng.integers(0, 3)
        color = int(rng.integers(0, 256))
        x, y = int(rng.integers(0, width)), int(rng.integers(0, height))
        if kind == 0:
            w, h = int(rng.integers(15, 90)), int(rng.integers(15, 90))
            cv2.rectangle(img, (x, y), (x + w, y + h), color, -1)
        elif kind == 1:
            cv2.circle(img, (x, y), int(rng.integers(8, 45)), color, -1)
        else:
            cv2.line(img, (x, y), (int(rng.integers(0, width)), int(rng.integers(0, height))), color, 3)
    for i in range(6):
        cv2.putText(img, f"KARMEL {rng.integers(100, 999)}", (int(rng.integers(0, width - 200)),
                    int(rng.integers(30, height))), cv2.FONT_HERSHEY_SIMPLEX, 1.0, int(rng.integers(0, 60)), 2)
    return img


def rotation_homography(K: np.ndarray, yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Pixels move by H = K R K^-1 when the camera only rotates (no translation -> no parallax)."""
    R = rot_z(roll) @ rot_x(pitch) @ rot_y(yaw)
    return K @ R @ np.linalg.inv(K)


def second_view(img: np.ndarray, H: np.ndarray, rng: np.random.Generator) -> np.ndarray:
    warped = cv2.warpPerspective(img, H, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR,
                                 borderValue=128)
    darker = warped.astype(np.float32) * 0.8 + 15            # exposure changed between frames
    return np.clip(darker + rng.normal(0, 3.0, img.shape), 0, 255).astype(np.uint8)


def edges_and_corners(gray: np.ndarray) -> dict:
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
    edges = cv2.Canny(blurred, 50, 150)
    harris = cv2.cornerHarris(np.float32(blurred), blockSize=2, ksize=3, k=0.04)
    shi = cv2.goodFeaturesToTrack(blurred, maxCorners=300, qualityLevel=0.01, minDistance=8)
    return {"edge_pixels": int(np.count_nonzero(edges)), "edges": edges,
            "harris_strong": int(np.count_nonzero(harris > 0.01 * harris.max())),
            "shi_tomasi": 0 if shi is None else len(shi), "shi_pts": shi}


def match_orb(img1: np.ndarray, img2: np.ndarray, n_features: int = 1000, ratio: float = 0.75,
              ransac_px: float = 3.0) -> dict:
    orb = cv2.ORB_create(nfeatures=n_features)
    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)
    result = {"kp1": kp1, "kp2": kp2, "raw": [], "good": [], "inliers": [], "H": None}
    if des1 is None or des2 is None or len(kp2) < 2:
        return result
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    knn = bf.knnMatch(des1, des2, k=2)
    raw = [m[0] for m in knn if len(m) >= 1]
    good = [m for m, n in (p for p in knn if len(p) == 2) if m.distance < ratio * n.distance]
    result.update(raw=raw, good=good)
    if len(good) >= 4:
        src = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)
        H, inl = cv2.findHomography(src, dst, cv2.RANSAC, ransac_px)
        if H is not None:
            result["H"] = H
            result["inliers"] = [m for m, keep in zip(good, inl.ravel()) if keep]
    return result


def match_errors(matches, kp1, kp2, H_true: np.ndarray) -> np.ndarray:
    """Distance (px) between where each match landed and where the true homography says it should."""
    if not matches:
        return np.array([])
    p1 = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
    p2 = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 2)
    expected = cv2.perspectiveTransform(p1, H_true).reshape(-1, 2)
    return np.linalg.norm(p2 - expected, axis=1)


def corner_error(H_est: np.ndarray, H_true: np.ndarray, w: int, h: int) -> float:
    c = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    return float(np.max(np.linalg.norm(cv2.perspectiveTransform(c, H_est) - cv2.perspectiveTransform(c, H_true), axis=2)))


def report(name: str, img1, img2, H_true, ratio: float = 0.75) -> dict:
    cv2.setRNGSeed(0)                                        # RANSAC is random: make runs repeatable
    match_orb(img1, img2, ratio=ratio)                       # warm-up (first call allocates)
    t0 = time.perf_counter()
    r = match_orb(img1, img2, ratio=ratio)
    ms = (time.perf_counter() - t0) * 1000
    print(f"--- {name}")
    print(f"keypoints {len(r['kp1'])}/{len(r['kp2'])}   ORB+match+RANSAC {ms:.0f} ms")
    for label in ("raw", "good", "inliers"):
        e = match_errors(r[label], r["kp1"], r["kp2"], H_true)
        correct = int(np.count_nonzero(e < 3.0)) if e.size else 0
        pct = 100.0 * correct / max(len(e), 1)
        print(f"{label:>8}: {len(r[label]):4d} matches, {correct:4d} correct (<3 px) = {pct:5.1f}%")
    if r["H"] is not None:
        r["corner_err"] = corner_error(r["H"], H_true, img1.shape[1], img1.shape[0])
        pts = np.float32([r["kp1"][m.queryIdx].pt for m in r["inliers"]]).reshape(-1, 1, 2)
        r["inlier_err"] = float(np.mean(np.linalg.norm(
            cv2.perspectiveTransform(pts, r["H"]) - cv2.perspectiveTransform(pts, H_true), axis=2)))
        print(f"estimated H vs true H: mean {r['inlier_err']:.2f} px where the features are, "
              f"max {r['corner_err']:.2f} px at the image corners")
    else:
        r["corner_err"] = float("inf")
        print("homography: not found")
    return r


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--ratio", type=float, default=0.75)
    args = ap.parse_args()
    rng = np.random.default_rng(4)
    cam = karmel_camera()

    if args.camera is not None:
        a = cv2.cvtColor(grab_frame(args.camera), cv2.COLOR_BGR2GRAY)
        input("turn the camera by ~5 degrees, then press Enter")
        b = cv2.cvtColor(grab_frame(args.camera), cv2.COLOR_BGR2GRAY)
        r = match_orb(a, b, ratio=args.ratio)
        print(f"keypoints {len(r['kp1'])}/{len(r['kp2'])}, good {len(r['good'])}, inliers {len(r['inliers'])}")
        vis = cv2.drawMatches(a, r["kp1"], b, r["kp2"], r["inliers"][:80], None)
        cv2.imwrite(str(out_path("13.04-camera-matches.png")), vis)
        return

    img1 = poster(rng)
    H_true = rotation_homography(cam.K, yaw=np.radians(8), pitch=np.radians(3), roll=np.radians(4))
    img2 = second_view(img1, H_true, rng)

    ec = edges_and_corners(img1)
    print(f"Canny edge pixels: {ec['edge_pixels']}   Harris strong responses: {ec['harris_strong']}   "
          f"Shi-Tomasi corners: {ec['shi_tomasi']}")
    cv2.imwrite(str(out_path("13.04-edges.png")), ec["edges"])

    r = report("textured poster, camera turned 8 deg", img1, img2, H_true, args.ratio)
    vis = cv2.drawMatches(img1, r["kp1"], img2, r["kp2"], r["inliers"][:60], None,
                          flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    cv2.imwrite(str(out_path("13.04-matches.png")), vis)

    tex, _ = checkerboard_texture(9, 6, px_per_square=48)
    board = np.full((480, 640), 128, np.uint8)
    board[(480 - tex.shape[0]) // 2:(480 - tex.shape[0]) // 2 + tex.shape[0],
          (640 - tex.shape[1]) // 2:(640 - tex.shape[1]) // 2 + tex.shape[1]] = tex
    H2 = rotation_homography(cam.K, yaw=np.radians(3), pitch=0.0, roll=np.radians(2))
    report("repetitive checkerboard, camera turned 3 deg", board, second_view(board, H2, rng), H2, args.ratio)


if __name__ == "__main__":
    main()
