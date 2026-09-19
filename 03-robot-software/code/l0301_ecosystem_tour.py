"""Lesson 03.01 — what each library in the Python robotics stack actually does for karmel.

Run it to see (a) which parts of the stack your machine has and (b) why the array libraries
exist, measured on the robot's own workloads instead of on a toy benchmark:

    python 03-robot-software/code/l0301_ecosystem_tour.py
    python 03-robot-software/code/l0301_ecosystem_tour.py --scans 60 --quick

Workloads
  transform  360-beam LiDAR scans -> world frame, pure Python vs numpy (SE(2) on every point)
  associate  match a scan against a 4000-point map: brute force numpy vs scipy's KD-tree
  vision     grayscale + blur + Canny on one 640x480 frame (OpenCV), reported as max FPS

Nothing here needs a robot. Tests: python -m pytest 03-robot-software/code/test_l0301_ecosystem.py
"""

from __future__ import annotations

import argparse
import importlib
import math
import platform
import sys
import time
from dataclasses import dataclass

# What the course uses each package for. Keep in sync with labs/requirements.txt.
STACK = [
    ("numpy", "arrays: poses, scans, images, covariances - the lingua franca of the stack"),
    ("scipy", "optimization, KD-trees, linear algebra, filters, rotations (scipy.spatial.transform)"),
    ("matplotlib", "plotting telemetry, trajectories and maps (headless on the robot)"),
    ("cv2", "OpenCV: camera capture, ArUco markers, calibration, image processing"),
    ("yaml", "PyYAML: karmel.yaml, ROS parameter and map files"),
    ("serial", "pyserial: the USB link to the Pico (robotlab.serial_base)"),
    ("pytest", "tests that run without hardware (lesson 03.08)"),
    ("rclpy", "ROS 2 client library - only inside a sourced ROS 2 environment (module 04)"),
    ("torch", "PyTorch - modules 16-18, on the Jetson rather than the Pi"),
]


@dataclass(frozen=True)
class Timing:
    """One measured workload."""

    name: str
    variant: str
    seconds: float
    note: str = ""

    @property
    def ms(self) -> float:
        return self.seconds * 1000.0


def library_versions() -> list[tuple[str, str, str]]:
    """[(import name, version or 'not installed', what the course uses it for)]."""
    rows = []
    for name, purpose in STACK:
        try:
            module = importlib.import_module(name)
        except Exception:  # noqa: BLE001 — a broken install should read as "not installed"
            rows.append((name, "not installed", purpose))
            continue
        rows.append((name, str(getattr(module, "__version__", "unknown")), purpose))
    return rows


# --- workload 1: transform LiDAR points into the world frame ---------------------------------------
def make_scan(samples: int) -> tuple[list[float], list[float]]:
    """A synthetic 360-beam scan: ranges in meters, angles in radians (like robotlab LaserScan)."""
    angles = [-math.pi + 2 * math.pi * i / samples for i in range(samples)]
    ranges = [1.0 + 0.5 * math.sin(3 * a) for a in angles]
    return ranges, angles


def transform_python(ranges, angles, pose) -> list[tuple[float, float]]:
    """Sensor polar -> world Cartesian, one beam at a time (the obvious C#-style loop)."""
    x, y, theta = pose
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    points = []
    for r, a in zip(ranges, angles):
        px, py = r * math.cos(a), r * math.sin(a)
        points.append((x + cos_t * px - sin_t * py, y + sin_t * px + cos_t * py))
    return points


def transform_numpy(ranges, angles, pose):
    """The same thing as four array operations. numpy loops in C, not in the interpreter."""
    import numpy as np

    x, y, theta = pose
    local = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
    rotation = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    return local @ rotation.T + np.array([x, y])


# --- workload 2: associate scan points with a map ---------------------------------------------------
def nearest_bruteforce(points, map_points):
    """Nearest map point for every scan point, by evaluating the whole N x M distance matrix."""
    import numpy as np

    diff = points[:, None, :] - map_points[None, :, :]
    return np.argmin(np.einsum("ijk,ijk->ij", diff, diff), axis=1)


def nearest_kdtree(points, map_points):
    """The same answer from a KD-tree: build once, query in O(log M) per point (ICP, lesson 11.03)."""
    from scipy.spatial import cKDTree

    return cKDTree(map_points).query(points)[1]


# --- workload 3: one camera frame --------------------------------------------------------------------
def vision_frame(width: int = 640, height: int = 480):
    """Grayscale + Gaussian blur + Canny on a synthetic frame; returns (seconds, edge pixels)."""
    import cv2
    import numpy as np

    rng = np.random.default_rng(0)
    frame = rng.integers(0, 255, size=(height, width, 3), dtype=np.uint8)
    cv2.rectangle(frame, (200, 150), (440, 330), (255, 255, 255), -1)

    def pipeline():
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
        return cv2.Canny(blurred, 60, 180)

    edges = pipeline()  # warm-up: the first OpenCV call also loads and initialises the library
    return _time(pipeline, repeats=5), int((edges > 0).sum())


def _time(func, *args, repeats: int = 1) -> float:
    """Best of `repeats` — the minimum is the run least disturbed by the OS."""
    best = float("inf")
    for _ in range(repeats):
        start = time.perf_counter()
        func(*args)
        best = min(best, time.perf_counter() - start)
    return best


def run(scans: int = 30, samples: int = 360, map_points: int = 4000, quick: bool = False) -> list[Timing]:
    import numpy as np

    ranges, angles = make_scan(samples)
    pose = (1.0, 1.3, 0.4)
    ranges_np, angles_np = np.asarray(ranges), np.asarray(angles)

    results = [
        Timing("transform", "pure Python", _time(lambda: [transform_python(ranges, angles, pose) for _ in range(scans)]),
               f"{scans} scans x {samples} beams"),
        Timing("transform", "numpy", _time(lambda: [transform_numpy(ranges_np, angles_np, pose) for _ in range(scans)],
                                           repeats=3), f"{scans} scans x {samples} beams"),
    ]

    points = np.asarray(transform_numpy(ranges_np, angles_np, pose))
    rng = np.random.default_rng(1)
    world = rng.uniform(-3.0, 3.0, size=(map_points, 2))
    if not quick:
        results.append(Timing("associate", "numpy brute force", _time(nearest_bruteforce, points, world),
                              f"{len(points)} x {map_points} distances"))
    results.append(Timing("associate", "scipy cKDTree", _time(nearest_kdtree, points, world, repeats=3),
                          f"{len(points)} queries, tree built each time"))

    try:
        seconds, edge_pixels = vision_frame()
        results.append(Timing("vision", "OpenCV", seconds,
                              f"640x480, {edge_pixels} edge pixels, {1.0 / seconds:.0f} fps ceiling"))
    except ImportError:
        results.append(Timing("vision", "OpenCV", float("nan"), "opencv-python-headless not installed"))
    return results


def main(argv: list[str] | None = None) -> list[Timing]:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scans", type=int, default=30, help="LiDAR scans to transform (default 30)")
    parser.add_argument("--samples", type=int, default=360, help="beams per scan (default 360)")
    parser.add_argument("--map-points", type=int, default=4000, help="map points to match against (default 4000)")
    parser.add_argument("--quick", action="store_true", help="skip the brute-force association")
    args = parser.parse_args(argv)

    print(f"python {platform.python_version()} ({platform.machine()}) on {platform.system()}: {sys.executable}")
    print(f"{'package':<12} {'version':<14} used for")
    for name, version, purpose in library_versions():
        print(f"{name:<12} {version:<14} {purpose}")

    results = run(args.scans, args.samples, args.map_points, args.quick)
    print(f"\n{'workload':<11} {'variant':<20} {'ms':>9}  note")
    for r in results:
        print(f"{r.name:<11} {r.variant:<20} {r.ms:>9.2f}  {r.note}")

    by_name: dict[str, list[Timing]] = {}
    for r in results:
        by_name.setdefault(r.name, []).append(r)
    for name, group in by_name.items():
        if len(group) > 1 and group[-1].seconds > 0:
            print(f"{name}: {group[0].variant} is {group[0].seconds / group[-1].seconds:.0f}x "
                  f"slower than {group[-1].variant}")
    return results


if __name__ == "__main__":
    main()
