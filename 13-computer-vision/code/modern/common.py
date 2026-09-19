"""Shared helpers for the modern-vision lessons 13.09-13.14.

* sample images (downloaded on first use, never committed) with hand-checked ground truth
* the Detection record every detector in this folder returns
* drawing, output paths and a latency benchmark that reports median and p90, not a single run

Only numpy + Pillow here, so the geometry/tracking code and its tests run without PyTorch.
"""
from __future__ import annotations

import io
import statistics
import time
import urllib.request
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
OUT = HERE / "out"

# Objects the robot cares about later (pick-and-place in 15.x, the LLM agent in 19.x).
# Names are COCO category names, which torchvision and RT-DETR checkpoints both use.
ROBOT_TARGETS = ("bottle", "cup", "sports ball")


@dataclass(frozen=True)
class SampleImage:
    url: str
    credit: str
    # Ground truth for the robot target classes only, (label, [x1, y1, x2, y2]) in pixels.
    # Copied from the COCO val2017 annotations (bbox x,y,w,h converted to corners).
    truth: tuple[tuple[str, tuple[float, float, float, float]], ...] = field(default=())


# Both photos are COCO val2017 images published on Flickr under CC BY 2.0.
SAMPLES: dict[str, SampleImage] = {
    "table": SampleImage(
        url="http://images.cocodataset.org/val2017/000000520871.jpg",
        credit="COCO val2017 #520871, Flickr photo 3711888643, CC BY 2.0",
        truth=(("cup", (463.0, 13.0, 567.0, 135.0)),
               ("bottle", (413.0, 1.0, 503.0, 190.0))),
    ),
    "shelf": SampleImage(
        url="http://images.cocodataset.org/val2017/000000578922.jpg",
        credit="COCO val2017 #578922, Flickr photo 3072908271, CC BY 2.0",
        truth=(("bottle", (422.0, 543.0, 525.0, 640.0)),
               ("bottle", (538.0, 590.0, 598.0, 639.0)),
               ("cup", (318.0, 499.0, 429.0, 640.0)),
               ("cup", (376.0, 293.0, 450.0, 391.0))),
    ),
}


def out_path(name: str) -> Path:
    OUT.mkdir(parents=True, exist_ok=True)
    return OUT / name


def load_image(source: str) -> Image.Image:
    """Load an RGB image from 'sample:<name>', an http(s) URL or a file path."""
    if source.startswith("sample:"):
        sample = SAMPLES[source.split(":", 1)[1]]
        cached = out_path("cache") / Path(sample.url).name
        cached.parent.mkdir(parents=True, exist_ok=True)
        if not cached.exists():
            print(f"downloading {sample.url}  ({sample.credit})")
            cached.write_bytes(_download(sample.url))
        return Image.open(cached).convert("RGB")
    if source.startswith(("http://", "https://")):
        return Image.open(io.BytesIO(_download(source))).convert("RGB")
    return Image.open(source).convert("RGB")


def _download(url: str) -> bytes:
    req = urllib.request.Request(url, headers={"User-Agent": "robotics-course/1.0"})
    with urllib.request.urlopen(req, timeout=30) as r:
        return r.read()


@dataclass(frozen=True)
class Detection:
    """One detected object. box = (x1, y1, x2, y2) in pixels of the ORIGINAL image."""

    label: str
    score: float
    box: tuple[float, float, float, float]

    @property
    def center(self) -> tuple[float, float]:
        x1, y1, x2, y2 = self.box
        return ((x1 + x2) / 2, (y1 + y2) / 2)

    def __str__(self) -> str:
        x1, y1, x2, y2 = self.box
        return f"{self.label:<12} {self.score:5.2f}  [{x1:6.1f} {y1:6.1f} {x2:6.1f} {y2:6.1f}]"


def draw_detections(img: Image.Image, dets: list[Detection], color: str = "lime") -> Image.Image:
    canvas = img.copy()
    d = ImageDraw.Draw(canvas)
    for det in dets:
        d.rectangle(det.box, outline=color, width=3)
        d.text((det.box[0] + 3, det.box[1] + 2), f"{det.label} {det.score:.2f}", fill=color)
    return canvas


@dataclass(frozen=True)
class Latency:
    median_ms: float
    p90_ms: float
    runs: int

    def __str__(self) -> str:
        return f"median {self.median_ms:7.1f} ms   p90 {self.p90_ms:7.1f} ms   ({self.runs} runs)"


def benchmark(fn: Callable[[], object], warmup: int = 3, runs: int = 20) -> Latency:
    """Time fn() after warm-up. The first calls are slow (allocations, kernel selection): skip them."""
    for _ in range(warmup):
        fn()
    times = []
    for _ in range(runs):
        t0 = time.perf_counter()
        fn()
        times.append((time.perf_counter() - t0) * 1000.0)
    times.sort()
    p90 = times[min(len(times) - 1, int(round(0.9 * (len(times) - 1))))]
    return Latency(statistics.median(times), p90, runs)


def pil_to_bgr(img: Image.Image) -> np.ndarray:
    """PIL (RGB) -> OpenCV (BGR) array. Use when handing frames to 13.01-13.08 code."""
    return np.asarray(img)[:, :, ::-1].copy()


def bgr_to_pil(bgr: np.ndarray) -> Image.Image:
    """OpenCV frame (BGR) -> PIL RGB. Every pretrained network in this folder expects RGB."""
    return Image.fromarray(np.ascontiguousarray(bgr[:, :, ::-1]))
