"""Minimal USB webcam access for the hardware variants of lessons 13.01-13.07 (catalog id `camera`).

On the robot the camera normally belongs to the ROS 2 driver (v4l2_camera, lesson 07.08). Stop
that node first, or these scripts will fail to open /dev/video0 ("device busy").
Works with a Logitech C920-class UVC webcam on Windows, macOS, Linux and the Raspberry Pi 5.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).resolve().parent / "out"


def out_path(name: str) -> Path:
    OUT.mkdir(exist_ok=True)
    return OUT / name


def open_camera(index: int = 0, width: int = 640, height: int = 480) -> cv2.VideoCapture:
    backend = cv2.CAP_DSHOW if sys.platform == "win32" else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open camera {index} (in use by another program or ROS node?)")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    return cap


def grab_frame(index: int = 0, width: int = 640, height: int = 480, warmup: int = 10) -> np.ndarray:
    """One BGR frame. The first frames are dropped: auto-exposure needs a moment to settle."""
    cap = open_camera(index, width, height)
    try:
        frame = None
        for _ in range(warmup + 1):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("camera opened but returned no frame")
            time.sleep(0.03)
        return frame
    finally:
        cap.release()
