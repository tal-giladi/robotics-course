"""16.02 — record synthetic "robot drives" as real ROS 2 bags (MCAP), one bag per session.

Stands in for `ros2 bag record` on karmel. Each bag has the same topics your robot records:

  /camera/image_raw/compressed   sensor_msgs/msg/CompressedImage   JPEG, 10 Hz (v4l2_camera + image_transport)
  /karmel/session_info           std_msgs/msg/String               one JSON message: room, lighting, operator

It also writes what a labeling tool would export for those frames (COCO JSON, one per session) to
data/labels/, so the rest of the pipeline can run without a labeling session. Real labels come
from CVAT or Label Studio (see the lesson).

Needs `pip install rosbags` (pure Python, Apache-2.0, no ROS install required).

    py make_demo_bags.py                      # -> data/bags/<session>/ and data/labels/<session>.coco.json
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import stat
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

from synthetic_objects import CLASSES, make_session, render

SESSIONS = [
    "kitchen-day-1", "kitchen-day-2", "kitchen-evening-1", "kitchen-evening-2",
    "living-day-1", "living-day-2", "living-evening-1", "living-night-lamp-1",
    "hallway-day-1", "hallway-day-2", "hallway-evening-1",
    "office-day-1", "office-evening-1",          # the room we will hold out as "unseen" (16.03, 16.06)
    "bedroom-day-1",                              # private room: excluded by the privacy policy (16.02)
]
FPS = 10
DURATION_S = 16.0
PAUSE = (4.0, 8.0)                                # the robot stops for 4 s: 40 near-identical frames


def trajectory(k: int, yaw_amp: float) -> tuple[float, float]:
    """(metres driven, heading) at frame k: 0.15 m/s with a stop, heading sweeping left/right."""
    t = k / FPS
    moving_time = min(t, PAUSE[0]) + max(0.0, t - PAUSE[1])
    frozen_t = min(t, PAUSE[0]) if PAUSE[0] <= t < PAUSE[1] else t
    return 0.15 * moving_time, yaw_amp * float(np.sin(frozen_t / 2.5))


def remove_tree(path: Path) -> None:
    """rosbag2 refuses to overwrite a bag, so delete the old one. On Windows, OneDrive marks synced
    folders read-only, which makes os.rmdir fail: clear the flag and retry."""
    def clear_readonly(func, p, _exc):
        os.chmod(p, stat.S_IWRITE)
        func(p)

    for _ in range(10):
        if not path.exists():
            return
        try:
            shutil.rmtree(path, onexc=clear_readonly) if sys.version_info >= (3, 12) else shutil.rmtree(path, onerror=clear_readonly)
        except PermissionError:
            time.sleep(0.5)
    if path.exists():
        raise SystemExit(f"cannot delete {path}: close programs using it or delete it by hand")


def record_session(session_id: str, root: Path, seed: int) -> dict:
    typestore = get_typestore(Stores.ROS2_JAZZY)
    CompressedImage = typestore.types["sensor_msgs/msg/CompressedImage"]
    Header = typestore.types["std_msgs/msg/Header"]
    Time = typestore.types["builtin_interfaces/msg/Time"]
    String = typestore.types["std_msgs/msg/String"]

    rng = np.random.default_rng(seed)
    session = make_session(session_id, rng)
    yaw_amp = float(rng.uniform(0.2, 0.5))
    bag_dir = root / "bags" / session_id
    remove_tree(bag_dir)
    t0_ns = 1_789_000_000_000_000_000 + seed * 3_600_000_000_000       # a date in Sept 2026, 1 h apart
    images, annotations = [], []
    with Writer(bag_dir, version=9, storage_plugin=StoragePlugin.MCAP) as writer:
        info = writer.add_connection("/karmel/session_info", String.__msgtype__, typestore=typestore)
        cam = writer.add_connection("/camera/image_raw/compressed", CompressedImage.__msgtype__, typestore=typestore)
        meta = {"session": session_id, "room": session.room, "lighting": session.lighting, "operator": "tal",
                "robot": "karmel", "camera": "usb-webcam-640x480-downscaled-320x240"}
        writer.write(info, t0_ns, typestore.serialize_cdr(String(data=json.dumps(meta)), String.__msgtype__))
        for k in range(int(DURATION_S * FPS)):
            stamp = t0_ns + k * 1_000_000_000 // FPS
            driven, yaw = trajectory(k, yaw_amp)
            img, boxes, labels = render(session, driven, yaw, rng)
            ok, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            assert ok
            msg = CompressedImage(
                header=Header(stamp=Time(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000),
                              frame_id="camera_optical_frame"),
                format="jpeg", data=np.frombuffer(jpeg.tobytes(), dtype=np.uint8))
            writer.write(cam, stamp, typestore.serialize_cdr(msg, CompressedImage.__msgtype__))
            image_id = len(images) + 1
            images.append({"id": image_id, "file_name": f"{session_id}/{stamp}.jpg", "width": img.shape[1],
                           "height": img.shape[0]})
            for (x1, y1, x2, y2), lab in zip(boxes.tolist(), labels.tolist()):
                annotations.append({"id": len(annotations) + 1, "image_id": image_id, "category_id": int(lab),
                                    "bbox": [x1, y1, x2 - x1, y2 - y1], "area": (x2 - x1) * (y2 - y1),
                                    "iscrowd": 0})
    coco = {"images": images, "annotations": annotations,
            "categories": [{"id": i, "name": n} for i, n in enumerate(CLASSES) if i > 0]}
    (root / "labels").mkdir(parents=True, exist_ok=True)
    (root / "labels" / f"{session_id}.coco.json").write_text(json.dumps(coco))
    return {"session": session_id, "frames": len(images), "boxes": len(annotations)}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=Path(__file__).with_name("data"))
    args = ap.parse_args()
    for seed, sid in enumerate(SESSIONS):
        r = record_session(sid, args.out, seed)
        size_kb = sum(f.stat().st_size for f in (args.out / "bags" / sid).iterdir()) // 1024
        print(f"{r['session']:22s} {r['frames']} frames  {r['boxes']:4d} labelled boxes  {size_kb:5d} kB")


if __name__ == "__main__":
    main()
