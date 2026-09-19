"""16.02 — turn rosbag2 recordings into an image dataset: sample, de-duplicate, protect privacy.

    py bag_to_dataset.py data/bags/* --out data/karmel-objects --every-s 0.5 --min-change 1.0 \
        --exclude-rooms bedroom,bathroom

Reads MCAP or SQLite3 bags with `rosbags` (pure Python, works on Windows/macOS without ROS).
On the robot you can do the same with `rosbag2_py` (see the lesson); the logic is identical.

For every bag:
  1. read /karmel/session_info (room, lighting) -> skip the whole bag if the room is excluded
  2. keep a frame only if >= --every-s seconds passed since the last kept frame   (time sampling)
  3. ... and it differs from the last kept frame by >= --min-change grey levels  (drops "robot parked")
  4. blur faces (YuNet or Haar cascade; a floor, not a guarantee — see the lesson)
  5. write images/<session>/<stamp_ns>.jpg and one JSON line per image to manifest.jsonl
"""
from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore

IMAGE_TOPIC = "/camera/image_raw/compressed"
INFO_TOPIC = "/karmel/session_info"


@dataclass
class ManifestRow:
    file: str            # relative to the dataset root, e.g. images/kitchen-day-1/1789000000500000000.jpg
    session: str
    room: str
    lighting: str
    stamp_ns: int
    sha256: str
    faces_blurred: int


def thumbnail(img: np.ndarray) -> np.ndarray:
    """Blurred 32x24 grey thumbnail: cheap to compare, and averaging ~100 pixels per cell pushes
    sensor noise well below the change a moving robot causes (measured: parked 0.1-0.9, driving 1-16)."""
    small = cv2.resize(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), (32, 24), interpolation=cv2.INTER_AREA)
    return cv2.GaussianBlur(small.astype(np.float32), (3, 3), 0)


def haar_file() -> Path | None:
    """pip's opencv-python ships the cascades in cv2.data; Ubuntu's apt package `opencv-data` in /usr/share."""
    candidates = [Path(cv2.data.haarcascades)] if hasattr(cv2, "data") else []
    candidates += [Path("/usr/share/opencv4/haarcascades"), Path("/usr/share/opencv/haarcascades")]
    return next((c / "haarcascade_frontalface_default.xml" for c in candidates
                 if (c / "haarcascade_frontalface_default.xml").exists()), None)


class FaceBlurrer:
    """Blur faces before an image is written anywhere. Best effort: detectors miss profiles,
    small and dark faces, so this complements the room exclusion policy, it doesn't replace it.

    Backends, in order: YuNet (``--yunet-model``, OpenCV >= 4.8, MIT-licensed model from
    opencv_zoo), then Haar cascades (OpenCV 4.x only; OpenCV 5 removed them). With neither, the
    tool refuses to run unless you pass ``--no-face-blur`` for recordings you know contain no people.
    """

    def __init__(self, yunet_model: Path | None = None, enabled: bool = True) -> None:
        self.backend = "off"
        if not enabled:
            return
        if yunet_model is not None:
            self.detector = cv2.FaceDetectorYN.create(str(yunet_model), "", (320, 240), 0.6)
            self.backend = "yunet"
        elif hasattr(cv2, "CascadeClassifier") and (xml := haar_file()) is not None:
            self.detector = cv2.CascadeClassifier(str(xml))
            self.backend = "haar"
        else:
            raise SystemExit("no face detector found (OpenCV 5 dropped Haar cascades; Ubuntu's python3-opencv "
                             "needs the opencv-data package). Pass --yunet-model face_detection_yunet_2023mar.onnx, "
                             "or --no-face-blur only if no people were recorded")

    def detect(self, img: np.ndarray) -> list[tuple[int, int, int, int]]:
        if self.backend == "yunet":
            self.detector.setInputSize((img.shape[1], img.shape[0]))
            _, faces = self.detector.detect(img)
            return [] if faces is None else [tuple(int(v) for v in f[:4]) for f in faces]
        if self.backend == "haar":
            grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            return [tuple(int(v) for v in f) for f in
                    self.detector.detectMultiScale(grey, scaleFactor=1.1, minNeighbors=5, minSize=(16, 16))]
        return []

    def __call__(self, img: np.ndarray) -> tuple[np.ndarray, int]:
        faces = self.detect(img)
        for x, y, w, h in faces:
            pad = int(0.2 * max(w, h))
            x0, y0 = max(0, x - pad), max(0, y - pad)
            x1, y1 = min(img.shape[1], x + w + pad), min(img.shape[0], y + h + pad)
            img[y0:y1, x0:x1] = cv2.GaussianBlur(img[y0:y1, x0:x1], (0, 0), sigmaX=max(w, h) / 3 + 1)
        return img, len(faces)


def extract_bag(bag: Path, out: Path, every_s: float, min_change: float, exclude_rooms: set[str],
                blur: FaceBlurrer) -> tuple[list[ManifestRow], dict]:
    typestore = get_typestore(Stores.ROS2_JAZZY)
    stats = {"bag": bag.name, "frames": 0, "kept": 0, "too_soon": 0, "unchanged": 0, "excluded": False}
    rows: list[ManifestRow] = []
    with Reader(bag) as reader:
        info_conns = [c for c in reader.connections if c.topic == INFO_TOPIC]
        info = {"session": bag.name, "room": "unknown", "lighting": "unknown"}
        for conn, _, raw in reader.messages(connections=info_conns):
            info.update(json.loads(typestore.deserialize_cdr(raw, conn.msgtype).data))
        if info["room"] in exclude_rooms:
            stats["excluded"] = True
            stats["frames"] = sum(c.msgcount for c in reader.connections if c.topic == IMAGE_TOPIC)
            return rows, stats

        last_t: int | None = None
        last_thumb: np.ndarray | None = None
        cam_conns = [c for c in reader.connections if c.topic == IMAGE_TOPIC]
        for conn, t_ns, raw in reader.messages(connections=cam_conns):
            stats["frames"] += 1
            if last_t is not None and (t_ns - last_t) < every_s * 1e9:
                stats["too_soon"] += 1
                continue
            msg = typestore.deserialize_cdr(raw, conn.msgtype)
            img = cv2.imdecode(np.asarray(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            thumb = thumbnail(img)
            if last_thumb is not None and float(np.mean(np.abs(thumb - last_thumb))) < min_change:
                stats["unchanged"] += 1
                last_t = t_ns                  # restart the clock: don't keep the next near-copy either
                continue
            img, n_faces = blur(img)
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec   # sensor time, not record time
            rel = Path("images") / info["session"] / f"{stamp}.jpg"
            (out / rel).parent.mkdir(parents=True, exist_ok=True)
            data = bytes(msg.data) if n_faces == 0 else cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
            (out / rel).write_bytes(data)
            rows.append(ManifestRow(rel.as_posix(), info["session"], info["room"], info["lighting"], stamp,
                                    hashlib.sha256(data).hexdigest(), n_faces))
            last_t, last_thumb = t_ns, thumb
            stats["kept"] += 1
    return rows, stats


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--every-s", type=float, default=0.5, help="minimum time between kept frames")
    ap.add_argument("--min-change", type=float, default=1.0, help="minimum mean abs grey-level change")
    ap.add_argument("--exclude-rooms", default="bedroom,bathroom", help="comma-separated privacy exclusions")
    ap.add_argument("--yunet-model", type=Path, help="face_detection_yunet_2023mar.onnx from opencv_zoo")
    ap.add_argument("--no-face-blur", action="store_true", help="only for recordings with no people in them")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    blur = FaceBlurrer(args.yunet_model, enabled=not args.no_face_blur)
    print(f"face blurring: {blur.backend}")
    exclude = {r.strip() for r in args.exclude_rooms.split(",") if r.strip()}
    all_rows: list[ManifestRow] = []
    print(f"{'bag':22s} {'frames':>6s} {'kept':>5s} {'too soon':>8s} {'unchanged':>9s}")
    for bag in sorted(args.bags):
        rows, s = extract_bag(bag, args.out, args.every_s, args.min_change, exclude, blur)
        all_rows += rows
        note = "  EXCLUDED (privacy policy)" if s["excluded"] else ""
        print(f"{s['bag']:22s} {s['frames']:6d} {s['kept']:5d} {s['too_soon']:8d} {s['unchanged']:9d}{note}")
    with (args.out / "manifest.jsonl").open("w", encoding="utf-8") as f:
        for r in all_rows:
            f.write(json.dumps(asdict(r)) + "\n")
    faces = sum(r.faces_blurred for r in all_rows)
    print(f"total kept {len(all_rows)} images, {faces} faces blurred -> {args.out / 'manifest.jsonl'}")


if __name__ == "__main__":
    main()
