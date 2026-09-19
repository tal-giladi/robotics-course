"""Synthetic webcam frames of karmel's target objects (bottle, cup) with exact bounding boxes.

Module 16 needs a detection dataset that anyone can regenerate on a laptop without a camera,
a download or a labeling session. Real data from your USB webcam replaces it in the practical
challenges; everything else (pipeline, splits, training, export, evaluation) stays the same.

What makes it useful for teaching rather than a toy:
  * frames come in SESSIONS (one drive through one ROOM under one LIGHTING). Consecutive
    frames of a session are near-duplicates, exactly like a 15 fps camera on a slow robot,
    so a random frame-level split leaks and a session-level split does not (16.03).
  * rooms and lighting differ in wall/floor colour, brightness, colour cast and noise, so a
    model trained on some rooms can be evaluated on an unseen room (distribution shift, 16.06).
  * distractors (books, balls, a vase-like shape) create realistic false positives.

    >>> rng = np.random.default_rng(0)
    >>> s = make_session("kitchen-evening", rng)
    >>> frames = list(drive_session(s, n_frames=5, rng=rng))
    >>> img, boxes, labels = frames[0]            # BGR uint8 (240, 320, 3), [[x1,y1,x2,y2]], [1|2]

Only numpy and OpenCV. All randomness comes from an explicit numpy Generator.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Iterator

import cv2
import numpy as np

WIDTH, HEIGHT = 320, 240
CLASSES = ("background", "bottle", "cup")     # index 0 is reserved for background (torchvision)
HORIZON_Y = 95                                 # wall/floor boundary in pixels

ROOMS: dict[str, dict] = {
    # wall BGR, floor BGR, floor tile size px
    "kitchen": {"wall": (200, 215, 225), "floor": (120, 140, 160), "tile": 28},
    "living": {"wall": (170, 190, 205), "floor": (60, 90, 130), "tile": 0},       # wooden floor, no tiles
    "office": {"wall": (215, 210, 200), "floor": (95, 95, 100), "tile": 40},
    "bedroom": {"wall": (190, 175, 200), "floor": (150, 160, 170), "tile": 0},
    "hallway": {"wall": (225, 225, 225), "floor": (110, 120, 125), "tile": 22},
}
LIGHTING: dict[str, dict] = {
    # global gain, BGR colour cast, sensor noise std, blur kernel (motion blur when > 1)
    "day": {"gain": 1.05, "cast": (1.00, 1.00, 1.00), "noise": 3.0, "blur": 1},
    "evening": {"gain": 0.70, "cast": (0.80, 0.95, 1.15), "noise": 7.0, "blur": 3},
    "night-lamp": {"gain": 0.45, "cast": (0.65, 0.90, 1.25), "noise": 11.0, "blur": 3},
}


@dataclass
class Session:
    """One recording: one room, one lighting, one arrangement of objects the robot drives past."""

    session_id: str
    room: str
    lighting: str
    objects: list[dict] = field(default_factory=list)   # each: kind, x (m-ish), depth, colour


def make_session(session_id: str, rng: np.random.Generator) -> Session:
    """``session_id`` is ``"<room>-<lighting>"`` optionally followed by ``-<n>``."""
    parts = session_id.split("-")
    room = parts[0]
    lighting = next(l for l in LIGHTING if session_id[len(room) + 1:].startswith(l))
    objects = []
    kinds = (["bottle"] * int(rng.integers(1, 4)) + ["cup"] * int(rng.integers(1, 3))      # cups are rarer
             + [str(k) for k in rng.choice(["book", "ball", "vase"], int(rng.integers(1, 3)))])
    for kind in kinds:
        objects.append({
            "kind": kind,
            "x": float(rng.uniform(-0.7, 0.7)),          # lateral position in the room, m
            "depth": float(rng.uniform(1.0, 2.8)),       # distance from the robot's start, m
            "colour": tuple(int(c) for c in rng.integers(30, 230, 3)),
            "hue": int(rng.integers(0, 4)),
        })
    return Session(session_id, room, lighting, objects)


# ------------------------------------------------------------------------------ drawing
def _background(room: str, rng: np.random.Generator) -> np.ndarray:
    r = ROOMS[room]
    img = np.empty((HEIGHT, WIDTH, 3), np.float32)
    img[:HORIZON_Y] = r["wall"]
    img[HORIZON_Y:] = r["floor"]
    if r["tile"]:
        for y in range(HORIZON_Y, HEIGHT, r["tile"] // 2):
            cv2.line(img, (0, y), (WIDTH, y), [c * 0.8 for c in r["floor"]], 1)
        for x in range(-WIDTH, 2 * WIDTH, r["tile"]):
            cv2.line(img, (WIDTH // 2 + (x - WIDTH // 2) // 4, HORIZON_Y), (x, HEIGHT),
                     [c * 0.8 for c in r["floor"]], 1)
    else:                                                  # wood grain / carpet texture
        grain = rng.normal(0, 8, (HEIGHT - HORIZON_Y, 1, 1)).astype(np.float32)
        img[HORIZON_Y:] += grain                           # horizontal stripes, broadcast over x
    return img


BOTTLE_COLOURS = [(60, 140, 40), (190, 150, 90), (235, 235, 235), (40, 60, 150)]   # green, blue, white, brown
CUP_COLOURS = [(40, 40, 200), (240, 240, 240), (40, 200, 230), (160, 90, 30)]       # red, white, yellow, blue


def _draw_bottle(img: np.ndarray, cx: int, base_y: int, h: int, colour) -> list[int]:
    w = max(4, int(h * 0.32))
    body_top = base_y - int(h * 0.62)
    shoulder = base_y - int(h * 0.78)
    neck_w = max(2, w // 3)
    cv2.rectangle(img, (cx - w // 2, body_top), (cx + w // 2, base_y), colour, -1)
    pts = np.array([[cx - w // 2, body_top], [cx + w // 2, body_top], [cx + neck_w // 2, shoulder],
                    [cx - neck_w // 2, shoulder]], np.int32)
    cv2.fillConvexPoly(img, pts, colour)
    cv2.rectangle(img, (cx - neck_w // 2, base_y - h), (cx + neck_w // 2, shoulder), colour, -1)
    cv2.rectangle(img, (cx - neck_w // 2 - 1, base_y - h), (cx + neck_w // 2 + 1, base_y - h + max(2, h // 12)),
                  (30, 30, 30), -1)                                                   # cap
    cv2.rectangle(img, (cx - w // 2 + 1, base_y - int(h * 0.45)), (cx + w // 2 - 1, base_y - int(h * 0.25)),
                  (250, 250, 250), -1)                                                # label
    cv2.line(img, (cx - w // 4, body_top + 2), (cx - w // 4, base_y - 3),
             [min(255, c + 50) for c in colour], 1)                                  # highlight
    return [cx - w // 2 - 1, base_y - h, cx + w // 2 + 1, base_y]


def _draw_cup(img: np.ndarray, cx: int, base_y: int, h: int, colour) -> list[int]:
    w_top, w_bot = int(h * 0.95), int(h * 0.7)
    top = base_y - h
    pts = np.array([[cx - w_top // 2, top], [cx + w_top // 2, top], [cx + w_bot // 2, base_y],
                    [cx - w_bot // 2, base_y]], np.int32)
    cv2.fillConvexPoly(img, pts, colour)
    cv2.ellipse(img, (cx, top), (w_top // 2, max(2, h // 8)), 0, 0, 360, [c * 0.6 for c in colour], -1)
    handle_r = max(3, h // 4)
    cv2.ellipse(img, (cx + w_top // 2, top + h // 2), (handle_r, handle_r + 1), 0, -90, 90, colour,
                max(2, h // 10))
    return [cx - w_top // 2 - 1, top - max(2, h // 8), cx + w_top // 2 + handle_r + max(2, h // 10), base_y]


def _draw_distractor(img: np.ndarray, kind: str, cx: int, base_y: int, h: int, colour) -> None:
    if kind == "book":
        cv2.rectangle(img, (cx - h // 2, base_y - h // 3), (cx + h // 2, base_y), colour, -1)
        cv2.line(img, (cx - h // 2, base_y - h // 6), (cx + h // 2, base_y - h // 6), (240, 240, 240), 1)
    elif kind == "ball":
        cv2.circle(img, (cx, base_y - h // 3), h // 3, colour, -1)
    else:                                                  # "vase": bottle-ish silhouette, round body
        cv2.ellipse(img, (cx, base_y - h // 3), (h // 4, h // 3), 0, 0, 360, colour, -1)
        cv2.rectangle(img, (cx - h // 12, base_y - h), (cx + h // 12, base_y - h // 2), colour, -1)


def _lighting(img: np.ndarray, lighting: str, rng: np.random.Generator) -> np.ndarray:
    l = LIGHTING[lighting]
    gradient = np.linspace(1.1, 0.85, WIDTH, dtype=np.float32)[None, :, None]   # lamp on the left
    out = img * l["gain"] * gradient * np.array(l["cast"], np.float32)
    if l["blur"] > 1:
        k = np.zeros((1, l["blur"]), np.float32) + 1.0 / l["blur"]
        out = cv2.filter2D(out, -1, k)
    out += rng.normal(0, l["noise"], out.shape).astype(np.float32)
    return np.clip(out, 0, 255).astype(np.uint8)


def render(session: Session, robot_y: float, robot_yaw: float, rng: np.random.Generator
           ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """One frame with the robot ``robot_y`` metres into the room, turned ``robot_yaw`` radians.

    Returns (BGR image, boxes float32 (N, 4) in x1,y1,x2,y2 pixels, labels int64 (N,)).
    Objects smaller than 8 px or mostly out of frame are not labelled (a human wouldn't either).
    """
    img = _background(session.room, rng)
    f = 260.0                                              # focal length in px (~63° FOV at 320 px)
    drawn = []
    for ob in session.objects:
        z = ob["depth"] + 0.8 - robot_y
        if z < 0.35:
            continue
        x_cam = ob["x"] * np.cos(robot_yaw) - z * np.sin(robot_yaw)
        z_cam = ob["x"] * np.sin(robot_yaw) + z * np.cos(robot_yaw)
        if z_cam < 0.35:
            continue
        cx = int(WIDTH / 2 + f * x_cam / z_cam)
        base_y = int(HORIZON_Y + 0.25 * f / z_cam)
        real_h = {"bottle": 0.25, "cup": 0.10, "book": 0.18, "ball": 0.12, "vase": 0.22}[ob["kind"]]
        h = int(f * real_h / z_cam * 1.6)
        drawn.append((z_cam, ob, cx, min(base_y, HEIGHT + h // 2), h))
    boxes, labels = [], []
    for z_cam, ob, cx, base_y, h in sorted(drawn, key=lambda d: -d[0]):   # far to near (occlusion)
        if ob["kind"] == "bottle":
            box = _draw_bottle(img, cx, base_y, h, BOTTLE_COLOURS[ob["hue"]])
        elif ob["kind"] == "cup":
            box = _draw_cup(img, cx, base_y, h, CUP_COLOURS[ob["hue"]])
        else:
            _draw_distractor(img, ob["kind"], cx, base_y, h, ob["colour"])
            continue
        x1, y1, x2, y2 = box
        cx1, cy1, cx2, cy2 = max(0, x1), max(0, y1), min(WIDTH - 1, x2), min(HEIGHT - 1, y2)
        visible = max(0, cx2 - cx1) * max(0, cy2 - cy1) / max(1, (x2 - x1) * (y2 - y1))
        if cx2 - cx1 >= 8 and cy2 - cy1 >= 8 and visible >= 0.5:
            boxes.append([cx1, cy1, cx2, cy2])
            labels.append(CLASSES.index(ob["kind"]))
    img = _lighting(img, session.lighting, rng)
    return (img, np.array(boxes, np.float32).reshape(-1, 4), np.array(labels, np.int64))


def drive_session(session: Session, n_frames: int, rng: np.random.Generator,
                  speed_m_per_frame: float = 0.01) -> Iterator[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """The robot drives slowly forward while sweeping its heading, like a search behaviour.

    At 15 fps and 0.15 m/s one frame is 1 cm of travel: neighbouring frames are almost identical.
    """
    yaw_amp = float(rng.uniform(0.2, 0.5))
    for k in range(n_frames):
        yield render(session, k * speed_m_per_frame, yaw_amp * np.sin(k / 25.0), rng)


if __name__ == "__main__":
    import pathlib
    out = pathlib.Path(__file__).resolve().parent.parent / "images"
    out.mkdir(exist_ok=True)
    rng = np.random.default_rng(3)
    tiles = []
    for sid in ["kitchen-day", "living-evening", "office-day", "bedroom-night-lamp"]:
        s = make_session(sid, rng)
        img, boxes, labels = next(drive_session(s, 1, rng))
        for (x1, y1, x2, y2), lab in zip(boxes.astype(int), labels):
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 1)
            cv2.putText(img, CLASSES[lab], (x1, max(8, y1 - 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 255, 0), 1)
        tiles.append(img)
        print(f"{sid:22s} objects={[o['kind'] for o in s.objects]} labelled={[CLASSES[l] for l in labels]}")
    grid = np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])
    cv2.imwrite(str(out / "16.02-synthetic-examples.png"), grid)
    print("wrote", out / "16.02-synthetic-examples.png")
