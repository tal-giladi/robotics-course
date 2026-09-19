"""16.02 — attach labels, split by session, check balance and leakage, and freeze a dataset version.

    py dataset_tool.py labels data/karmel-objects --coco data/labels/*.coco.json
    py dataset_tool.py split  data/karmel-objects --test-rooms office --val-per-room 1
    py dataset_tool.py report data/karmel-objects
    py dataset_tool.py freeze data/karmel-objects --name karmel-objects

Files inside the dataset root (all plain JSON, diff-able, small):
  manifest.jsonl        one line per image (from bag_to_dataset.py)
  annotations.json      COCO: only the images in the manifest, boxes checked
  split.json            {"train": [sessions], "val": [...], "test": [...]}   <- sessions, never frames
  dataset_card.json     version id, counts, sources, policies (written by `freeze`)
"""
from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter, defaultdict
from datetime import date
from pathlib import Path

import cv2
import numpy as np


def load_manifest(root: Path) -> list[dict]:
    return [json.loads(line) for line in (root / "manifest.jsonl").read_text(encoding="utf-8").splitlines() if line]


# ------------------------------------------------------------------------------ labels
def cmd_labels(root: Path, coco_files: list[Path]) -> None:
    """Merge labeling-tool exports; keep only images we kept; reject broken boxes loudly."""
    manifest = {Path(r["file"]).relative_to("images").as_posix(): r for r in load_manifest(root)}
    images, annotations, categories, problems = [], [], None, Counter()
    for f in coco_files:
        coco = json.loads(f.read_text(encoding="utf-8"))
        categories = categories or coco["categories"]
        if coco["categories"] != categories:
            raise SystemExit(f"{f}: category list differs from the first export — fix the label schema first")
        by_image = defaultdict(list)
        for a in coco["annotations"]:
            by_image[a["image_id"]].append(a)
        for im in coco["images"]:
            if im["file_name"] not in manifest:
                continue                                    # frame was sampled away or excluded
            new_id = len(images) + 1
            images.append({**im, "id": new_id, "file_name": "images/" + im["file_name"]})
            for a in by_image[im["id"]]:
                x, y, w, h = a["bbox"]
                if w < 2 or h < 2:
                    problems["degenerate box"] += 1
                    continue
                if x < 0 or y < 0 or x + w > im["width"] + 1 or y + h > im["height"] + 1:
                    problems["box outside image"] += 1
                    continue
                annotations.append({**a, "id": len(annotations) + 1, "image_id": new_id})
    missing = len(manifest) - len(images)
    (root / "annotations.json").write_text(json.dumps(
        {"images": images, "annotations": annotations, "categories": categories}), encoding="utf-8")
    print(f"labelled images {len(images)}, boxes {len(annotations)}, images with no label file {missing}, "
          f"rejected {dict(problems) or 0}")


# ------------------------------------------------------------------------------ split
def cmd_split(root: Path, test_rooms: set[str], val_per_room: int) -> None:
    """Whole rooms go to test (unseen place), whole sessions go to val. Deterministic, no randomness:
    within a room, the session whose name hashes lowest goes to val, so re-running gives the same split."""
    sessions_by_room: dict[str, set[str]] = defaultdict(set)
    for r in load_manifest(root):
        sessions_by_room[r["room"]].add(r["session"])
    split = {"train": [], "val": [], "test": []}
    for room, sessions in sorted(sessions_by_room.items()):
        if room in test_rooms:
            split["test"] += sorted(sessions)
            continue
        ordered = sorted(sessions, key=lambda s: hashlib.sha256(s.encode()).hexdigest())
        n_val = min(val_per_room, len(ordered) - 1)          # never put a room's only session in val
        split["val"] += sorted(ordered[:n_val])
        split["train"] += sorted(ordered[n_val:])
    (root / "split.json").write_text(json.dumps(split, indent=2), encoding="utf-8")
    for k, v in split.items():
        print(f"{k:5s} {len(v):2d} sessions: {', '.join(v)}")


def nearest_neighbour_distance(root: Path, reference: list[str], queries: list[str]) -> float:
    """Median distance from each query image to its closest reference image (blurred 32x24 grey
    thumbnails, mean absolute grey-level difference). A frame whose twin is in the training set
    sits ~1 grey level away; a genuinely new view sits much further. A leakage smell detector."""
    from bag_to_dataset import thumbnail
    ref = np.stack([thumbnail(cv2.imread(str(root / f))) for f in reference])
    dists = [float(np.abs(ref - thumbnail(cv2.imread(str(root / f)))).mean(axis=(1, 2)).min()) for f in queries]
    return float(np.median(dists))


# ------------------------------------------------------------------------------ report
def cmd_report(root: Path) -> dict:
    manifest = load_manifest(root)
    coco = json.loads((root / "annotations.json").read_text(encoding="utf-8"))
    split = json.loads((root / "split.json").read_text(encoding="utf-8"))
    session_to_split = {s: k for k, v in split.items() for s in v}
    names = {c["id"]: c["name"] for c in coco["categories"]}
    session_of = {im["id"]: im["file_name"].split("/")[1] for im in coco["images"]}

    boxes = defaultdict(Counter)          # split -> class -> boxes
    images_with = defaultdict(Counter)    # split -> class -> images containing it
    negatives = Counter()                 # split -> images with no box at all
    per_image = defaultdict(set)
    for a in coco["annotations"]:
        k = session_to_split[session_of[a["image_id"]]]
        boxes[k][names[a["category_id"]]] += 1
        per_image[a["image_id"]].add(names[a["category_id"]])
    for im in coco["images"]:
        k = session_to_split[session_of[im["id"]]]
        for c in per_image[im["id"]]:
            images_with[k][c] += 1
        if not per_image[im["id"]]:
            negatives[k] += 1
    n_images = Counter(session_to_split[r["session"]] for r in manifest)

    print(f"{'split':5s} {'images':>6s} {'no-object':>9s} " + " ".join(f"{n + ' boxes':>12s} {n + ' imgs':>11s}" for n in names.values()))
    for k in ("train", "val", "test"):
        print(f"{k:5s} {n_images[k]:6d} {negatives[k]:9d} " +
              " ".join(f"{boxes[k][n]:12d} {images_with[k][n]:11d}" for n in names.values()))
    total = Counter()
    for k in boxes:
        total.update(boxes[k])
    common, rare = total.most_common()[0], total.most_common()[-1]
    print(f"imbalance: {common[0]}:{rare[0]} = {common[1] / max(1, rare[1]):.1f} : 1 (boxes, all splits)")

    files = defaultdict(list)
    for r in manifest:
        files[session_to_split[r["session"]]].append(r["file"])
    nn_session = nearest_neighbour_distance(root, files["train"], files["val"])
    shuffled = [r["file"] for r in manifest]                 # what a random FRAME split would have done
    np.random.default_rng(0).shuffle(shuffled)
    cut = len(files["train"])
    nn_frame = nearest_neighbour_distance(root, shuffled[:cut], shuffled[cut:cut + len(files["val"])])
    print(f"median val->train nearest-neighbour distance: session split {nn_session:.2f}, "
          f"random frame split {nn_frame:.2f} grey levels (small = near-copies = leakage)")
    return {"images": dict(n_images), "boxes": {k: dict(v) for k, v in boxes.items()},
            "negatives": dict(negatives), "val_train_nn_distance": round(nn_session, 2)}


# ------------------------------------------------------------------------------ freeze
def cmd_freeze(root: Path, name: str) -> None:
    """Version = hash of everything that defines the dataset: image bytes (via their sha256),
    labels and split. Change one box or move one session and the id changes."""
    h = hashlib.sha256()
    for r in sorted(load_manifest(root), key=lambda r: r["file"]):
        h.update(f"{r['file']} {r['sha256']}\n".encode())
    for f in ("annotations.json", "split.json"):
        h.update(json.dumps(json.loads((root / f).read_text(encoding="utf-8")), sort_keys=True).encode())
    version = h.hexdigest()[:12]
    stats = cmd_report(root)
    card = {
        "name": name, "version": version, "id": f"{name}@{version}", "created": date.today().isoformat(),
        "classes": ["bottle", "cup"], "stats": stats,
        "sources": sorted({r["session"] for r in load_manifest(root)}),
        "sampling": "bag_to_dataset.py --every-s 0.5 --min-change 1.0",
        "split_policy": "by room (test) and by session (val); never by frame",
        "privacy": "bedroom/bathroom sessions excluded at extraction; faces blurred; stored locally only",
        "license": "private, do not publish (contains images of a home)",
    }
    (root / "dataset_card.json").write_text(json.dumps(card, indent=2), encoding="utf-8")
    print(f"dataset id: {card['id']}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("labels"); p.add_argument("root", type=Path); p.add_argument("--coco", nargs="+", type=Path, required=True)
    p = sub.add_parser("split"); p.add_argument("root", type=Path)
    p.add_argument("--test-rooms", default="office"); p.add_argument("--val-per-room", type=int, default=1)
    p = sub.add_parser("report"); p.add_argument("root", type=Path)
    p = sub.add_parser("freeze"); p.add_argument("root", type=Path); p.add_argument("--name", default="karmel-objects")
    a = ap.parse_args()
    if a.cmd == "labels":
        cmd_labels(a.root, a.coco)
    elif a.cmd == "split":
        cmd_split(a.root, {r for r in a.test_rooms.split(",") if r}, a.val_per_room)
    elif a.cmd == "report":
        cmd_report(a.root)
    else:
        cmd_freeze(a.root, a.name)


if __name__ == "__main__":
    main()
