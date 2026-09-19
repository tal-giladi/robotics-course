"""16.03 — see data leakage in 20 seconds, without training a detector.

    py leakage_demo.py data/karmel-objects

A deliberately dumb "model": 1-nearest-neighbour on 32x24 colour thumbnails, answering
"how many bottles are in this frame?" It can only memorise. If the validation score is high,
the validation set is made of frames the model has effectively already seen.

We score it three ways:
  frame split    random 70/30 split of the frames of the non-test rooms       (the mistake)
  session split  split.json from 16.02: whole sessions in val                  (honest)
  unseen room    train on all non-test sessions, evaluate on the office room   (harder, and what the robot meets)
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np


def features(path: Path) -> np.ndarray:
    small = cv2.resize(cv2.imread(str(path)), (32, 24), interpolation=cv2.INTER_AREA).astype(np.float32)
    return (small / 255.0).ravel()


def one_nn_accuracy(x_train: np.ndarray, y_train: np.ndarray, x_eval: np.ndarray, y_eval: np.ndarray) -> float:
    d = ((x_eval[:, None, :] - x_train[None, :, :]) ** 2).sum(-1)
    return float(np.mean(y_train[d.argmin(1)] == y_eval))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path)
    root = ap.parse_args().root
    coco = json.loads((root / "annotations.json").read_text(encoding="utf-8"))
    split = json.loads((root / "split.json").read_text(encoding="utf-8"))
    bottles = Counter(a["image_id"] for a in coco["annotations"] if a["category_id"] == 1)
    imgs = coco["images"]
    x = np.stack([features(root / im["file_name"]) for im in imgs])
    y = np.array([min(bottles[im["id"]], 3) for im in imgs])                 # 0, 1, 2, 3+ bottles
    session = np.array([im["file_name"].split("/")[1] for im in imgs])
    in_ = lambda names: np.isin(session, names)                               # noqa: E731

    pool = np.nonzero(~in_(split["test"]))[0]
    perm = np.random.default_rng(0).permutation(pool)
    cut = int(0.7 * len(perm))
    majority = np.mean(y[in_(split["val"])] == Counter(y[in_(split["train"])]).most_common(1)[0][0])
    rows = [
        ("frame split (random frames)", one_nn_accuracy(x[perm[:cut]], y[perm[:cut]], x[perm[cut:]], y[perm[cut:]])),
        ("session split (split.json)", one_nn_accuracy(x[in_(split["train"])], y[in_(split["train"])],
                                                       x[in_(split["val"])], y[in_(split["val"])])),
        ("unseen room (office)", one_nn_accuracy(x[pool], y[pool], x[in_(split["test"])], y[in_(split["test"])])),
        ("always guess the most common count", majority),
    ]
    print("1-NN 'how many bottles?' accuracy")
    for name, acc in rows:
        print(f"  {name:36s} {acc:6.1%}")


if __name__ == "__main__":
    main()
