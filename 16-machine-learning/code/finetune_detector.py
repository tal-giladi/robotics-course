"""16.03 — fine-tune a torchvision Faster R-CNN on karmel's objects (bottle, cup) and measure it honestly.

    # the real run: COCO-pretrained MobileNetV3 detector (downloads ~74 MB of weights once)
    py finetune_detector.py data/karmel-objects --arch mobilenet320 --weights coco --iters 600

    # the 1-minute smoke test: tiny run, no download, proves the pipeline works end to end
    py finetune_detector.py data/karmel-objects --arch mobilenet320 --weights none --iters 5 --eval-limit 8

    # the leakage experiment: split frames at random instead of by session
    py finetune_detector.py data/karmel-objects --split-mode frame ...

Reads the dataset produced in 16.02 (annotations.json + split.json). Trains on `train` sessions,
picks the checkpoint by `val` AP, and evaluates `test` (an unseen room) exactly once at the end.
Code: torchvision (BSD-3-Clause). Pretrained weights carry their own dataset terms (COCO) — see lesson.
"""
from __future__ import annotations

import argparse
import json
import math
import time
from collections import defaultdict
from pathlib import Path

import numpy as np
import torch
from torchvision import tv_tensors
from torchvision.io import ImageReadMode, decode_image
from torchvision.models.detection import (FasterRCNN_MobileNet_V3_Large_320_FPN_Weights,
                                          FasterRCNN_ResNet50_FPN_V2_Weights,
                                          fasterrcnn_mobilenet_v3_large_320_fpn, fasterrcnn_resnet50_fpn_v2)
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.transforms import v2

from detection_metrics import average_precision

CLASS_NAMES = ["background", "bottle", "cup"]


# ------------------------------------------------------------------------------ data
def load_split(root: Path, mode: str, seed: int = 0) -> dict[str, list[dict]]:
    """Return {'train'|'val'|'test': [coco image dict with its 'boxes' and 'labels']}.

    mode='session': use split.json (whole sessions). mode='frame': keep the same test room, but
    shuffle all other FRAMES into train/val with the same sizes — the classic leakage mistake.
    """
    coco = json.loads((root / "annotations.json").read_text(encoding="utf-8"))
    split = json.loads((root / "split.json").read_text(encoding="utf-8"))
    anns = defaultdict(list)
    for a in coco["annotations"]:
        anns[a["image_id"]].append(a)
    session_to_split = {s: k for k, v in split.items() for s in v}
    out: dict[str, list[dict]] = {"train": [], "val": [], "test": []}
    for im in coco["images"]:
        x = [(a["bbox"][0], a["bbox"][1], a["bbox"][0] + a["bbox"][2], a["bbox"][1] + a["bbox"][3]) for a in anns[im["id"]]]
        item = {**im, "boxes": np.array(x, np.float32).reshape(-1, 4),
                "labels": np.array([a["category_id"] for a in anns[im["id"]]], np.int64)}
        out[session_to_split[im["file_name"].split("/")[1]]].append(item)
    if mode == "frame":
        pool = out["train"] + out["val"]
        np.random.default_rng(seed).shuffle(pool)
        n_train = len(out["train"])
        out["train"], out["val"] = pool[:n_train], pool[n_train:]
    return out


class DetectionDataset(torch.utils.data.Dataset):
    def __init__(self, root: Path, items: list[dict], augment: bool) -> None:
        self.root, self.items = root, items
        # Augmentations a moving robot actually sees: mirror-image scenes, lighting and white balance
        # changes, slight zoom (distance), blur. NOT vertical flips: bottles are never upside down on a floor.
        self.transform = v2.Compose([
            v2.RandomHorizontalFlip(0.5),
            v2.RandomApply([v2.ColorJitter(brightness=0.5, contrast=0.4, saturation=0.4, hue=0.04)], p=0.8),
            v2.RandomApply([v2.RandomAffine(degrees=0, scale=(0.8, 1.2), translate=(0.05, 0.05))], p=0.5),
            v2.RandomApply([v2.GaussianBlur(kernel_size=3, sigma=(0.1, 1.2))], p=0.3),
            v2.SanitizeBoundingBoxes(min_size=4),
        ]) if augment else None

    def __len__(self) -> int:
        return len(self.items)

    def __getitem__(self, i: int):
        it = self.items[i]
        img = decode_image(str(self.root / it["file_name"]), mode=ImageReadMode.RGB)       # uint8 CHW, RGB
        boxes = tv_tensors.BoundingBoxes(torch.from_numpy(it["boxes"]), format="XYXY", canvas_size=img.shape[-2:])
        target = {"boxes": boxes, "labels": torch.from_numpy(it["labels"])}
        img = tv_tensors.Image(img)
        if self.transform is not None:
            img, target = self.transform(img, target)
        return v2.functional.to_dtype(img, torch.float32, scale=True), target


def collate(batch):
    return tuple(zip(*batch))


# ------------------------------------------------------------------------------ model
def build_model(arch: str, weights: str, num_classes: int = len(CLASS_NAMES)) -> torch.nn.Module:
    """COCO-pretrained detector with a NEW 3-class box predictor (background, bottle, cup)."""
    if arch == "mobilenet320":
        w = FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT if weights == "coco" else None
        model = fasterrcnn_mobilenet_v3_large_320_fpn(weights=w, weights_backbone=None)
    elif arch == "resnet50v2":
        w = FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT if weights == "coco" else None
        model = fasterrcnn_resnet50_fpn_v2(weights=w, weights_backbone=None, min_size=240, max_size=320)
    else:
        raise ValueError(arch)
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)   # the only fresh layer
    return model


@torch.no_grad()
def evaluate(model: torch.nn.Module, loader) -> dict[str, float]:
    model.eval()
    preds, gts = [], []
    for images, targets in loader:
        for out, t in zip(model(list(images)), targets):
            preds.append((out["boxes"].numpy(), out["scores"].numpy(), out["labels"].numpy()))
            gts.append((t["boxes"].numpy(), t["labels"].numpy()))
    ap = {name: average_precision(preds, gts, c) for c, name in enumerate(CLASS_NAMES) if c > 0}
    ap["mAP50"] = float(np.nanmean(list(ap.values())))
    return ap


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("root", type=Path)
    ap.add_argument("--arch", choices=["mobilenet320", "resnet50v2"], default="mobilenet320")
    ap.add_argument("--weights", choices=["coco", "none"], default="coco")
    ap.add_argument("--iters", type=int, default=600)
    ap.add_argument("--batch", type=int, default=4)
    ap.add_argument("--lr", type=float, default=0.01)
    ap.add_argument("--freeze-backbone", action="store_true", help="train only FPN, RPN and heads")
    ap.add_argument("--no-augment", action="store_true")
    ap.add_argument("--split-mode", choices=["session", "frame"], default="session")
    ap.add_argument("--eval-every", type=int, default=200)
    ap.add_argument("--eval-limit", type=int, default=0, help="evaluate on at most N images (smoke tests)")
    ap.add_argument("--threads", type=int, default=0, help="torch CPU threads (0 = default)")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=Path("data/detector.pt"))
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    if args.threads:
        torch.set_num_threads(args.threads)
    split = load_split(args.root, args.split_mode, args.seed)
    lim = (lambda xs: xs[: args.eval_limit]) if args.eval_limit else (lambda xs: xs)
    train_ds = DetectionDataset(args.root, split["train"], augment=not args.no_augment)
    val_dl = torch.utils.data.DataLoader(DetectionDataset(args.root, lim(split["val"]), False), batch_size=4, collate_fn=collate)
    test_dl = torch.utils.data.DataLoader(DetectionDataset(args.root, lim(split["test"]), False), batch_size=4, collate_fn=collate)
    print(f"split mode {args.split_mode}: train {len(split['train'])} val {len(split['val'])} test {len(split['test'])} images")

    model = build_model(args.arch, args.weights)
    if args.freeze_backbone:
        for p in model.backbone.body.parameters():
            p.requires_grad = False
    params = [p for p in model.parameters() if p.requires_grad]
    print(f"{args.arch} weights={args.weights}: {sum(p.numel() for p in params) / 1e6:.1f} M trainable parameters")
    opt = torch.optim.SGD(params, lr=args.lr, momentum=0.9, weight_decay=1e-4)
    warmup = min(50, args.iters // 5 + 1)
    sched = torch.optim.lr_scheduler.LambdaLR(                     # linear warm-up, then cosine decay
        opt, lambda i: (i + 1) / warmup if i < warmup else 0.5 * (1 + math.cos(math.pi * (i - warmup) / max(1, args.iters - warmup))))

    sampler = torch.utils.data.RandomSampler(train_ds, replacement=True, num_samples=args.iters * args.batch)
    train_dl = torch.utils.data.DataLoader(train_ds, batch_size=args.batch, sampler=sampler, collate_fn=collate)
    best, t0, running = -1.0, time.perf_counter(), []
    for it, (images, targets) in enumerate(train_dl, start=1):
        model.train()
        losses = model(list(images), list(targets))
        loss = sum(losses.values())
        opt.zero_grad()
        loss.backward()
        opt.step()
        sched.step()
        running.append(float(loss))
        if it % 10 == 0 or it == args.iters:
            print(f"iter {it:4d}  loss {np.mean(running[-10:]):.3f}  lr {sched.get_last_lr()[0]:.4f}  "
                  f"{(time.perf_counter() - t0) / it:.2f} s/iter", flush=True)
        if it % args.eval_every == 0 or it == args.iters:
            val = evaluate(model, val_dl)
            print(f"  val  AP50 bottle {val['bottle']:.3f}  cup {val['cup']:.3f}  mAP50 {val['mAP50']:.3f}", flush=True)
            if val["mAP50"] > best:
                best = val["mAP50"]
                args.out.parent.mkdir(parents=True, exist_ok=True)
                torch.save({"arch": args.arch, "state_dict": model.state_dict(), "classes": CLASS_NAMES,
                            "val": val, "iters": it, "split_mode": args.split_mode}, args.out)

    ckpt = torch.load(args.out, weights_only=False)
    model.load_state_dict(ckpt["state_dict"])
    test = evaluate(model, test_dl)                                  # touched exactly once
    print(f"best val mAP50 {best:.3f} (iter {ckpt['iters']})  ->  TEST (unseen room) AP50 bottle {test['bottle']:.3f}  "
          f"cup {test['cup']:.3f}  mAP50 {test['mAP50']:.3f}")
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
