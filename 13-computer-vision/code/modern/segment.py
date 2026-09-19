"""Lesson 13.11 — semantic, instance and promptable segmentation, and what a robot does with a mask.

  py segment.py --mode semantic            # DeepLabV3-MobileNetV3: a class for every pixel (VOC classes)
  py segment.py --mode instance            # Mask R-CNN: one mask per object
  py segment.py --mode sam                 # detector box -> SAM 2.1 tiny -> precise mask (needs `transformers`)
  py segment.py --mode sam --point 458,120 # click-style prompt instead of a box

Licenses (checked 2026-09): torchvision BSD-3-Clause; SAM 2.1 code + checkpoints Apache-2.0.
SAM 3 (facebook/sam3) is under Meta's custom "SAM License" and gated on Hugging Face — see the lesson.
"""
from __future__ import annotations

import argparse
import time
from dataclasses import dataclass

import numpy as np
import torch
from PIL import Image

from common import ROBOT_TARGETS, load_image, out_path


@dataclass(frozen=True)
class MaskStats:
    """Everything the grasp planner (15.05) wants from a mask, in pixels."""

    area_px: int
    centroid: tuple[float, float]          # (u, v) — better grasp point than the box centre
    major_axis_deg: float                  # orientation of the long axis, 0 = image x axis, CCW positive
    elongation: float                      # sqrt(eigenvalue ratio); ~1 = round, >2 = bottle-like
    box_fill: float                        # mask area / bounding-box area; bottles lying diagonally are low


def mask_stats(mask: np.ndarray) -> MaskStats:
    """Moments of a binary mask: centroid and principal axis (PCA of the pixel coordinates)."""
    vs, us = np.nonzero(mask)
    if len(us) == 0:
        raise ValueError("empty mask")
    cu, cv = us.mean(), vs.mean()
    cov = np.cov(np.stack([us - cu, vs - cv]))
    evals, evecs = np.linalg.eigh(cov)                     # ascending eigenvalues
    major = evecs[:, 1]
    # image v points down, so flip the sign to report a conventional counter-clockwise angle
    angle = float(np.degrees(np.arctan2(-major[1], major[0])))
    angle = (angle + 90.0) % 180.0 - 90.0                  # an axis has no direction: keep (-90, 90]
    box_area = (us.max() - us.min() + 1) * (vs.max() - vs.min() + 1)
    return MaskStats(int(len(us)), (float(cu), float(cv)), angle,
                     float(np.sqrt(evals[1] / max(evals[0], 1e-9))), float(len(us) / box_area))


def overlay(img: Image.Image, mask: np.ndarray, rgb=(255, 0, 180), alpha: float = 0.5) -> Image.Image:
    arr = np.asarray(img).astype(np.float32)
    arr[mask] = (1 - alpha) * arr[mask] + alpha * np.array(rgb, dtype=np.float32)
    return Image.fromarray(arr.astype(np.uint8))


def semantic(img: Image.Image) -> None:
    from torchvision.models.segmentation import (DeepLabV3_MobileNet_V3_Large_Weights,
                                                 deeplabv3_mobilenet_v3_large)
    weights = DeepLabV3_MobileNet_V3_Large_Weights.DEFAULT
    model = deeplabv3_mobilenet_v3_large(weights=weights).eval()
    categories = weights.meta["categories"]                # 21 Pascal VOC classes: no "cup"!
    batch = weights.transforms()(img).unsqueeze(0)          # resizes the short side to 520
    with torch.inference_mode():
        model(batch)                                        # warm-up: the first call is always slow
        t0 = time.perf_counter()
        logits = model(batch)["out"][0]                     # (21, H', W')
    print(f"network {1000 * (time.perf_counter() - t0):.0f} ms, logits {tuple(logits.shape)}")
    classes = logits.argmax(0).numpy()
    total = classes.size
    for idx in np.unique(classes):
        print(f"  {categories[idx]:<14} {100 * np.sum(classes == idx) / total:5.1f} % of pixels")
    small = img.resize((classes.shape[1], classes.shape[0]))
    bottle = classes == categories.index("bottle")
    overlay(small, bottle).save(out_path("13.11-semantic-bottle.jpg"))
    print("saved", out_path("13.11-semantic-bottle.jpg"), "(bottle pixels highlighted)")


def instance(img: Image.Image, threshold: float) -> None:
    from torchvision.models.detection import MaskRCNN_ResNet50_FPN_V2_Weights, maskrcnn_resnet50_fpn_v2
    weights = MaskRCNN_ResNet50_FPN_V2_Weights.DEFAULT
    model = maskrcnn_resnet50_fpn_v2(weights=weights).eval()
    categories = weights.meta["categories"]
    x = weights.transforms()(img)
    with torch.inference_mode():
        model([x])                                          # warm-up
        t0 = time.perf_counter()
        out = model([x])[0]
    print(f"network {1000 * (time.perf_counter() - t0):.0f} ms")
    canvas = img
    for i, (label, score, m) in enumerate(zip(out["labels"], out["scores"], out["masks"])):
        name = categories[int(label)]
        if score < threshold or name not in ROBOT_TARGETS:
            continue
        mask = m[0].numpy() > 0.5                            # soft mask (0..1) at full image size
        s = mask_stats(mask)
        print(f"  {name:<8} {float(score):.2f}  area {s.area_px:6d} px  centroid ({s.centroid[0]:.0f}, "
              f"{s.centroid[1]:.0f})  axis {s.major_axis_deg:+.0f} deg  elongation {s.elongation:.1f}  "
              f"box fill {s.box_fill:.2f}")
        canvas = overlay(canvas, mask, rgb=[(255, 0, 180), (0, 200, 255), (255, 200, 0)][i % 3])
    canvas.save(out_path("13.11-instance.jpg"))
    print("saved", out_path("13.11-instance.jpg"))


def sam(img: Image.Image, box: list[float] | None, point: list[float] | None,
        checkpoint: str = "facebook/sam2.1-hiera-tiny") -> np.ndarray:
    from transformers import Sam2Model, Sam2Processor
    processor = Sam2Processor.from_pretrained(checkpoint)
    model = Sam2Model.from_pretrained(checkpoint).eval()
    if point is not None:
        inputs = processor(images=img, input_points=[[[point]]], input_labels=[[[1]]], return_tensors="pt")
    else:
        inputs = processor(images=img, input_boxes=[[box]], return_tensors="pt")
    with torch.inference_mode():
        model(**inputs)                                     # warm-up
        t0 = time.perf_counter()
        outputs = model(**inputs)
    ms = 1000 * (time.perf_counter() - t0)
    masks = processor.post_process_masks(outputs.pred_masks.cpu(), inputs["original_sizes"])[0]
    scores = outputs.iou_scores[0, 0]                         # the model's own quality estimate per mask
    best = int(scores.argmax())
    mask = masks[0, best].numpy().astype(bool)
    print(f"SAM 2.1 tiny: {ms:.0f} ms (image encoder + decoder), {masks.shape[1]} candidate masks, "
          f"predicted IoU {[round(float(s), 2) for s in scores]}, using #{best}")
    return mask


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["semantic", "instance", "sam"], default="semantic")
    ap.add_argument("--image", default="sample:table")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--box", help="x1,y1,x2,y2 prompt for SAM (default: best robot-target detection)")
    ap.add_argument("--point", help="u,v positive click prompt for SAM")
    ap.add_argument("--threads", type=int, default=0)
    args = ap.parse_args()
    if args.threads:
        torch.set_num_threads(args.threads)
    img = load_image(args.image)

    if args.mode == "semantic":
        semantic(img)
    elif args.mode == "instance":
        instance(img, args.threshold)
    else:
        point = [float(v) for v in args.point.split(",")] if args.point else None
        box = [float(v) for v in args.box.split(",")] if args.box else None
        if box is None and point is None:
            from detect import build_detector, keep
            dets = keep(build_detector("ssdlite")(img), 0.3, ROBOT_TARGETS)
            if not dets:
                raise SystemExit("no robot target detected; pass --box or --point")
            box = list(dets[0].box)
            print(f"prompt box from detector: {dets[0]}")
        mask = sam(img, box, point)
        s = mask_stats(mask)
        prompt = f"prompt box area {int((box[2] - box[0]) * (box[3] - box[1]))} px" if box else "point prompt"
        print(f"mask area {s.area_px} px ({prompt}), centroid "
              f"({s.centroid[0]:.0f}, {s.centroid[1]:.0f}), axis {s.major_axis_deg:+.0f} deg, "
              f"elongation {s.elongation:.1f}, box fill {s.box_fill:.2f}")
        overlay(img, mask).save(out_path("13.11-sam.jpg"))
        print("saved", out_path("13.11-sam.jpg"))


if __name__ == "__main__":
    main()
