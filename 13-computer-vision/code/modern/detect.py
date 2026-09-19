"""Lesson 13.10 — pretrained object detectors with permissive licenses, on CPU.

  py detect.py                                         # SSDLite on the sample photo, robot targets only
  py detect.py --model frcnn_mobile --threshold 0.3 --all-classes
  py detect.py --model rtdetr_v2_r18                   # transformer detector, no NMS (needs `transformers`)
  py detect.py --no-nms                                # Faster R-CNN with NMS switched off: see the duplicates
  py detect.py --image my_desk.jpg --bench --threads 4

Every model here is trained on COCO (80 classes, incl. bottle, cup, sports ball).
Licenses (checked 2026-09): torchvision code BSD-3-Clause; RT-DETRv2 checkpoints Apache-2.0.
The COCO-trained weights inherit dataset terms — see the lesson's license section.
"""
from __future__ import annotations

import argparse
from collections.abc import Callable

import numpy as np
import torch
from PIL import Image
from torchvision.models import detection as tvd

from boxes import iou_matrix
from common import ROBOT_TARGETS, Detection, benchmark, draw_detections, load_image, out_path

Detector = Callable[[Image.Image], list[Detection]]

TORCHVISION = {
    # name: (builder, weights)                                    COCO box mAP (torchvision docs)
    "ssdlite": (tvd.ssdlite320_mobilenet_v3_large, tvd.SSDLite320_MobileNet_V3_Large_Weights.DEFAULT),  # 21.3
    "frcnn_mobile_320": (tvd.fasterrcnn_mobilenet_v3_large_320_fpn,
                         tvd.FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT),                      # 22.8
    "frcnn_mobile": (tvd.fasterrcnn_mobilenet_v3_large_fpn,
                     tvd.FasterRCNN_MobileNet_V3_Large_FPN_Weights.DEFAULT),                              # 32.8
    "frcnn_r50_v2": (tvd.fasterrcnn_resnet50_fpn_v2, tvd.FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT),    # 46.7
}
HF = {"rtdetr_v2_r18": "PekingU/rtdetr_v2_r18vd"}


def torchvision_detector(name: str, nms_iou: float | None = None) -> Detector:
    builder, weights = TORCHVISION[name]
    kwargs = {}
    if nms_iou is not None:
        # Faster R-CNN exposes its internal NMS IoU; 1.0 means "never suppress".
        kwargs["box_nms_thresh"] = nms_iou
    # box_score_thresh=0.01 keeps low-confidence boxes so YOU choose the threshold (and can plot PR curves)
    if name.startswith("frcnn"):
        kwargs["box_score_thresh"] = 0.01
    model = builder(weights=weights, **kwargs).eval()
    categories = weights.meta["categories"]
    to_tensor = weights.transforms()   # for detection: just uint8 -> float [0, 1]; the model resizes itself

    @torch.inference_mode()
    def run(img: Image.Image) -> list[Detection]:
        out = model([to_tensor(img)])[0]        # boxes come back in original-image pixels
        return [Detection(categories[int(l)], float(s), tuple(float(v) for v in b))
                for b, s, l in zip(out["boxes"], out["scores"], out["labels"])]

    return run


def rtdetr_detector(name: str = "rtdetr_v2_r18") -> Detector:
    from transformers import AutoImageProcessor, RTDetrV2ForObjectDetection

    checkpoint = HF[name]
    processor = AutoImageProcessor.from_pretrained(checkpoint)
    model = RTDetrV2ForObjectDetection.from_pretrained(checkpoint).eval()

    @torch.inference_mode()
    def run(img: Image.Image) -> list[Detection]:
        inputs = processor(images=img, return_tensors="pt")          # resizes to 640x640, scales to [0, 1]
        outputs = model(**inputs)
        result = processor.post_process_object_detection(
            outputs, target_sizes=torch.tensor([(img.height, img.width)]), threshold=0.01)[0]
        return [Detection(model.config.id2label[int(l)], float(s), tuple(float(v) for v in b))
                for b, s, l in zip(result["boxes"], result["scores"], result["labels"])]

    return run


def build_detector(name: str, nms_iou: float | None = None) -> Detector:
    if name in TORCHVISION:
        return torchvision_detector(name, nms_iou)
    if name in HF:
        return rtdetr_detector(name)
    raise ValueError(f"unknown detector {name}; choose from {sorted(TORCHVISION) + sorted(HF)}")


def keep(dets: list[Detection], threshold: float, classes: tuple[str, ...] | None) -> list[Detection]:
    return sorted((d for d in dets if d.score >= threshold and (classes is None or d.label in classes)),
                  key=lambda d: -d.score)


def count_duplicates(dets: list[Detection], iou: float = 0.5) -> int:
    """How many boxes overlap a higher-scoring box of the same class by more than `iou`."""
    dup = 0
    for i, d in enumerate(dets):
        higher = [e.box for e in dets[:i] if e.label == d.label]
        if higher and iou_matrix(np.array(d.box), np.array(higher)).max() > iou:
            dup += 1
    return dup


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="ssdlite", choices=sorted(TORCHVISION) + sorted(HF))
    ap.add_argument("--image", default="sample:table")
    ap.add_argument("--threshold", type=float, default=0.5)
    ap.add_argument("--all-classes", action="store_true", help="show all 80 COCO classes, not only robot targets")
    ap.add_argument("--no-nms", action="store_true", help="Faster R-CNN only: disable the internal NMS")
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--bench", action="store_true")
    args = ap.parse_args()

    if args.threads:
        torch.set_num_threads(args.threads)
    if args.no_nms and not args.model.startswith("frcnn"):
        ap.error("--no-nms is only wired for the frcnn_* models")

    detector = build_detector(args.model, nms_iou=1.0 if args.no_nms else None)
    img = load_image(args.image)
    classes = None if args.all_classes else ROBOT_TARGETS
    dets = keep(detector(img), args.threshold, classes)
    print(f"{args.model} on {img.width}x{img.height}, threshold {args.threshold}: {len(dets)} detections")
    for d in dets:
        print("  ", d)
    print(f"boxes overlapping a better box of the same class (IoU > 0.5): {count_duplicates(dets)}")
    path = out_path(f"13.10-{args.model}{'-nonms' if args.no_nms else ''}.jpg")
    draw_detections(img, dets).save(path)
    print("saved", path)

    if args.bench:
        print(f"latency ({torch.get_num_threads()} threads, pre+network+post): {benchmark(lambda: detector(img), runs=15)}")


if __name__ == "__main__":
    main()
