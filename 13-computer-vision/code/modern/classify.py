"""Lesson 13.09 — run a pretrained torchvision classifier, see its preprocessing, measure latency.

  py classify.py                                   # MobileNetV3-Small on the sample photo
  py classify.py --model resnet18 --image my.jpg
  py classify.py --crop 413,1,503,190              # classify only the bottle region
  py classify.py --bench --threads 4               # latency with 4 CPU threads (like a Pi 5)
  py classify.py --bgr-bug                         # what happens if you feed OpenCV's BGR order
  py classify.py --compare-preprocessing           # manual normalisation == weights.transforms()

Weights download once to ~/.cache/torch/hub (MobileNetV3-Small ≈ 10 MB, ResNet-18 ≈ 45 MB).
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np
import torch
import torchvision
from PIL import Image
from torchvision import models
from torchvision.transforms import functional as TF

from common import benchmark, load_image

# ImageNet channel statistics every torchvision ImageNet checkpoint was trained with.
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)

MODELS = {
    # name: (builder, weights enum) — DEFAULT picks the best published weights for that builder
    "mobilenet_v3_small": (models.mobilenet_v3_small, models.MobileNet_V3_Small_Weights.DEFAULT),
    "mobilenet_v3_large": (models.mobilenet_v3_large, models.MobileNet_V3_Large_Weights.DEFAULT),
    "resnet18": (models.resnet18, models.ResNet18_Weights.DEFAULT),
    "efficientnet_b0": (models.efficientnet_b0, models.EfficientNet_B0_Weights.DEFAULT),
}


@dataclass(frozen=True)
class Prediction:
    label: str
    probability: float


class Classifier:
    """A pretrained ImageNet classifier with its own preprocessing. Load once, call per frame."""

    def __init__(self, name: str = "mobilenet_v3_small") -> None:
        builder, weights = MODELS[name]
        self.name = name
        self.weights = weights
        self.model = builder(weights=weights).eval()   # eval(): dropout off, batch-norm frozen
        self.transform = weights.transforms()           # resize -> center crop -> tensor -> normalise
        self.categories: list[str] = weights.meta["categories"]

    def preprocess(self, img: Image.Image) -> torch.Tensor:
        return self.transform(img).unsqueeze(0)          # (1, 3, 224, 224) float32

    @torch.inference_mode()
    def logits(self, batch: torch.Tensor) -> torch.Tensor:
        return self.model(batch)

    def top_k(self, img: Image.Image, k: int = 5) -> list[Prediction]:
        probs = torch.softmax(self.logits(self.preprocess(img))[0], dim=0)
        p, idx = probs.topk(k)
        return [Prediction(self.categories[i], float(v)) for v, i in zip(p, idx)]

    def embedding(self, img: Image.Image) -> torch.Tensor:
        """The feature vector before the last layer — the start of transfer learning (13.09 Level 4)."""
        with torch.inference_mode():
            if self.name.startswith("resnet"):
                trunk = torch.nn.Sequential(*list(self.model.children())[:-1])
                return trunk(self.preprocess(img)).flatten(1)[0]
            feats = self.model.avgpool(self.model.features(self.preprocess(img)))
            return feats.flatten(1)[0]


def manual_preprocess(img: Image.Image, resize: int = 256, crop: int = 224) -> torch.Tensor:
    """The same steps as weights.transforms(), written out so nothing is magic."""
    img = TF.resize(img, resize, interpolation=TF.InterpolationMode.BILINEAR)  # short side -> 256
    img = TF.center_crop(img, crop)                                             # 224 x 224 middle
    x = torch.from_numpy(np.asarray(img, dtype=np.float32) / 255.0)             # HWC in [0, 1]
    x = x.permute(2, 0, 1)                                                      # -> CHW
    mean = torch.tensor(IMAGENET_MEAN).view(3, 1, 1)
    std = torch.tensor(IMAGENET_STD).view(3, 1, 1)
    return ((x - mean) / std).unsqueeze(0)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default="mobilenet_v3_small", choices=sorted(MODELS))
    ap.add_argument("--image", default="sample:table")
    ap.add_argument("--crop", help="x1,y1,x2,y2 region to classify")
    ap.add_argument("--threads", type=int, default=0, help="torch CPU threads (0 = PyTorch default)")
    ap.add_argument("--bench", action="store_true")
    ap.add_argument("--bgr-bug", action="store_true")
    ap.add_argument("--compare-preprocessing", action="store_true")
    args = ap.parse_args()

    if args.threads:
        torch.set_num_threads(args.threads)
    print(f"torch {torch.__version__}, torchvision {torchvision.__version__}, threads={torch.get_num_threads()}")

    clf = Classifier(args.model)
    n_params = sum(p.numel() for p in clf.model.parameters())
    print(f"{args.model}: {n_params / 1e6:.1f} M parameters, "
          f"ImageNet top-1 {clf.weights.meta['_metrics']['ImageNet-1K']['acc@1']:.1f} %")
    print("preprocessing:", clf.transform)

    img = load_image(args.image)
    if args.crop:
        x1, y1, x2, y2 = (int(v) for v in args.crop.split(","))
        img = img.crop((x1, y1, x2, y2))
    print(f"input image {img.size[0]}x{img.size[1]}")

    for pred in clf.top_k(img):
        print(f"  {pred.probability:6.3f}  {pred.label}")

    if args.bgr_bug:
        bgr_as_rgb = Image.fromarray(np.asarray(img)[:, :, ::-1].copy())
        print("same image with R and B swapped (OpenCV frame passed straight in):")
        for pred in clf.top_k(bgr_as_rgb):
            print(f"  {pred.probability:6.3f}  {pred.label}")

    if args.compare_preprocessing:
        a = clf.preprocess(img)
        b = manual_preprocess(img)
        print(f"shape {tuple(a.shape)}, max |weights.transforms() - manual| = {float((a - b).abs().max()):.2e}")
        print(f"tensor mean per channel {a.mean(dim=(0, 2, 3)).numpy().round(3)}, "
              f"std {a.std(dim=(0, 2, 3)).numpy().round(3)}")

    if args.bench:
        batch = clf.preprocess(img)
        pre = benchmark(lambda: clf.preprocess(img))
        net = benchmark(lambda: clf.logits(batch))
        print(f"preprocess: {pre}")
        print(f"network:    {net}")
        print(f"=> about {1000.0 / (pre.median_ms + net.median_ms):.0f} frames/s for classification alone")


if __name__ == "__main__":
    main()
