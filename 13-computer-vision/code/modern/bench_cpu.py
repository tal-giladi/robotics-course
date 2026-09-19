"""Measure CPU latency of every model used in 13.09-13.13 on YOUR machine, in one table.

  py bench_cpu.py                         # threads 4 (a Raspberry Pi 5 has 4 cores) and all cores
  py bench_cpu.py --only classify detect  # skip the slow groups
  py bench_cpu.py --threads 1 4

Numbers are end-to-end per frame: preprocessing + network + post-processing, median of N runs after warm-up.
Run it with nothing else busy on the machine, plugged in (laptops throttle on battery).
"""
from __future__ import annotations

import argparse
import os
import platform

import torch

from common import benchmark, load_image


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--threads", type=int, nargs="+", default=[4, os.cpu_count() or 4])
    ap.add_argument("--only", nargs="+", default=["classify", "detect", "segment", "embed"])
    ap.add_argument("--runs", type=int, default=10)
    args = ap.parse_args()

    print(f"{platform.processor() or platform.machine()} | {os.cpu_count()} logical CPUs | "
          f"torch {torch.__version__} | Python {platform.python_version()}")
    img = load_image("sample:table")
    jobs: list[tuple[str, object]] = []

    if "classify" in args.only:
        from classify import Classifier
        for name in ("mobilenet_v3_small", "resnet18", "efficientnet_b0"):
            clf = Classifier(name)
            jobs.append((f"classify {name} 224px", lambda c=clf: c.logits(c.preprocess(img))))
    if "detect" in args.only:
        from detect import build_detector
        for name in ("ssdlite", "frcnn_mobile_320", "frcnn_mobile", "rtdetr_v2_r18"):
            jobs.append((f"detect {name}", build_detector(name)))
    if "segment" in args.only:
        from torchvision.models.segmentation import LRASPP_MobileNet_V3_Large_Weights, lraspp_mobilenet_v3_large
        w = LRASPP_MobileNet_V3_Large_Weights.DEFAULT
        seg = lraspp_mobilenet_v3_large(weights=w).eval()

        def run_seg(m=seg, t=w.transforms()):
            with torch.inference_mode():
                return m(t(img).unsqueeze(0))
        jobs.append(("segment lraspp_mobilenet_v3 520px", run_seg))

        from transformers import Sam2Model, Sam2Processor
        proc = Sam2Processor.from_pretrained("facebook/sam2.1-hiera-tiny")
        sam = Sam2Model.from_pretrained("facebook/sam2.1-hiera-tiny").eval()
        inputs = proc(images=img, input_boxes=[[[412.0, 27.0, 501.0, 198.0]]], return_tensors="pt")

        def run_sam(m=sam, i=inputs):
            with torch.inference_mode():
                return m(**i)
        jobs.append(("segment SAM 2.1 tiny (encode+decode)", run_sam))
    if "embed" in args.only:
        from open_vocab import ImageTextEmbedder
        emb = ImageTextEmbedder("openai/clip-vit-base-patch32")
        jobs.append(("embed CLIP ViT-B/32 image", lambda e=emb: e.image_vectors([img])))

    header = "".join(f"{f'{t} thr median/p90 ms':>26}" for t in args.threads)
    print(f"{'model':<40}{header}")
    for label, fn in jobs:
        cells = []
        for t in args.threads:
            torch.set_num_threads(t)
            lat = benchmark(lambda f=fn: f(img) if label.startswith("detect") else f(), warmup=2, runs=args.runs)
            cells.append(f"{lat.median_ms:>17.0f} / {lat.p90_ms:<6.0f}")
        print(f"{label:<40}" + "".join(cells), flush=True)


if __name__ == "__main__":
    main()
