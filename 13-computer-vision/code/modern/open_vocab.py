"""Lesson 13.13 — embeddings and open-vocabulary detection.

  py open_vocab.py clip                                  # zero-shot: which text best describes each crop?
  py open_vocab.py clip --model google/siglip2-base-patch16-224
  py open_vocab.py similar                               # image-to-image cosine similarity between crops
  py open_vocab.py owlv2 --queries "a glass bottle" "a drinking glass" "a fork"
  py open_vocab.py gdino --queries "a glass bottle" "a drinking glass" "a fork"

Checkpoints (licenses checked 2026-09 on the Hugging Face model cards):
  openai/clip-vit-base-patch32            MIT; card says deployed use is out of scope (research model)
  google/siglip2-base-patch16-224         Apache-2.0
  google/owlv2-base-patch16-ensemble      Apache-2.0 (card: research output)
  IDEA-Research/grounding-dino-tiny       Apache-2.0
First run downloads 0.6-0.9 GB per detector model to ~/.cache/huggingface.
"""
from __future__ import annotations

import argparse
import time

import torch
from PIL import Image

from common import Detection, SAMPLES, draw_detections, load_image, out_path


def timed(fn, *args, **kwargs):
    fn(*args, **kwargs)                        # warm-up
    t0 = time.perf_counter()
    result = fn(*args, **kwargs)
    return result, 1000 * (time.perf_counter() - t0)


class ImageTextEmbedder:
    """CLIP / SigLIP: one encoder for images, one for text, both into the same vector space."""

    def __init__(self, checkpoint: str = "openai/clip-vit-base-patch32") -> None:
        from transformers import AutoModel, AutoProcessor
        self.checkpoint = checkpoint
        self.processor = AutoProcessor.from_pretrained(checkpoint)
        self.model = AutoModel.from_pretrained(checkpoint).eval()
        self.is_siglip = "siglip" in checkpoint

    @torch.inference_mode()
    def image_vectors(self, images: list[Image.Image]) -> torch.Tensor:
        inputs = self.processor(images=images, return_tensors="pt")
        v = self.model.get_image_features(**inputs)
        v = v if isinstance(v, torch.Tensor) else v.pooler_output
        return torch.nn.functional.normalize(v, dim=-1)       # unit length: dot product = cosine

    @torch.inference_mode()
    def text_vectors(self, texts: list[str]) -> torch.Tensor:
        kwargs = {"padding": "max_length", "max_length": 64} if self.is_siglip else {"padding": True}
        inputs = self.processor(text=texts, return_tensors="pt", **kwargs)
        v = self.model.get_text_features(**inputs)
        v = v if isinstance(v, torch.Tensor) else v.pooler_output
        return torch.nn.functional.normalize(v, dim=-1)

    @torch.inference_mode()
    def zero_shot(self, image: Image.Image, labels: list[str]) -> list[tuple[str, float, float]]:
        """(label, cosine similarity, probability). CLIP: softmax over labels. SigLIP: independent sigmoids."""
        cos = (self.image_vectors([image]) @ self.text_vectors(labels).T)[0]
        scale = self.model.logit_scale.exp()
        if self.is_siglip:
            probs = torch.sigmoid(cos * scale + self.model.logit_bias)
        else:
            probs = torch.softmax(cos * scale, dim=0)
        return sorted(zip(labels, cos.tolist(), probs.tolist()), key=lambda t: -t[1])


def owlv2(img: Image.Image, queries: list[str], threshold: float) -> list[Detection]:
    from transformers import Owlv2ForObjectDetection, Owlv2Processor
    ckpt = "google/owlv2-base-patch16-ensemble"
    processor = Owlv2Processor.from_pretrained(ckpt)
    model = Owlv2ForObjectDetection.from_pretrained(ckpt).eval()
    text_labels = [queries]
    inputs = processor(text=text_labels, images=img, return_tensors="pt")
    with torch.inference_mode():
        outputs, ms = timed(model, **inputs)
    # OWLv2 pads the image to a square before resizing; post-processing maps boxes back to target_sizes
    result = processor.post_process_grounded_object_detection(
        outputs=outputs, target_sizes=torch.tensor([(img.height, img.width)]), threshold=threshold,
        text_labels=text_labels)[0]
    print(f"OWLv2 network {ms:.0f} ms")
    return [Detection(lbl, float(s), tuple(float(v) for v in b))
            for b, s, lbl in zip(result["boxes"], result["scores"], result["text_labels"])]


def grounding_dino(img: Image.Image, queries: list[str], threshold: float, text_threshold: float) -> list[Detection]:
    from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
    ckpt = "IDEA-Research/grounding-dino-tiny"
    processor = AutoProcessor.from_pretrained(ckpt)
    model = AutoModelForZeroShotObjectDetection.from_pretrained(ckpt).eval()
    text_labels = [[q.lower() for q in queries]]              # the model expects lower-case phrases
    inputs = processor(images=img, text=text_labels, return_tensors="pt")
    with torch.inference_mode():
        outputs, ms = timed(model, **inputs)
    result = processor.post_process_grounded_object_detection(
        outputs, inputs.input_ids, threshold=threshold, text_threshold=text_threshold,
        target_sizes=[(img.height, img.width)])[0]
    print(f"Grounding DINO network {ms:.0f} ms")
    return [Detection(lbl or "?", float(s), tuple(float(v) for v in b))
            for b, s, lbl in zip(result["boxes"], result["scores"], result["text_labels"])]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("task", choices=["clip", "similar", "owlv2", "gdino"])
    ap.add_argument("--image", default="sample:table")
    ap.add_argument("--model", default="openai/clip-vit-base-patch32")
    ap.add_argument("--queries", nargs="+", default=["a glass bottle", "a drinking glass", "a fork"])
    ap.add_argument("--threshold", type=float, default=0.3)
    ap.add_argument("--text-threshold", type=float, default=0.25)
    args = ap.parse_args()
    img = load_image(args.image)

    if args.task in ("clip", "similar"):
        emb = ImageTextEmbedder(args.model)
        crops = {f"{lbl}#{i}": img.crop(tuple(int(v) for v in box))
                 for i, (lbl, box) in enumerate(SAMPLES["table"].truth)}
        crops["plate-region"] = img.crop((40, 200, 300, 380))
        if args.task == "clip":
            labels = ["a photo of a bottle of oil", "a photo of a water bottle", "a photo of a coffee mug",
                      "a photo of a drinking glass", "a photo of a tennis ball", "a photo of a pizza"]
            for name, crop in crops.items():
                ranked = emb.zero_shot(crop, labels)
                print(f"{name:<13} " + "  ".join(f"{l.replace('a photo of ', '')}: cos {c:.3f} p {p:.2f}"
                                                  for l, c, p in ranked[:3]))
        else:
            names = list(crops)
            vecs = emb.image_vectors([crops[n] for n in names])
            sim = vecs @ vecs.T
            print(" " * 13 + "".join(f"{n:>13}" for n in names))
            for i, n in enumerate(names):
                print(f"{n:<13}" + "".join(f"{float(sim[i, j]):13.3f}" for j in range(len(names))))
        return

    dets = owlv2(img, args.queries, args.threshold) if args.task == "owlv2" else \
        grounding_dino(img, args.queries, args.threshold, args.text_threshold)
    dets.sort(key=lambda d: -d.score)
    for d in dets:
        print("  ", d)
    path = out_path(f"13.13-{args.task}.jpg")
    draw_detections(img, dets).save(path)
    print("saved", path)


if __name__ == "__main__":
    main()
