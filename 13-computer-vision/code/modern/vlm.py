"""Lesson 13.14 — vision-language models on a robot: ask about the scene, get JSON, then VERIFY it.

  py vlm.py tokens --size 640x480                      # how many image tokens / dollars a frame costs
  py vlm.py local --question "What objects are on the table?"
  py vlm.py local --scene-json                         # ask for structured JSON, check it against the detector
  py vlm.py local --mirror-test                        # spatial-reasoning probe: flip the image, does the answer flip?
  py vlm.py claude --scene-json                        # same through the Claude API (needs ANTHROPIC_API_KEY, costs money)

Local model: HuggingFaceTB/SmolVLM2-256M-Video-Instruct (Apache-2.0, ~0.3 B params, runs on a laptop CPU).
Swap in HuggingFaceTB/SmolVLM2-2.2B-Instruct or Qwen/Qwen3-VL-2B-Instruct (both Apache-2.0) with --checkpoint
if you have the RAM/GPU; answers get much better, latency much worse.
API model: Claude via the official `anthropic` SDK (pip install anthropic). Docs:
https://platform.claude.com/docs/en/build-with-claude/vision
"""
from __future__ import annotations

import argparse
import base64
import io
import json
import math
import re
import time
from dataclasses import dataclass

from PIL import Image, ImageOps

from common import ROBOT_TARGETS, Detection, load_image

SCENE_PROMPT = (
    "You are the perception module of a small mobile robot with a gripper. "
    "List the graspable objects you can see. For each give a short name, one of "
    "[bottle, cup, sports ball, other] as category, and its horizontal position in the image as "
    "left, center or right. Reply with JSON only, in this form: "
    '{"objects": [{"name": "...", "category": "...", "position": "left|center|right"}]}'
)

SCENE_SCHEMA = {
    "type": "object",
    "properties": {
        "objects": {
            "type": "array",
            "items": {
                "type": "object",
                "properties": {
                    "name": {"type": "string"},
                    "category": {"type": "string", "enum": ["bottle", "cup", "sports ball", "other"]},
                    "position": {"type": "string", "enum": ["left", "center", "right"]},
                },
                "required": ["name", "category", "position"],
                "additionalProperties": False,
            },
        }
    },
    "required": ["objects"],
    "additionalProperties": False,
}


# ----------------------------------------------------------------------------- cost / tokens
def claude_image_tokens(width: int, height: int, max_long_edge: int = 1568, max_tokens: int = 1568) -> tuple[int, int, int]:
    """Visual tokens for one image: ceil(w/28) * ceil(h/28) after downscaling to the model's limits.

    Standard tier: long edge 1568 px / 1568 tokens. Claude 4.7 and later: 2576 px / 4784 tokens.
    (Claude vision docs, checked 2026-09.) Returns (tokens, sent_width, sent_height).
    """
    long_side = max(width, height)
    for edge in range(min(long_side, max_long_edge), 27, -1):   # largest size that fits both limits
        s = edge / long_side
        w, h = round(width * s), round(height * s)
        if math.ceil(w / 28) * math.ceil(h / 28) <= max_tokens:
            break
    return math.ceil(w / 28) * math.ceil(h / 28), w, h


def downscale(img: Image.Image, long_edge: int) -> Image.Image:
    """Send the size you need, not the size you have: tokens, bytes and latency all scale with pixels."""
    s = long_edge / max(img.size)
    return img if s >= 1 else img.resize((round(img.width * s), round(img.height * s)), Image.Resampling.LANCZOS)


# ----------------------------------------------------------------------------- parsing + verification
def extract_json(text: str, required_key: str | None = None) -> dict:
    """Small models wrap JSON in prose or ``` fences, or break it. Find the first {...} that parses
    (and, if given, contains required_key — otherwise a nested fragment could be mistaken for the answer)."""
    fenced = re.findall(r"```(?:json)?\s*(\{.*?\})\s*```", text, re.S)
    candidates = fenced + [text[i:] for i, ch in enumerate(text) if ch == "{"]
    decoder = json.JSONDecoder()
    for cand in candidates:
        try:
            obj, _ = decoder.raw_decode(cand.strip())
            if isinstance(obj, dict) and (required_key is None or required_key in obj):
                return obj
        except json.JSONDecodeError:
            continue
    raise ValueError(f"no JSON object{f' with key {required_key!r}' if required_key else ''} "
                     f"in model output: {text[:120]!r}")


def position_of(det: Detection, image_width: int) -> str:
    u = det.center[0] / image_width
    return "left" if u < 1 / 3 else "right" if u > 2 / 3 else "center"


@dataclass(frozen=True)
class Verified:
    name: str
    category: str
    position: str
    status: str               # "confirmed", "wrong position", "not found by detector", "not checkable"
    detection: Detection | None


def verify(scene: dict, detections: list[Detection], image_width: int) -> list[Verified]:
    """Cross-check every object the VLM claims against an independent detector.

    Never let a VLM's free-text claim become a grasp target without a box from a geometric
    pipeline behind it (13.16 turns that box into a 3D position).
    """
    results = []
    unused = list(detections)
    for obj in scene.get("objects", []):
        cat, pos = str(obj.get("category", "other")), str(obj.get("position", "?"))
        if cat not in ROBOT_TARGETS:
            results.append(Verified(obj.get("name", "?"), cat, pos, "not checkable", None))
            continue
        same = [d for d in unused if d.label == cat]
        if not same:
            results.append(Verified(obj.get("name", "?"), cat, pos, "not found by detector", None))
            continue
        match = next((d for d in same if position_of(d, image_width) == pos), None)
        det = match or same[0]
        unused.remove(det)
        results.append(Verified(obj.get("name", "?"), cat, pos, "confirmed" if match else "wrong position", det))
    return results


# ----------------------------------------------------------------------------- backends
class LocalVLM:
    def __init__(self, checkpoint: str = "HuggingFaceTB/SmolVLM2-256M-Video-Instruct") -> None:
        import torch
        from transformers import AutoModelForImageTextToText, AutoProcessor
        self.torch = torch
        self.checkpoint = checkpoint
        self.processor = AutoProcessor.from_pretrained(checkpoint)
        self.model = AutoModelForImageTextToText.from_pretrained(checkpoint, dtype=torch.float32).eval()

    def ask(self, img: Image.Image, question: str, max_new_tokens: int = 120) -> tuple[str, float]:
        messages = [{"role": "user", "content": [{"type": "image", "image": img},
                                                 {"type": "text", "text": question}]}]
        inputs = self.processor.apply_chat_template(messages, add_generation_prompt=True, tokenize=True,
                                                    return_dict=True, return_tensors="pt")
        t0 = time.perf_counter()
        with self.torch.inference_mode():
            out = self.model.generate(**inputs, max_new_tokens=max_new_tokens, do_sample=False)
        seconds = time.perf_counter() - t0
        new_tokens = out[0, inputs["input_ids"].shape[1]:]
        return self.processor.decode(new_tokens, skip_special_tokens=True).strip(), seconds


class ClaudeVLM:
    def __init__(self, model: str = "claude-opus-5", long_edge: int = 1024) -> None:
        import anthropic
        self.client = anthropic.Anthropic()        # reads ANTHROPIC_API_KEY (or an `ant auth login` profile)
        self.model = model
        self.long_edge = long_edge

    def _image_block(self, img: Image.Image) -> dict:
        buf = io.BytesIO()
        downscale(img, self.long_edge).save(buf, format="JPEG", quality=90)
        data = base64.standard_b64encode(buf.getvalue()).decode("ascii")
        return {"type": "image", "source": {"type": "base64", "media_type": "image/jpeg", "data": data}}

    def ask(self, img: Image.Image, question: str, schema: dict | None = None) -> tuple[str, float]:
        kwargs = {"output_config": {"format": {"type": "json_schema", "schema": schema}}} if schema else {}
        t0 = time.perf_counter()
        response = self.client.messages.create(
            model=self.model,
            max_tokens=2000,
            messages=[{"role": "user", "content": [self._image_block(img), {"type": "text", "text": question}]}],
            **kwargs,
        )
        seconds = time.perf_counter() - t0
        if response.stop_reason == "refusal":
            raise RuntimeError("model declined the request")
        text = next((b.text for b in response.content if b.type == "text"), "")
        print(f"usage: {response.usage.input_tokens} input tokens, {response.usage.output_tokens} output tokens")
        return text, seconds


def mirror_test(vlm, img: Image.Image, question: str) -> None:
    """If the model really localises, 'left' in the image must become 'right' in the mirrored image."""
    for name, im in (("original", img), ("mirrored", ImageOps.mirror(img))):
        answer, s = vlm.ask(im, question)
        print(f"{name:<9} ({s:5.1f} s): {answer}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("backend", choices=["local", "claude", "tokens"])
    ap.add_argument("--image", default="sample:table")
    ap.add_argument("--question", default="Describe what is on the table in one sentence.")
    ap.add_argument("--scene-json", action="store_true")
    ap.add_argument("--mirror-test", action="store_true")
    ap.add_argument("--checkpoint", default="HuggingFaceTB/SmolVLM2-256M-Video-Instruct")
    ap.add_argument("--model", default="claude-opus-5")
    ap.add_argument("--size", default="640x480")
    ap.add_argument("--price-per-mtok", type=float, default=5.0, help="USD per million input tokens")
    args = ap.parse_args()

    if args.backend == "tokens":
        w, h = (int(v) for v in args.size.split("x"))
        for tier, edge, cap in (("standard", 1568, 1568), ("high-res (4.7+)", 2576, 4784)):
            tokens, sw, sh = claude_image_tokens(w, h, edge, cap)
            usd = tokens * args.price_per_mtok / 1e6
            print(f"{tier:<16} {w}x{h} -> sent {sw}x{sh}: {tokens} image tokens, ${usd:.5f} per frame, "
                  f"${usd * 3600:.2f} per hour at 1 frame/s")
        return

    img = load_image(args.image)
    vlm = LocalVLM(args.checkpoint) if args.backend == "local" else ClaudeVLM(args.model)
    if args.mirror_test:
        mirror_test(vlm, img, "Is the bottle on the left side or the right side of the image? Answer with one word.")
        return
    if not args.scene_json:
        answer, s = vlm.ask(img, args.question)
        print(f"({s:.1f} s) {answer}")
        return

    if args.backend == "claude":
        text, s = vlm.ask(img, SCENE_PROMPT, schema=SCENE_SCHEMA)
    else:
        text, s = vlm.ask(img, SCENE_PROMPT, max_new_tokens=200)
    print(f"raw answer ({s:.1f} s): {text}")
    try:
        scene = extract_json(text, required_key="objects")
    except ValueError as e:
        print("PARSE FAILURE:", e)
        return
    from detect import build_detector, keep
    dets = keep(build_detector("rtdetr_v2_r18")(img), 0.5, ROBOT_TARGETS)
    for v in verify(scene, dets, img.width):
        print(f"  {v.name:<20} {v.category:<12} {v.position:<7} -> {v.status}" + (f"  [{v.detection}]" if v.detection else ""))


if __name__ == "__main__":
    main()
