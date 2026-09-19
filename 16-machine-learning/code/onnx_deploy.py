"""16.04 — take the 16.03 detector to robot compute: export to ONNX, check it, measure latency,
quantize to INT8, and measure what that cost in accuracy.

    py onnx_deploy.py export   --ckpt data/detector.pt --out data/detector.onnx
    py onnx_deploy.py check    --ckpt data/detector.pt --onnx data/detector.onnx
    py onnx_deploy.py bench    --ckpt data/detector.pt --onnx data/detector.onnx --threads 1 4
    py onnx_deploy.py quantize --onnx data/detector.onnx --data data/karmel-objects
    py onnx_deploy.py accuracy --onnx data/detector.onnx data/detector.int8.onnx --data data/karmel-objects

Without --ckpt, `export` uses an untrained model of --arch: fine for latency (it doesn't depend on
the weights), useless for accuracy.

On the robot you only need `onnxruntime`, `numpy` and `opencv` — no PyTorch:
    py onnx_deploy.py bench --onnx data/detector.onnx --threads 4        (Pi 5: 4 Cortex-A76 cores)
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

from detection_metrics import average_precision

H, W = 240, 320
HERE = Path(__file__).resolve().parent


def load_rgb(path: Path) -> np.ndarray:
    """The robot-side preprocessing, identical to training: RGB, float32 in [0, 1], CHW."""
    bgr = cv2.imread(str(path))
    return np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB).transpose(2, 0, 1), dtype=np.float32) / 255.0


def torch_model(ckpt: Path | None, arch: str):
    import torch
    from finetune_detector import build_model
    if ckpt is not None:
        state = torch.load(ckpt, weights_only=False, map_location="cpu")
        model = build_model(state["arch"], "none")
        model.load_state_dict(state["state_dict"])
    else:
        torch.manual_seed(0)                    # untrained but reproducible: export and check see the same weights
        model = build_model(arch, "none")
    return model.eval()


def cmd_export(ckpt: Path | None, arch: str, out: Path) -> None:
    import torch
    model = torch_model(ckpt, arch)
    dummy = torch.rand(3, H, W)
    out.parent.mkdir(parents=True, exist_ok=True)
    # The TorchScript-based exporter (dynamo=False) is what torchvision's detection models are tested
    # with. Input: ONE image, fixed 3x240x320 (the robot's camera size). Outputs have a variable length N.
    torch.onnx.export(model, ([dummy],), str(out), dynamo=False, opset_version=17,
                      input_names=["image"], output_names=["boxes", "labels", "scores"],
                      dynamic_axes={"boxes": {0: "N"}, "labels": {0: "N"}, "scores": {0: "N"}})
    print(f"exported {out} ({out.stat().st_size / 1e6:.1f} MB)")


def session(path: Path, threads: int) -> ort.InferenceSession:
    opts = ort.SessionOptions()
    opts.intra_op_num_threads = threads
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    return ort.InferenceSession(str(path), opts, providers=["CPUExecutionProvider"])


def cmd_check(ckpt: Path | None, arch: str, onnx_path: Path, data: Path) -> None:
    """Same inputs through PyTorch and ONNX Runtime: outputs must agree to ~1e-3 px."""
    import torch
    model, sess = torch_model(ckpt, arch), session(onnx_path, 4)
    files = sorted((data / "images").rglob("*.jpg"))[:: 25][:8]
    worst = 0.0
    for f in files:
        x = load_rgb(f)
        with torch.no_grad():
            ref = model([torch.from_numpy(x)])[0]
        boxes, labels, scores = sess.run(None, {"image": x})
        n = min(len(boxes), len(ref["boxes"]))
        if len(boxes) != len(ref["boxes"]):
            print(f"  {f.name}: {len(ref['boxes'])} torch vs {len(boxes)} onnx detections")
        if n:
            worst = max(worst, float(np.abs(boxes[:n] - ref["boxes"][:n].numpy()).max()))
    print(f"checked {len(files)} images: max box difference torch vs onnxruntime = {worst:.4f} px")


def latency_ms(fn, runs: int = 30, warmup: int = 5) -> tuple[float, float]:
    for _ in range(warmup):
        fn()
    t = []
    for _ in range(runs):
        t0 = time.perf_counter()
        fn()
        t.append((time.perf_counter() - t0) * 1000)
    return float(np.percentile(t, 50)), float(np.percentile(t, 90))


def cmd_bench(ckpt: Path | None, arch: str, onnx_paths: list[Path], threads: list[int], data: Path, with_torch: bool) -> None:
    img = load_rgb(sorted((data / "images").rglob("*.jpg"))[0])
    print(f"{'runtime':34s} {'threads':>7s} {'p50 ms':>7s} {'p90 ms':>7s} {'fps':>5s}")
    if with_torch:
        import torch
        model = torch_model(ckpt, arch)
        x = torch.from_numpy(img)
        for n in threads:
            torch.set_num_threads(n)
            with torch.no_grad():
                p50, p90 = latency_ms(lambda: model([x]))
            print(f"{'PyTorch eager':34s} {n:7d} {p50:7.1f} {p90:7.1f} {1000 / p50:5.1f}")
    for path in onnx_paths:
        for n in threads:
            sess = session(path, n)
            p50, p90 = latency_ms(lambda: sess.run(None, {"image": img}))
            print(f"{'onnxruntime ' + path.name:34s} {n:7d} {p50:7.1f} {p90:7.1f} {1000 / p50:5.1f}")


class ImageReader:
    """Feeds calibration images to the static quantizer (ONNX Runtime's CalibrationDataReader protocol)."""

    def __init__(self, files: list[Path]) -> None:
        self.it = iter({"image": load_rgb(f)} for f in files)

    def get_next(self):
        return next(self.it, None)

    def rewind(self) -> None:
        pass


def cmd_quantize(onnx_path: Path, data: Path, n_calib: int) -> None:
    from onnxruntime.quantization import CalibrationDataReader, QuantFormat, QuantType, quantize_dynamic, quantize_static
    from onnxruntime.quantization.shape_inference import quant_pre_process
    CalibrationDataReader.register(ImageReader)
    pre = onnx_path.with_suffix(".pre.onnx")
    quant_pre_process(str(onnx_path), str(pre), skip_symbolic_shape=True)
    split = json.loads((data / "split.json").read_text(encoding="utf-8"))
    manifest = [json.loads(l) for l in (data / "manifest.jsonl").read_text(encoding="utf-8").splitlines() if l]
    train = [data / r["file"] for r in manifest if r["session"] in split["train"]]
    calib = train[:: max(1, len(train) // n_calib)][:n_calib]          # spread over all training sessions
    out = onnx_path.with_suffix(".int8.onnx")
    t0 = time.perf_counter()
    # Static INT8: weights AND activations quantized, activation ranges measured on real robot images.
    # QDQ format + per-channel weights is ONNX Runtime's recommendation for CNNs. Only Conv/MatMul
    # are quantized: box decoding, NMS and RoIAlign stay in float, as they must.
    quantize_static(str(pre), str(out), ImageReader(calib), quant_format=QuantFormat.QDQ, per_channel=True,
                    activation_type=QuantType.QUInt8, weight_type=QuantType.QInt8,
                    op_types_to_quantize=["Conv", "MatMul", "Gemm"])
    print(f"static INT8 ({len(calib)} calibration images, {time.perf_counter() - t0:.0f} s): {out} "
          f"({out.stat().st_size / 1e6:.1f} MB, FP32 was {onnx_path.stat().st_size / 1e6:.1f} MB)")
    dyn = onnx_path.with_suffix(".dynint8.onnx")
    quantize_dynamic(str(pre), str(dyn), weight_type=QuantType.QInt8, op_types_to_quantize=["MatMul", "Gemm"])
    print(f"dynamic INT8 (weights only, Gemm/MatMul): {dyn} ({dyn.stat().st_size / 1e6:.1f} MB)")


def cmd_accuracy(onnx_paths: list[Path], data: Path, split_name: str) -> None:
    coco = json.loads((data / "annotations.json").read_text(encoding="utf-8"))
    split = json.loads((data / "split.json").read_text(encoding="utf-8"))
    boxes_of: dict[int, list] = {}
    for a in coco["annotations"]:
        x, y, w, h = a["bbox"]
        boxes_of.setdefault(a["image_id"], []).append((x, y, x + w, y + h, a["category_id"]))
    images = [im for im in coco["images"] if im["file_name"].split("/")[1] in split[split_name]]
    gts = [(np.array([b[:4] for b in boxes_of.get(im["id"], [])], np.float32).reshape(-1, 4),
            np.array([b[4] for b in boxes_of.get(im["id"], [])], np.int64)) for im in images]
    print(f"{split_name} set: {len(images)} images")
    for path in onnx_paths:
        sess = session(path, 4)
        preds = [tuple(sess.run(None, {"image": load_rgb(data / im["file_name"])})[i] for i in (0, 2, 1)) for im in images]
        ap = {c: average_precision(preds, gts, i) for i, c in ((1, "bottle"), (2, "cup"))}
        print(f"  {path.name:28s} AP50 bottle {ap['bottle']:.3f}  cup {ap['cup']:.3f}  mAP50 {np.nanmean(list(ap.values())):.3f}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["export", "check", "bench", "quantize", "accuracy"])
    ap.add_argument("--ckpt", type=Path)
    ap.add_argument("--arch", default="mobilenet320", choices=["mobilenet320", "resnet50v2"])
    ap.add_argument("--onnx", type=Path, nargs="+", default=[HERE / "data" / "detector.onnx"])
    ap.add_argument("--out", type=Path, default=HERE / "data" / "detector.onnx")
    ap.add_argument("--data", type=Path, default=HERE / "data" / "karmel-objects")
    ap.add_argument("--threads", type=int, nargs="+", default=[1, 4])
    ap.add_argument("--no-torch", action="store_true", help="bench ONNX only (robot without PyTorch)")
    ap.add_argument("--calib", type=int, default=32)
    ap.add_argument("--split", default="test")
    a = ap.parse_args()
    if a.cmd == "export":
        cmd_export(a.ckpt, a.arch, a.out)
    elif a.cmd == "check":
        cmd_check(a.ckpt, a.arch, a.onnx[0], a.data)
    elif a.cmd == "bench":
        cmd_bench(a.ckpt, a.arch, a.onnx, a.threads, a.data, with_torch=not a.no_torch)
    elif a.cmd == "quantize":
        cmd_quantize(a.onnx[0], a.data, a.calib)
    else:
        cmd_accuracy(a.onnx, a.data, a.split)


if __name__ == "__main__":
    main()
