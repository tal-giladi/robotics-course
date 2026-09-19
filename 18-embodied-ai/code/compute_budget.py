"""compute_budget.py - back-of-the-envelope GPU memory and latency budgets for robot policies (18.01).

Run:  py compute_budget.py          (standard library only)

Rules of thumb (they give a FLOOR, not the real number - activations, images and framework
overhead come on top; always compare with the official compute guide):
  inference, bf16/fp16 weights ............ 2 bytes per parameter
  full training, mixed precision + AdamW .. 16 bytes per parameter
      (4 fp32 master weights + 2 half weights + 2 half gradients + 8 Adam moments)
  LoRA fine-tuning ........................ 2 bytes per frozen parameter + 16 per trainable one

Parameter counts and "official" figures are dated (verified 2026-09 against the LeRobot compute
guide, the ACT paper and the openpi README) - see references/research/embodied-ai-landscape-2026-09.md.
"""
from __future__ import annotations

from dataclasses import dataclass

GB = 1e9


@dataclass(frozen=True)
class Model:
    name: str
    params: float                 # number of parameters
    official: str                 # what the maintainers say, for comparison


MODELS = [
    Model("ACT (ResNet-18 + transformer)", 80e6, "train 2-6 GB (LeRobot guide)"),
    Model("SmolVLA", 450e6, "train/fine-tune 10-16 GB (LeRobot guide)"),
    Model("pi0 / pi0.5 (openpi), ~3B", 3.0e9, "inference >8 GB, LoRA >22.5 GB, full >70 GB (openpi)"),
    Model("OpenVLA, ~7B", 7.0e9, "inference ~16 GB bf16 (paper/README)"),
]


def inference_gb(params: float, bytes_per_param: float = 2.0) -> float:
    return params * bytes_per_param / GB


def full_finetune_gb(params: float) -> float:
    return params * 16.0 / GB


def lora_gb(params: float, trainable_fraction: float = 0.01) -> float:
    return (params * (1 - trainable_fraction) * 2.0 + params * trainable_fraction * 16.0) / GB


@dataclass(frozen=True)
class ControlBudget:
    control_hz: float             # rate at which the arm receives joint targets
    chunk_size: int               # actions predicted per inference
    execute_steps: int            # actions executed before asking the policy again
    inference_s: float            # time for one policy call

    @property
    def open_loop_s(self) -> float:
        """How long the robot acts without looking at a new observation."""
        return self.execute_steps / self.control_hz

    @property
    def synchronous_duty(self) -> float:
        """Fraction of wall time spent moving if the robot waits (frozen) during every inference.

        One control period (1/control_hz) of compute is free: the loop has to wait for it anyway.
        """
        freeze = max(0.0, self.inference_s - 1.0 / self.control_hz)
        return self.open_loop_s / (self.open_loop_s + freeze)

    @property
    def stale_steps(self) -> float:
        """With asynchronous inference: control steps that pass while the next chunk is computed."""
        return self.inference_s * self.control_hz


def main() -> None:
    print("GPU memory floors from parameter count (GB)")
    print(f"{'model':32s} {'params':>8s} {'infer bf16':>10s} {'LoRA 1%':>8s} {'full FT':>8s}   official")
    for m in MODELS:
        print(f"{m.name:32s} {m.params / 1e6:7.0f}M {inference_gb(m.params):10.1f} {lora_gb(m.params):8.1f} "
              f"{full_finetune_gb(m.params):8.1f}   {m.official}")

    print("\nControl-loop budgets")
    cases = {
        "ACT on a laptop GPU, 30 Hz, chunk 100, run 100": ControlBudget(30, 100, 100, 0.01),
        "ACT with temporal ensembling (query every step)": ControlBudget(30, 100, 1, 0.01),
        "Diffusion, 30 Hz, run 8 of 16, 100 DDPM steps": ControlBudget(30, 16, 8, 1.0),
        "Diffusion, 30 Hz, run 8 of 16, 10 DDIM steps": ControlBudget(30, 16, 8, 0.1),
        "SmolVLA on Jetson Orin-class, chunk 50, run 50": ControlBudget(30, 50, 50, 1.0),
    }
    for name, b in cases.items():
        print(f"{name:50s} open loop {b.open_loop_s:5.2f} s   moving {b.synchronous_duty:5.1%} of the time if "
              f"synchronous   {b.stale_steps:4.1f} steps stale if async")


if __name__ == "__main__":
    main()
