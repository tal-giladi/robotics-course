"""16.01 — should this part of the robot be learned or engineered? A scorecard you can argue with.

    py ml_fit_scorecard.py                 # karmel's stack, layer by layer
    py ml_fit_scorecard.py --explain perception.object_detection

Each component is scored on six questions. Positive points favour a learned component, negative
points favour an engineered (classical) one. The numbers are judgement calls for karmel in a home
in 2026: change them, and the point of the exercise is to defend your changes.

  model    Can you write the physics/geometry down?            yes -> -2 ... no -> +2
  data     Can you get enough labelled or logged examples?     no  -> -2 ... cheaply -> +2
  variety  How varied/unstructured are the inputs?             fixed -> -2 ... open world -> +2
  latency  Does the loop leave room for a model's latency?     <10 ms hard -> -2 ... seconds -> +2
  failure  What does a wrong answer cost?                      injury/damage -> -2 ... a retry -> +2
  verify   Must you prove/inspect why it did what it did?      certify -> -2 ... "works in tests" -> +2
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass

QUESTIONS = ("model", "data", "variety", "latency", "failure", "verify")


@dataclass(frozen=True)
class Component:
    layer: str
    name: str
    scores: tuple[int, int, int, int, int, int]
    loop_hz: float
    note: str

    @property
    def total(self) -> int:
        return sum(self.scores)

    @property
    def verdict(self) -> str:
        if self.total <= -4:
            return "engineer it"
        if self.total >= 4:
            return "learn it"
        return "hybrid: engineered frame, learned part"


STACK = [
    Component("actuation", "wheel_speed_pid", (-2, 2, -2, -2, -1, -1), 100,
              "first-order motor model is known (08.02); PID + feedforward, learning only the feedforward (16.05)"),
    Component("actuation", "motor_feedforward_model", (0, 2, -1, -1, 1, 0), 100,
              "a 3-parameter physics fit usually beats an MLP; learn residuals only if data shows nonlinearity"),
    Component("safety", "obstacle_stop", (-2, -2, -1, -2, -2, -2), 50,
              "ToF/LiDAR threshold + watchdog; must work the first time, every time; never a network"),
    Component("state_estimation", "odometry_and_ekf", (-2, 0, -2, -2, -1, -1), 50,
              "kinematics and Kalman filters are exact enough and give covariances (09, 10)"),
    Component("state_estimation", "slam", (-1, 0, 0, 0, -1, -1), 10,
              "slam_toolbox is classical; learned features/depth help in texture-poor rooms"),
    Component("perception", "object_detection", (2, 1, 2, 1, 1, 1), 5,
              "bottles and cups vary endlessly in look: the textbook case for a fine-tuned detector (16.03)"),
    Component("perception", "fiducial_markers", (-2, 0, -2, -1, 0, -1), 15,
              "AprilTags are solved geometry (13.07); ML adds nothing"),
    Component("navigation", "global_planning", (-2, -1, -1, 0, -1, -1), 1,
              "A*/Smac on a costmap is optimal, inspectable and fast (12)"),
    Component("navigation", "local_control", (-1, -1, 0, -1, -2, -1), 20,
              "MPPI/DWB with hard costmaps; learned policies are research in homes (17.09)"),
    Component("manipulation", "grasp_selection", (1, 0, 2, 0, 0, 0), 1,
              "heuristics for boxes and cylinders first; learned grasps for clutter (15.05)"),
    Component("manipulation", "dexterous_pick_policy", (2, 0, 2, 0, 1, 1), 30,
              "imitation learning (ACT, 18) when the motion is hard to write down; slow, arm-only, supervised"),
    Component("task", "language_to_goals", (2, 2, 2, 2, 0, 0), 0.2,
              "an LLM turns 'bring me a drink' into typed skill calls; never motor commands (19)"),
]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--explain", help="layer.name, e.g. perception.object_detection")
    args = ap.parse_args()
    if args.explain:
        c = next(c for c in STACK if f"{c.layer}.{c.name}" == args.explain)
        print(f"{c.layer}.{c.name}  (loop {c.loop_hz:g} Hz, one cycle = {1000 / c.loop_hz:.0f} ms)")
        for q, s in zip(QUESTIONS, c.scores):
            print(f"  {q:8s} {s:+d}  {'learned' if s > 0 else 'engineered' if s < 0 else 'neutral'}")
        print(f"  total {c.total:+d} -> {c.verdict}\n  {c.note}")
        return
    print(f"{'layer':17s} {'component':24s} " + " ".join(f"{q[:4]:>4s}" for q in QUESTIONS) + f" {'total':>5s}  verdict")
    for c in STACK:
        print(f"{c.layer:17s} {c.name:24s} " + " ".join(f"{s:+4d}" for s in c.scores) + f" {c.total:+5d}  {c.verdict}")
    learned = sum(c.verdict == "learn it" for c in STACK)
    print(f"\n{learned} of {len(STACK)} components are clearly 'learn it'. Everything that keeps karmel safe is engineered.")


if __name__ == "__main__":
    main()
