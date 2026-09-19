"""inspect_demos.py - audit a demonstration dataset before you spend GPU hours on it (lesson 18.03).

For every episode it computes duration, idle fraction (the arm is not moving), jerkiness, how
far the gripper opens and closes, and flags episodes that look unlike the others. Bad demos
(hesitations, re-grasps, aborted attempts, a forgotten gripper) are the cheapest bug to fix in
imitation learning: delete or re-record them.

Run on synthetic SO-101-like demos (no hardware, numpy only):
    py inspect_demos.py --synthetic
Run on a LeRobot dataset (local or on the Hub; needs `pip install 'lerobot[dataset]'`, Python >= 3.12):
    python inspect_demos.py --repo-id lerobot/svla_so101_pickplace
    python inspect_demos.py --repo-id ${HF_USER}/pick_bottle --root ~/my_datasets/pick_bottle

Version-sensitive: the loader uses the LeRobot v0.6.x dataset API (LeRobotDataset, dataset v3.0).
Only `load_lerobot()` touches LeRobot; everything else works on plain numpy arrays.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Episode:
    index: int
    actions: np.ndarray          # (T, D) commanded joint positions, one row per frame
    names: list[str]             # D joint names, e.g. "shoulder_pan.pos" ... "gripper.pos"
    fps: float


@dataclass(frozen=True)
class EpisodeStats:
    index: int
    duration_s: float
    idle_fraction: float         # share of frames where no joint moves faster than idle_speed
    jerk_rms: float              # RMS of the second difference of the arm joints, units/s^2
    gripper_range: float         # max - min of the gripper command over the episode
    longest_pause_s: float


def episode_stats(ep: Episode, idle_speed: float = 2.0) -> EpisodeStats:
    """idle_speed is in action units per second (degrees/s for SO-101 joints in LeRobot)."""
    a = ep.actions
    gripper_cols = [i for i, n in enumerate(ep.names) if "gripper" in n]
    arm_cols = [i for i in range(a.shape[1]) if i not in gripper_cols]
    vel = np.diff(a[:, arm_cols], axis=0) * ep.fps
    moving = np.abs(vel).max(axis=1) > idle_speed
    acc = np.diff(a[:, arm_cols], n=2, axis=0) * ep.fps**2
    longest, run = 0, 0
    for m in moving:
        run = 0 if m else run + 1
        longest = max(longest, run)
    g = a[:, gripper_cols[0]] if gripper_cols else np.zeros(len(a))
    return EpisodeStats(
        index=ep.index,
        duration_s=len(a) / ep.fps,
        idle_fraction=float(1.0 - moving.mean()) if len(moving) else 1.0,
        jerk_rms=float(np.sqrt(np.mean(acc**2))) if len(acc) else 0.0,
        gripper_range=float(g.max() - g.min()),
        longest_pause_s=longest / ep.fps,
    )


def flag(stats: list[EpisodeStats], min_gripper_range: float = 10.0) -> dict[int, list[str]]:
    """Robust outlier rules: compare every episode with the median episode of the dataset."""
    dur = np.median([s.duration_s for s in stats])
    jerk = np.median([s.jerk_rms for s in stats])
    flags: dict[int, list[str]] = {}
    for s in stats:
        f = []
        if s.duration_s > 1.8 * dur:
            f.append(f"long ({s.duration_s:.1f} s vs median {dur:.1f} s): hesitation or re-grasp?")
        if s.duration_s < 0.5 * dur:
            f.append(f"short ({s.duration_s:.1f} s): aborted attempt?")
        if s.longest_pause_s > 2.0:
            f.append(f"pause of {s.longest_pause_s:.1f} s: policies learn to freeze here")
        if s.jerk_rms > 3.0 * jerk:
            f.append(f"jerky (RMS jerk {s.jerk_rms:.0f} vs median {jerk:.0f})")
        if s.gripper_range < min_gripper_range:
            f.append(f"gripper barely moved (range {s.gripper_range:.1f}): no grasp?")
        if f:
            flags[s.index] = f
    return flags


def synthetic_episodes(n: int = 20, fps: float = 30.0, seed: int = 0) -> list[Episode]:
    """SO-101-like pick-and-place demos in degrees; episodes 3, 7 and 12 are deliberately bad."""
    rng = np.random.default_rng(seed)
    names = ["shoulder_pan.pos", "shoulder_lift.pos", "elbow_flex.pos",
             "wrist_flex.pos", "wrist_roll.pos", "gripper.pos"]
    episodes = []
    for i in range(n):
        dur = rng.uniform(9.0, 12.0)
        t = np.linspace(0.0, 1.0, int(dur * fps))
        reach = 0.5 - 0.5 * np.cos(np.pi * np.clip(t / 0.4, 0, 1))    # 0-40 %: reach the object
        carry = 0.5 - 0.5 * np.cos(np.pi * np.clip((t - 0.5) / 0.4, 0, 1))  # 50-90 %: carry it
        target = rng.uniform(-30, 30)
        arm = np.stack([target * reach - 2 * target * carry, -40 * reach + 10 * carry,
                        60 * reach - 15 * carry, 20 * reach, 5 * reach], axis=1)
        grip = np.where((t > 0.42) & (t < 0.92), 5.0, 40.0)            # close to grasp, open to place
        a = np.column_stack([arm, grip]) + rng.normal(0, 0.02, (len(t), 6))
        if i == 3:                                                    # operator hesitated for 3 s
            a = np.concatenate([a[: len(a) // 3], np.repeat(a[len(a) // 3: len(a) // 3 + 1], int(3 * fps), 0), a[len(a) // 3:]])
        if i == 7:                                                    # forgot to close the gripper
            a[:, 5] = 40.0 + rng.normal(0, 0.2, len(a))
        if i == 12:                                                   # shaky teleoperation
            a[:, :5] += rng.normal(0, 3.0, (len(a), 5))
        episodes.append(Episode(i, a, names, fps))
    return episodes


def load_lerobot(repo_id: str, root: str | None = None) -> list[Episode]:
    """Read only the action column of a LeRobotDataset (no video download or decoding)."""
    from lerobot.datasets import LeRobotDataset   # Version-sensitive: LeRobot v0.6.x

    ds = LeRobotDataset(repo_id, root=root, download_videos=False)
    table = ds.hf_dataset.with_format("numpy")
    actions = np.asarray(table["action"], dtype=np.float64)
    ep_index = np.asarray(table["episode_index"])
    names = list(ds.meta.features["action"]["names"])
    return [Episode(int(e), actions[ep_index == e], names, float(ds.meta.fps)) for e in np.unique(ep_index)]


def report(episodes: list[Episode]) -> dict[int, list[str]]:
    stats = [episode_stats(e) for e in episodes]
    print(f"{len(episodes)} episodes, {sum(len(e.actions) for e in episodes)} frames at {episodes[0].fps:.0f} fps, "
          f"action = {episodes[0].names}")
    print(" ep  duration  idle  longest-pause  jerk-RMS  gripper-range")
    for s in stats:
        print(f"{s.index:3d}  {s.duration_s:6.1f} s  {s.idle_fraction:4.0%}  {s.longest_pause_s:9.1f} s  "
              f"{s.jerk_rms:8.0f}  {s.gripper_range:10.1f}")
    flags = flag(stats)
    print(f"flagged {len(flags)} of {len(stats)} episodes:" if flags else "no episode flagged")
    for idx, reasons in flags.items():
        print(f"  episode {idx}: " + "; ".join(reasons))
    return flags


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Audit demonstration episodes before training")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--synthetic", action="store_true", help="generated demos, no LeRobot needed")
    src.add_argument("--repo-id", help="LeRobot dataset repo id, e.g. lerobot/svla_so101_pickplace")
    ap.add_argument("--root", default=None, help="local dataset folder (optional)")
    args = ap.parse_args(argv)
    episodes = synthetic_episodes() if args.synthetic else load_lerobot(args.repo_id, args.root)
    report(episodes)


if __name__ == "__main__":
    main()
