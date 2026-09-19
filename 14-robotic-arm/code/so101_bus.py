"""so101_bus — talk to a real SO-101 follower through LeRobot's Feetech driver (lessons 14.02, 14.03).

Requires the pinned LeRobot with the Feetech extra (it installs ``feetech-servo-sdk``/``scservo_sdk``):

    pip install "lerobot[feetech]==0.6.1"

Commands (all print, none moves the arm unless its name says so):

    py so101_bus.py scan          --port COM5                  # 14.02: which ids answer, model, voltage
    py so101_bus.py read          --port COM5 --robot-id karmel_follower     # torque OFF, stream telemetry
    py so101_bus.py record-limits --port COM5 --robot-id karmel_follower --out limits.json
    py so101_bus.py hold          --port COM5 --robot-id karmel_follower --limits limits.json   # MOVES: holds pose
    py so101_bus.py wiggle        --port COM5 --robot-id karmel_follower --limits limits.json \\
                                  --joint wrist_flex --delta 10                                 # MOVES one joint

Positions are LeRobot units (v0.6.1, use_degrees=True): degrees from the middle of the calibrated
range; the gripper is 0–100 %. The calibration comes from ``lerobot-calibrate`` (14.02).
Safety logic lives in servo_tools.py and is tested there with a fake bus.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from collections.abc import Mapping
from pathlib import Path

from servo_tools import (
    SO101_JOINTS,
    JointLimits,
    SafetyConfig,
    SafetyError,
    first_run_limits,
    health_problems,
    move_smoothly,
    read_telemetry,
    safe_enable_torque,
)

SO101_IDS = {name: i + 1 for i, name in enumerate(SO101_JOINTS)}   # shoulder_pan = 1 ... gripper = 6
STS3215_MODEL_NUMBER = 777


def default_calibration_path(robot_id: str) -> Path:
    """Where lerobot-calibrate (v0.6.1) writes a follower's calibration.

    HF_LEROBOT_CALIBRATION overrides the root; the default root is $HF_HOME/lerobot/calibration,
    and HF_HOME defaults to ~/.cache/huggingface.
    """
    root = os.environ.get("HF_LEROBOT_CALIBRATION")
    if root is None:
        hf_home = os.environ.get("HF_HOME", str(Path.home() / ".cache" / "huggingface"))
        root = str(Path(hf_home) / "lerobot" / "calibration")
    return Path(root).expanduser() / "robots" / "so_follower" / f"{robot_id}.json"


def make_bus(port: str, calibration_file: Path | None):
    """A FeetechMotorsBus with the SO-101 follower's motors, as LeRobot's SOFollower builds it."""
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    motors = {name: Motor(i, "sts3215", MotorNormMode.DEGREES) for name, i in SO101_IDS.items()}
    motors["gripper"] = Motor(SO101_IDS["gripper"], "sts3215", MotorNormMode.RANGE_0_100)
    calibration = None
    if calibration_file is not None:
        raw = json.loads(Path(calibration_file).read_text(encoding="utf-8"))
        calibration = {name: MotorCalibration(**values) for name, values in raw.items()}
    return FeetechMotorsBus(port=port, motors=motors, calibration=calibration)


class LeRobotArmBus:
    """servo_tools.ArmBus on top of a connected FeetechMotorsBus."""

    def __init__(self, bus) -> None:
        self.bus = bus
        self.joint_names = SO101_JOINTS

    def read_positions(self) -> dict[str, float]:
        return {k: float(v) for k, v in self.bus.sync_read("Present_Position", num_retry=2).items()}

    def write_goal_positions(self, goals: Mapping[str, float]) -> None:
        self.bus.sync_write("Goal_Position", dict(goals))

    def enable_torque(self) -> None:
        self.bus.enable_torque()

    def disable_torque(self) -> None:
        self.bus.disable_torque(num_retry=3)

    def read_register(self, register: str, joint: str) -> int:
        return int(self.bus.read(register, joint, normalize=False, num_retry=2))

    def write_register(self, register: str, joint: str, value: int) -> None:
        self.bus.write(register, joint, int(value), normalize=False, num_retry=2)


def load_limits(path: Path | None) -> dict[str, JointLimits]:
    if path is None:
        print("no --limits file: using the narrow first-run limits (±30 deg, gripper 5–60 %)")
        return first_run_limits()
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    return {name: JointLimits(v["lower"], v["upper"]) for name, v in raw.items()}


def print_telemetry(arm: LeRobotArmBus) -> None:
    for name, t in read_telemetry(arm).items():
        print(f"{name:13s} pos {t.position:7.1f}  load {t.load_percent:5.1f} %  "
              f"{t.voltage_v:4.1f} V  {t.temperature_c:3d} °C  {t.current_a:5.2f} A")


# ------------------------------------------------------------------------------------------
# Commands
# ------------------------------------------------------------------------------------------
def cmd_scan(args: argparse.Namespace) -> None:
    bus = make_bus(args.port, None)
    bus.connect(handshake=False)          # don't insist that all six motors exist yet
    try:
        for name, motor_id in SO101_IDS.items():
            model = bus.ping(name, num_retry=2)
            if model is None:
                print(f"id {motor_id} ({name}): no answer")
                continue
            volts = bus.read("Present_Voltage", name, normalize=False) / 10
            ticks = bus.read("Present_Position", name, normalize=False)
            kind = "STS3215" if model == STS3215_MODEL_NUMBER else f"model {model}"
            print(f"id {motor_id} ({name}): {kind}, {volts:.1f} V, raw position {ticks}")
    finally:
        bus.disconnect(disable_torque=False)


def _connect(args: argparse.Namespace) -> LeRobotArmBus:
    cal = Path(args.calibration) if args.calibration else default_calibration_path(args.robot_id)
    if not cal.is_file():
        raise SystemExit(f"calibration file {cal} not found — run lerobot-calibrate first (14.02)")
    bus = make_bus(args.port, cal)
    bus.connect()                          # handshake: all six STS3215 must answer
    return LeRobotArmBus(bus)


def cmd_read(args: argparse.Namespace) -> None:
    arm = _connect(args)
    try:
        arm.disable_torque()
        print("torque OFF — move the joints by hand; Ctrl+C to stop")
        while True:
            print_telemetry(arm)
            print()
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        arm.bus.disconnect(disable_torque=True)


def cmd_record_limits(args: argparse.Namespace) -> None:
    arm = _connect(args)
    lo: dict[str, float] = {}
    hi: dict[str, float] = {}
    try:
        arm.disable_torque()
        print(f"torque OFF — move EVERY joint slowly to both ends of the range you want to allow, "
              f"for {args.seconds:.0f} s")
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            for k, v in arm.read_positions().items():
                lo[k] = min(lo.get(k, v), v)
                hi[k] = max(hi.get(k, v), v)
            time.sleep(0.05)
    finally:
        arm.bus.disconnect(disable_torque=True)
    limits = {}
    for k in SO101_JOINTS:
        lower, upper = lo[k] + args.margin, hi[k] - args.margin
        if lower >= upper:
            raise SystemExit(f"{k}: range {lo[k]:.1f}..{hi[k]:.1f} is too small for a {args.margin} margin")
        limits[k] = {"lower": round(lower, 1), "upper": round(upper, 1)}
        print(f"{k:13s} seen {lo[k]:7.1f} .. {hi[k]:7.1f}  -> limits {lower:7.1f} .. {upper:7.1f}")
    Path(args.out).write_text(json.dumps(limits, indent=2), encoding="utf-8")
    print("wrote", args.out)


def _config(args: argparse.Namespace) -> SafetyConfig:
    window = (4.5, 8.4) if args.servo_voltage == "7.4" else (9.0, 12.8)
    return SafetyConfig(limits=load_limits(Path(args.limits) if args.limits else None),
                        torque_percent=args.torque, max_load_percent=args.max_load,
                        speed_deg_s=args.speed, voltage_window_v=window)


def cmd_hold(args: argparse.Namespace) -> None:
    arm = _connect(args)
    cfg = _config(args)
    try:
        held = safe_enable_torque(arm, cfg)
        print("torque ON, holding", {k: round(v, 1) for k, v in held.items()})
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            problems = health_problems(read_telemetry(arm), cfg)
            if problems:
                raise SafetyError("; ".join(problems))
            time.sleep(0.2)
    except SafetyError as exc:
        print("SAFETY STOP:", exc)
    finally:
        arm.bus.disconnect(disable_torque=True)   # the arm goes limp: support it with a hand
        print("torque OFF")


def cmd_wiggle(args: argparse.Namespace) -> None:
    if args.joint not in SO101_JOINTS:
        raise SystemExit(f"--joint must be one of {SO101_JOINTS}")
    arm = _connect(args)
    cfg = _config(args)
    try:
        start = safe_enable_torque(arm, cfg)
        for goal in (start[args.joint] + args.delta, start[args.joint] - args.delta, start[args.joint]):
            reached = move_smoothly(arm, {args.joint: goal}, cfg)
            print(f"{args.joint}: asked {goal:.1f}, reached {reached[args.joint]:.1f}")
    except SafetyError as exc:
        print("SAFETY STOP:", exc)
    finally:
        arm.bus.disconnect(disable_torque=True)
        print("torque OFF")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)

    def common(sp: argparse.ArgumentParser, calibrated: bool = True) -> None:
        sp.add_argument("--port", required=True, help="COM5, /dev/ttyACM0, ... (lerobot-find-port)")
        if calibrated:
            sp.add_argument("--robot-id", default="karmel_follower", help="the --robot.id used with lerobot-calibrate")
            sp.add_argument("--calibration", help="path to the calibration JSON (default: LeRobot's location)")

    def moving(sp: argparse.ArgumentParser) -> None:
        sp.add_argument("--limits", help="JSON from record-limits (default: narrow first-run limits)")
        sp.add_argument("--torque", type=float, default=30.0, help="Torque_Limit, %% of stall")
        sp.add_argument("--max-load", type=float, default=25.0, help="abort above this |load| %%")
        sp.add_argument("--speed", type=float, default=30.0, help="servo speed limit, deg/s")
        sp.add_argument("--servo-voltage", choices=("7.4", "12"), default="7.4")

    s = sub.add_parser("scan", help="ping ids 1-6")
    common(s, calibrated=False)
    s.set_defaults(func=cmd_scan)
    s = sub.add_parser("read", help="torque off, print telemetry")
    common(s)
    s.set_defaults(func=cmd_read)
    s = sub.add_parser("record-limits", help="torque off, record hand-moved ranges")
    common(s)
    s.add_argument("--seconds", type=float, default=60.0)
    s.add_argument("--margin", type=float, default=5.0, help="degrees kept away from each recorded end")
    s.add_argument("--out", default="limits.json")
    s.set_defaults(func=cmd_record_limits)
    s = sub.add_parser("hold", help="safe torque enable, hold the pose")
    common(s)
    moving(s)
    s.add_argument("--seconds", type=float, default=10.0)
    s.set_defaults(func=cmd_hold)
    s = sub.add_parser("wiggle", help="move one joint +delta, -delta, back")
    common(s)
    moving(s)
    s.add_argument("--joint", default="wrist_flex")
    s.add_argument("--delta", type=float, default=10.0)
    s.set_defaults(func=cmd_wiggle)
    return p


def main(argv: list[str] | None = None) -> None:
    args = build_parser().parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
