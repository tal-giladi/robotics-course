"""Shared plumbing for the scripts in ``labs/robot`` — connection options, config, kinematics.

Lessons: 01.11-01.15 use the scripts; 03.05-03.06 look inside this file (dependency injection:
every script gets a ``DifferentialBase`` and doesn't care whether it is the real robot or the fake).

Every script accepts the same connection options:

    --port /dev/ttyACM0            the Pico over USB (default: serial.device from karmel.yaml)
    --port socket://localhost:5760 a fake Pico started with `python -m robotlab.fake_pico`
    --fake                         start a fake Pico + simulator inside this process
"""

from __future__ import annotations

import argparse
import contextlib
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

LABS_DIR = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = LABS_DIR / "config" / "karmel.yaml"

try:  # robotlab installed (`pip install -e labs/python`) or already on the path
    import robotlab  # noqa: F401
except ImportError:  # running straight from a checkout: use labs/python
    sys.path.insert(0, str(LABS_DIR / "python"))

from robotlab.serial_base import SerialBase  # noqa: E402


@dataclass(frozen=True)
class RobotParams:
    """The handful of karmel.yaml values the robot scripts need."""

    wheel_radius_m: float
    wheel_separation_m: float
    ticks_per_wheel_rev: int
    max_wheel_speed_rad_s: float
    max_linear_speed_m_s: float
    max_angular_speed_rad_s: float
    serial_device: str
    serial_baud: int
    battery_cells: int
    battery_full_v: float
    battery_low_warning_v: float
    battery_cutoff_v: float

    @property
    def meters_per_tick(self) -> float:
        return 2.0 * math.pi * self.wheel_radius_m / self.ticks_per_wheel_rev


def load_params(path: str | Path | None = None) -> RobotParams:
    """Read karmel.yaml through robotlab.config; fall back to a plain YAML load."""
    try:
        from robotlab.config import load_config
    except ImportError:
        load_config = None
    if load_config is not None:
        cfg = load_config(path)
        d, b, s = cfg.drive, cfg.battery, cfg.serial
        return RobotParams(
            d.wheel_radius_m, d.wheel_separation_m, d.ticks_per_wheel_rev, d.max_wheel_speed_rad_s,
            d.max_linear_speed_m_s, d.max_angular_speed_rad_s, s.device, s.baud,
            b.cells_series, b.full_v, b.low_warning_v, b.cutoff_v,
        )

    import yaml

    raw = yaml.safe_load(Path(path or DEFAULT_CONFIG).read_text(encoding="utf-8"))
    d, b, s = raw["drive"], raw["battery"], raw["serial"]
    ticks = round(d["encoder_cpr_motor"] * d["gear_ratio"] * d["quadrature_multiplier"])
    return RobotParams(
        d["wheel_radius_m"], d["wheel_separation_m"], ticks, d["max_wheel_speed_rad_s"],
        d["max_linear_speed_m_s"], d["max_angular_speed_rad_s"], s["device"], s["baud"],
        b["cells_series"], b["full_v"], b["low_warning_v"], b["cutoff_v"],
    )


def add_connection_args(parser: argparse.ArgumentParser) -> None:
    group = parser.add_argument_group("connection")
    group.add_argument("--port", help="serial device or pyserial URL (default: serial.device in karmel.yaml)")
    group.add_argument("--baud", type=int, help="baud rate (default: serial.baud in karmel.yaml)")
    group.add_argument("--fake", action="store_true", help="run against a fake Pico + simulator in this process")
    group.add_argument("--world", default="room", choices=("room", "apartment", "empty"),
                       help="simulated world for --fake (default: room, a wall 3 m ahead)")
    group.add_argument("--config", help="robot config file (default: labs/config/karmel.yaml)")


@dataclass
class Connection:
    base: SerialBase
    params: RobotParams
    fake_server: object | None = None  # robotlab.fake_pico.FakePicoServer when --fake

    def true_pose(self) -> tuple[float, float, float] | None:
        """Ground-truth pose from the simulator (only with --fake), for comparing with odometry."""
        if self.fake_server is None:
            return None
        pose = self.fake_server.fake.sim.pose  # type: ignore[attr-defined]
        return pose.x, pose.y, pose.theta


@contextlib.contextmanager
def connect(args: argparse.Namespace, **serial_options: object) -> Iterator[Connection]:
    """Open the robot described by the command-line options; always stops it on exit."""
    params = load_params(args.config)
    server = None
    if args.fake:
        from robotlab.fake_pico import FakePico, FakePicoServer, default_sim

        server = FakePicoServer(FakePico(default_sim(args.world)), port=0).start()
        port = server.url
    else:
        port = args.port or params.serial_device
    try:
        base = SerialBase(port, args.baud or params.serial_baud,
                          max_wheel_speed_rad_s=params.max_wheel_speed_rad_s, **serial_options)
        try:
            yield Connection(base, params, server)
        finally:
            base.close()  # SAFETY: close() brakes the motors before releasing the port
    finally:
        if server is not None:
            server.stop()


def body_to_wheels(v_m_s: float, omega_rad_s: float, params: RobotParams) -> tuple[float, float]:
    """Differential-drive inverse kinematics: body velocity -> wheel angular velocities.

    Each wheel's rim speed is v -/+ omega * (track / 2); divide by the radius for rad/s.
    """
    half_track = params.wheel_separation_m / 2.0
    left = (v_m_s - omega_rad_s * half_track) / params.wheel_radius_m
    right = (v_m_s + omega_rad_s * half_track) / params.wheel_radius_m
    return left, right
