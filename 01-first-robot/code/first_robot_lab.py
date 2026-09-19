"""Shared plumbing for the module-01 measurement scripts (01.11–01.15).

The scripts here *measure* things the lessons quote: the duty→speed curve, how repeatable a
square is, how long a command takes to reach the wheels. They talk to the same
:class:`robotlab.hal.DifferentialBase` as ``labs/robot/*.py``, so every one of them runs

* against the real robot   — ``--port /dev/ttyACM0``
* against a fake Pico      — ``--fake`` (in-process) or ``--port socket://localhost:5760``

``--fake`` here adds two options ``labs/robot`` does not have, because these scripts are about
error and repeatability: ``--realistic`` (mismatched motors and wheels, slip, sensor noise) and
``--seed`` (a different "robot" each run).
"""

from __future__ import annotations

import argparse
import contextlib
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
for path in (ROOT / "labs" / "python", ROOT / "labs" / "robot"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from robot_common import RobotParams, load_params  # noqa: E402
from robotlab.serial_base import SerialBase  # noqa: E402


def add_target_args(parser: argparse.ArgumentParser) -> None:
    """The connection options every script in this folder accepts."""
    group = parser.add_argument_group("robot")
    group.add_argument("--port", help="serial device or pyserial URL (default: karmel.yaml, e.g. /dev/ttyACM0)")
    group.add_argument("--baud", type=int, help="baud rate (default: serial.baud in karmel.yaml)")
    group.add_argument("--fake", action="store_true", help="simulated robot inside this process")
    group.add_argument("--realistic", action="store_true",
                       help="with --fake: mismatched motors and wheels, slip, sensor noise")
    group.add_argument("--seed", type=int, default=0, help="with --fake: simulator random seed")
    group.add_argument("--world", default="empty", choices=("empty", "room", "apartment"),
                       help="with --fake: which world (default: empty, no walls)")
    group.add_argument("--config", help="robot config file (default: labs/config/karmel.yaml)")


@dataclass
class Target:
    """An open robot: the base, its parameters, and (in simulation) ground truth."""

    base: SerialBase
    params: RobotParams
    server: object | None = None

    @property
    def simulated(self) -> bool:
        return self.server is not None

    def true_pose(self) -> tuple[float, float, float] | None:
        """Ground truth ``(x, y, theta)`` — only the simulator knows it."""
        if self.server is None:
            return None
        pose = self.server.fake.sim.pose  # type: ignore[attr-defined]
        return pose.x, pose.y, pose.theta


@contextlib.contextmanager
def open_target(args: argparse.Namespace, **serial_options: object) -> Iterator[Target]:
    """Open the robot the command line describes; always stop the motors on the way out."""
    params = load_params(getattr(args, "config", None))
    server = None
    if args.fake:
        from robotlab.fake_pico import FakePico, FakePicoServer, default_sim

        sim = default_sim(args.world, realistic=args.realistic, seed=args.seed)
        server = FakePicoServer(FakePico(sim), port=0).start()
        port = server.url
    else:
        port = args.port or params.serial_device
    try:
        base = SerialBase(port, args.baud or params.serial_baud,
                          max_wheel_speed_rad_s=params.max_wheel_speed_rad_s, **serial_options)
        try:
            yield Target(base, params, server)
        finally:
            base.close()  # SAFETY: close() brakes the motors before releasing the port
    finally:
        if server is not None:
            server.stop()


def plt_headless():
    """matplotlib with a non-interactive backend (these scripts run over SSH and in CI)."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    return plt
