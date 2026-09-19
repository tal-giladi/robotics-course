"""Watch a Pi <-> Pico serial link: connection state, telemetry age and the SerialBase counters.

Lesson 03.03 (robust serial communication). Use it to break a link on purpose and watch it heal.

    python 03-robot-software/code/l0303_link_monitor.py --fake --chaos 2.0      # in-process fake, cable "pulled" every 2 s
    python -m robotlab.fake_pico --port 5760                                   # terminal 1 (kill + restart it)
    python 03-robot-software/code/l0303_link_monitor.py --port socket://localhost:5760   # terminal 2
    python 03-robot-software/code/l0303_link_monitor.py --port /dev/karmel     # the real robot (udev symlink)

One line per interval:
    t=  3.0s  UP    age=  12ms  telem=150  bad=0  ack_to=0  reconnects=1  fw_restarts=0
The monitor never commands the motors: it only sends the hello at connect time.
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "labs" / "python"))

from robotlab.serial_base import SerialBase, TelemetryTimeoutError  # noqa: E402


def status_line(base: SerialBase, elapsed_s: float) -> str:
    """One human-readable line describing the link right now."""
    age = f"{base_age_ms(base):4.0f}ms"
    if not base.connected:
        up = "DOWN"  # the reader thread saw an I/O error and is retrying every reconnect_interval_s
    else:
        try:
            base.read()  # raises if the newest sample is older than max_telemetry_age_s (0.5 s)
            up = "UP"
        except TelemetryTimeoutError:
            up = "STALE"  # port open, but no fresh telemetry: firmware hung, wrong device, baud…
    s = base.stats
    return (f"t={elapsed_s:5.1f}s  {up:5} age={age}  telem={s.telemetry}  bad={s.bad_lines}  "
            f"ack_to={s.ack_timeouts}  reconnects={s.reconnects}  fw_restarts={s.firmware_restarts}")


def base_age_ms(base: SerialBase) -> float:
    """Age of the newest telemetry sample in ms (inf before the first one)."""
    arrived = base._telemetry_time  # read-only peek for monitoring; SerialBase has no public getter
    return float("inf") if arrived == 0.0 else (time.monotonic() - arrived) * 1000.0


def monitor(base: SerialBase, duration_s: float, interval_s: float, out=print) -> list[str]:
    lines = []
    start = time.monotonic()
    while (elapsed := time.monotonic() - start) < duration_s:
        line = status_line(base, elapsed)
        lines.append(line)
        out(line)
        time.sleep(interval_s)
    return lines


def main(argv: list[str] | None = None) -> dict[str, int]:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="socket://localhost:5760", help="device or pyserial URL")
    parser.add_argument("--fake", action="store_true", help="start a fake Pico in this process")
    parser.add_argument("--chaos", type=float, metavar="SECONDS",
                        help="with --fake: drop the connection every SECONDS (like pulling the cable)")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--interval", type=float, default=0.5)
    args = parser.parse_args(argv)

    server = None
    port = args.port
    if args.fake:
        from robotlab.fake_pico import FakePico, FakePicoServer

        server = FakePicoServer(FakePico(), port=0).start()
        port = server.url
    stop_chaos = threading.Event()
    if server is not None and args.chaos:
        def chaos() -> None:
            while not stop_chaos.wait(args.chaos):
                server.disconnect_client()
        threading.Thread(target=chaos, daemon=True).start()

    try:
        with SerialBase(port, reconnect_interval_s=0.2) as base:
            monitor(base, args.duration, args.interval)
            stats = dict(vars(base.stats))
    finally:
        stop_chaos.set()
        if server is not None:
            server.stop()
    return stats


if __name__ == "__main__":
    main()
