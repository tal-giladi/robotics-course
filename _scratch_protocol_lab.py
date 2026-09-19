"""01.10 — take protocol v1 apart, then watch the watchdog stop the robot.

Run from the repository root. `--fake` starts a software Pico inside this process, so every
command works with no hardware at all:

    python protocol_lab.py checksum "M 7 500 -500"   # the XOR, byte by byte
    python protocol_lab.py roundtrip                 # encode/decode every message type
    python protocol_lab.py corrupt                   # what a damaged line does
    python protocol_lab.py talk     --fake           # hello, drive, telemetry (raw lines)
    python protocol_lab.py watchdog --fake           # stop sending and time the motor stop
    python protocol_lab.py library  --fake           # the same through robotlab.SerialBase

Against a real Pico running the karmel firmware, replace --fake with
    --port /dev/ttyACM0        (Linux)      --port COM5      (Windows)
or against a separately started software Pico
    --port socket://localhost:5760

Needs `robotlab` and pyserial (labs/README.md).
"""
from __future__ import annotations

import argparse
import contextlib
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent
for candidate in (REPO / "labs" / "python", Path.cwd() / "labs" / "python"):
    if candidate.is_dir():
        sys.path.insert(0, str(candidate))
        break

from robotlab import protocol  # noqa: E402


# --- 1. framing -------------------------------------------------------------------------------
def cmd_checksum(payload: str) -> None:
    value = 0
    print(f"payload: {payload!r}")
    print("  char  ascii   running XOR")
    for char in payload:
        value ^= ord(char)
        print(f"    {char!r:>4}   {ord(char):3d}      0x{value:02X}")
    line = protocol.frame(payload)
    print(f"\nchecksum      : 0x{value:02X}")
    print(f"complete line : {line!r}")
    print(f"unframes back : {protocol.unframe(line)!r}")
    print("\nThe checksum covers the payload only - not the '*', not the newline. Two hex digits")
    print("are emitted uppercase; a decoder accepts either case.")


def cmd_roundtrip() -> None:
    messages = [
        protocol.Hello(1),
        protocol.DutyCommand(7, 500, -500),
        protocol.VelocityCommand(8, 4444, 4444),
        protocol.StopCommand(9),
        protocol.ResetEncoders(10),
        protocol.SetParam(11, "kp", 0.05),
        protocol.SetParam(12, "watchdog_ms", 300),
        protocol.HelloReply("0.1.0", protocol.PROTOCOL_VERSION),
        protocol.Ack(7, True),
        protocol.Ack(13, False, "out_of_range"),
        protocol.Telemetry(5340, 1210, 1198, 4431, 4402, 11850, 523, 8),
    ]
    print(f"{'message':<28} {'on the wire':<52} bytes")
    for message in messages:
        line = protocol.encode(message)
        assert protocol.decode(line) == message, message      # encode -> decode -> same object
        print(f"{type(message).__name__:<28} {line.strip()!r:<52} {len(line)}")
    print("\nEvery line decoded back to exactly the object it came from.")
    print(f"Telemetry is {len(protocol.encode(messages[-1]))} bytes; at 50 Hz that is "
          f"{len(protocol.encode(messages[-1])) * 50} bytes/s.")


def cmd_corrupt() -> None:
    good = protocol.encode(protocol.DutyCommand(7, 500, -500))
    print(f"good line            : {good!r} -> {protocol.decode(good)}")
    cases = {
        "one digit changed": good.replace("500", "600", 1),
        "one checksum digit changed": good.strip()[:-1] + "8\n",
        "line cut in half (cable pulled)": good[:8] + "\n",
        "no checksum at all": "M 7 500 -500\n",
        "a tab instead of a space": good.replace(" ", "\t", 1),
        "unknown command": protocol.frame("Z 7 500 -500"),
        "duty out of range": protocol.frame("M 7 5000 0"),
    }
    for name, line in cases.items():
        try:
            message = protocol.decode(line)
            verdict = f"accepted: {message}"
        except protocol.ChecksumError as exc:
            verdict = f"ChecksumError  ({exc})"
        except protocol.FramingError as exc:
            verdict = f"FramingError   ({exc})"
        except protocol.MessageError as exc:
            verdict = f"MessageError   ({exc})"
        print(f"  {name:<32} {line.strip()!r:<26} {verdict}")
    print("\nFraming and checksum errors are DROPPED by the firmware without a reply: the sequence")
    print("number in a corrupted line cannot be trusted, so there is nobody to answer. Message")
    print("errors keep a trustworthy seq, so they get 'A <seq> ERR <reason>'.")


# --- 2. talking to a Pico ---------------------------------------------------------------------
class LineReader:
    """Read complete protocol lines from a pyserial port opened with timeout=0."""

    def __init__(self, port) -> None:
        self.port = port
        self._buffer = b""

    def lines(self, seconds: float):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            chunk = self.port.read(4096)
            if not chunk:
                time.sleep(0.002)
                continue
            self._buffer += chunk
            while b"\n" in self._buffer:
                line, self._buffer = self._buffer.split(b"\n", 1)
                if line.strip():
                    yield line


@contextlib.contextmanager
def open_link(args):
    """Yield (pyserial port, url). With --fake, a software Pico runs in this process."""
    import serial

    server = None
    url = args.port
    if args.fake:
        from robotlab.fake_pico import FakePico, FakePicoServer

        server = FakePicoServer(FakePico(), port=0).start()
        url = server.url
        print(f"software Pico listening on {url}\n")
    port = serial.serial_for_url(url, baudrate=115200, timeout=0)
    try:
        yield port, url
    finally:
        port.close()
        if server is not None:
            server.stop()


def send(port, message) -> None:
    print(f"  Pi   -> Pico : {protocol.encode(message).strip()}")
    port.write(protocol.encode_bytes(message))


def show(line: bytes) -> object | None:
    try:
        message = protocol.decode_pico_message(line)
    except protocol.ProtocolError as exc:
        print(f"  Pico -> Pi   : {line!r}   REJECTED: {exc}")
        return None
    print(f"  Pico -> Pi   : {line.decode().strip():<52} {type(message).__name__}")
    return message


def listen(reader: LineReader, seconds: float, max_telemetry: int = 4) -> None:
    """Print replies for `seconds`, but only the first few telemetry lines (50 Hz is a lot)."""
    shown = hidden = 0
    for line in reader.lines(seconds):
        if line.startswith(b"T "):
            shown += 1
            if shown > max_telemetry:
                hidden += 1
                continue
        show(line)
    if hidden:
        print(f"  ... and {hidden} more telemetry lines")


def cmd_talk(args) -> None:
    with open_link(args) as (port, _url):
        reader = LineReader(port)
        send(port, protocol.Hello(1))
        listen(reader, 0.3, max_telemetry=2)
        print("\n--- drive both wheels at 30 % duty for 0.4 s ---")
        send(port, protocol.DutyCommand(2, 300, 300))
        listen(reader, 0.4)
        print("\n--- stop ---")
        send(port, protocol.StopCommand(3))
        listen(reader, 0.2, max_telemetry=2)
        print("\nThe tick counts rise while the duty command is active, the speed estimate")
        print("follows, and flag 8 (velocity mode) stays clear because M is open loop.")


def cmd_watchdog(args) -> None:
    """Send one drive command, then go quiet, and time how long until flag 1 appears."""
    with open_link(args) as (port, _url):
        reader = LineReader(port)
        send(port, protocol.Hello(1))
        listen(reader, 0.2, max_telemetry=1)

        print("\n--- one M command, then SILENCE (this is what a crashed host looks like) ---")
        sent_at = time.monotonic()
        send(port, protocol.DutyCommand(2, 400, 400))

        tripped_at = None
        last = None
        for line in reader.lines(1.2):
            try:
                message = protocol.decode_pico_message(line)
            except protocol.ProtocolError:
                continue
            if not isinstance(message, protocol.Telemetry):
                continue
            flags = message.flags
            if last is None or (flags & protocol.FLAG_WATCHDOG) != (last & protocol.FLAG_WATCHDOG):
                elapsed = (time.monotonic() - sent_at) * 1000
                state = "TRIPPED - motors braked" if flags & protocol.FLAG_WATCHDOG else "running"
                print(f"  t = {elapsed:6.0f} ms   flags = {flags}   {state}")
                if flags & protocol.FLAG_WATCHDOG and tripped_at is None:
                    tripped_at = elapsed
            last = flags

        if tripped_at is None:
            print("\n  the watchdog did not trip - is watchdog_ms larger than the 1.2 s we waited?")
        else:
            print(f"\n  watchdog tripped {tripped_at:.0f} ms after the last drive command "
                  f"(firmware default: 300 ms).")
            print("  A robot at 0.3 m/s travels about "
                  f"{0.3 * tripped_at / 1000 * 100:.0f} cm in that time, plus braking distance.")


def cmd_library(args) -> None:
    """The same conversation through robotlab.SerialBase, which handles all of it for you."""
    from robotlab.serial_base import SerialBase

    with open_link(args) as (port, url):
        port.close()                      # SerialBase opens the URL itself
        with SerialBase(url) as base:
            print(f"firmware      : {base.firmware}")
            base.reset_encoders()
            base.set_wheel_velocity(5.0, 5.0)     # SAFETY: on a real robot the wheels turn here
            state = base.read()
            for _ in range(6):
                # newer_than uses the ROBOT's clock, so this samples about every 100 ms
                state = base.wait_for_telemetry(timeout_s=1.0, newer_than=state.t + 0.09)
                print(f"  t={state.t:7.3f} s  ticks L/R {state.left_ticks:6d}/{state.right_ticks:6d}  "
                      f"speed {state.left_rad_s:5.2f}/{state.right_rad_s:5.2f} rad/s  "
                      f"batt {state.battery_v}  flags {state.flags}")
            base.stop()
            print(f"stats         : {base.stats}")
        print("\nSerialBase did all of this for you: sequence numbers, acknowledgements with")
        print("retries for stop/reset/param, a keep-alive thread that re-sends the drive command")
        print("every 100 ms so the watchdog never trips while you are still commanding, unit")
        print("conversion to SI, and reconnection if the cable is pulled.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("checksum", help="XOR a payload, byte by byte")
    p.add_argument("payload")
    sub.add_parser("roundtrip", help="encode and decode every message type")
    sub.add_parser("corrupt", help="what happens to damaged lines")
    for name, helptext in (("talk", "hello, drive, telemetry, stop"),
                           ("watchdog", "go silent and time the motor stop"),
                           ("library", "the same through robotlab.SerialBase")):
        p = sub.add_parser(name, help=helptext)
        p.add_argument("--port", default="/dev/ttyACM0", help="serial device or pyserial URL")
        p.add_argument("--fake", action="store_true", help="run a software Pico in this process")

    args = parser.parse_args()
    if args.command == "checksum":
        cmd_checksum(args.payload)
    elif args.command == "roundtrip":
        cmd_roundtrip()
    elif args.command == "corrupt":
        cmd_corrupt()
    elif args.command == "talk":
        cmd_talk(args)
    elif args.command == "watchdog":
        cmd_watchdog(args)
    else:
        cmd_library(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
