"""Drive the robot with the keyboard — on the Pi directly, or from your laptop over SSH.

Lesson 01.15 (keyboard teleop over the network, dead-man switch).

    python labs/robot/teleop_keyboard.py --fake
    ssh pi@karmel.local  then  python labs/robot/teleop_keyboard.py

Keys:   w / s  forward / backward        a / d  turn left / right
        q / e  forward + turn            space  stop
        + / -  faster / slower           x or Ctrl-C  quit

Hold a key to keep moving. There is no "latched" drive command: this is a dead-man switch.
Three layers stop the robot, from the fastest to the last resort:

1. This script: no key for --deadman seconds (0.5 s) -> stop. Terminal key repeat sends a
   held key ~30 times per second, so holding a key keeps the robot moving. (The first repeat
   comes after the OS repeat delay, ~250-500 ms: increase --deadman if the robot stutters.)
2. SerialBase's command_timeout_s: if this script's loop hangs, the keep-alive stops
   refreshing the last command after the same --deadman time.
3. The firmware watchdog: if the Pi program dies or the USB cable falls out, the Pico stops the
   motors after 300 ms.

Over SSH, a dropped Wi-Fi connection ends the SSH session: the kernel sends SIGHUP (terminal
hung up) and reading the keyboard fails. Both are caught below and stop the robot. If the
network just freezes, no keys arrive and layer 1 stops the robot.

No curses: keyboard input uses msvcrt on Windows and termios + select on Linux/macOS, so the
same script runs everywhere.

SAFETY: start with the default slow speed in an open space.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time

from robot_common import add_connection_args, body_to_wheels, connect

# key -> (forward fraction, turn fraction) of the current speed limits
KEY_COMMANDS = {
    "w": (1.0, 0.0),
    "s": (-1.0, 0.0),
    "a": (0.0, 1.0),
    "d": (0.0, -1.0),
    "q": (1.0, 0.6),
    "e": (1.0, -0.6),
}


KEY_HELP = """w/s forward/backward   a/d turn   q/e forward+turn   space stop   +/- speed   x quit
Hold a key to keep moving; release it and the robot stops within the dead-man time."""


class KeyReader:
    """Non-blocking single-key reads without curses. Use as a context manager."""

    def __enter__(self) -> KeyReader:
        if os.name == "nt":
            import msvcrt

            self._msvcrt = msvcrt
        else:
            import termios
            import tty

            self._fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)  # keys arrive immediately, without Enter; Ctrl-C still works
        return self

    def __exit__(self, *exc: object) -> None:
        if os.name != "nt":
            import termios

            try:
                termios.tcsetattr(self._fd, termios.TCSADRAIN, self._saved)
            except termios.error:
                pass  # the terminal is already gone (SSH dropped): nothing to restore

    def read_keys(self) -> list[str]:
        """All keys pressed since the last call (possibly none). Raises EOFError if the terminal
        is gone (e.g. the SSH connection dropped)."""
        keys = []
        if os.name == "nt":
            while self._msvcrt.kbhit():
                keys.append(self._msvcrt.getwch())
        else:
            import select

            while select.select([sys.stdin], [], [], 0)[0]:
                data = os.read(self._fd, 32)
                if not data:
                    raise EOFError("terminal closed")
                keys.extend(data.decode(errors="ignore"))
        return keys


class HangUp(Exception):
    """The controlling terminal hung up (SSH session closed)."""


def _on_hangup(signum: int, frame: object) -> None:
    raise HangUp()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, default=0.15, help="initial forward speed, m/s (default 0.15)")
    parser.add_argument("--turn-rate", type=float, default=1.2, help="turn rate, rad/s (default 1.2)")
    parser.add_argument("--deadman", type=float, default=0.5, help="stop after this long without a key, s")
    parser.add_argument("--rate", type=float, default=20.0, help="command rate, Hz (default 20)")
    add_connection_args(parser)
    args = parser.parse_args(argv)
    if not sys.stdin.isatty():
        parser.error("teleop needs an interactive terminal")

    if hasattr(signal, "SIGHUP"):  # not on Windows
        signal.signal(signal.SIGHUP, _on_hangup)

    with connect(args, command_timeout_s=args.deadman) as conn, KeyReader() as keyboard:
        base, params = conn.base, conn.params
        speed = min(args.speed, params.max_linear_speed_m_s)
        turn_rate = min(args.turn_rate, params.max_angular_speed_rad_s)
        print(KEY_HELP)
        command = (0.0, 0.0)
        last_key_time = 0.0
        moving = False
        period = 1.0 / args.rate
        try:
            while True:
                loop_start = time.monotonic()
                for key in keyboard.read_keys():
                    key = key.lower()
                    if key in KEY_COMMANDS:
                        command = KEY_COMMANDS[key]
                        last_key_time = loop_start
                    elif key == " ":
                        command, last_key_time = (0.0, 0.0), 0.0
                    elif key in "+=":
                        speed = min(speed + 0.05, params.max_linear_speed_m_s)
                    elif key in "-_":
                        speed = max(speed - 0.05, 0.05)
                    elif key in ("x", "\x03"):
                        return

                deadman_expired = loop_start - last_key_time > args.deadman
                if deadman_expired or command == (0.0, 0.0):
                    if moving:
                        base.stop()
                        moving = False
                else:
                    left, right = body_to_wheels(command[0] * speed, command[1] * turn_rate, params)
                    base.set_wheel_velocity(left, right)  # SAFETY: the wheels turn here
                    moving = True

                state = base.read()
                status = "MOVING " if moving else "stopped"
                battery = "--.-" if state.battery_v is None else f"{state.battery_v:4.1f}"
                rng = " --- " if state.range_m is None else f"{state.range_m:5.2f}"
                print(f"\r{status} speed {speed:.2f} m/s  battery {battery} V  range {rng} m  flags {state.flags}   ",
                      end="", flush=True)
                time.sleep(max(0.0, period - (time.monotonic() - loop_start)))
        except (HangUp, EOFError, OSError) as exc:
            # SAFETY: the terminal/network is gone - stop now (close() below also stops).
            print(f"\nconnection to the terminal lost ({type(exc).__name__}): stopping")
        except KeyboardInterrupt:
            pass
        finally:
            print()


if __name__ == "__main__":
    main()
