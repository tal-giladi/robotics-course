"""A fake Raspberry Pi Pico that speaks protocol v1 — test base_node without hardware.

It opens a pseudo-terminal (pty) that looks exactly like the Pico's USB serial port, or a TCP
port for pyserial's ``socket://`` URLs, and simulates two geared DC motors with encoders, a
battery and a front range sensor facing a wall.

    ros2 run karmel_base fake_pico --link /tmp/ttyKARMEL            # prints the pty path
    ros2 run karmel_base base_node --ros-args -p port:=/tmp/ttyKARMEL

    ros2 run karmel_base fake_pico --tcp 5760
    ros2 run karmel_base base_node --ros-args -p port:=socket://localhost:5760

Fault injection for robustness tests:
    --corrupt-rate 0.05          flip a byte in 5 % of outgoing lines (checksum must catch it)
    --disconnect-after 10        close the port after 10 s (unplugged cable) ...
    --reconnect-delay 3          ... and bring it back 3 s later (same --link path)

The pty transport needs Linux/macOS (it runs in the course Docker image). No ROS required.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import select
import socket
import sys
import time

from karmel_base import protocol as p
from karmel_base import wire

FIRMWARE_VERSION = 'fake-1.0'


class FakePicoModel:
    """The simulated firmware: command handling, motor model, watchdog, telemetry. No I/O."""

    def __init__(self, ticks_per_rev: int = 2464, wheel_radius: float = 0.045,
                 max_wheel_speed: float = 17.0, time_constant: float = 0.08,
                 wall_distance: float = 2.0, battery_v: float = 12.3, low_battery_v: float = 10.5):
        self.ticks_per_rev = ticks_per_rev
        self.wheel_radius = wheel_radius
        self.max_wheel_speed = max_wheel_speed
        self.time_constant = time_constant
        self.wall_distance = wall_distance
        self.battery_v = battery_v
        self.low_battery_v = low_battery_v
        self.params = {'kp': 0.5, 'ki': 5.0, 'kd': 0.0, 'ff': 0.05, 'watchdog_ms': 300, 'telemetry_hz': 50}
        self.reset()

    def reset(self) -> None:
        self.ms = 0.0
        self.left_pos = 0.0          # wheel angle, rad
        self.right_pos = 0.0
        self.left_speed = 0.0        # rad/s
        self.right_speed = 0.0
        self.left_target = 0.0
        self.right_target = 0.0
        self.tick_offset_left = 0.0
        self.tick_offset_right = 0.0
        self.velocity_mode = False
        self.last_command_ms = -1e9
        self.watchdog = True         # motors start stopped until a command arrives
        self.forward_travel = 0.0

    # ---- host -> pico ---------------------------------------------------------------------
    # Parameter ranges, identical to the firmware (see labs/python/robotlab/fake_pico.py).
    PARAM_RANGES = {'kp': (0.0, 10.0), 'ki': (0.0, 10.0), 'kd': (0.0, 10.0), 'ff': (0.0, 2.0),
                    'watchdog_ms': (50, 5000), 'telemetry_hz': (1, 200)}

    def handle_line(self, line: bytes) -> list[bytes]:
        """Process one received line, return the lines to send back (like the firmware)."""
        try:
            payload = p.unframe(line)
        except p.ProtocolError as exc:
            # Bad framing or checksum: the seq cannot be trusted, so the firmware stays silent.
            print(f'[fake_pico] dropped {line!r}: {exc}', file=sys.stderr)
            return []
        try:
            msg = p.parse_payload(payload)
        except p.MessageError:
            return self._error_reply(payload)
        if isinstance(msg, p.Hello):
            return [p.encode_bytes(p.HelloReply(FIRMWARE_VERSION, p.PROTOCOL_VERSION))]
        if isinstance(msg, p.VelocityCommand):
            self.velocity_mode = True
            self.left_target = self._limit(msg.left_mrad_s / 1000.0)
            self.right_target = self._limit(msg.right_mrad_s / 1000.0)
            self._feed_watchdog()
        elif isinstance(msg, p.DutyCommand):
            self.velocity_mode = False
            self.left_target = msg.left / 1000.0 * self.max_wheel_speed
            self.right_target = msg.right / 1000.0 * self.max_wheel_speed
            self._feed_watchdog()
        elif isinstance(msg, p.StopCommand):
            self.left_target = self.right_target = 0.0
        elif isinstance(msg, p.ResetEncoders):
            self.tick_offset_left = self.left_pos
            self.tick_offset_right = self.right_pos
        elif isinstance(msg, p.SetParam):
            low, high = self.PARAM_RANGES[msg.key]
            if not low <= msg.value <= high:
                return [p.encode_bytes(p.Ack(msg.seq, False, 'out_of_range'))]
            self.params[msg.key] = msg.value
        else:  # a Pico -> host message sent to the Pico
            seq = getattr(msg, 'seq', None)
            return [] if seq is None else [p.encode_bytes(p.Ack(seq, False, 'unknown_command'))]
        return [p.encode_bytes(p.Ack(msg.seq, True))]

    @staticmethod
    def _error_reply(payload: str) -> list[bytes]:
        """Firmware error codes for a well-framed but invalid payload."""
        fields = payload.split(' ')
        seq = fields[1] if len(fields) > 1 else ''
        if not seq.isdigit() or int(seq) > p.SEQ_MAX or '' in fields:
            return []
        if fields[0] not in ('H', 'M', 'V', 'S', 'R', 'P'):
            reason = 'unknown_command'
        elif fields[0] == 'P' and len(fields) == 4 and fields[2] not in p.PARAM_KEYS:
            reason = 'unknown_param'
        elif fields[0] == 'M' and len(fields) == 4 and all(f.lstrip('-').isdigit() for f in fields[2:]):
            reason = 'out_of_range'
        else:
            reason = 'bad_args'
        return [p.encode_bytes(p.Ack(int(seq), False, reason))]

    def _limit(self, speed: float) -> float:
        return max(-self.max_wheel_speed, min(self.max_wheel_speed, speed))

    def _feed_watchdog(self) -> None:
        self.last_command_ms = self.ms
        self.watchdog = False

    # ---- physics --------------------------------------------------------------------------
    def step(self, dt: float) -> None:
        self.ms += dt * 1000.0
        if self.ms - self.last_command_ms > self.params['watchdog_ms']:
            self.watchdog = True
        left_target, right_target = (0.0, 0.0) if self.watchdog else (self.left_target, self.right_target)
        alpha = 1.0 - math.exp(-dt / self.time_constant) if self.time_constant > 0 else 1.0
        self.left_speed += alpha * (left_target - self.left_speed)
        self.right_speed += alpha * (right_target - self.right_speed)
        self.left_pos += self.left_speed * dt
        self.right_pos += self.right_speed * dt
        self.forward_travel += self.wheel_radius * (self.left_speed + self.right_speed) / 2.0 * dt
        load = abs(self.left_speed) + abs(self.right_speed)
        self.battery_v -= dt * (2e-5 + 2e-6 * load)   # slow drain, faster when driving

    def telemetry(self) -> p.Telemetry:
        ticks = self.ticks_per_rev / (2.0 * math.pi)
        distance = self.wall_distance - self.forward_travel
        range_mm = int(distance * 1000) if 0.04 <= distance <= 4.0 else -1
        flags = 0
        if self.watchdog:
            flags |= p.FLAG_WATCHDOG
        if self.battery_v < self.low_battery_v:
            flags |= p.FLAG_LOW_BATTERY
        if self.velocity_mode:
            flags |= p.FLAG_VELOCITY_MODE
        noise = random.gauss
        return p.Telemetry(
            ms=int(self.ms),
            left_ticks=int(round((self.left_pos - self.tick_offset_left) * ticks)),
            right_ticks=int(round((self.right_pos - self.tick_offset_right) * ticks)),
            left_mrad_s=int(round(self.left_speed * 1000)),
            right_mrad_s=int(round(self.right_speed * 1000)),
            battery_mv=int(self.battery_v * 1000 + noise(0, 5)),
            range_mm=range_mm,
            flags=flags,
        )


# ------------------------------------------------------------------------------------------------
# Transports
# ------------------------------------------------------------------------------------------------
class PtyTransport:
    def __init__(self, link: str | None):
        import tty
        self.link = link
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)          # no echo, no CR/LF translation: a clean byte pipe like USB CDC
        tty.setraw(self.master)
        self.path = os.ttyname(self.slave)
        if link:
            if os.path.lexists(link):
                os.remove(link)
            os.symlink(self.path, link)
        print(f'[fake_pico] serial port: {link or self.path}' + (f' -> {self.path}' if link else ''), flush=True)

    def fileno(self) -> int:
        return self.master

    def read(self) -> bytes:
        try:
            return os.read(self.master, 1024)
        except OSError:
            return b''

    def write(self, data: bytes) -> None:
        try:
            os.write(self.master, data)
        except OSError:
            pass

    def poll_fds(self):
        return [self.master]

    def close(self) -> None:
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        if self.link and os.path.islink(self.link):
            os.remove(self.link)


class TcpTransport:
    def __init__(self, port: int):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', port))
        self.server.listen(1)
        self.client = None
        print(f'[fake_pico] listening on socket://localhost:{port}', flush=True)

    def poll_fds(self):
        return [self.client if self.client else self.server]

    def read(self) -> bytes:
        if self.client is None:
            self.client, addr = self.server.accept()
            print(f'[fake_pico] client connected from {addr}', flush=True)
            return b''
        try:
            data = self.client.recv(1024)
        except OSError:
            data = b''
        if not data:
            print('[fake_pico] client disconnected', flush=True)
            self.client.close()
            self.client = None
        return data

    def write(self, data: bytes) -> None:
        if self.client is not None:
            try:
                self.client.sendall(data)
            except OSError:
                pass

    def close(self) -> None:
        if self.client:
            self.client.close()
        self.server.close()


def corrupt(line: bytes, rate: float) -> bytes:
    if rate <= 0 or random.random() >= rate or len(line) < 4:
        return line
    i = random.randrange(0, len(line) - 4)
    b = bytearray(line)
    b[i] ^= 0x01                      # a single-bit error in the payload: the XOR checksum catches it
    return bytes(b)


def run(args) -> None:
    model = FakePicoModel(ticks_per_rev=args.ticks_per_rev, wheel_radius=args.wheel_radius,
                          max_wheel_speed=args.max_wheel_speed, wall_distance=args.wall_distance)

    def open_transport():
        return TcpTransport(args.tcp) if args.tcp else PtyTransport(args.link)

    transport = open_transport()
    buffer = wire.LineBuffer()
    start = last = time.monotonic()
    next_telemetry = start
    disconnected_at = None
    try:
        while True:
            now = time.monotonic()
            # --- fault injection: unplug and replug the "USB cable"
            if args.disconnect_after and transport and disconnected_at is None and now - start > args.disconnect_after:
                print('[fake_pico] simulating disconnect', flush=True)
                transport.close()
                transport = None
                disconnected_at = now
            if transport is None:
                model.step(now - last)
                last = now
                if args.reconnect_delay >= 0 and now - disconnected_at > args.reconnect_delay:
                    model.reset()             # a replugged Pico reboots: clock and encoders restart
                    transport = open_transport()
                    buffer = wire.LineBuffer()
                    start = now
                    args.disconnect_after = 0
                time.sleep(0.01)
                continue

            period = 1.0 / max(1.0, float(model.params['telemetry_hz']))
            timeout = max(0.0, next_telemetry - now)
            readable, _, _ = select.select(transport.poll_fds(), [], [], timeout)
            if readable:
                for line in buffer.feed(transport.read()):
                    for reply in model.handle_line(line):
                        transport.write(corrupt(reply, args.corrupt_rate))
            now = time.monotonic()
            model.step(now - last)
            last = now
            if now >= next_telemetry:
                transport.write(corrupt(p.encode_bytes(model.telemetry()), args.corrupt_rate))
                next_telemetry += period
                if next_telemetry < now:
                    next_telemetry = now + period
    except KeyboardInterrupt:
        pass
    finally:
        if transport:
            transport.close()


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--link', default=None, help='create a symlink to the pty at this path, e.g. /tmp/ttyKARMEL')
    parser.add_argument('--tcp', type=int, default=0, help='serve on this TCP port instead of a pty')
    parser.add_argument('--ticks-per-rev', type=int, default=2464)
    parser.add_argument('--wheel-radius', type=float, default=0.045)
    parser.add_argument('--max-wheel-speed', type=float, default=17.0)
    parser.add_argument('--wall-distance', type=float, default=2.0, help='distance to the wall ahead, m')
    parser.add_argument('--corrupt-rate', type=float, default=0.0)
    parser.add_argument('--disconnect-after', type=float, default=0.0)
    parser.add_argument('--reconnect-delay', type=float, default=3.0, help='-1 = never come back')
    # ros2 run appends --ros-args ...; ignore anything we do not know
    args, _unknown = parser.parse_known_args(argv)
    run(args)


if __name__ == '__main__':
    main()
