"""A software Pico: speaks protocol v1 over TCP, driven by the course simulator.

Lessons: 03.05 (fakes behind interfaces), 03.08 (testing without hardware). Every script in
``labs/robot`` can run against it, on Windows, macOS and Linux, with no robot attached::

    python -m robotlab.fake_pico --port 5760                  # terminal 1: the "robot"
    python labs/robot/drive_square.py --port socket://localhost:5760   # terminal 2

or in one process (what the scripts' ``--fake`` flag does)::

    with FakePicoServer(FakePico()) as server:                # port=0: pick a free port
        with SerialBase(server.url) as base:
            base.set_wheel_velocity(5.0, 5.0)

Why TCP? A real USB serial link is just a byte stream. pyserial opens ``socket://host:port``
exactly like ``/dev/ttyACM0`` or ``COM5``, so :class:`robotlab.serial_base.SerialBase` runs
unchanged — the whole stack (protocol encoding, reader thread, acks, keep-alive, reconnect) is
exercised, not bypassed. Virtual serial-port pairs would need different tools per OS.

What is emulated, like ``labs/firmware/pico/main.py``:

* commands H, M, V, S, R, P with the same acknowledgements and error codes;
* the command watchdog (``watchdog_ms``, flag 1), velocity-mode flag 8, low-battery flag 2;
* telemetry at ``telemetry_hz`` with ticks, speed estimates, battery and front range from
  :class:`robotlab.sim.DiffDriveSim` (which also runs the same PID + feedforward controller).

Time runs in real time: the simulator advances ``1 / control_hz`` per step, paced by the wall
clock. One client at a time; a new connection replaces the old one (like re-plugging USB).
"""

from __future__ import annotations

import argparse
import logging
import math
import select
import socket
import threading
import time
from typing import Any

from robotlab import protocol
from robotlab.protocol import (
    FLAG_LOW_BATTERY,
    FLAG_VELOCITY_MODE,
    FLAG_WATCHDOG,
    DutyCommand,
    Hello,
    HelloReply,
    ResetEncoders,
    SetParam,
    StopCommand,
    Telemetry,
    VelocityCommand,
)

log = logging.getLogger(__name__)

FIRMWARE_VERSION = "fake-0.1.0"


class FakePico:
    """The firmware's behaviour, without timing or I/O: feed lines in, step time, read lines out.

    ``sim`` is a :class:`robotlab.sim.DiffDriveSim` (default: an empty 4 m x 3 m room with the
    robot at x=1, y=1.5 facing +x, so a wall is 3 m ahead - within the 4 m ToF range).
    """

    def __init__(
        self,
        sim: Any | None = None,
        *,
        watchdog_ms: int = 300,
        telemetry_hz: int = 50,
        low_battery_v: float | None = None,
    ) -> None:
        if sim is None:
            sim = default_sim()
        self.sim = sim
        self.watchdog_ms = watchdog_ms
        self.telemetry_hz = telemetry_hz
        self.low_battery_v = low_battery_v if low_battery_v is not None else _sim_low_battery_v(sim)
        self.watchdog_tripped = True  # like the firmware: stopped until the first drive command
        self._last_drive_t = sim.t
        self._tick_offset = (0, 0)
        self.bad_lines = 0

    # --- protocol -------------------------------------------------------------------------------
    def handle_line(self, line: str | bytes) -> str | None:
        """One received line -> the reply line, or None (dropped, like the firmware does)."""
        try:
            payload = protocol.unframe(line)
        except protocol.ProtocolError:
            self.bad_lines += 1
            return None
        try:
            message = protocol.parse_payload(payload)
        except protocol.MessageError:
            return self._error_reply(payload)
        return self._execute(message)

    def _error_reply(self, payload: str) -> str | None:
        """Mirror the firmware's error codes for a payload the strict parser rejected."""
        fields = payload.split(" ")
        seq = fields[1] if len(fields) > 1 else ""
        if not seq.isdigit() or int(seq) > protocol.SEQ_MAX or "" in fields:
            self.bad_lines += 1
            return None
        kind = fields[0]
        if kind not in ("H", "M", "V", "S", "R", "P"):
            reason = "unknown_command"
        elif kind == "P" and len(fields) == 4 and fields[2] not in protocol.PARAM_KEYS:
            reason = "unknown_param"
        elif kind == "M" and len(fields) == 4 and all(_is_int(f) for f in fields[2:]):
            reason = "out_of_range"
        else:
            reason = "bad_args"
        return protocol.encode(protocol.Ack(int(seq), False, reason))

    def _execute(self, message: protocol.Message) -> str | None:
        sim = self.sim
        if isinstance(message, Hello):
            return protocol.encode(HelloReply(FIRMWARE_VERSION, protocol.PROTOCOL_VERSION))
        if isinstance(message, DutyCommand):
            self._feed_watchdog()
            sim.set_duty(message.left / protocol.DUTY_SCALE, message.right / protocol.DUTY_SCALE)
        elif isinstance(message, VelocityCommand):
            self._feed_watchdog()
            limit = sim.params.max_wheel_speed_rad_s
            sim.set_velocity(
                max(-limit, min(limit, message.left_mrad_s / 1000.0)),
                max(-limit, min(limit, message.right_mrad_s / 1000.0)),
            )
        elif isinstance(message, StopCommand):
            sim.stop()
        elif isinstance(message, ResetEncoders):
            self._tick_offset = sim.ticks
        elif isinstance(message, SetParam):
            error = self._set_param(message.key, message.value)
            if error:
                return protocol.encode(protocol.Ack(message.seq, False, error))
        else:  # a Pico -> host message sent to us
            seq = getattr(message, "seq", None)
            return None if seq is None else protocol.encode(protocol.Ack(seq, False, "unknown_command"))
        return protocol.encode(protocol.Ack(message.seq, True))

    def _set_param(self, key: str, value: float) -> str | None:
        """Same ranges as the firmware's Robot.set_param. Returns an error code or None."""
        controller = getattr(self.sim, "_controller", None)  # the sim's firmware-PID model
        if key in ("kp", "ki", "kd"):
            if not 0.0 <= value <= 10.0:
                return "out_of_range"
            if controller is not None:
                setattr(controller, key, value)
        elif key == "ff":
            if not 0.0 <= value <= 2.0:
                return "out_of_range"
            if controller is not None:
                controller.ff = value
        elif key == "watchdog_ms":
            if not 50 <= value <= 5000:
                return "out_of_range"
            self.watchdog_ms = int(value)
        elif key == "telemetry_hz":
            if not 1 <= value <= 200:
                return "out_of_range"
            self.telemetry_hz = int(value)
        return None

    def _feed_watchdog(self) -> None:
        self._last_drive_t = self.sim.t
        self.watchdog_tripped = False

    # --- time and telemetry ----------------------------------------------------------------------
    def step(self, dt: float) -> None:
        """Advance the simulated robot by ``dt`` seconds, applying the command watchdog first."""
        if not self.watchdog_tripped and (self.sim.t - self._last_drive_t) * 1000.0 >= self.watchdog_ms - 1e-6:
            self.watchdog_tripped = True
            self.sim.stop()
        self.sim.step(dt)

    def telemetry(self) -> Telemetry:
        sim = self.sim
        left, right = sim.ticks
        left_rad_s, right_rad_s = sim.wheel_velocity_estimate
        battery_v = sim.battery_v
        range_m = sim.front_range()
        flags = 0
        if self.watchdog_tripped:
            flags |= FLAG_WATCHDOG
        if battery_v < self.low_battery_v:
            flags |= FLAG_LOW_BATTERY
        if sim.velocity_mode:
            flags |= FLAG_VELOCITY_MODE
        return Telemetry(
            ms=int(round(sim.t * 1000.0)),
            left_ticks=left - self._tick_offset[0],
            right_ticks=right - self._tick_offset[1],
            left_mrad_s=int(round(left_rad_s * 1000.0)),
            right_mrad_s=int(round(right_rad_s * 1000.0)),
            battery_mv=int(round(battery_v * 1000.0)),
            range_mm=-1 if range_m is None or not math.isfinite(range_m) else int(round(range_m * 1000.0)),
            flags=flags,
        )

    def telemetry_line(self) -> str:
        return protocol.encode(self.telemetry())


class FakePicoServer:
    """Serve a :class:`FakePico` on a TCP port in a background thread.

    ``url`` is what you pass to :class:`robotlab.serial_base.SerialBase`.
    """

    def __init__(self, fake: FakePico | None = None, host: str = "127.0.0.1", port: int = 5760,
                 control_hz: int = 100) -> None:
        self.fake = fake if fake is not None else FakePico()
        self.control_hz = control_hz
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((host, port))
        self._listener.listen(1)
        self._listener.setblocking(False)
        self.host, self.port = self._listener.getsockname()[:2]
        self._client: socket.socket | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="FakePicoServer", daemon=True)

    @property
    def url(self) -> str:
        return f"socket://{self.host}:{self.port}"

    def start(self) -> FakePicoServer:
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.disconnect_client()
        self._listener.close()

    def disconnect_client(self) -> None:
        """Drop the current connection, like pulling the USB cable (tests use it)."""
        with self._lock:
            client, self._client = self._client, None
        if client is not None:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            client.close()

    def __enter__(self) -> FakePicoServer:
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.stop()

    # --- the loop --------------------------------------------------------------------------------
    def _run(self) -> None:
        dt = 1.0 / self.control_hz
        next_step = time.monotonic()
        next_telemetry = next_step
        buffer = b""
        while not self._stop.is_set():
            self._accept()
            with self._lock:
                client = self._client
            timeout = max(0.0, min(next_step, next_telemetry) - time.monotonic())
            if client is None:
                time.sleep(min(timeout, 0.01))
            else:
                buffer = self._receive(client, buffer, timeout)

            now = time.monotonic()
            while now >= next_step:
                self.fake.step(dt)
                next_step += dt
                if now - next_step > 0.5:  # fell far behind (debugger, sleep): don't replay it
                    next_step = now
            if now >= next_telemetry:
                self._send(self.fake.telemetry_line())
                next_telemetry += 1.0 / self.fake.telemetry_hz
                if now - next_telemetry > 0.5:
                    next_telemetry = now

    def _accept(self) -> None:
        try:
            client, address = self._listener.accept()
        except (BlockingIOError, OSError):
            return
        # Accepted sockets may inherit the listener's non-blocking mode (OS-dependent): make the
        # behaviour explicit. A client that stops reading for 1 s is dropped, like a dead USB link.
        client.settimeout(1.0)
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # send each line immediately
        log.info("client connected from %s", address)
        self.disconnect_client()
        with self._lock:
            self._client = client

    def _receive(self, client: socket.socket, buffer: bytes, timeout: float) -> bytes:
        try:
            readable, _, _ = select.select([client], [], [], timeout)
            if not readable:
                return buffer
            data = client.recv(4096)
        except OSError:
            data = b""
        if not data:
            log.info("client disconnected")
            with self._lock:
                if self._client is client:
                    self._client = None
            client.close()
            return b""
        buffer += data
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            reply = self.fake.handle_line(line)
            if reply:
                self._send(reply)
        if len(buffer) > protocol.MAX_LINE_LENGTH:
            buffer = b""
        return buffer

    def _send(self, line: str) -> None:
        with self._lock:
            client = self._client
        if client is None:
            return
        try:
            client.sendall(line.encode("ascii"))
        except OSError:
            self.disconnect_client()


# --- helpers ---------------------------------------------------------------------------------------
def default_sim(world: str = "room", pose: tuple[float, float, float] | None = None,
                realistic: bool = False, seed: int | None = 0) -> Any:
    """Build a :class:`robotlab.sim.DiffDriveSim` for the fake robot."""
    from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, World

    if world == "apartment":
        world_obj, default_pose = World.apartment(), (1.0, 1.3, 0.0)
    elif world == "room":
        world_obj, default_pose = World.rectangle_room(4.0, 3.0), (1.0, 1.5, 0.0)
    elif world == "empty":
        world_obj, default_pose = World(), (0.0, 0.0, 0.0)
    else:
        raise ValueError(f"unknown world {world!r} (apartment, room, empty)")
    params = DiffDriveParams.realistic() if realistic else DiffDriveParams.ideal()
    sensors = SensorParams.realistic() if realistic else SensorParams.ideal()
    return DiffDriveSim(world_obj, params, sensors, pose=pose or default_pose, seed=seed)


def _sim_low_battery_v(sim: Any) -> float:
    params = getattr(sim, "params", None)
    return getattr(params, "battery_low_v", 10.5)


def _is_int(token: str) -> bool:
    digits = token[1:] if token.startswith("-") else token
    return digits.isdigit()


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        prog="python -m robotlab.fake_pico",
        description="Emulate the karmel Pico firmware (protocol v1) on a TCP port, backed by the simulator.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="interface to listen on (default: localhost only)")
    parser.add_argument("--port", type=int, default=5760, help="TCP port (default: 5760)")
    parser.add_argument("--world", choices=("room", "apartment", "empty"), default="room")
    parser.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "THETA"), help="start pose (m, m, rad)")
    parser.add_argument("--realistic", action="store_true", help="motor/wheel mismatch, slip and sensor noise")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING, format="%(message)s")

    sim = default_sim(args.world, tuple(args.pose) if args.pose else None, args.realistic, args.seed)
    with FakePicoServer(FakePico(sim), host=args.host, port=args.port) as server:
        print(f"fake Pico listening on {server.url}  (connect with --port {server.url}; Ctrl-C to quit)")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("bye")


if __name__ == "__main__":
    main()
