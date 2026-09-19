"""``SerialBase`` — the real robot (Pico firmware over USB serial) behind ``DifferentialBase``.

Lessons: 01.10 (protocol and watchdog), 01.11-01.15 (the first robot scripts), 03.03 (robust
serial: timeouts, reconnection), 03.05 (hardware abstraction).

    >>> from robotlab.serial_base import SerialBase
    >>> with SerialBase("/dev/ttyACM0") as base:          # or "COM5", or "socket://localhost:5760"
    ...     base.set_wheel_velocity(5.0, 5.0)             # rad/s; the Pico runs the PID
    ...     state = base.read()                           # BaseState in SI units

What happens behind those three lines:

* **Reader thread.** A background thread reads lines from the port, decodes them with
  :mod:`robotlab.protocol`, keeps the newest telemetry sample and matches acknowledgements to
  the commands that were sent. Your code never blocks on the serial port.
* **Acknowledgements.** Every command carries a sequence number. ``stop()``, ``reset_encoders()``
  and ``set_param()`` wait for their ``A <seq> OK`` and retry on timeout — they must not get
  lost. Drive commands (``set_wheel_*``) don't wait: a control loop sends them 20-50 times per
  second, so a lost one is replaced by the next one anyway. Missing acks are counted in
  :attr:`SerialBase.stats`.
* **Keep-alive.** The firmware stops the motors if no drive command arrives for ``watchdog_ms``
  (300 ms). A script that says ``set_wheel_duty(0.4, 0.4); time.sleep(2)`` would trip it, so a
  keep-alive thread re-sends the last drive command every ``keepalive_s`` (100 ms). If your
  whole program dies, the thread dies with it and the watchdog stops the robot — as intended.
  ``command_timeout_s`` adds a software dead-man: stop refreshing a command that your code
  hasn't renewed for that long (teleop uses it).
* **Reconnection.** If the USB cable is unplugged, the reader thread closes the port and
  retries every ``reconnect_interval_s``. The last drive command is *forgotten*: after a
  reconnect the robot stays stopped until your code commands it again.
* **Units.** The wire uses integers (mrad/s, mV, mm, per-mille duty); this class speaks SI.

``port`` is anything :func:`serial.serial_for_url` understands: a device (``/dev/ttyACM0``,
``COM5``) or a URL such as ``socket://localhost:5760`` for :mod:`robotlab.fake_pico`.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable

from robotlab import protocol
from robotlab.hal import BaseState
from robotlab.protocol import (
    Ack,
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


# --- errors ------------------------------------------------------------------------------------
class SerialBaseError(RuntimeError):
    """Base class for SerialBase failures."""


class NotConnectedError(SerialBaseError):
    """The port is closed or currently disconnected."""


class AckTimeoutError(SerialBaseError):
    """A command was not acknowledged after all retries."""


class CommandRejectedError(SerialBaseError):
    """The firmware answered ``A <seq> ERR <reason>``."""

    def __init__(self, command: protocol.HostMessage, reason: str) -> None:
        super().__init__(f"firmware rejected {command.payload()!r}: {reason}")
        self.command = command
        self.reason = reason


class TelemetryTimeoutError(SerialBaseError):
    """No (fresh) telemetry arrived in time."""


# --- unit conversions ----------------------------------------------------------------------------
def rad_s_to_mrad_s(rad_s: float) -> int:
    """5.0 rad/s -> 5000 mrad/s (rounded to the nearest integer)."""
    return int(round(rad_s * 1000.0))


def mrad_s_to_rad_s(mrad_s: int) -> float:
    return mrad_s / 1000.0


def duty_to_permille(duty: float) -> int:
    """0.4 -> 400. Clamped to -1000..1000 (the protocol rejects anything outside)."""
    if not math.isfinite(duty):
        raise ValueError(f"duty must be finite, got {duty!r}")
    return int(round(max(-1.0, min(1.0, duty)) * protocol.DUTY_SCALE))


def mv_to_v(millivolts: int) -> float | None:
    """-1 (no reading) -> None, 11850 -> 11.85."""
    return None if millivolts < 0 else millivolts / 1000.0


def mm_to_m(millimetres: int) -> float | None:
    """-1 (no valid reading) -> None, 523 -> 0.523."""
    return None if millimetres < 0 else millimetres / 1000.0


def telemetry_to_state(t: Telemetry) -> BaseState:
    """Wire integers -> SI :class:`BaseState`."""
    return BaseState(
        t=t.ms / 1000.0,
        left_ticks=t.left_ticks,
        right_ticks=t.right_ticks,
        left_rad_s=mrad_s_to_rad_s(t.left_mrad_s),
        right_rad_s=mrad_s_to_rad_s(t.right_mrad_s),
        battery_v=mv_to_v(t.battery_mv),
        range_m=mm_to_m(t.range_mm),
        flags=t.flags,
    )


# --- bookkeeping ---------------------------------------------------------------------------------
@dataclass
class SerialStats:
    """Counters for debugging a flaky link (print ``base.stats``)."""

    lines_received: int = 0
    bad_lines: int = 0  # failed checksum/framing or unexpected message type
    telemetry: int = 0
    commands_sent: int = 0
    acks_ok: int = 0
    acks_err: int = 0
    ack_timeouts: int = 0  # waited-for commands that were retried or failed
    unacked_drive_commands: int = 0  # fire-and-forget commands whose ack never came
    reconnects: int = 0
    firmware_restarts: int = 0  # telemetry clock went backwards: the Pico rebooted


@dataclass
class _Pending:
    command: protocol.HostMessage
    sent_at: float
    wait: bool
    event: threading.Event
    ack: Ack | None = None


SerialFactory = Callable[[str, int, float], Any]
"""``(port, baud, timeout_s) -> object`` with pyserial's ``read``, ``write`` and ``close``."""


def _open_serial(port: str, baud: int, timeout: float) -> Any:
    import serial  # pyserial; imported here so robotlab works without it until you need it

    return serial.serial_for_url(port, baudrate=baud, timeout=timeout)


# --- the base ------------------------------------------------------------------------------------
class SerialBase:
    """:class:`robotlab.hal.DifferentialBase` over the Pi <-> Pico serial protocol v1.

    Parameters
    ----------
    port, baud:
        Serial device or pyserial URL, and baud rate (ignored by USB CDC, kept for UARTs).
    ack_timeout_s, retries:
        How long to wait for each acknowledgement, and how many times to re-send.
    keepalive_s:
        Re-send the active drive command this often. Must be well below the firmware
        ``watchdog_ms``.
    command_timeout_s:
        ``None`` (default): keep a drive command alive until ``stop()``. A number: stop
        refreshing it once your code hasn't called ``set_wheel_*`` for that long, so the
        firmware watchdog stops the robot (a software dead-man switch).
    max_wheel_speed_rad_s:
        Velocity setpoints are clamped to this (default: ``drive.max_wheel_speed_rad_s`` from
        karmel.yaml if it can be loaded).
    connect_timeout_s:
        How long the constructor waits for the firmware's hello reply and first telemetry.
    reconnect:
        Re-open the port automatically after a disconnect.
    serial_factory:
        ``(port, baud, timeout) -> serial-like object``; tests inject fakes here.
    """

    def __init__(
        self,
        port: str = "/dev/ttyACM0",
        baud: int = 115200,
        *,
        ack_timeout_s: float = 0.25,
        retries: int = 2,
        keepalive_s: float = 0.1,
        command_timeout_s: float | None = None,
        max_wheel_speed_rad_s: float | None = None,
        connect_timeout_s: float = 3.0,
        max_telemetry_age_s: float = 0.5,
        reconnect: bool = True,
        reconnect_interval_s: float = 0.5,
        serial_factory: SerialFactory | None = None,
    ) -> None:
        self.port = port
        self.baud = baud
        self.ack_timeout_s = ack_timeout_s
        self.retries = retries
        self.keepalive_s = keepalive_s
        self.command_timeout_s = command_timeout_s
        self.max_telemetry_age_s = max_telemetry_age_s
        self.reconnect = reconnect
        self.reconnect_interval_s = reconnect_interval_s
        self.max_wheel_speed_rad_s = (
            max_wheel_speed_rad_s if max_wheel_speed_rad_s is not None else _config_max_wheel_speed()
        )
        self.stats = SerialStats()
        self.firmware: HelloReply | None = None

        self._factory: SerialFactory = serial_factory or _open_serial
        self._serial: Any = None
        self._write_lock = threading.Lock()
        # Held while the drive command is changed or re-sent, so the keep-alive thread can never
        # re-send an old drive command AFTER stop() has sent S.
        self._drive_lock = threading.Lock()
        self._state_lock = threading.Condition()
        self._seq = 0
        self._pending: dict[int, _Pending] = {}
        self._telemetry: Telemetry | None = None
        self._telemetry_time = 0.0  # time.monotonic() when it arrived
        self._hello_event = threading.Event()
        self._drive_command: DutyCommand | VelocityCommand | None = None
        self._drive_command_set_at = 0.0
        self._last_drive_send = 0.0
        self._connected = threading.Event()
        self._closing = threading.Event()

        self._serial = self._factory(port, baud, 0.0)
        self._connected.set()
        self._reader = threading.Thread(target=self._reader_loop, name="SerialBase-reader", daemon=True)
        self._keepalive = threading.Thread(target=self._keepalive_loop, name="SerialBase-keepalive", daemon=True)
        self._reader.start()
        self._keepalive.start()
        try:
            self.firmware = self.hello(timeout_s=connect_timeout_s)
            self.wait_for_telemetry(timeout_s=connect_timeout_s)
        except SerialBaseError:
            self.close()
            raise
        if self.firmware.protocol_version != protocol.PROTOCOL_VERSION:
            log.warning(
                "firmware speaks protocol v%d, this library v%d",
                self.firmware.protocol_version, protocol.PROTOCOL_VERSION,
            )

    # --- DifferentialBase ------------------------------------------------------------------------
    def set_wheel_duty(self, left: float, right: float) -> None:
        """Open-loop duty, each -1.0..1.0 (clamped). SAFETY: the wheels start turning."""
        command = DutyCommand(self._next_seq(), duty_to_permille(left), duty_to_permille(right))
        self._set_drive_command(command)

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        """Wheel speed setpoints in rad/s (clamped to the max wheel speed). SAFETY: the wheels
        start turning."""
        limit = self.max_wheel_speed_rad_s
        left = max(-limit, min(limit, left_rad_s))
        right = max(-limit, min(limit, right_rad_s))
        command = VelocityCommand(self._next_seq(), rad_s_to_mrad_s(left), rad_s_to_mrad_s(right))
        self._set_drive_command(command)

    def stop(self) -> None:
        """Forget the drive command and brake. Waits for the firmware's acknowledgement."""
        with self._drive_lock, self._state_lock:
            self._drive_command = None
        self._send_and_wait(lambda seq: StopCommand(seq))

    def read(self, max_age_s: float | None = None) -> BaseState:
        """The newest telemetry sample in SI units.

        Raises :class:`TelemetryTimeoutError` if the newest sample is older than ``max_age_s``
        (default ``max_telemetry_age_s``) — e.g. the cable is unplugged. Stale data is worse than
        no data for a robot: it would happily keep driving toward an obstacle seen a second ago.
        """
        limit = self.max_telemetry_age_s if max_age_s is None else max_age_s
        with self._state_lock:
            telemetry, arrived = self._telemetry, self._telemetry_time
        if telemetry is None:
            raise TelemetryTimeoutError("no telemetry received yet")
        age = time.monotonic() - arrived
        if age > limit:
            raise TelemetryTimeoutError(f"newest telemetry is {age:.2f} s old (connected={self.connected})")
        return telemetry_to_state(telemetry)

    def close(self) -> None:
        """Stop the motors (best effort), stop the threads and close the port."""
        if self._closing.is_set():
            return
        if self.connected:
            try:
                self.stop()
            except SerialBaseError as exc:
                log.warning("could not stop the motors while closing: %s", exc)
        self._closing.set()
        for thread in (getattr(self, "_reader", None), getattr(self, "_keepalive", None)):
            if thread is not None and thread is not threading.current_thread() and thread.is_alive():
                thread.join(timeout=1.0)
        self._close_port()

    def __enter__(self) -> SerialBase:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # --- more commands -------------------------------------------------------------------------
    def hello(self, timeout_s: float = 1.0) -> HelloReply:
        """Send ``H`` and wait for the ``I`` reply (retried until ``timeout_s``)."""
        deadline = time.monotonic() + timeout_s
        self._hello_event.clear()
        while True:
            self._write(Hello(self._next_seq()))
            remaining = deadline - time.monotonic()
            if self._hello_event.wait(timeout=max(0.0, min(self.ack_timeout_s, remaining))):
                assert self.firmware is not None
                return self.firmware
            if time.monotonic() >= deadline:
                raise AckTimeoutError(
                    f"no hello reply from {self.port} within {timeout_s:.1f} s — "
                    "is the karmel firmware (main.py) running on the Pico?"
                )

    def reset_encoders(self) -> None:
        """Zero both tick counters (waits for the acknowledgement)."""
        self._send_and_wait(lambda seq: ResetEncoders(seq))

    def set_param(self, key: str, value: float) -> None:
        """Set a firmware parameter: kp, ki, kd, ff, watchdog_ms, telemetry_hz."""
        self._send_and_wait(lambda seq: SetParam(seq, key, value))

    def wait_for_telemetry(self, timeout_s: float = 1.0, newer_than: float | None = None) -> BaseState:
        """Block until a telemetry sample arrives (newer than robot time ``newer_than`` if given).

        Handy to synchronise a control loop with the 50 Hz telemetry stream::

            state = base.wait_for_telemetry(newer_than=state.t)
        """
        deadline = time.monotonic() + timeout_s
        with self._state_lock:
            while True:
                t = self._telemetry
                if t is not None and (newer_than is None or t.ms / 1000.0 > newer_than + 1e-9):
                    return telemetry_to_state(t)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TelemetryTimeoutError(f"no telemetry from {self.port} within {timeout_s:.1f} s")
                self._state_lock.wait(timeout=remaining)

    @property
    def connected(self) -> bool:
        return self._connected.is_set() and not self._closing.is_set()

    # --- internals: sending ----------------------------------------------------------------------
    def _next_seq(self) -> int:
        with self._state_lock:
            self._seq = (self._seq + 1) % (protocol.SEQ_MAX + 1)
            return self._seq

    def _write(self, command: protocol.HostMessage) -> None:
        if not self.connected:
            raise NotConnectedError(f"{self.port} is not connected")
        data = protocol.encode_bytes(command)
        try:
            with self._write_lock:
                self._serial.write(data)
        except Exception as exc:  # pyserial raises SerialException/OSError; sockets OSError
            self._handle_disconnect(exc)
            raise NotConnectedError(f"write to {self.port} failed: {exc}") from exc
        self.stats.commands_sent += 1

    def _track(self, command: protocol.HostMessage, wait: bool) -> _Pending:
        pending = _Pending(command, time.monotonic(), wait, threading.Event())
        with self._state_lock:
            self._expire_pending_locked()
            self._pending[command.seq] = pending  # type: ignore[union-attr]
        return pending

    def _send_and_wait(self, make: Callable[[int], protocol.HostMessage]) -> Ack:
        last_error: Exception | None = None
        for attempt in range(self.retries + 1):
            command = make(self._next_seq())
            pending = self._track(command, wait=True)
            self._write(command)
            if pending.event.wait(self.ack_timeout_s):
                assert pending.ack is not None
                if not pending.ack.ok:
                    raise CommandRejectedError(command, pending.ack.reason or "")
                return pending.ack
            with self._state_lock:
                self._pending.pop(command.seq, None)  # type: ignore[union-attr]
            self.stats.ack_timeouts += 1
            last_error = AckTimeoutError(f"no ack for {command.payload()!r} (attempt {attempt + 1})")
            log.debug("%s", last_error)
        raise AckTimeoutError(f"{last_error}; gave up after {self.retries + 1} attempts")

    def _set_drive_command(self, command: DutyCommand | VelocityCommand) -> None:
        with self._drive_lock:
            with self._state_lock:
                self._drive_command = command
                self._drive_command_set_at = time.monotonic()
            self._send_drive(command)

    def _send_drive(self, command: DutyCommand | VelocityCommand) -> None:
        self._track(command, wait=False)
        self._last_drive_send = time.monotonic()
        self._write(command)

    def _keepalive_loop(self) -> None:
        while not self._closing.wait(self.keepalive_s / 2):
            if not self.connected:
                continue
            with self._drive_lock:
                with self._state_lock:
                    command = self._drive_command
                    set_at = self._drive_command_set_at
                if command is None:
                    continue
                now = time.monotonic()
                if self.command_timeout_s is not None and now - set_at > self.command_timeout_s:
                    continue  # dead-man: the application stopped commanding; let the watchdog trip
                if now - self._last_drive_send >= self.keepalive_s:
                    refreshed = type(command)(self._next_seq(), *_drive_values(command))
                    try:
                        self._send_drive(refreshed)
                    except SerialBaseError:
                        pass  # disconnected; the reader thread handles reconnection

    # --- internals: receiving --------------------------------------------------------------------
    def _reader_loop(self) -> None:
        buffer = b""
        while not self._closing.is_set():
            if not self._connected.is_set():
                self._try_reconnect()
                buffer = b""
                continue
            try:
                # The port is opened with timeout=0: read() returns whatever has arrived, at once.
                # Reading big chunks matters - one select()+recv() per byte can't keep up with
                # 50 Hz telemetry in a Python thread, and the data would pile up.
                chunk = self._serial.read(4096)
            except Exception as exc:
                if self._closing.is_set():
                    break
                self._handle_disconnect(exc)
                continue
            if not chunk:
                time.sleep(0.002)  # nothing yet: yield instead of spinning the CPU
                continue
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                self._handle_line(line)
            if len(buffer) > 4 * protocol.MAX_LINE_LENGTH:
                buffer = b""  # a flood of garbage without newlines
                self.stats.bad_lines += 1

    def _handle_line(self, line: bytes) -> None:
        if not line.strip():
            return
        self.stats.lines_received += 1
        try:
            message = protocol.decode_pico_message(line)
        except protocol.ProtocolError as exc:
            self.stats.bad_lines += 1
            log.debug("ignoring line %r: %s", line, exc)
            return

        if isinstance(message, Telemetry):
            with self._state_lock:
                previous = self._telemetry
                if previous is not None and message.ms < previous.ms:
                    self.stats.firmware_restarts += 1
                    log.warning("robot clock went backwards: the Pico restarted")
                self._telemetry = message
                self._telemetry_time = time.monotonic()
                self.stats.telemetry += 1
                self._state_lock.notify_all()
        elif isinstance(message, Ack):
            with self._state_lock:
                pending = self._pending.pop(message.seq, None)
            if message.ok:
                self.stats.acks_ok += 1
            else:
                self.stats.acks_err += 1
                log.warning("firmware error for seq %d: %s", message.seq, message.reason)
            if pending is not None:
                pending.ack = message
                pending.event.set()
        elif isinstance(message, HelloReply):
            self.firmware = message
            self._hello_event.set()

    def _expire_pending_locked(self) -> None:
        """Drop fire-and-forget commands whose ack is long overdue (counted, not raised)."""
        now = time.monotonic()
        for seq in [s for s, p in self._pending.items() if now - p.sent_at > 5 * self.ack_timeout_s]:
            if not self._pending.pop(seq).wait:
                self.stats.unacked_drive_commands += 1

    # --- internals: connection -------------------------------------------------------------------
    def _handle_disconnect(self, exc: BaseException) -> None:
        if not self._connected.is_set() or self._closing.is_set():
            return
        log.warning("lost connection to %s: %s", self.port, exc)
        with self._state_lock:
            # SAFETY: forget the drive command, so a reconnect never silently restarts the motors.
            self._drive_command = None
        self._connected.clear()
        self._close_port()
        if not self.reconnect:
            log.error("reconnect disabled; SerialBase stays disconnected")

    def _try_reconnect(self) -> None:
        if not self.reconnect or self._closing.wait(self.reconnect_interval_s):
            return
        try:
            self._serial = self._factory(self.port, self.baud, 0.0)
        except Exception as exc:
            log.debug("reconnect to %s failed: %s", self.port, exc)
            return
        self._connected.set()
        self.stats.reconnects += 1
        log.info("reconnected to %s", self.port)
        try:
            self._write(Hello(self._next_seq()))
        except SerialBaseError:
            pass

    def _close_port(self) -> None:
        port, self._serial = self._serial, None
        if port is not None:
            try:
                port.close()
            except Exception:  # closing a dead port can fail in many ways; nothing to do
                pass


def _drive_values(command: DutyCommand | VelocityCommand) -> tuple[int, int]:
    if isinstance(command, DutyCommand):
        return command.left, command.right
    return command.left_mrad_s, command.right_mrad_s


def _config_max_wheel_speed() -> float:
    try:
        from robotlab.config import load_config

        return load_config().drive.max_wheel_speed_rad_s
    except Exception:  # no config file available: fall back to the karmel default
        return 17.0
