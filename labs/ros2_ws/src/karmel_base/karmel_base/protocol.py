"""Pi <-> Pico serial protocol, version 1 — the canonical (host-side) implementation.

Taught in lesson 01.10 (design) and 03.03 (robustness). The wire format is specified in
``labs/README.md``; this module is the reference every other copy must agree with:

* ``labs/firmware/pico/protocol.py`` — a MicroPython-compatible subset running on the Pico,
* ``labs/ros2_ws`` — the ROS 2 hardware interface copies this file.

``labs/python/tests/test_protocol.py`` round-trips lines between this module and the firmware
copy, so a change here that the firmware doesn't follow fails the tests.

Wire format
-----------
One message per line, ASCII, NMEA-style checksum::

    <payload>*<hh>\\n        hh = XOR of every payload byte, two uppercase hex digits

    >>> encode(DutyCommand(seq=7, left=500, right=-500))
    'M 7 500 -500*77\\n'
    >>> decode('A 7 OK*72\\n')
    Ack(seq=7, ok=True, reason=None)

Payload fields are separated by exactly one space. Integers are plain decimal: ``-12`` is
valid, ``+12`` and ``12.0`` are not.

Details the README table leaves open, fixed here so every implementation agrees:

* ``seq`` is a 16-bit sequence number, 0..65535; the host wraps back to 0 after 65535.
* ``batt_mV`` in telemetry is ``-1`` when the firmware has no battery reading (like
  ``range_mm``).
* The checksum hex digits are *emitted* uppercase; a decoder accepts either case.
* The ``ERR`` reason is free text (one or more words) without ``*``. The firmware uses
  short snake_case codes: ``bad_args``, ``out_of_range``, ``unknown_command``, ``unknown_param``.
* A line whose checksum is wrong is dropped by the firmware *without* a reply — its ``seq`` can't
  be trusted. The host notices through the missing acknowledgement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Union

PROTOCOL_VERSION = 1

SEQ_MAX = 65535
"""Sequence numbers are 16-bit: 0..SEQ_MAX."""

DUTY_SCALE = 1000
"""Open-loop duty is sent as integer per-mille: 1000 = 100 % duty."""

PARAM_KEYS = ("kp", "ki", "kd", "ff", "watchdog_ms", "telemetry_hz")
"""Keys accepted by the ``P`` (set parameter) command."""

INTEGER_PARAMS = ("watchdog_ms", "telemetry_hz")
"""Parameters whose value must be a whole number."""

# Telemetry flag bits (same values as robotlab.hal.FLAG_*).
FLAG_WATCHDOG = 1
FLAG_LOW_BATTERY = 2
FLAG_RANGE_ERROR = 4
FLAG_VELOCITY_MODE = 8

MAX_LINE_LENGTH = 120
"""Longest line (including checksum) either side must accept. Longer input is a framing error."""


# --- errors ----------------------------------------------------------------------------------------
class ProtocolError(ValueError):
    """A line or message that doesn't follow protocol v1."""


class FramingError(ProtocolError):
    """The line isn't ``<payload>*<hh>``: missing ``*``, bad hex, non-ASCII, too long…"""


class ChecksumError(ProtocolError):
    """The two hex digits don't match the XOR of the payload — the line was corrupted."""

    def __init__(self, payload: str, expected: int, received: int) -> None:
        super().__init__(
            f"checksum mismatch for {payload!r}: computed {expected:02X}, line says {received:02X}"
        )
        self.payload = payload
        self.expected = expected
        self.received = received


class MessageError(ProtocolError):
    """The framing is fine but the payload isn't a valid message (unknown type, bad fields)."""


# --- framing ---------------------------------------------------------------------------------------
def checksum(payload: str) -> int:
    """XOR of all payload bytes (0..255). The ``*`` and the newline are not included."""
    value = 0
    for byte in payload.encode("ascii"):
        value ^= byte
    return value


def frame(payload: str) -> str:
    """Wrap a payload into a complete line: ``payload*HH\\n``."""
    _check_payload_chars(payload)
    return f"{payload}*{checksum(payload):02X}\n"


def unframe(line: str | bytes) -> str:
    """Verify a received line and return its payload.

    Accepts a trailing ``\\n`` or ``\\r\\n`` (MicroPython's USB console turns ``\\n`` into
    ``\\r\\n``). Raises :class:`FramingError` or :class:`ChecksumError`.
    """
    if isinstance(line, bytes):
        try:
            line = line.decode("ascii")
        except UnicodeDecodeError as exc:
            raise FramingError(f"line is not ASCII: {line!r}") from exc
    if line.endswith("\n"):
        line = line[:-1]
    if line.endswith("\r"):
        line = line[:-1]
    if len(line) > MAX_LINE_LENGTH:
        raise FramingError(f"line longer than {MAX_LINE_LENGTH} characters")
    if len(line) < 4 or line[-3] != "*":
        raise FramingError(f"expected '<payload>*<hh>', got {line!r}")
    payload, hex_digits = line[:-3], line[-2:]
    _check_payload_chars(payload)
    if not all(c in "0123456789abcdefABCDEF" for c in hex_digits):
        raise FramingError(f"checksum {hex_digits!r} is not two hex digits")
    received = int(hex_digits, 16)
    expected = checksum(payload)
    if received != expected:
        raise ChecksumError(payload, expected, received)
    return payload


def _check_payload_chars(payload: str) -> None:
    if not payload:
        raise FramingError("empty payload")
    for c in payload:
        if not " " <= c <= "~" or c == "*":
            raise FramingError(f"payload {payload!r} contains {c!r} (only printable ASCII, no '*')")


# --- field helpers (shared by encode validation and decode) ----------------------------------------
def _parse_int(token: str, name: str) -> int:
    """Plain decimal integer: optional '-', then at least one digit. No '+', no spaces."""
    digits = token[1:] if token.startswith("-") else token
    if not digits or not all("0" <= c <= "9" for c in digits):
        raise MessageError(f"{name}: {token!r} is not an integer")
    return int(token)


def _parse_number(token: str, name: str) -> float:
    """A finite decimal number such as ``0.02``, ``-3``, ``1e-05``."""
    if not token or not all(c in "0123456789.-+eE" for c in token) or not any(c.isdigit() for c in token):
        raise MessageError(f"{name}: {token!r} is not a number")
    try:
        value = float(token)
    except ValueError as exc:
        raise MessageError(f"{name}: {token!r} is not a number") from exc
    if not math.isfinite(value):
        raise MessageError(f"{name}: {token!r} is not finite")
    return value


def _check_int(value: object, name: str, low: int | None = None, high: int | None = None) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise MessageError(f"{name} must be an int, got {value!r}")
    if (low is not None and value < low) or (high is not None and value > high):
        raise MessageError(f"{name}={value} is outside {low}..{high}")


def _check_seq(seq: object) -> None:
    _check_int(seq, "seq", 0, SEQ_MAX)


def _check_token(text: object, name: str) -> None:
    if not isinstance(text, str) or not text or " " in text:
        raise MessageError(f"{name} must be a non-empty string without spaces, got {text!r}")
    _check_payload_chars(text)


# --- host -> Pico messages -------------------------------------------------------------------------
@dataclass(frozen=True)
class Hello:
    """``H <seq>`` — ask the firmware who it is. Reply: :class:`HelloReply`."""

    seq: int

    def __post_init__(self) -> None:
        _check_seq(self.seq)

    def payload(self) -> str:
        return f"H {self.seq}"


@dataclass(frozen=True)
class DutyCommand:
    """``M <seq> <left> <right>`` — open-loop duty in per-mille (-1000..1000)."""

    seq: int
    left: int
    right: int

    def __post_init__(self) -> None:
        _check_seq(self.seq)
        _check_int(self.left, "left", -DUTY_SCALE, DUTY_SCALE)
        _check_int(self.right, "right", -DUTY_SCALE, DUTY_SCALE)

    def payload(self) -> str:
        return f"M {self.seq} {self.left} {self.right}"


@dataclass(frozen=True)
class VelocityCommand:
    """``V <seq> <left_mrad_s> <right_mrad_s>`` — wheel speed setpoints, closed loop on the Pico."""

    seq: int
    left_mrad_s: int
    right_mrad_s: int

    def __post_init__(self) -> None:
        _check_seq(self.seq)
        _check_int(self.left_mrad_s, "left_mrad_s")
        _check_int(self.right_mrad_s, "right_mrad_s")

    def payload(self) -> str:
        return f"V {self.seq} {self.left_mrad_s} {self.right_mrad_s}"


@dataclass(frozen=True)
class StopCommand:
    """``S <seq>`` — brake both motors."""

    seq: int

    def __post_init__(self) -> None:
        _check_seq(self.seq)

    def payload(self) -> str:
        return f"S {self.seq}"


@dataclass(frozen=True)
class ResetEncoders:
    """``R <seq>`` — zero both encoder counts."""

    seq: int

    def __post_init__(self) -> None:
        _check_seq(self.seq)

    def payload(self) -> str:
        return f"R {self.seq}"


@dataclass(frozen=True)
class SetParam:
    """``P <seq> <key> <value>`` — change a firmware parameter (see :data:`PARAM_KEYS`)."""

    seq: int
    key: str
    value: float

    def __post_init__(self) -> None:
        _check_seq(self.seq)
        if self.key not in PARAM_KEYS:
            raise MessageError(f"unknown parameter {self.key!r}; expected one of {', '.join(PARAM_KEYS)}")
        if isinstance(self.value, bool) or not isinstance(self.value, (int, float)):
            raise MessageError(f"parameter value must be a number, got {self.value!r}")
        if not math.isfinite(self.value):
            raise MessageError(f"parameter value must be finite, got {self.value!r}")
        if self.key in INTEGER_PARAMS and self.value != int(self.value):
            raise MessageError(f"{self.key} must be a whole number, got {self.value!r}")

    def value_text(self) -> str:
        if self.key in INTEGER_PARAMS:
            return str(int(self.value))
        return repr(float(self.value))  # shortest text that parses back to the same float

    def payload(self) -> str:
        return f"P {self.seq} {self.key} {self.value_text()}"


# --- Pico -> host messages -------------------------------------------------------------------------
@dataclass(frozen=True)
class HelloReply:
    """``I <firmware_version> <protocol_version>``."""

    firmware_version: str
    protocol_version: int

    def __post_init__(self) -> None:
        _check_token(self.firmware_version, "firmware_version")
        _check_int(self.protocol_version, "protocol_version", 0)

    def payload(self) -> str:
        return f"I {self.firmware_version} {self.protocol_version}"


@dataclass(frozen=True)
class Ack:
    """``A <seq> OK`` or ``A <seq> ERR <reason>``."""

    seq: int
    ok: bool
    reason: str | None = None

    def __post_init__(self) -> None:
        _check_seq(self.seq)
        if self.ok and self.reason is not None:
            raise MessageError("an OK acknowledgement has no reason")
        if not self.ok:
            if not isinstance(self.reason, str) or not self.reason.strip() or self.reason != self.reason.strip():
                raise MessageError(f"an ERR acknowledgement needs a reason, got {self.reason!r}")
            if "  " in self.reason:
                raise MessageError("reason words must be separated by single spaces")
            _check_payload_chars(self.reason)

    def payload(self) -> str:
        return f"A {self.seq} OK" if self.ok else f"A {self.seq} ERR {self.reason}"


@dataclass(frozen=True)
class Telemetry:
    """``T <ms> <lticks> <rticks> <l_mrad_s> <r_mrad_s> <batt_mV> <range_mm> <flags>``.

    Raw integers exactly as on the wire; :class:`robotlab.serial_base.SerialBase` converts them to
    SI units in a :class:`robotlab.hal.BaseState`.
    """

    ms: int
    left_ticks: int
    right_ticks: int
    left_mrad_s: int
    right_mrad_s: int
    battery_mv: int  # -1 = no reading
    range_mm: int  # -1 = no valid reading
    flags: int

    def __post_init__(self) -> None:
        _check_int(self.ms, "ms", 0)
        _check_int(self.left_ticks, "left_ticks")
        _check_int(self.right_ticks, "right_ticks")
        _check_int(self.left_mrad_s, "left_mrad_s")
        _check_int(self.right_mrad_s, "right_mrad_s")
        _check_int(self.battery_mv, "battery_mv", -1)
        _check_int(self.range_mm, "range_mm", -1)
        _check_int(self.flags, "flags", 0, 255)

    def payload(self) -> str:
        return (
            f"T {self.ms} {self.left_ticks} {self.right_ticks} {self.left_mrad_s} {self.right_mrad_s} "
            f"{self.battery_mv} {self.range_mm} {self.flags}"
        )


HostMessage = Union[Hello, DutyCommand, VelocityCommand, StopCommand, ResetEncoders, SetParam]
PicoMessage = Union[HelloReply, Ack, Telemetry]
Message = Union[HostMessage, PicoMessage]

HOST_MESSAGE_TYPES = (Hello, DutyCommand, VelocityCommand, StopCommand, ResetEncoders, SetParam)
PICO_MESSAGE_TYPES = (HelloReply, Ack, Telemetry)


# --- encode / decode -------------------------------------------------------------------------------
def encode(message: Message) -> str:
    """Message dataclass -> complete line (with checksum and ``\\n``)."""
    return frame(message.payload())


def encode_bytes(message: Message) -> bytes:
    """Like :func:`encode`, as bytes ready for ``serial.write``."""
    return encode(message).encode("ascii")


def decode(line: str | bytes) -> Message:
    """Complete line -> message dataclass. Raises a :class:`ProtocolError` subclass."""
    return parse_payload(unframe(line))


def decode_host_message(line: str | bytes) -> HostMessage:
    """Decode a line that must be a host -> Pico command (what the firmware receives)."""
    message = decode(line)
    if not isinstance(message, HOST_MESSAGE_TYPES):
        raise MessageError(f"{type(message).__name__} is not a host -> Pico command")
    return message


def decode_pico_message(line: str | bytes) -> PicoMessage:
    """Decode a line that must come from the Pico (what the host receives)."""
    message = decode(line)
    if not isinstance(message, PICO_MESSAGE_TYPES):
        raise MessageError(f"{type(message).__name__} is not a Pico -> host message")
    return message


def parse_payload(payload: str) -> Message:
    """Payload text (no checksum) -> message dataclass."""
    fields = payload.split(" ")
    if any(f == "" for f in fields):
        raise MessageError(f"fields must be separated by exactly one space: {payload!r}")
    kind, args = fields[0], fields[1:]
    parser = _PARSERS.get(kind)
    if parser is None:
        raise MessageError(f"unknown message type {kind!r} in {payload!r}")
    try:
        return parser(args)
    except MessageError as exc:
        raise MessageError(f"{payload!r}: {exc}") from None


def _expect(args: list[str], count: int, usage: str) -> None:
    if len(args) != count:
        raise MessageError(f"expected '{usage}' ({count} fields after the type), got {len(args)}")


def _parse_hello(args: list[str]) -> Hello:
    _expect(args, 1, "H <seq>")
    return Hello(_parse_int(args[0], "seq"))


def _parse_duty(args: list[str]) -> DutyCommand:
    _expect(args, 3, "M <seq> <left> <right>")
    return DutyCommand(_parse_int(args[0], "seq"), _parse_int(args[1], "left"), _parse_int(args[2], "right"))


def _parse_velocity(args: list[str]) -> VelocityCommand:
    _expect(args, 3, "V <seq> <left_mrad_s> <right_mrad_s>")
    return VelocityCommand(
        _parse_int(args[0], "seq"), _parse_int(args[1], "left_mrad_s"), _parse_int(args[2], "right_mrad_s")
    )


def _parse_stop(args: list[str]) -> StopCommand:
    _expect(args, 1, "S <seq>")
    return StopCommand(_parse_int(args[0], "seq"))


def _parse_reset(args: list[str]) -> ResetEncoders:
    _expect(args, 1, "R <seq>")
    return ResetEncoders(_parse_int(args[0], "seq"))


def _parse_param(args: list[str]) -> SetParam:
    _expect(args, 3, "P <seq> <key> <value>")
    seq = _parse_int(args[0], "seq")
    key = args[1]
    if key in INTEGER_PARAMS:
        value: float = _parse_int(args[2], key)
    else:
        value = _parse_number(args[2], key)
    return SetParam(seq, key, value)


def _parse_hello_reply(args: list[str]) -> HelloReply:
    _expect(args, 2, "I <firmware_version> <protocol_version>")
    return HelloReply(args[0], _parse_int(args[1], "protocol_version"))


def _parse_ack(args: list[str]) -> Ack:
    if len(args) == 2 and args[1] == "OK":
        return Ack(_parse_int(args[0], "seq"), True)
    if len(args) >= 3 and args[1] == "ERR":
        return Ack(_parse_int(args[0], "seq"), False, " ".join(args[2:]))
    raise MessageError("expected 'A <seq> OK' or 'A <seq> ERR <reason>'")


def _parse_telemetry(args: list[str]) -> Telemetry:
    _expect(args, 8, "T <ms> <lticks> <rticks> <l_mrad_s> <r_mrad_s> <batt_mV> <range_mm> <flags>")
    names = ("ms", "left_ticks", "right_ticks", "left_mrad_s", "right_mrad_s", "battery_mv", "range_mm", "flags")
    values = [_parse_int(token, name) for token, name in zip(args, names)]
    return Telemetry(*values)


_PARSERS = {
    "H": _parse_hello,
    "M": _parse_duty,
    "V": _parse_velocity,
    "S": _parse_stop,
    "R": _parse_reset,
    "P": _parse_param,
    "I": _parse_hello_reply,
    "A": _parse_ack,
    "T": _parse_telemetry,
}
