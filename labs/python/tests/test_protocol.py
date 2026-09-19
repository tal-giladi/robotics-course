"""Protocol v1: the host implementation (robotlab.protocol) and the firmware copy
(labs/firmware/pico/protocol.py) must agree on every line, in both directions."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

from robotlab import protocol as p

FIRMWARE_PROTOCOL = Path(__file__).resolve().parents[2] / "firmware" / "pico" / "protocol.py"


@pytest.fixture(scope="module")
def fwp():
    """The firmware protocol module, loaded under a private name (it is pure Python)."""
    spec = importlib.util.spec_from_file_location("pico_firmware_protocol", FIRMWARE_PROTOCOL)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


# --- framing -----------------------------------------------------------------------------------------
def test_checksum_is_xor_of_payload_bytes():
    assert p.checksum("A") == 0x41
    assert p.checksum("AB") == 0x41 ^ 0x42
    assert p.frame("A 7 OK") == "A 7 OK*72\n"
    assert p.encode(p.DutyCommand(7, 500, -500)) == "M 7 500 -500*77\n"


@pytest.mark.parametrize("line", ["S 9*4A\n", "S 9*4A\r\n", "S 9*4a", b"S 9*4A\n"])
def test_unframe_accepts_crlf_lowercase_and_bytes(line):
    assert p.checksum("S 9") == 0x4A
    assert p.unframe(line) == "S 9"


@pytest.mark.parametrize(
    ("line", "error"),
    [
        ("S 1\n", p.FramingError),  # no checksum
        ("S 1*6\n", p.FramingError),
        ("S 1*G0\n", p.FramingError),
        ("*00\n", p.FramingError),  # empty payload
        ("S*1*00\n", p.FramingError),  # '*' inside the payload
        ("S\t1*00\n", p.FramingError),  # control character
        ("S 1*6E\n", p.ChecksumError),
        (b"S \xff*00\n", p.FramingError),  # not ASCII
        ("T " + "1 " * 80 + "*00\n", p.FramingError),  # too long
    ],
)
def test_unframe_rejects(line, error):
    with pytest.raises(error):
        p.unframe(line)


# --- messages ------------------------------------------------------------------------------------------
ALL_MESSAGES = [
    p.Hello(0),
    p.DutyCommand(1, 1000, -1000),
    p.DutyCommand(65535, 0, 0),
    p.VelocityCommand(2, 5000, -12345),
    p.StopCommand(3),
    p.ResetEncoders(4),
    p.SetParam(5, "kp", 0.02),
    p.SetParam(6, "ki", 1e-05),
    p.SetParam(7, "ff", 1.0),
    p.SetParam(8, "watchdog_ms", 300),
    p.SetParam(9, "telemetry_hz", 50),
    p.HelloReply("0.1.0", 1),
    p.Ack(10, True),
    p.Ack(11, False, "out_of_range"),
    p.Ack(12, False, "two words"),
    p.Telemetry(123456, -5, 2464, 6283, -6283, 11850, -1, 1 | 8),
]


@pytest.mark.parametrize("message", ALL_MESSAGES, ids=lambda m: m.payload())
def test_encode_decode_round_trip(message):
    line = p.encode(message)
    assert line.endswith("\n") and line.count("*") == 1
    assert p.decode(line) == message
    assert p.decode(p.encode_bytes(message)) == message


@pytest.mark.parametrize(
    "payload",
    [
        "M 1 500",  # missing field
        "M 1 500 500 500",  # extra field
        "M 1  500 500",  # double space
        "M 1 +500 500",  # '+' sign
        "M 1 5.0 500",  # not an integer
        "M 1 1001 0",  # out of range
        "M -1 0 0",  # negative seq
        "M 65536 0 0",  # seq beyond 16 bits
        "H",
        "S 1 2",
        "P 1 gain 0.5",  # unknown key
        "P 1 kp nan",
        "P 1 kp inf",
        "P 1 kp 0x10",
        "P 1 watchdog_ms 300.5",
        "A 1 MAYBE",
        "A 1 ERR",
        "T 1 2 3",
        "Z 1",
        "I 0.1.0",
    ],
)
def test_host_decoder_rejects_invalid_payloads(payload):
    with pytest.raises(p.MessageError):
        p.decode(p.frame(payload))


def test_dataclass_validation_catches_mistakes_before_sending():
    with pytest.raises(p.MessageError):
        p.DutyCommand(1, 0.5, 0)  # type: ignore[arg-type]  # duty must be per-mille ints
    with pytest.raises(p.MessageError):
        p.SetParam(1, "watchdog_ms", 12.5)
    with pytest.raises(p.MessageError):
        p.Ack(1, True, "why")
    with pytest.raises(p.MessageError):
        p.HelloReply("has space", 1)


def test_decode_by_direction():
    assert isinstance(p.decode_host_message(p.encode(p.StopCommand(1))), p.StopCommand)
    assert isinstance(p.decode_pico_message(p.encode(p.Ack(1, True))), p.Ack)
    with pytest.raises(p.MessageError):
        p.decode_pico_message(p.encode(p.StopCommand(1)))
    with pytest.raises(p.MessageError):
        p.decode_host_message(p.encode(p.Ack(1, True)))


# --- host <-> firmware agreement -------------------------------------------------------------------------
HOST_COMMANDS = [m for m in ALL_MESSAGES if isinstance(m, p.HOST_MESSAGE_TYPES)] + [
    p.SetParam(20, "kd", 0.000123456789),
    p.SetParam(21, "kp", 1e-12),
    p.SetParam(22, "ff", 2.0),
    p.VelocityCommand(23, 0, -1),
]


def _firmware_view(message):
    """What the firmware's parse_command should return for a host command."""
    if isinstance(message, (p.Hello, p.StopCommand, p.ResetEncoders)):
        return (message.payload()[0], message.seq, ())
    if isinstance(message, p.DutyCommand):
        return ("M", message.seq, (message.left, message.right))
    if isinstance(message, p.VelocityCommand):
        return ("V", message.seq, (message.left_mrad_s, message.right_mrad_s))
    assert isinstance(message, p.SetParam)
    return ("P", message.seq, (message.key, message.value))


@pytest.mark.parametrize("message", HOST_COMMANDS, ids=lambda m: m.payload())
def test_firmware_parses_every_host_command(fwp, message):
    payload = fwp.unframe(p.encode(message))
    assert fwp.parse_command(payload) == _firmware_view(message)


@pytest.mark.parametrize("message", HOST_COMMANDS, ids=lambda m: m.payload())
def test_firmware_frames_identically(fwp, message):
    assert fwp.frame(message.payload()) == p.encode(message)
    assert fwp.checksum(message.payload()) == p.checksum(message.payload())


def test_firmware_replies_decode_on_the_host(fwp):
    assert p.decode(fwp.format_hello_reply("0.1.0")) == p.HelloReply("0.1.0", 1)
    assert p.decode(fwp.format_ack(42)) == p.Ack(42, True)
    assert p.decode(fwp.format_error(43, fwp.ERR_BAD_ARGS)) == p.Ack(43, False, "bad_args")
    line = fwp.format_telemetry(99999, -3, 7, 1500, -1500, 12010, 845, 8)
    assert p.decode(line) == p.Telemetry(99999, -3, 7, 1500, -1500, 12010, 845, 8)
    assert line == p.encode(p.Telemetry(99999, -3, 7, 1500, -1500, 12010, 845, 8))


@pytest.mark.parametrize(
    "payload",
    ["M 1 500", "M 1  500 500", "M 1 +500 500", "M 1 5.0 500", "M 1 1001 0", "M -1 0 0", "M 65536 0 0",
     "H", "S 1 2", "P 1 gain 0.5", "P 1 kp nan", "P 1 kp inf", "P 1 kp 0x10", "P 1 watchdog_ms 300.5", "Z 1"],
)
def test_both_sides_reject_the_same_commands(fwp, payload):
    with pytest.raises(p.MessageError):
        p.decode(p.frame(payload))
    with pytest.raises(fwp.ProtocolError):
        fwp.parse_command(fwp.unframe(fwp.frame(payload)))


@pytest.mark.parametrize("line", ["S 1\n", "S 1*6E\n", "S 1*G0\n", "S\t1*00\n"])
def test_both_sides_reject_the_same_frames(fwp, line):
    with pytest.raises(p.ProtocolError):
        p.unframe(line)
    with pytest.raises(fwp.ProtocolError):
        fwp.unframe(line)
