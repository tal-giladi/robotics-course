"""Protocol v1 in karmel_base: identical to robotlab and matching the README table line by line."""

from pathlib import Path

import pytest

from karmel_base import protocol as p
from karmel_base import wire

HERE = Path(__file__).resolve()
COPY = HERE.parent.parent / 'karmel_base' / 'protocol.py'
# labs/ros2_ws/src/karmel_base/test -> labs/python/robotlab/protocol.py
CANONICAL = HERE.parents[4] / 'python' / 'robotlab' / 'protocol.py' if len(HERE.parents) > 4 else None


@pytest.mark.skipif(CANONICAL is None or not CANONICAL.is_file(),
                    reason='labs/python/robotlab/protocol.py not present (installed copy only)')
def test_copy_is_byte_identical_to_robotlab():
    assert COPY.read_bytes() == CANONICAL.read_bytes(), (
        'karmel_base/protocol.py drifted from labs/python/robotlab/protocol.py — copy it again')


# ------------------------------------------------------------------------------ framing
def test_checksum_is_xor_of_payload_bytes():
    assert p.checksum('A') == 0x41
    assert p.checksum('AB') == 0x41 ^ 0x42
    assert p.frame('H 1') == 'H 1*59\n'          # 0x48 ^ 0x20 ^ 0x31 = 0x59 (same literal as the C++ test)


def test_wire_lines_from_the_readme_table():
    cases = [
        (p.Hello(1), 'H 1'),
        (p.DutyCommand(2, -1000, 1000), 'M 2 -1000 1000'),
        (p.VelocityCommand(4, 1500, -1500), 'V 4 1500 -1500'),
        (p.StopCommand(5), 'S 5'),
        (p.ResetEncoders(6), 'R 6'),
        (p.SetParam(7, 'kp', 0.8), 'P 7 kp 0.8'),
        (p.SetParam(8, 'watchdog_ms', 300), 'P 8 watchdog_ms 300'),
        (p.SetParam(9, 'telemetry_hz', 50), 'P 9 telemetry_hz 50'),
        (p.HelloReply('1.2.0', 1), 'I 1.2.0 1'),
        (p.Ack(12, True), 'A 12 OK'),
        (p.Ack(13, False, 'bad_args'), 'A 13 ERR bad_args'),
        (p.Telemetry(123456, -2464, 1232, 6283, -3141, 11850, -1, 13), 'T 123456 -2464 1232 6283 -3141 11850 -1 13'),
    ]
    for msg, payload in cases:
        line = p.frame(payload)
        assert p.encode(msg) == line
        assert p.encode_bytes(msg) == line.encode('ascii')
        assert p.decode(line) == msg
        assert p.decode(line.encode('ascii')) == msg


def test_crlf_accepted_and_corruption_rejected():
    line = p.encode(p.Telemetry(100, 5, 6, 7, 8, 12000, 250, 8))
    assert p.decode(line[:-1] + '\r\n') == p.decode(line)
    for i in range(len(line) - 4):              # flip one bit in every payload byte
        corrupted = line[:i] + chr(ord(line[i]) ^ 0x04) + line[i + 1:]
        with pytest.raises(p.ProtocolError):
            p.decode(corrupted)


@pytest.mark.parametrize('payload', ['H', 'V 1 2', 'V 1 1.5 2', 'V 1 +5 2', 'M 1 0 1001', 'P 1 gain 1',
                                     'P 1 watchdog_ms 2.5', 'T 1 2 3 4 5 6 7', 'Q 1', 'A 1 MAYBE'])
def test_malformed_payloads_rejected(payload):
    with pytest.raises(p.ProtocolError):
        p.decode(p.frame(payload))


def test_flag_values_match_readme():
    assert (p.FLAG_WATCHDOG, p.FLAG_LOW_BATTERY, p.FLAG_RANGE_ERROR, p.FLAG_VELOCITY_MODE) == (1, 2, 4, 8)
    assert set(p.PARAM_KEYS) == {'kp', 'ki', 'kd', 'ff', 'watchdog_ms', 'telemetry_hz'}


# ------------------------------------------------------------------------------ wire.py helpers
def test_telemetry_unit_helpers():
    t = p.decode('T 1 0 0 0 0 12000 350 5*' + format(p.checksum('T 1 0 0 0 0 12000 350 5'), '02X') + '\n')
    assert wire.battery_v(t) == pytest.approx(12.0)
    assert wire.range_m(t) == pytest.approx(0.35)
    assert wire.has_flag(t, p.FLAG_WATCHDOG) and wire.has_flag(t, p.FLAG_RANGE_ERROR)
    assert not wire.has_flag(t, p.FLAG_LOW_BATTERY)
    invalid = p.Telemetry(1, 0, 0, 0, 0, -1, -1, 0)
    assert wire.battery_v(invalid) is None and wire.range_m(invalid) is None


def test_seq_wraps_at_16_bits():
    assert wire.next_seq(0) == 1
    assert wire.next_seq(p.SEQ_MAX) == 0


def test_rad_s_to_mrad_s():
    assert wire.rad_s_to_mrad_s(17.0) == 17000
    assert wire.rad_s_to_mrad_s(-0.0004) == 0
    assert wire.rad_s_to_mrad_s(1.2346) == 1235


def test_line_buffer_reassembles_chunks():
    buf = wire.LineBuffer()
    data = p.encode_bytes(p.Telemetry(1, 2, 3, 4, 5, 12000, 100, 0)) + p.encode_bytes(p.Ack(9, True))
    lines = []
    for i in range(0, len(data), 5):
        lines += buf.feed(data[i:i + 5])
    assert [p.decode(line) for line in lines] == [p.Telemetry(1, 2, 3, 4, 5, 12000, 100, 0), p.Ack(9, True)]


def test_line_buffer_discards_garbage_without_newline():
    buf = wire.LineBuffer(max_length=32)
    assert buf.feed(b'x' * 100) == []
    assert buf.overflows == 1
    assert buf.feed(p.encode_bytes(p.StopCommand(1))) == [p.encode_bytes(p.StopCommand(1))]
