"""Checker for 03.03 — A line-framing parser that survives a real serial link.

Run: ``python course.py check 03.03`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import random

import pytest

from robotlab import protocol

T_LINE = protocol.encode_bytes(protocol.Telemetry(1500, 10, -20, 6283, -100, 11100, 350, 9))
ACK_LINE = protocol.encode_bytes(protocol.Ack(7, True))


# --- checksums ----------------------------------------------------------------------------------
def test_xor_checksum_matches_protocol_v1(impl):
    assert impl.xor_checksum(b"A") == 0x41
    assert impl.xor_checksum(b"S 9") == 0x4A  # 'S'=0x53 ^ ' '=0x20 ^ '9'=0x39 = 0x4A
    assert impl.xor_checksum(b"") == 0
    for payload in ("A 7 OK", "M 7 500 -500", "T 1 2 3 4 5 6 7 8"):
        assert impl.xor_checksum(payload.encode()) == protocol.checksum(payload)


def test_crc16_check_value(impl):
    # Every CRC catalogue lists the CRC of the ASCII string "123456789" as the "check" value.
    assert impl.crc16_ccitt_false(b"123456789") == 0x29B1
    assert impl.crc16_ccitt_false(b"") == 0xFFFF


def test_xor_misses_swapped_bytes_but_crc_catches_them(impl):
    a, b = b"V 12 5000 -3000", b"V 12 5000 -0030"  # same bytes, different order
    assert impl.xor_checksum(a) == impl.xor_checksum(b), "XOR is order-blind: this corruption passes"
    assert impl.crc16_ccitt_false(a) != impl.crc16_ccitt_false(b), "a CRC depends on byte positions"


def test_crc_detects_every_single_bit_flip(impl):
    data = bytearray(b"T 1500 10 -20 6283 -100 11100 350 9")
    good = impl.crc16_ccitt_false(bytes(data))
    for i in range(len(data)):
        for bit in range(8):
            data[i] ^= 1 << bit
            assert impl.crc16_ccitt_false(bytes(data)) != good, f"missed a flip of bit {bit} in byte {i}"
            data[i] ^= 1 << bit


# --- frame ----------------------------------------------------------------------------------------
def test_frame(impl):
    assert impl.frame("S 9") == b"S 9*4A\n"
    assert impl.frame("A 7 OK") == b"A 7 OK*72\n"
    assert impl.frame("M 7 500 -500") == protocol.encode_bytes(protocol.DutyCommand(7, 500, -500))


@pytest.mark.parametrize("payload", ["", "S*1", "S 1\n", "S 1\r"])
def test_frame_rejects_payloads_that_would_break_framing(impl, payload):
    with pytest.raises(ValueError):
        impl.frame(payload)


# --- LineReader: the happy path -------------------------------------------------------------------
def test_one_complete_line(impl):
    reader = impl.LineReader()
    assert reader.feed(ACK_LINE) == ["A 7 OK"]
    assert (reader.bad_lines, reader.overflows) == (0, 0)


def test_line_split_across_chunks(impl):
    reader = impl.LineReader()
    assert reader.feed(T_LINE[:5]) == []
    assert reader.feed(T_LINE[5:20]) == []
    assert reader.feed(T_LINE[20:]) == [T_LINE[:-4].decode()]


def test_byte_by_byte(impl):
    reader = impl.LineReader()
    out = []
    for i in range(len(T_LINE)):
        out += reader.feed(T_LINE[i:i + 1])
    assert out == [T_LINE[:-4].decode()]


def test_concatenated_lines_in_one_chunk(impl):
    reader = impl.LineReader()
    stream = T_LINE + ACK_LINE + T_LINE
    assert reader.feed(stream) == [T_LINE[:-4].decode(), "A 7 OK", T_LINE[:-4].decode()]


def test_crlf_and_lowercase_hex_are_accepted(impl):
    reader = impl.LineReader()
    assert reader.feed(b"S 9*4A\r\nS 9*4a\n") == ["S 9", "S 9"]


def test_empty_lines_are_ignored_without_counting(impl):
    reader = impl.LineReader()
    assert reader.feed(b"\n\r\n" + ACK_LINE + b"\n") == ["A 7 OK"]
    assert reader.bad_lines == 0


# --- LineReader: a hostile stream -----------------------------------------------------------------
@pytest.mark.parametrize(
    "bad",
    [
        b"S 9*4B\n",  # wrong checksum
        b"S 9\n",  # no checksum
        b"S 9*4\n",  # one hex digit
        b"S 9*G1\n",  # not hex
        b"*00\n",  # empty payload
        b"S\t9*00\n",  # control character in the payload
        b"S \xff9*00\n",  # not ASCII
        b"S*9*00\n",  # '*' inside the payload
    ],
)
def test_bad_lines_are_rejected_and_counted(impl, bad):
    reader = impl.LineReader()
    assert reader.feed(bad + ACK_LINE) == ["A 7 OK"], "a bad line must not poison the next good one"
    assert reader.bad_lines == 1


def test_partial_first_line_after_opening_the_port(impl):
    # You open the port in the middle of a telemetry line: the first "line" is its tail.
    reader = impl.LineReader()
    assert reader.feed(T_LINE[17:] + ACK_LINE) == ["A 7 OK"]
    assert reader.bad_lines == 1


def test_overlong_garbage_is_discarded_until_the_next_newline(impl):
    reader = impl.LineReader(max_line_length=120)
    garbage = b"x" * 500  # wrong baud rate, a flood without newlines
    out = reader.feed(garbage[:200]) + reader.feed(garbage[200:]) + reader.feed(b"tail*00\n" + ACK_LINE)
    assert out == ["A 7 OK"]
    assert reader.overflows == 1, "one overlong line = one overflow, however many chunks it came in"
    assert reader.bad_lines == 0, "the discarded tail of an overlong line is not a separate bad line"


def test_a_valid_looking_line_after_overflow_on_the_same_line_is_not_accepted(impl):
    # 200 bytes of noise then "A 7 OK*72\n": the newline ends ONE overlong line, which is garbage.
    reader = impl.LineReader()
    assert reader.feed(b"n" * 200 + ACK_LINE) == []
    assert reader.feed(ACK_LINE) == ["A 7 OK"]


def test_max_length_line_is_accepted(impl):
    payload = "T " + "9" * 115  # 117 + "*hh" = 120 characters: the limit, inclusive
    line = impl.frame(payload)
    assert len(line) == 121
    reader = impl.LineReader(max_line_length=120)
    assert reader.feed(line) == [payload]
    assert reader.feed(line[:-1] + b"\r\n") == [payload]
    assert reader.feed(b"1" + line) == [] and reader.overflows + reader.bad_lines == 1


def test_fuzz_random_chunking_with_noise_lines(impl):
    rng = random.Random(3)
    messages = [protocol.Telemetry(ms, ms % 97, -ms, 5000, -5000, 11800, 420, 8) for ms in range(0, 4000, 20)]
    stream = bytearray()
    expected = []
    noise_lines = 0
    for message in messages:
        if rng.random() < 0.3:  # a corrupted line: random printable noise, then newline
            stream += bytes(rng.randrange(32, 127) for _ in range(rng.randrange(1, 60))) + b"\n"
            noise_lines += 1
        line = bytearray(protocol.encode_bytes(message))
        if rng.random() < 0.1:  # flip one bit in the payload: must be rejected
            i = rng.randrange(0, len(line) - 4)
            line[i] ^= 1 << rng.randrange(0, 7)
            if line[i] in (0x0A, 0x2A):  # don't create a newline or '*' (a different failure mode)
                line[i] ^= 0x01
        if protocol.checksum(bytes(line[:-4]).decode("latin-1")) == int(line[-3:-1], 16) and all(
            32 <= b <= 126 and b != 42 for b in line[:-4]
        ):
            expected.append(message.payload())
        stream += line
    reader = impl.LineReader()
    out: list[str] = []
    position = 0
    while position < len(stream):
        size = rng.randrange(1, 64)
        out += reader.feed(bytes(stream[position:position + size]))
        position += size
    assert out == expected
    assert reader.bad_lines >= noise_lines
