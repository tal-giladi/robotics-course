"""COBS framing + CRC-16 — the upgrade path from protocol v1's text lines to a binary protocol.

Lesson 03.03. Not used by the course robot (protocol v1 stays human-readable); this is what you
would build when you need binary payloads (IMU samples, images, float arrays) or a stronger check.

Frame on the wire:   COBS( payload + CRC-16/CCITT-FALSE(payload), big-endian ) + one 0x00 byte

COBS (Consistent Overhead Byte Stuffing, Cheshire & Baker 1999) removes every 0x00 byte from the
data, so 0x00 can mark the end of a frame unambiguously: a receiver that joins mid-stream
resynchronizes at the next zero. Overhead: at most 1 byte per 254 bytes, plus the delimiter.

    >>> cobs_encode(bytes([0x11, 0x22, 0x00, 0x33])).hex(" ")
    '03 11 22 02 33'
    >>> cobs_decode(bytes.fromhex("0311220233")).hex(" ")
    '11 22 00 33'
"""

from __future__ import annotations

import struct


DELIMITER = bytes([0])


class CobsError(ValueError):
    """Malformed COBS data or a frame whose CRC does not match."""


def cobs_encode(data: bytes) -> bytes:
    """Encode ``data`` so the result contains no 0x00 byte (the delimiter is NOT appended)."""
    out = bytearray(b"\x00")  # placeholder for the first code byte
    code_index, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_index] = code
            code_index, code = len(out), 1
            out.append(0)  # placeholder for the next code byte
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:  # 254 non-zero bytes: close this block, start a new one
                out[code_index] = code
                code_index, code = len(out), 1
                out.append(0)
    out[code_index] = code
    return bytes(out)


def cobs_decode(encoded: bytes) -> bytes:
    """Inverse of :func:`cobs_encode` (input without the 0x00 delimiter)."""
    out = bytearray()
    i, n = 0, len(encoded)
    while i < n:
        code = encoded[i]
        if code == 0:
            raise CobsError(f"zero byte inside COBS data at index {i}")
        if i + code > n:
            raise CobsError("truncated COBS block")
        block = encoded[i + 1:i + code]
        if 0 in block:
            raise CobsError("zero byte inside a COBS block")
        out += block
        i += code
        if code != 0xFF and i < n:
            out.append(0)  # every block shorter than 254 bytes was followed by a zero
    return bytes(out)


def crc16_ccitt_false(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, not reflected. check(b"123456789") = 0x29B1."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(payload: bytes) -> bytes:
    """payload -> COBS(payload + CRC16) + 0x00, ready for ``serial.write``."""
    return cobs_encode(payload + struct.pack(">H", crc16_ccitt_false(payload))) + b"\x00"


def decode_frame(frame: bytes) -> bytes:
    """One frame WITHOUT its trailing 0x00 -> payload. Raises CobsError on bad COBS or CRC."""
    raw = cobs_decode(frame)
    if len(raw) < 2:
        raise CobsError("frame shorter than its CRC")
    payload, (received,) = raw[:-2], struct.unpack(">H", raw[-2:])
    if crc16_ccitt_false(payload) != received:
        raise CobsError("CRC mismatch")
    return payload


class FrameReader:
    """Byte chunks in, verified payloads out — the binary twin of a line reader."""

    def __init__(self, max_frame: int = 512) -> None:
        self.max_frame = max_frame
        self.bad_frames = 0
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[bytes]:
        payloads = []
        *complete, rest = data.split(DELIMITER)  # every part except the last ended with a delimiter
        for part in complete:
            self._buffer += part
            if self._buffer and len(self._buffer) <= self.max_frame:
                try:
                    payloads.append(decode_frame(bytes(self._buffer)))
                except CobsError:
                    self.bad_frames += 1
            elif self._buffer:
                self.bad_frames += 1
            self._buffer.clear()
        self._buffer += rest
        if len(self._buffer) > self.max_frame:  # bounded memory: drop it, the zero will resync us
            self._buffer.clear()
            self.bad_frames += 1
        return payloads
