"""Reference solution for 03.03 — A line-framing parser that survives a real serial link.

Don't read this until you have made an honest attempt at ``student.py``.
"""

from __future__ import annotations

MAX_LINE_LENGTH = 120
HEX_DIGITS = b"0123456789abcdefABCDEF"


def xor_checksum(payload: bytes) -> int:
    """XOR of all payload bytes (0..255) — protocol v1's NMEA-style checksum."""
    value = 0
    for byte in payload:
        value ^= byte
    return value


def crc16_ccitt_false(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (a.k.a. CRC-16/IBM-3740): poly 0x1021, init 0xFFFF, no reflection."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def frame(payload: str) -> bytes:
    """``"S 9"`` -> ``b"S 9*4A\\n"``: payload, ``*``, two uppercase hex digits, newline."""
    raw = payload.encode("ascii")
    if not raw or b"*" in raw or b"\n" in raw or b"\r" in raw:
        raise ValueError(f"payload {payload!r} must be non-empty and contain no '*', CR or LF")
    return raw + b"*" + f"{xor_checksum(raw):02X}".encode("ascii") + b"\n"


class LineReader:
    """Turn an arbitrary stream of byte chunks into verified payloads.

    ``feed`` may be called with any split of the stream: half a line, many lines, garbage.
    Counters: ``bad_lines`` (complete lines that failed framing/checksum) and ``overflows``
    (lines longer than ``max_line_length`` that were discarded).
    """

    def __init__(self, max_line_length: int = MAX_LINE_LENGTH) -> None:
        self.max_line_length = max_line_length
        self.bad_lines = 0
        self.overflows = 0
        self._buffer = bytearray()
        self._discarding = False

    def feed(self, data: bytes) -> list[str]:
        payloads: list[str] = []
        start = 0
        while True:
            newline = data.find(b"\n", start)
            if newline < 0:
                self._append(data[start:])
                return payloads
            self._append(data[start:newline])
            if self._discarding:
                self._discarding = False  # the newline ends the overlong line: resynchronized
            else:
                payload = self._check(bytes(self._buffer))
                if payload is not None:
                    payloads.append(payload)
            self._buffer.clear()
            start = newline + 1

    def _append(self, chunk: bytes) -> None:
        if self._discarding or not chunk:
            return
        self._buffer += chunk
        # A line may end with '\r' before the '\n', so allow one extra byte.
        if len(self._buffer) > self.max_line_length + 1:
            self._buffer.clear()
            self._discarding = True
            self.overflows += 1

    def _check(self, line: bytes) -> str | None:
        if line.endswith(b"\r"):
            line = line[:-1]
        if not line:
            return None  # empty line: ignore silently
        if len(line) > self.max_line_length or len(line) < 4 or line[-3:-2] != b"*":
            self.bad_lines += 1
            return None
        payload, digits = line[:-3], line[-2:]
        if any(b < 0x20 or b > 0x7E or b == 0x2A for b in payload) or any(d not in HEX_DIGITS for d in digits):
            self.bad_lines += 1
            return None
        if int(digits, 16) != xor_checksum(payload):
            self.bad_lines += 1
            return None
        return payload.decode("ascii")
