"""03.03 — A line-framing parser that survives a real serial link.

Fill in every ``TODO(student)``. Run the checker with ``python course.py check 03.03``.
Only the Python standard library is needed.

Wire format (protocol v1, ``labs/README.md``)::

    <payload>*<hh>\\n      hh = XOR of all payload bytes, two hex digits (emit uppercase, accept both)

The payload is printable ASCII (0x20..0x7E) without ``*``. A line may end in ``\\r\\n``.
"""

from __future__ import annotations

MAX_LINE_LENGTH = 120
"""Longest line accepted, counting payload + ``*hh`` but not the line ending."""


def xor_checksum(payload: bytes) -> int:
    """XOR of all payload bytes, 0..255. ``b""`` -> 0."""
    # TODO(student): one loop, one operator.
    raise NotImplementedError("xor_checksum")


def crc16_ccitt_false(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (CRC-16/IBM-3740): poly 0x1021, init 0xFFFF, MSB first, no final XOR.

    Bitwise algorithm, for each byte:
        crc ^= byte << 8
        repeat 8 times: if the top bit (0x8000) is set, crc = (crc << 1) ^ 0x1021, else crc <<= 1
        keep crc to 16 bits (& 0xFFFF)
    Check value: crc16_ccitt_false(b"123456789") == 0x29B1.
    """
    # TODO(student): implement the bitwise CRC above.
    raise NotImplementedError("crc16_ccitt_false")


def frame(payload: str) -> bytes:
    """``"S 9"`` -> ``b"S 9*4A\\n"``.

    Raise ``ValueError`` for an empty payload or one containing ``*``, ``\\r`` or ``\\n`` — any of
    those would make the receiver cut the line in the wrong place.
    """
    # TODO(student): validate, then payload + b"*" + two UPPERCASE hex digits + b"\n".
    raise NotImplementedError("frame")


class LineReader:
    """Turn an arbitrary stream of byte chunks into verified payload strings.

    ``feed(chunk)`` may receive any split of the stream: half a line, several lines, garbage.
    It returns the payloads (without ``*hh`` and line ending) of every *complete, valid* line that
    the chunk finished, in order.

    Rules:
    * A line ends at ``\\n``; strip one ``\\r`` before it. Empty lines are ignored (not counted).
    * A complete line that is not ``<printable payload without *>*<2 hex digits>`` with a matching
      XOR checksum, or is longer than ``max_line_length``, is dropped and ``bad_lines += 1``.
    * Memory must stay bounded: if the bytes buffered for the current line exceed
      ``max_line_length + 1`` (the +1 allows a ``\\r``) *before* a newline arrives, discard them,
      ``overflows += 1``, and ignore everything up to and including the next ``\\n``.
      That whole overlong line counts as one overflow and not as a bad line.
    """

    def __init__(self, max_line_length: int = MAX_LINE_LENGTH) -> None:
        self.max_line_length = max_line_length
        self.bad_lines = 0
        self.overflows = 0
        # TODO(student): the state you need between feed() calls (a buffer, a "discarding" flag).

    def feed(self, data: bytes) -> list[str]:
        # TODO(student):
        #   1. find each b"\n" in data; bytes before it complete the buffered line
        #   2. check the complete line (unless you were discarding an overlong one)
        #   3. keep the bytes after the last newline for the next call, respecting the overflow rule
        raise NotImplementedError("LineReader.feed")
