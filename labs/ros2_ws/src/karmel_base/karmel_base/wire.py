"""Serial-stream helpers around protocol v1 (pure Python, no ROS).

``karmel_base/protocol.py`` is a byte-identical copy of ``labs/python/robotlab/protocol.py`` (the
canonical implementation; ``test/test_protocol.py`` checks the copy). Anything the ROS driver
needs on top of it lives here, so the copy never has to be edited.
"""
from __future__ import annotations

from karmel_base import protocol as p


class LineBuffer:
    """Accumulates bytes from a serial port and returns complete lines.

    Serial reads return arbitrary chunks ("T 12 3", "4 5 ...*1A\\nT 13"...). Feed every chunk in and
    handle the complete lines that come back. Over-long garbage without a newline is discarded.
    """

    def __init__(self, max_length: int = p.MAX_LINE_LENGTH + 2):
        self._buf = bytearray()
        self._max = max_length
        self.overflows = 0

    def feed(self, data: bytes) -> list[bytes]:
        self._buf.extend(data)
        lines = []
        while True:
            newline = self._buf.find(b'\n')
            if newline < 0:
                break
            lines.append(bytes(self._buf[:newline + 1]))
            del self._buf[:newline + 1]
        if len(self._buf) > self._max:
            self._buf.clear()
            self.overflows += 1
        return lines


def rad_s_to_mrad_s(value: float) -> int:
    """Wheel speed in rad/s -> integer milliradians/s for a V command (rounded)."""
    return int(round(value * 1000.0))


def next_seq(seq: int) -> int:
    """16-bit sequence counter: 0..SEQ_MAX, then wraps to 0."""
    return 0 if seq >= p.SEQ_MAX else seq + 1


def battery_v(t: p.Telemetry) -> float | None:
    return None if t.battery_mv < 0 else t.battery_mv / 1000.0


def range_m(t: p.Telemetry) -> float | None:
    return None if t.range_mm < 0 else t.range_mm / 1000.0


def has_flag(t: p.Telemetry, flag: int) -> bool:
    return bool(t.flags & flag)
