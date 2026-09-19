"""CPython stand-in for MicroPython's ``time``/``utime`` tick functions (tests only).

The clock is FAKE and only moves when a test calls :func:`set_ms` or :func:`advance_ms`, so
timing logic can be tested exactly. Like on the RP2 port, tick values wrap around at 2**30 ms;
``ticks_diff`` and ``ticks_add`` handle the wrap like MicroPython does.
"""

from __future__ import annotations

TICKS_PERIOD = 1 << 30
_TICKS_MAX = TICKS_PERIOD - 1
_TICKS_HALFPERIOD = TICKS_PERIOD // 2

_now_ms = 0


def set_ms(value: int) -> None:
    global _now_ms
    _now_ms = value & _TICKS_MAX


def advance_ms(delta: int) -> None:
    set_ms(_now_ms + delta)


def ticks_ms() -> int:
    return _now_ms


def ticks_us() -> int:
    return (_now_ms * 1000) & _TICKS_MAX


def ticks_add(ticks: int, delta: int) -> int:
    return (ticks + delta) & _TICKS_MAX


def ticks_diff(ticks1: int, ticks2: int) -> int:
    """Signed difference ticks1 - ticks2, correct across one wrap-around (MicroPython semantics)."""
    return ((ticks1 - ticks2 + _TICKS_HALFPERIOD) & _TICKS_MAX) - _TICKS_HALFPERIOD


def sleep_ms(ms: int) -> None:
    advance_ms(ms)


def sleep_us(us: int) -> None:
    advance_ms(us // 1000)


def sleep(seconds: float) -> None:
    advance_ms(int(seconds * 1000))
