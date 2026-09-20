"""FC.05 — reference solution: quadrature decoding, debouncing, and a saturated ISR."""

from __future__ import annotations

from dataclasses import dataclass, field

TRANSITION_TABLE = (
    0, +1, -1, 0,     # previous 00
    -1, 0, 0, +1,     # previous 01
    +1, 0, 0, -1,     # previous 10
    0, -1, +1, 0,     # previous 11
)


class QuadratureDecoder:
    """Decode a quadrature signal one sample at a time (see student.py for the contract)."""

    def __init__(self, invert: bool = False) -> None:
        self.invert = invert
        self.count = 0
        self.missed = 0
        self._state: int | None = None

    def update(self, a: int, b: int) -> int:
        state = ((b & 1) << 1) | (a & 1)
        if self._state is None:
            self._state = state
            return 0
        previous = self._state
        step = TRANSITION_TABLE[(previous << 2) | state]
        if step == 0 and previous != state:
            # A jump of two positions: 00 -> 11 or 01 -> 10. One edge was not observed.
            self.missed += 1
        self._state = state
        if self.invert:
            step = -step
        self.count += step
        return step

    def reset(self) -> None:
        self.count = 0
        self.missed = 0


class Debouncer:
    """Accept a new level only after it has held steady for stable_ms."""

    def __init__(self, stable_ms: int, initial: int = 0) -> None:
        self.stable_ms = stable_ms
        self.level = initial & 1
        self.transitions = 0
        self._candidate_since: int | None = None

    def update(self, now_ms: int, raw: int) -> int:
        raw &= 1
        if raw == self.level:
            self._candidate_since = None        # bounce rejected
            return self.level
        if self._candidate_since is None:
            self._candidate_since = now_ms
            return self.level
        if ticks_diff(now_ms, self._candidate_since) >= self.stable_ms:
            self.level = raw
            self.transitions += 1
            self._candidate_since = None
        return self.level


TICKS_PERIOD = 1 << 30
TICKS_HALF = TICKS_PERIOD // 2


def ticks_diff(new_ms: int, old_ms: int) -> int:
    """Signed distance from old_ms to new_ms on a counter that wraps at 2**30."""
    return ((new_ms - old_ms + TICKS_HALF) % TICKS_PERIOD) - TICKS_HALF


@dataclass
class IsrResult:
    counted: int = 0
    lost: int = 0
    busy_us: float = 0.0
    events: list[str] = field(default_factory=list)


def simulate_isr(edge_times_us: list[float], handler_us: float) -> IsrResult:
    """Model a pin-change handler with one pending flag (see student.py for the state machine)."""
    result = IsrResult()
    busy_until = float("-inf")
    pending = False
    for t in edge_times_us:
        while pending and busy_until <= t:
            result.counted += 1
            busy_until += handler_us
            pending = False
        if t >= busy_until:
            result.counted += 1
            busy_until = t + handler_us
        elif not pending:
            pending = True
        else:
            result.lost += 1
    if pending:
        result.counted += 1
        busy_until += handler_us
    result.busy_us = result.counted * handler_us
    return result


def max_sustainable_rate_hz(handler_us: float) -> float:
    return 1e6 / handler_us


def cpu_load_fraction(edge_rate_hz: float, handler_us: float) -> float:
    return edge_rate_hz * handler_us / 1e6
