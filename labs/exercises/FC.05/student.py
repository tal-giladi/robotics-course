"""FC.05 — interrupt-safe counting: quadrature decoding, debouncing, and a saturated ISR.

Fill in every ``TODO(student)``. Check your work with ``python course.py check FC.05``.
Standard library only. Everything here is pure logic, so it runs on your PC, in MicroPython on
the Pico, and in the tests — the same code that would live in an interrupt handler.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Index = (previous_state << 2) | new_state, where a state is (B << 1) | A.
# +1 forward, -1 backward, 0 = no change or an impossible two-step jump (an edge was missed).
# Identical to TRANSITION_TABLE in labs/firmware/pico/encoders.py.
TRANSITION_TABLE = (
    0, +1, -1, 0,     # previous 00
    -1, 0, 0, +1,     # previous 01
    +1, 0, 0, -1,     # previous 10
    0, -1, +1, 0,     # previous 11
)


class QuadratureDecoder:
    """Decode a quadrature signal one sample at a time.

    ``update(a, b)`` takes the two pin levels (0 or 1) and returns the step for this sample:
    ``+1``, ``-1`` or ``0``. ``count`` accumulates the steps. The first ``update`` after
    construction establishes the starting state and must return ``0``.

    ``missed`` counts samples where the state jumped by two positions at once (table entry 0 with
    a *different* previous and new state) — the signature of an edge that arrived while you were
    not looking.
    """

    def __init__(self, invert: bool = False) -> None:
        # TODO(student): remember `invert`, set count = 0, missed = 0, and mark that no previous
        #   state has been seen yet.
        raise NotImplementedError("QuadratureDecoder.__init__")

    def update(self, a: int, b: int) -> int:
        # TODO(student): build the 2-bit state (b << 1) | a, look the transition up in
        #   TRANSITION_TABLE, update count and missed, remember the new state, and return the
        #   step (negated when `invert` is set).
        raise NotImplementedError("QuadratureDecoder.update")

    def reset(self) -> None:
        # TODO(student): make count and missed 0 again, keeping the current state.
        raise NotImplementedError("QuadratureDecoder.reset")


class Debouncer:
    """Filter contact bounce from a mechanical switch (a bumper, a limit switch, an e-stop).

    A raw level is accepted only after it has held steady for ``stable_ms``.

        d = Debouncer(stable_ms=20, initial=0)
        d.update(now_ms, raw_level)   -> the debounced level (0 or 1)

    Rules:
      * ``update`` returns the current *debounced* level every time it is called.
      * When the raw level differs from the debounced level, start (or continue) timing. Once it
        has differed for at least ``stable_ms``, the debounced level changes and ``transitions``
        increments.
      * If the raw level goes back to the debounced level before ``stable_ms`` has passed, the
        candidate is forgotten: bounce is rejected, ``transitions`` does not move.
      * Times are millisecond integers that may wrap (``time.ticks_ms``): compare them only
        through :func:`ticks_diff` below, never with ``a - b``.
    """

    def __init__(self, stable_ms: int, initial: int = 0) -> None:
        # TODO(student)
        raise NotImplementedError("Debouncer.__init__")

    def update(self, now_ms: int, raw: int) -> int:
        # TODO(student)
        raise NotImplementedError("Debouncer.update")


TICKS_PERIOD = 1 << 30          # the rp2 port's ticks_ms wraps at 2**30
TICKS_HALF = TICKS_PERIOD // 2


def ticks_diff(new_ms: int, old_ms: int) -> int:
    """``time.ticks_diff`` for the rp2 port: the signed distance from ``old_ms`` to ``new_ms``.

    The counter wraps at 2**30, so the result is brought into [-2**29, 2**29).
    ``ticks_diff(200, 1073741000)`` must be ``1024``, not ``-1073740800``.
    """
    # TODO(student)
    raise NotImplementedError("ticks_diff")


@dataclass
class IsrResult:
    counted: int = 0
    lost: int = 0
    busy_us: float = 0.0
    events: list[str] = field(default_factory=list)


def simulate_isr(edge_times_us: list[float], handler_us: float) -> IsrResult:
    """Model a pin-change interrupt handler that takes ``handler_us`` to run.

    The hardware has **one** pending flag per pin: an edge that arrives while the handler is
    running sets the flag and is serviced as soon as the handler returns. A *second* edge while
    the flag is already set is lost — the flag is already 1 and nothing records the extra edge.

    Process the edges in the order given (they are already sorted) with this state machine:

        busy_until = -inf ; pending = False ; counted = 0 ; lost = 0
        for t in edge_times_us:
            while pending and busy_until <= t:      # the handler finished; service the flag
                counted += 1 ; busy_until += handler_us ; pending = False
            if t >= busy_until:                     # the CPU is free: service immediately
                counted += 1 ; busy_until = t + handler_us
            elif not pending:                       # busy, flag free: latch it
                pending = True
            else:                                   # busy and the flag is already set
                lost += 1
        if pending:                                 # flush the last latched edge
            counted += 1 ; busy_until += handler_us

    ``busy_us`` is the total time spent inside the handler: ``counted * handler_us``.
    ``events`` is left empty (it exists so you can add your own debug notes).
    """
    # TODO(student)
    raise NotImplementedError("simulate_isr")


def max_sustainable_rate_hz(handler_us: float) -> float:
    """The highest steady edge rate this handler can keep up with, in edges per second.

    One handler run per edge, so the answer is ``1e6 / handler_us``.
    """
    # TODO(student)
    raise NotImplementedError("max_sustainable_rate_hz")


def cpu_load_fraction(edge_rate_hz: float, handler_us: float) -> float:
    """Fraction of the CPU spent in the handler at this edge rate (1.0 = nothing else runs).

    ``edge_rate_hz * handler_us / 1e6``. It may exceed 1.0 — that is the point: karmel's two
    wheels make about 16,000 edges/s at full speed, and a 70 us MicroPython handler would need
    112 % of the CPU.
    """
    # TODO(student)
    raise NotImplementedError("cpu_load_fraction")
