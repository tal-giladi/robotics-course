"""Checker for FC.05 — interrupt-safe counting.

Run: ``python course.py check FC.05`` (or ``--solution`` to see the reference pass).
"""

from __future__ import annotations

import pytest

# karmel: 11 hall pulses per motor turn x 4 edges x 56:1 gearbox (labs/firmware/pico/config.py)
TICKS_PER_WHEEL_REV = 2464


def forward_samples(steps: int) -> list[tuple[int, int]]:
    """(a, b) samples walking the quadrature cycle forward: 00 -> 10 -> 11 -> 01 -> 00."""
    cycle = [(0, 0), (1, 0), (1, 1), (0, 1)]
    return [cycle[i % 4] for i in range(steps + 1)]


# --- quadrature ------------------------------------------------------------------------------------
def test_first_sample_only_establishes_the_state(impl):
    decoder = impl.QuadratureDecoder()
    assert decoder.update(0, 0) == 0
    assert decoder.count == 0


def test_forward_cycle_counts_up(impl):
    decoder = impl.QuadratureDecoder()
    steps = [decoder.update(a, b) for a, b in forward_samples(8)]
    assert steps[0] == 0
    assert steps[1:] == [1] * 8
    assert decoder.count == 8
    assert decoder.missed == 0


def test_backward_cycle_counts_down(impl):
    decoder = impl.QuadratureDecoder()
    samples = list(reversed(forward_samples(8)))
    for a, b in samples:
        decoder.update(a, b)
    assert decoder.count == -8


def test_invert_flips_the_sign(impl):
    decoder = impl.QuadratureDecoder(invert=True)
    for a, b in forward_samples(4):
        decoder.update(a, b)
    assert decoder.count == -4


def test_one_wheel_revolution(impl):
    decoder = impl.QuadratureDecoder()
    for a, b in forward_samples(TICKS_PER_WHEEL_REV):
        decoder.update(a, b)
    assert decoder.count == TICKS_PER_WHEEL_REV


def test_a_two_step_jump_is_reported_as_missed_and_counts_nothing(impl):
    decoder = impl.QuadratureDecoder()
    decoder.update(0, 0)          # state 00
    assert decoder.update(1, 1) == 0   # 00 -> 11: impossible in one sample
    assert decoder.missed == 1
    assert decoder.count == 0


def test_no_change_is_not_a_missed_edge(impl):
    decoder = impl.QuadratureDecoder()
    decoder.update(0, 0)
    decoder.update(0, 0)
    decoder.update(0, 0)
    assert decoder.missed == 0
    assert decoder.count == 0


def test_reset_keeps_the_state_but_zeroes_the_count(impl):
    decoder = impl.QuadratureDecoder()
    for a, b in forward_samples(4):
        decoder.update(a, b)
    decoder.reset()
    assert decoder.count == 0 and decoder.missed == 0
    assert decoder.update(1, 0) == 1, "the decoder must remember it was at state 00"


# --- ticks_diff ------------------------------------------------------------------------------------
@pytest.mark.parametrize(
    ("new", "old", "expected"),
    [
        (100, 50, 50),
        (50, 100, -50),
        (200, 1073741000, 1024),          # the wrap example from FC.02
        (0, 0, 0),
        (1073741823, 1073741820, 3),
    ],
)
def test_ticks_diff(impl, new, old, expected):
    assert impl.ticks_diff(new, old) == expected


# --- debouncer -------------------------------------------------------------------------------------
def test_a_clean_press_is_accepted_after_stable_ms(impl):
    d = impl.Debouncer(stable_ms=20, initial=0)
    assert d.update(0, 0) == 0
    assert d.update(5, 1) == 0, "too early: the level has been high for 0 ms"
    assert d.update(20, 1) == 0, "still 15 ms of high"
    assert d.update(25, 1) == 1, "high for 20 ms -> accepted"
    assert d.transitions == 1


def test_bounce_shorter_than_stable_ms_is_rejected(impl):
    d = impl.Debouncer(stable_ms=20, initial=0)
    for t, raw in [(0, 0), (2, 1), (4, 0), (6, 1), (9, 0), (12, 1), (15, 0)]:
        assert d.update(t, raw) == 0
    assert d.transitions == 0


def test_release_is_debounced_too(impl):
    d = impl.Debouncer(stable_ms=20, initial=0)
    d.update(0, 1)
    d.update(30, 1)
    assert d.level == 1
    d.update(100, 0)
    assert d.update(115, 0) == 1, "not stable long enough yet"
    assert d.update(125, 0) == 0
    assert d.transitions == 2


def test_debouncer_survives_the_tick_wrap(impl):
    d = impl.Debouncer(stable_ms=20, initial=0)
    d.update(1073741800, 1)
    assert d.update(1073741820, 1) == 1, "20 ms later across the 2**30 wrap"


# --- ISR saturation ---------------------------------------------------------------------------------
def evenly_spaced(n: int, period_us: float) -> list[float]:
    return [i * period_us for i in range(n)]


def test_a_slow_signal_loses_nothing(impl):
    result = impl.simulate_isr(evenly_spaced(1000, 1000.0), handler_us=25.0)
    assert result.counted == 1000
    assert result.lost == 0
    assert result.busy_us == pytest.approx(25000.0)


def test_karmel_full_speed_with_a_fast_handler(impl):
    # 16,000 edges/s (both wheels) = one edge every 62.5 us; a 25 us handler keeps up.
    result = impl.simulate_isr(evenly_spaced(2000, 62.5), handler_us=25.0)
    assert result.lost == 0
    assert result.counted == 2000


def test_a_slow_handler_starts_losing_edges(impl):
    # The same signal with a 70 us MicroPython handler: it cannot finish before the next edge.
    result = impl.simulate_isr(evenly_spaced(2000, 62.5), handler_us=70.0)
    assert result.lost > 0
    assert result.counted < 2000
    assert result.counted + result.lost == 2000


def test_the_pending_flag_saves_exactly_one_edge(impl):
    # Two edges arrive while the handler runs: the first is latched, the second is lost.
    result = impl.simulate_isr([0.0, 10.0, 20.0], handler_us=100.0)
    assert result.counted == 2
    assert result.lost == 1


def test_throughput_saturates_at_one_edge_per_handler(impl):
    # Far too fast: the handler runs back to back and nothing else gets through.
    edges = evenly_spaced(1000, 1.0)
    result = impl.simulate_isr(edges, handler_us=50.0)
    assert result.counted + result.lost == 1000
    # 1000 edges arrive in 999 us; the handler can serve at most ceil(999/50)+2 of them.
    assert result.counted <= 999 / 50 + 2


def test_rate_and_load_helpers(impl):
    assert impl.max_sustainable_rate_hz(25.0) == pytest.approx(40_000.0)
    assert impl.max_sustainable_rate_hz(70.0) == pytest.approx(14_285.7, rel=1e-4)
    assert impl.cpu_load_fraction(16_000, 25.0) == pytest.approx(0.40)
    assert impl.cpu_load_fraction(16_000, 70.0) == pytest.approx(1.12)
