# FC.05 — Interrupt-safe counting

Lesson: [FC.05 Interrupts, timers and the Pico's PIO](../../../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md)

Three pieces of logic that every interrupt handler in this course needs, written so they run
unchanged on your PC and on the Pico: decode a quadrature signal, debounce a mechanical contact,
and predict when an interrupt handler stops keeping up.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `QuadratureDecoder` | `update(a, b)` → `+1`, `-1` or `0`; accumulates `count`; counts two-step jumps as `missed` |
| `ticks_diff(new, old)` | `time.ticks_diff` for the rp2 port: signed distance on a counter that wraps at 2³⁰ |
| `Debouncer` | accepts a new level only after it has held for `stable_ms`; counts `transitions` |
| `simulate_isr(edges, handler_us)` | one pending flag per pin: how many edges are counted, how many are lost |
| `max_sustainable_rate_hz`, `cpu_load_fraction` | the two numbers that decide IRQ vs PIO |

## Check

```bash
python course.py check FC.05              # your code
python course.py check FC.05 --solution   # the reference
```

The tests walk a full wheel revolution (2464 ticks), feed the decoder an impossible 00 → 11 jump,
bounce a switch seven times in 15 ms, cross the 2³⁰ tick wrap, and run karmel's 16,000 edges/s
through a 25 µs handler (keeps up) and a 70 µs handler (does not).

## Hints

* The transition table is given at the top of `student.py`. A table entry of `0` with a
  *different* previous and new state is the "I missed an edge" case; `0` with the same state is
  just a sample where nothing changed.
* `ticks_diff` is three tokens: shift by half the period, take the modulus, shift back.
* In `Debouncer`, "the candidate is forgotten" means setting the pending timestamp back to `None`
  — not resetting it to `now_ms`.
* `simulate_isr`'s state machine is written out line by line in the docstring. Implement it
  exactly; the flush at the end matters.
