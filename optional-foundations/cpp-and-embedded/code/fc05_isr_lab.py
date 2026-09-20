# FC.05 - what you may and may not do inside an interrupt handler, measured on the board.
#
# Run on a Pico 2 with MicroPython:  mpremote run fc05_isr_lab.py
# It uses GPIO 16 and GPIO 17 (free on karmel: see labs/firmware/pico/config.py) wired to each
# other with one jumper wire, so the program can generate its own edges. Nothing else is touched:
# the motor pins are never configured.
#
#     GP16 (output, "signal generator")  ------ jumper ------  GP17 (input, interrupt source)
#
# Four experiments:
#   1. a soft IRQ vs a hard IRQ: which one is allowed to allocate memory
#   2. how long the handler itself takes
#   3. a shared counter read from the main loop: why you copy it in one statement
#   4. how many edges per second the handler can survive before it starts losing them

import gc
import time

import machine
import micropython
from machine import Pin

micropython.alloc_emergency_exception_buf(100)   # so a crash inside a hard IRQ can print a traceback

OUT_PIN = 16
IN_PIN = 17

pulse = Pin(OUT_PIN, Pin.OUT, value=0)
signal = Pin(IN_PIN, Pin.IN, Pin.PULL_DOWN)

# --- state shared between the handler and the main loop ------------------------------------------
# Small ints only: incrementing a small int allocates nothing (FC.02). A float or a list here
# would allocate inside the handler and eventually raise MemoryError in a hard IRQ.
count = 0
last_us = 0
worst_gap_us = 0


def on_edge_hard(_pin):
    """A hard IRQ: runs immediately, inside the interrupt, and MUST NOT allocate."""
    global count, last_us, worst_gap_us
    now = time.ticks_us()
    gap = time.ticks_diff(now, last_us)
    if gap > worst_gap_us:
        worst_gap_us = gap
    last_us = now
    count += 1


def on_edge_allocating(_pin):
    """The same handler with one innocent-looking line that allocates a float and a string."""
    global count
    count += 1
    _ = "edge %d at %.3f" % (count, time.ticks_us() / 1000.0)   # <- allocates


def burst(n, delay_us=0):
    """Generate n rising edges on OUT_PIN."""
    for _ in range(n):
        pulse.on()
        if delay_us:
            time.sleep_us(delay_us)
        pulse.off()
        if delay_us:
            time.sleep_us(delay_us)


def reset_counters():
    global count, last_us, worst_gap_us
    count = 0
    worst_gap_us = 0
    last_us = time.ticks_us()


print("1. soft IRQ with an allocating handler")
gc.collect()
signal.irq(on_edge_allocating, Pin.IRQ_RISING, hard=False)
reset_counters()
burst(200, delay_us=200)
time.sleep_ms(50)
signal.irq(None)
print("   soft + allocating: %d of 200 edges counted (allocation is legal here)" % count)

print("2. hard IRQ with the same allocating handler")
gc.collect()
signal.irq(on_edge_allocating, Pin.IRQ_RISING, hard=True)
reset_counters()
burst(200, delay_us=200)
time.sleep_ms(50)
signal.irq(None)
print("   hard + allocating: %d of 200 edges counted" % count)
print("   (watch the output above: a MemoryError traceback from the handler means the heap")
print("    had to be touched while the interrupt was running - forbidden in a hard IRQ)")

print("3. hard IRQ, allocation-free handler")
gc.collect()
signal.irq(on_edge_hard, Pin.IRQ_RISING, hard=True)
reset_counters()
burst(200, delay_us=200)
time.sleep_ms(50)
signal.irq(None)
print("   hard, no allocation: %d of 200 edges, worst gap %d us" % (count, worst_gap_us))

print("4. how fast can it go before edges are lost?")
signal.irq(on_edge_hard, Pin.IRQ_RISING, hard=True)
for delay_us in (200, 50, 20, 10, 5, 0):
    gc.collect()
    reset_counters()
    t0 = time.ticks_us()
    burst(1000, delay_us=delay_us)
    elapsed_us = time.ticks_diff(time.ticks_us(), t0)
    time.sleep_ms(20)
    rate = 1000 * 1_000_000 // max(elapsed_us, 1)
    print("   delay %3d us -> %6d edges/s generated, %4d of 1000 counted (%.1f %% lost)"
          % (delay_us, rate, count, 100.0 * (1000 - count) / 1000.0))
signal.irq(None)

print("\nkarmel at full speed: about 8,000 edges/s per wheel, 16,000 for both.")
print("Compare that with the last row above - and see labs/firmware/pico/encoders.py,")
print("which is why the encoders are counted by PIO and not by interrupts.")
pulse.off()
