# FC.05 - the same pulse-counting job, done by PIO instead of by the CPU.
#
# Run on a Pico 2 with MicroPython:  mpremote run fc05_pio_counter.py
# Wiring: the same single jumper as fc05_isr_lab.py, GP16 -> GP17.
#
# Two state machines:
#   sm_gen   generates a square wave on GP16 at a frequency we choose - no CPU involvement
#   sm_count counts rising edges on GP17 and reports the count on demand - no CPU per edge
#
# The point: the CPU sets both up, then does nothing at all while millions of edges go by.
# This is the mechanism behind labs/firmware/pico/encoders.py.

import time

import rp2
from machine import Pin

GEN_PIN = 16
COUNT_PIN = 17


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def square_wave():
    """One high period and one low period per loop. Each instruction is one PIO clock,
    so the output frequency is sm_freq / 4 (two `set` plus two `nop` cycles)."""
    set(pins, 1)    [0]
    nop()           [0]
    set(pins, 0)    [0]
    nop()           [0]


@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT)
def edge_counter():
    """Count rising edges in the Y register; push the count whenever the CPU asks.

    `wait(1, pin, 0)` blocks until the pin is high, `wait(0, pin, 0)` until it is low,
    so one loop = one complete rising-then-falling cycle. PIO cannot add, only decrement,
    so we count DOWN from 0 and negate on the Python side (the same trick, inverted, as
    encoders.py)."""
    wrap_target()
    wait(1, pin, 0)
    wait(0, pin, 0)
    jmp(y_dec, "publish")       # y = y - 1, always taken except when y is 0 (then it also decrements)
    label("publish")
    mov(isr, y)
    push(noblock)               # if the FIFO is full the value is dropped; we only want the newest
    wrap()


def read_count(sm):
    """Drain the stale values, take the freshest one, and negate it back to a positive count."""
    latest = 0
    while sm.rx_fifo():
        latest = sm.get()
    return (-latest) & 0xFFFFFFFF


# State machines 0..3 are in PIO block 0; karmel's encoders use 4 and 5 in block 1, so this
# program does not disturb them even if the firmware is running.
sm_count = rp2.StateMachine(0, edge_counter, in_base=Pin(COUNT_PIN, Pin.IN, Pin.PULL_DOWN))
sm_count.active(1)

print("%12s %14s %14s %10s" % ("target Hz", "expected edges", "counted edges", "error %"))
for target_hz in (1_000, 10_000, 100_000, 1_000_000):
    sm_gen = rp2.StateMachine(1, square_wave, freq=target_hz * 4, set_base=Pin(GEN_PIN))
    sm_count.active(0)
    sm_count.exec("set(y, 0)")       # zero the counter register without rebuilding the program
    sm_count.active(1)
    t0 = time.ticks_us()
    sm_gen.active(1)
    time.sleep_ms(200)
    sm_gen.active(0)
    elapsed_us = time.ticks_diff(time.ticks_us(), t0)
    counted = read_count(sm_count)
    sm_gen.active(0)
    expected = target_hz * elapsed_us // 1_000_000
    error = 100.0 * (counted - expected) / max(expected, 1)
    print("%12d %14d %14d %10.2f" % (target_hz, expected, counted, error))

sm_count.active(0)
Pin(GEN_PIN, Pin.IN)
print("\nThe CPU executed about ten Python statements during all of that.")
print("An interrupt handler at 1 MHz would need one edge every microsecond;")
print("a MicroPython hard IRQ needs tens of microseconds just to enter and leave.")
