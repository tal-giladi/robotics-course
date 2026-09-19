# encoders.py - count quadrature encoder ticks: PIO (hardware) and IRQ (software) versions.
#
# Lesson 01.09 (reading encoders). Used by main.py and examples/03_encoder_count.py.
#
# What a quadrature encoder sends
# -------------------------------
# The Yahboom 520 motor has a magnet disc and two hall sensors, A and B, placed so their
# square waves are a quarter period apart:
#
#     A  __|‾‾‾‾|____|‾‾‾‾|____
#     B  ____|‾‾‾‾|____|‾‾‾‾|__        forward: the state (A,B) walks 00 -> 10 -> 11 -> 01 -> 00
#                                      backward: the same walk in reverse
#
# Every edge of either signal is one "tick": 11 pulses per motor turn x 4 edges x 56:1 gearbox
# = 2464 ticks per wheel turn. Which neighbour the new state is tells us the direction.
#
# Why PIO and not interrupts
# --------------------------
# At full speed (about 21 rad/s at the wheel) one wheel produces ~8,000 edges per second,
# two wheels ~16,000. A MicroPython interrupt handler takes tens of microseconds, so the CPU
# would spend most of its time in handlers, and edges that arrive while one is running are
# lost - the count silently drifts. (Note: MicroPython's machine.Encoder class does NOT
# exist on the rp2 port, only on ESP32 and i.MX RT.)
#
# The RP2350 has PIO: small state machines, separate from the CPU, that run a tiny program of
# at most 32 instructions at millions of steps per second. We give each encoder its own state
# machine. It watches the two pins and keeps the count in its Y register; the CPU just asks
# for the current count whenever it wants it. Nothing is lost, and the CPU does no work per
# edge.
#
# QuadratureEncoderIRQ below is the interrupt version, kept for teaching: it is easy to read,
# fine for slow hand-turned wheels, and you can see it lose ticks at speed (lesson 01.09
# exercise: spin a wheel at full duty and compare the two counts).
#
# Comparison
#   PIO: exact at any speed, zero CPU per edge, count read on demand; harder to read.
#   IRQ: plain Python, any pins; CPU load grows with speed, misses edges above a few kHz.

import rp2
from machine import Pin

import config

# --------------------------------------------------------------------------------------------
# The decoding rule, as a table. Index = (previous_state << 2) | new_state, where a state is
# the two pin levels read together as a 2-bit number (bit 1 = B, bit 0 = A, because PIO's
# `in_(pins, 2)` reads the base pin A into the lowest bit).
#   +1 = one step forward, -1 = one step backward, 0 = no change (or an impossible jump of two
#   steps, which only happens if we sampled too slowly - we ignore it).
# Both implementations below use exactly this table.
# --------------------------------------------------------------------------------------------
#                 new: 00  01  10  11
TRANSITION_TABLE = (0, +1, -1, 0,     # previous 00
                    -1, 0, 0, +1,     # previous 01
                    +1, 0, 0, -1,     # previous 10
                    0, -1, +1, 0)     # previous 11


def to_signed32(value):
    """PIO registers are 32-bit unsigned; turn 0xFFFFFFFF back into -1."""
    value &= 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


# --------------------------------------------------------------------------------------------
# PIO program
#
# Registers used:  Y   = the tick count
#                  OSR = the previous 2-bit pin state
#                  ISR = scratch
#
# The first 16 instructions are a jump table, one entry per row/column of TRANSITION_TABLE.
# `mov(pc, isr)` jumps to the entry whose address equals the 4-bit (previous, new) value.
# That jump uses ABSOLUTE addresses 0..15, so the program must be loaded at address 0 of the
# PIO block. MicroPython places a program at the END of the 32-instruction memory, so we pad
# the program to exactly 32 instructions: then address 0 is the only place it fits.
# Consequence: this PIO block (config: PIO block 1) must hold nothing else. Both encoders share
# the same copy of the program.
#
# Incrementing: PIO can only DEcrement (`jmp(y_dec, ...)`). To add one we flip all bits
# (~y = -y - 1), subtract one, and flip back: ~(~y - 1) = y + 1.
# --------------------------------------------------------------------------------------------
@rp2.asm_pio(in_shiftdir=rp2.PIO.SHIFT_LEFT, out_shiftdir=rp2.PIO.SHIFT_RIGHT)
def quadrature_program():
    # addresses 0..15: jump table (keep in sync with TRANSITION_TABLE)
    jmp("update")           # 0000  00 -> 00  no change
    jmp("increment")        # 0001  00 -> 01  +1
    jmp("decrement")        # 0010  00 -> 10  -1
    jmp("update")           # 0011  00 -> 11  invalid, ignore
    jmp("decrement")        # 0100  01 -> 00  -1
    jmp("update")           # 0101  01 -> 01  no change
    jmp("update")           # 0110  01 -> 10  invalid, ignore
    jmp("increment")        # 0111  01 -> 11  +1
    jmp("increment")        # 1000  10 -> 00  +1
    jmp("update")           # 1001  10 -> 01  invalid, ignore
    jmp("update")           # 1010  10 -> 10  no change
    jmp("decrement")        # 1011  10 -> 11  -1
    jmp("update")           # 1100  11 -> 00  invalid, ignore
    jmp("decrement")        # 1101  11 -> 01  -1
    jmp("increment")        # 1110  11 -> 10  +1
    jmp("update")           # 1111  11 -> 11  no change

    label("decrement")      # 16
    jmp(y_dec, "update")    # y = y - 1 (jumps only if y was not 0; either way y is decremented)
    jmp("update")           # y was 0 and is now -1: continue to update as well

    label("increment")      # 18
    mov(y, invert(y))       # y = ~y
    jmp(y_dec, "inc_done")  # y = y - 1 (always lands on the next line)
    label("inc_done")
    mov(y, invert(y))       # y = ~y  -> net effect y + 1

    label("update")         # 21
    mov(isr, y)             # publish the count:
    push(noblock)           #   into the RX FIFO; if the FIFO is full, this value is dropped
    out(isr, 2)             # ISR = previous state (lowest 2 bits of OSR)
    in_(pins, 2)            # ISR = (previous << 2) | new state   (reads pins A and B)
    mov(osr, isr)           # remember this 4-bit value; its low 2 bits are the next "previous"
    mov(pc, isr)            # jump into the table at address (previous << 2) | new

    # addresses 27..31: padding so the program is exactly 32 instructions (never executed)
    nop()
    nop()
    nop()
    nop()
    nop()


class QuadratureEncoderPIO:
    """Count ticks with a PIO state machine.

        enc = QuadratureEncoderPIO(sm_id=4, pin_a=10)   # pin B must be pin_a + 1
        enc.count()     # signed ticks since start (or since reset())
    """

    def __init__(self, sm_id, pin_a, invert=False):
        # The Yahboom hall encoder has its own pull-ups; power it from 3V3 so its outputs never
        # exceed 3.3 V. The internal pull-ups are a harmless extra. (Don't rely on internal
        # PULL-DOWNS on early RP2350 chips: erratum RP2350-E9, fixed in the A4 stepping.)
        Pin(pin_a, Pin.IN, Pin.PULL_UP)
        Pin(pin_a + 1, Pin.IN, Pin.PULL_UP)
        self.invert = invert
        self._offset = 0
        # State machines 4..7 belong to PIO block 1. freq=-1 (default) runs PIO at full speed.
        self._sm = rp2.StateMachine(sm_id, quadrature_program, in_base=Pin(pin_a))
        self._sm.active(1)

    def _raw_count(self):
        # The state machine pushes the count on every step, so the 4-entry FIFO is full of
        # slightly old values. Throw those away, then wait for a fresh one - it arrives within
        # microseconds because the program loops millions of times per second.
        for _ in range(self._sm.rx_fifo()):
            self._sm.get()
        return to_signed32(self._sm.get())

    def count(self):
        value = self._raw_count() - self._offset
        return -value if self.invert else value

    def reset(self):
        """Make count() return 0 from now on. We remember an offset instead of writing to the
        state machine's Y register, which could race with an increment in progress."""
        self._offset = self._raw_count()

    def deinit(self):
        self._sm.active(0)


class QuadratureEncoderIRQ:
    """Count ticks in pin-change interrupts - the teaching version. Loses ticks at speed."""

    def __init__(self, pin_a, pin_b, invert=False):
        self._pin_a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self._pin_b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.invert = invert
        self._count = 0
        self._state = self._read_state()
        trigger = Pin.IRQ_RISING | Pin.IRQ_FALLING
        # hard=True runs the handler immediately inside the interrupt: fastest, but the
        # handler must not allocate memory (no lists, no floats, no print).
        self._pin_a.irq(self._on_edge, trigger, hard=True)
        self._pin_b.irq(self._on_edge, trigger, hard=True)

    def _read_state(self):
        return (self._pin_b.value() << 1) | self._pin_a.value()

    def _on_edge(self, _pin):
        new_state = self._read_state()
        self._count += TRANSITION_TABLE[(self._state << 2) | new_state]
        self._state = new_state

    def count(self):
        return -self._count if self.invert else self._count

    def reset(self):
        self._count = 0

    def deinit(self):
        self._pin_a.irq(None)
        self._pin_b.irq(None)


class WheelEncoders:
    """Left and right PIO encoders, built from config.py."""

    def __init__(self):
        if config.ENCODER_LEFT_B != config.ENCODER_LEFT_A + 1 or \
                config.ENCODER_RIGHT_B != config.ENCODER_RIGHT_A + 1:
            raise ValueError("PIO encoders need B on the pin right after A (see config.py)")
        self.left = QuadratureEncoderPIO(4, config.ENCODER_LEFT_A, config.ENCODER_LEFT_REVERSED)
        self.right = QuadratureEncoderPIO(5, config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_REVERSED)

    def counts(self):
        return self.left.count(), self.right.count()

    def reset(self):
        self.left.reset()
        self.right.reset()
