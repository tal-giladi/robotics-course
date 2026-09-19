"""CPython stand-in for MicroPython's ``rp2`` module (tests only): a PIO assembler and a tiny PIO
interpreter.

``@asm_pio`` works like the real decorator: it runs the program function twice with the PIO
instruction names injected as globals (pass 0 collects labels, pass 1 emits instructions).
Instead of 16-bit machine words it produces readable tuples, and :class:`StateMachine` executes
them against the shared pin levels in ``machine.Pin.levels``.

Only the instructions and operands the course firmware uses are implemented; anything else
raises ``NotImplementedError`` so a test fails loudly instead of silently passing. The
interpreter follows the RP2040/RP2350 datasheet semantics for these instructions, including
32-bit register wrap-around, ``jmp y--`` testing Y *before* decrementing, ``push`` clearing
the ISR, and the RX FIFO holding at most 4 words.
"""

from __future__ import annotations

from collections import deque

import machine

MASK32 = 0xFFFFFFFF
MAX_INSTRUCTIONS = 32


class PIOASMError(Exception):
    pass


class PIO:
    SHIFT_LEFT = 0
    SHIFT_RIGHT = 1
    JOIN_NONE = 0
    JOIN_TX = 1
    JOIN_RX = 2
    IN_LOW = 0
    IN_HIGH = 1
    OUT_LOW = 2
    OUT_HIGH = 3

    def __init__(self, id):
        self.id = id


class Program:
    def __init__(self, instructions, labels, in_shiftdir, out_shiftdir):
        self.instructions = instructions
        self.labels = labels
        self.in_shiftdir = in_shiftdir
        self.out_shiftdir = out_shiftdir


class _Emitter:
    def __init__(self):
        self.labels: dict[str, int] = {}
        self.instructions: list[tuple] = []
        self.pass_ = 0

    def _emit(self, instruction):
        self.instructions.append(instruction)

    def _addr(self, label):
        if self.pass_ == 0:
            return 0
        if label not in self.labels:
            raise PIOASMError(f"unknown label {label}")
        return self.labels[label]

    def label(self, name):
        if self.pass_ == 0:
            if name in self.labels:
                raise PIOASMError(f"duplicate label {name}")
            self.labels[name] = len(self.instructions)

    def wrap_target(self):
        pass

    def wrap(self):
        pass

    def jmp(self, cond, label=None):
        if label is None:
            cond, label = None, cond
        self._emit(("jmp", cond, self._addr(label)))

    def mov(self, dest, src):
        self._emit(("mov", dest, src))

    def push(self, mode="block"):
        self._emit(("push", mode))

    def out(self, dest, bits):
        self._emit(("out", dest, bits))

    def in_(self, src, bits):
        self._emit(("in", src, bits))

    def nop(self):
        self._emit(("nop",))

    def _unsupported(self, *args, **kwargs):
        raise NotImplementedError("instruction not implemented in the rp2 test shim")


def asm_pio(*, in_shiftdir=PIO.SHIFT_LEFT, out_shiftdir=PIO.SHIFT_LEFT, **_other):
    emitter = _Emitter()

    def decorator(function):
        names = {
            # operands
            "x": "x", "y": "y", "isr": "isr", "osr": "osr", "pc": "pc", "pins": "pins",
            "null": "null", "y_dec": "y_dec", "x_dec": "x_dec", "not_x": "not_x", "not_y": "not_y",
            "x_not_y": "x_not_y", "noblock": "noblock", "block": "block",
            "invert": lambda operand: ("invert", operand),
        }
        for name in ("label", "wrap_target", "wrap", "jmp", "mov", "push", "out", "in_", "nop"):
            names[name] = getattr(emitter, name)
        for name in ("set", "wait", "pull", "irq", "word"):
            names[name] = emitter._unsupported

        module_globals = function.__globals__
        saved = module_globals.copy()
        module_globals.clear()
        try:
            module_globals.update(names)
            for pass_ in (0, 1):
                emitter.pass_ = pass_
                emitter.instructions = []
                function()
        finally:
            module_globals.clear()
            module_globals.update(saved)
        if len(emitter.instructions) > MAX_INSTRUCTIONS:
            raise PIOASMError("program too long")
        return Program(emitter.instructions, dict(emitter.labels), in_shiftdir, out_shiftdir)

    return decorator


class StateMachine:
    """Executes a shim :class:`Program`. ``get()`` runs the program until a word is available."""

    STEPS_PER_GET = 64

    def __init__(self, id, program=None, freq=-1, *, in_base=None, **_other):
        self.id = id
        self.program = program
        self.in_base = in_base.id if in_base is not None else 0
        self.running = False
        self.restart()

    def restart(self):
        self.pc = 0
        self.x = self.y = self.isr = self.osr = 0
        self.rx = deque()

    def active(self, value=None):
        if value is None:
            return self.running
        self.running = bool(value)
        return None

    def rx_fifo(self):
        # On real hardware the program runs millions of steps between two CPU reads; let some
        # "time" pass here too, so the FIFO holds words pushed after the latest pin change.
        self._run(self.STEPS_PER_GET)
        return len(self.rx)

    def _run(self, steps):
        if self.running:
            for _ in range(steps):
                self.step()

    def get(self):
        self._run(self.STEPS_PER_GET)
        if not self.rx:
            raise RuntimeError("program pushed nothing (a real get() would block forever)")
        return self.rx.popleft()

    # --- interpreter -------------------------------------------------------------------------
    def _read_pins(self, bits):
        value = 0
        for i in range(bits):
            value |= machine.Pin.levels.get(self.in_base + i, 0) << i
        return value

    def _source(self, src):
        if isinstance(src, tuple) and src[0] == "invert":
            return ~self._source(src[1]) & MASK32
        if src in ("x", "y", "isr", "osr"):
            return getattr(self, src)
        if src == "null":
            return 0
        if src == "pins":
            return self._read_pins(32)
        raise NotImplementedError(f"mov source {src!r}")

    def step(self):
        instruction = self.program.instructions[self.pc]
        op = instruction[0]
        next_pc = (self.pc + 1) % len(self.program.instructions)

        if op == "jmp":
            _, cond, target = instruction
            if cond is None:
                take = True
            elif cond == "y_dec":
                take = self.y != 0
                self.y = (self.y - 1) & MASK32
            elif cond == "x_dec":
                take = self.x != 0
                self.x = (self.x - 1) & MASK32
            elif cond == "not_x":
                take = self.x == 0
            elif cond == "not_y":
                take = self.y == 0
            elif cond == "x_not_y":
                take = self.x != self.y
            else:
                raise NotImplementedError(f"jmp condition {cond!r}")
            if take:
                next_pc = target
        elif op == "mov":
            _, dest, src = instruction
            value = self._source(src)
            if dest == "pc":
                next_pc = value % MAX_INSTRUCTIONS
            elif dest in ("x", "y", "isr", "osr"):
                setattr(self, dest, value)
            else:
                raise NotImplementedError(f"mov destination {dest!r}")
        elif op == "push":
            if len(self.rx) < 4:
                self.rx.append(self.isr)
            elif instruction[1] != "noblock":
                raise NotImplementedError("blocking push on a full FIFO")
            self.isr = 0
        elif op == "out":
            _, dest, bits = instruction
            if self.program.out_shiftdir == PIO.SHIFT_RIGHT:
                data = self.osr & ((1 << bits) - 1)
                self.osr >>= bits
            else:
                data = self.osr >> (32 - bits)
                self.osr = (self.osr << bits) & MASK32
            if dest in ("x", "y", "isr"):
                setattr(self, dest, data)
            else:
                raise NotImplementedError(f"out destination {dest!r}")
        elif op == "in":
            _, src, bits = instruction
            if src != "pins":
                raise NotImplementedError(f"in source {src!r}")
            data = self._read_pins(bits)
            if self.program.in_shiftdir == PIO.SHIFT_LEFT:
                self.isr = ((self.isr << bits) | data) & MASK32
            else:
                self.isr = (self.isr >> bits) | (data << (32 - bits))
        elif op == "nop":
            pass
        else:
            raise NotImplementedError(op)
        self.pc = next_pc
