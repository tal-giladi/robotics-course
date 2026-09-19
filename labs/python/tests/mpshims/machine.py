"""CPython stand-in for MicroPython's ``machine`` module (tests only).

Just enough of Pin, PWM, ADC, I2C, WDT and time_pulse_us for the firmware modules in
``labs/firmware/pico`` to import and run their logic on a PC. Objects remember what was done
to them so tests can assert on it (e.g. the PWM duty that a motor command produced).
"""

from __future__ import annotations


class Pin:
    IN = 0
    OUT = 1
    PULL_UP = 1
    PULL_DOWN = 2
    IRQ_RISING = 4
    IRQ_FALLING = 8

    levels: dict = {}
    """Logic level of every GPIO, shared by all Pin objects for the same id (tests set these)."""

    def __init__(self, id, mode=-1, pull=-1, *, value=None):
        self.id = id
        self.mode = mode
        self.pull = pull
        if value is not None:
            Pin.levels[id] = 1 if value else 0
        self.irq_handler = None

    def value(self, v=None):
        if v is None:
            return Pin.levels.get(self.id, 0)
        Pin.levels[self.id] = 1 if v else 0
        return None

    def on(self):
        self.value(1)

    def off(self):
        self.value(0)

    def toggle(self):
        self.value(0 if self.value() else 1)

    def irq(self, handler=None, trigger=0, hard=False):
        self.irq_handler = handler


class PWM:
    def __init__(self, pin, freq=0, duty_u16=0):
        self.pin = pin
        self._freq = freq
        self._duty = duty_u16

    def freq(self, value=None):
        if value is None:
            return self._freq
        self._freq = value
        return None

    def duty_u16(self, value=None):
        if value is None:
            return self._duty
        if not 0 <= value <= 65535:
            raise ValueError("duty_u16 out of range")
        self._duty = int(value)
        return None

    def deinit(self):
        self._duty = 0


class ADC:
    def __init__(self, pin):
        self.pin = pin
        self.value_u16 = 0

    def read_u16(self):
        return self.value_u16


class I2C:
    """A bus with no devices unless a test registers some in ``devices`` (address -> object with
    ``readfrom_mem(register, n)`` and ``writeto_mem(register, data)``)."""

    def __init__(self, id=0, *, scl=None, sda=None, freq=400_000):
        self.devices: dict[int, object] = {}

    def scan(self):
        return sorted(self.devices)

    def _device(self, addr):
        if addr not in self.devices:
            raise OSError(19, "ENODEV")
        return self.devices[addr]

    def readfrom_mem(self, addr, memaddr, nbytes, *, addrsize=8):
        return self._device(addr).readfrom_mem(memaddr, nbytes)

    def writeto_mem(self, addr, memaddr, buf, *, addrsize=8):
        self._device(addr).writeto_mem(memaddr, bytes(buf))


class WDT:
    def __init__(self, id=0, timeout=5000):
        self.timeout = timeout
        self.feeds = 0

    def feed(self):
        self.feeds += 1


PULSE_RESULT_US = -2
"""What :func:`time_pulse_us` returns; tests may overwrite it."""


def time_pulse_us(pin, pulse_level, timeout_us=1_000_000):
    return PULSE_RESULT_US


def reset():
    raise SystemExit("machine.reset()")
