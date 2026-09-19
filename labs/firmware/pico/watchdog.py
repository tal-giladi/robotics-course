# watchdog.py - stop the motors when the commands stop coming.
#
# Lesson 01.10 (command watchdog), 03.03 (robust communication).
#
# Why a watchdog?
#   The Pi sends "drive at 30 %" and the Pico keeps doing that until told otherwise. If the
#   Pi program crashes, the USB cable falls out, or the Wi-Fi to your laptop drops, the next
#   command never arrives - and a robot without a watchdog drives into the wall at 30 %.
#
#   So the rule is: a drive command is only valid for a short time (300 ms by default). The
#   host must keep repeating it (robotlab.serial_base does this automatically). If no M or V
#   command arrives in time, the firmware stops the motors and sets flag 1 in telemetry.
#
# Two watchdogs, two different failures:
#   CommandWatchdog  - the HOST went silent. Software timer, checked every control step.
#   HardwareWatchdog - the PICO ITSELF hung (infinite loop, crashed task). machine.WDT is a
#                      hardware timer: if our control loop stops feeding it, the chip resets,
#                      and after a reset the motor pins are inputs again, so the motors coast.

try:                                    # MicroPython
    from time import ticks_diff
except ImportError:                     # CPython (tests): labs/python/tests/mpshims/utime.py
    from utime import ticks_diff


class CommandWatchdog:
    """Software watchdog for host commands. Times are time.ticks_ms() values.

        wd = CommandWatchdog(timeout_ms=300, now_ms=time.ticks_ms())
        wd.feed(now)            # a valid M or V command arrived
        if wd.check(now):       # call every control step
            motors.brake()

    ticks_ms() wraps around (after ~12 days on the Pico), so we never subtract two tick values
    directly: time.ticks_diff() handles the wrap-around correctly.
    """

    def __init__(self, timeout_ms, now_ms):
        self.timeout_ms = timeout_ms
        self._last_feed_ms = now_ms
        # Start tripped: after boot the motors are stopped until the first command arrives.
        self.tripped = True

    def feed(self, now_ms):
        self._last_feed_ms = now_ms
        self.tripped = False

    def elapsed_ms(self, now_ms):
        return ticks_diff(now_ms, self._last_feed_ms)

    def check(self, now_ms):
        """Return True exactly once, on the step where the watchdog trips.

        While already tripped it returns False (the motors were stopped then; don't stop them
        again every step). `self.tripped` stays True until the next feed().
        """
        if self.tripped:
            return False
        if self.elapsed_ms(now_ms) >= self.timeout_ms:
            self.tripped = True
            return True
        return False


class HardwareWatchdog:
    """Thin wrapper around machine.WDT that can be switched off in config.py.

    timeout_ms = 0 disables it. Once enabled it cannot be stopped until the next reset.
    """

    def __init__(self, timeout_ms):
        self._wdt = None
        if timeout_ms > 0:
            from machine import WDT
            self._wdt = WDT(timeout=timeout_ms)

    @property
    def enabled(self):
        return self._wdt is not None

    def feed(self):
        if self._wdt is not None:
            self._wdt.feed()
