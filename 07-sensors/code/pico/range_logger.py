# range_logger.py - static-test logger: many readings from the US-100 and the VL53L1X, as CSV lines.
#
# Lessons 07.01, 07.03, 07.04. MicroPython on the Pico 2; uses the course firmware's drivers.
# Run it from the firmware folder so `config`, `sensors` and `vl53l1x` import, and pipe the output
# into the tagger, which adds the tape-measured distance and appends to a CSV:
#
#     cd labs/firmware/pico
#     mpremote mount . run ../../../07-sensors/code/pico/range_logger.py | python ../../../07-sensors/code/sensor_stats.py tag --true-m 0.50 --out bench.csv
#
# Output, one line per reading:   us100,<mm>   or   vl53l1x,<mm>      (-1 = no valid reading)
#
# The two sensors take turns so they see the same scene at the same time. They don't disturb each
# other: one is sound at 40 kHz, the other infrared light at 940 nm. A missing sensor is skipped.

import time

from machine import I2C, Pin

import config
from sensors import US100
from vl53l1x import VL53L1X

SAMPLES = 200               # per sensor; ~25 s at the rates below
US_SETTLE_MS = 60           # let the previous ping's echoes die out before the next one
TOF_TIMEOUT_MS = 200        # the default 100 ms timing budget gives ~10 readings per second


def open_sensors():
    sonar = US100(config.ULTRASONIC_TRIG, config.ULTRASONIC_ECHO)
    tof = None
    try:
        i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ_HZ)
        tof = VL53L1X(i2c)
    except OSError as exc:
        print("# no VL53L1X:", exc)
    return sonar, tof


def as_mm(value):
    return -1 if value is None else int(value)


def run(samples=SAMPLES):
    sonar, tof = open_sensors()
    print("# logging %d readings per sensor - keep still, keep the scene still" % samples)
    t0 = time.ticks_ms()
    for _ in range(samples):
        print("us100,%d" % as_mm(sonar.read_mm()))
        if tof is not None:
            print("vl53l1x,%d" % as_mm(tof.read_mm(timeout_ms=TOF_TIMEOUT_MS)))
        time.sleep_ms(US_SETTLE_MS)
    print("# done in %.1f s" % (time.ticks_diff(time.ticks_ms(), t0) / 1000))


if __name__ == "__main__":
    run()
