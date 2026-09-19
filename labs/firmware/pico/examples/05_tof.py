# 05_tof.py - measure distance with the VL53L1X time-of-flight sensor over I2C.
#
# Lesson 01.13. Needs vl53l1x.py and config.py:
#     cd labs/firmware/pico
#     mpremote mount . run examples/05_tof.py
#
# Wiring (Pololu VL53L1X carrier): VIN -> 3V3, GND -> GND, SDA -> GPIO 4, SCL -> GPIO 5.
# Run examples/07_i2c_scan.py first: the sensor should answer at address 0x29.

from machine import I2C, Pin

import config
from vl53l1x import VL53L1X

i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ_HZ)
tof = VL53L1X(i2c)

print("Ctrl-C to stop.")
while True:
    mm = tof.read_mm(timeout_ms=200)        # waits for the next measurement (~10 per second)
    if mm is None:
        print("no valid target (range status %s)" % tof.last_status)
    else:
        print("%4d mm  %s" % (mm, "#" * (mm // 50)))
