# 06_battery.py - read the battery voltage (and current) with an INA219, or a resistor divider.
#
# Lesson 01.14. Needs sensors.py and config.py:
#     cd labs/firmware/pico
#     mpremote mount . run examples/06_battery.py
#
# INA219 wiring: VCC -> 3V3, GND -> GND, SDA -> GPIO 4, SCL -> GPIO 5,
#                VIN+ -> battery side, VIN- -> load side (fuse, buck converter, motor drivers).
# Divider wiring: battery + -> 100k -> GPIO 26 -> 22k -> GND.
#
# SAFETY: measure the divider's middle point with a multimeter BEFORE connecting it to the
# Pico: with a full battery it must be below 3.3 V (12.6 V -> about 2.27 V).
#
# A 3S Li-ion pack: 12.6 V full, 10.8 V nominal (3 x 3.6 V, Samsung 35E), 10.5 V low
# (config.BATTERY_LOW_WARNING_V).

import time

from machine import I2C, Pin

import config
from sensors import INA219, DividerBattery

USE_DIVIDER = False

if USE_DIVIDER:
    battery = DividerBattery()
else:
    i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ_HZ)
    battery = INA219(i2c)

print("Ctrl-C to stop.")
while True:
    volts = battery.read_mv() / 1000
    line = "battery %.2f V (%.2f V per cell)" % (volts, volts / config.BATTERY_CELLS)
    if not USE_DIVIDER:
        line += "  current %.2f A" % battery.current_a()
    if volts < config.BATTERY_LOW_WARNING_V:
        line += "  LOW - charge soon"
    print(line)
    time.sleep(1)
