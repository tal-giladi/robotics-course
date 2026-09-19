# 07_i2c_scan.py - list every device that answers on the I2C bus.
#
# Lessons 01.13, 01.14. The first thing to run when an I2C sensor "doesn't work":
#     cd labs/firmware/pico
#     mpremote mount . run examples/07_i2c_scan.py
#
# Expected on karmel:  0x29 = VL53L1X ToF sensor,  0x40 = INA219 battery monitor.
# Nothing found? Check 3V3 and GND, that SDA goes to GPIO 4 and SCL to GPIO 5 (not swapped),
# and that the bus has pull-up resistors (the Pololu and INA219 boards include them).

from machine import I2C, Pin

import config

KNOWN_DEVICES = {0x29: "VL53L1X time-of-flight", 0x40: "INA219 current/voltage"}

i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ_HZ)
addresses = i2c.scan()

if not addresses:
    print("No I2C devices found.")
for address in addresses:
    print("0x%02X  %s" % (address, KNOWN_DEVICES.get(address, "unknown device")))
