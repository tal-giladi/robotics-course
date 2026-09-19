# 04_ultrasonic.py - measure distance with the US-100 ultrasonic sensor (pulse mode).
#
# Lesson 01.13. Needs sensors.py and config.py:
#     cd labs/firmware/pico
#     mpremote mount . run examples/04_ultrasonic.py
#
# Wiring: VCC -> 3V3, GND -> GND, Trig/TX -> GPIO 14, Echo/RX -> GPIO 15.
# REMOVE the jumper on the back of the US-100 (with the jumper fitted it talks UART instead).
# Powered at 3.3 V its echo output is 3.3 V: safe for the Pico, no divider needed.
#
# Try: a flat wall (good echo), a soft sweater (absorbs sound), a wall at 45 degrees (the echo
# bounces away - no reading). That is one reason the robot also has a ToF sensor.

import time

import config
from sensors import US100

sonar = US100(config.ULTRASONIC_TRIG, config.ULTRASONIC_ECHO)

print("Ctrl-C to stop.")
while True:
    mm = sonar.read_mm()
    if mm is None:
        print("no echo")
    else:
        print("%4d mm  %s" % (mm, "#" * (mm // 50)))     # a crude bar graph
    time.sleep_ms(100)      # let echoes of the previous ping die out
