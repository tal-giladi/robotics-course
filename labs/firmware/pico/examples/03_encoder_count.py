# 03_encoder_count.py - print the tick count of both wheel encoders.
#
# Lesson 01.09. Needs encoders.py and config.py:
#     cd labs/firmware/pico
#     mpremote mount . run examples/03_encoder_count.py
#
# Turn each wheel by hand exactly one revolution (put a tape mark on the tyre). The count
# should change by about 2464 (11 pulses x 4 edges x 56:1 gearbox). Rolling the robot
# FORWARD must make both counts go UP - if not, flip ENCODER_*_REVERSED in config.py.
#
# Set USE_IRQ = True to try the interrupt-based counter instead of PIO. Turned by hand, both
# agree. With a motor at full speed (wheels off the ground!) the IRQ version falls behind:
# Python interrupt handlers can't keep up with ~8,000 edges per second.

import time

import config
from encoders import QuadratureEncoderIRQ, QuadratureEncoderPIO

USE_IRQ = False

if USE_IRQ:
    left = QuadratureEncoderIRQ(config.ENCODER_LEFT_A, config.ENCODER_LEFT_B, config.ENCODER_LEFT_REVERSED)
    right = QuadratureEncoderIRQ(config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_B, config.ENCODER_RIGHT_REVERSED)
else:
    # State machines 4 and 5 are in PIO block 1 (see the comment above quadrature_program).
    left = QuadratureEncoderPIO(4, config.ENCODER_LEFT_A, config.ENCODER_LEFT_REVERSED)
    right = QuadratureEncoderPIO(5, config.ENCODER_RIGHT_A, config.ENCODER_RIGHT_REVERSED)

print("Turn the wheels by hand. Ctrl-C to stop.")
try:
    while True:
        left_ticks, right_ticks = left.count(), right.count()
        print("left %7d ticks (%6.2f rev)   right %7d ticks (%6.2f rev)" % (
            left_ticks, left_ticks / config.TICKS_PER_WHEEL_REV,
            right_ticks, right_ticks / config.TICKS_PER_WHEEL_REV))
        time.sleep_ms(200)
finally:
    left.deinit()
    right.deinit()
