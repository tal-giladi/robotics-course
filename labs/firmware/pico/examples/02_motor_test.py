# 02_motor_test.py - spin each motor forward and backward at a few duty cycles.
#
# Lesson 01.08. Needs motors.py and config.py - run it with the firmware folder mounted:
#     cd labs/firmware/pico
#     mpremote mount . run examples/02_motor_test.py
#
# SAFETY - read before running:
#   * Put the robot on a box or a mug so BOTH WHEELS ARE OFF THE GROUND. This script drives
#     the motors without any feedback; on the table the robot would drive off the edge.
#   * Keep fingers, hair and cables away from the wheels.
#   * Keep a hand on the battery switch. Ctrl-C stops the script and the `finally` block
#     below brakes the motors.
#
# What to check (and fix in config.py):
#   * "forward" must turn each wheel the way that would move the ROBOT forward. If a wheel
#     turns the wrong way, flip MOTOR_LEFT_REVERSED or MOTOR_RIGHT_REVERSED.
#   * Note the smallest duty at which each wheel starts turning: that is the deadband
#     (config.DUTY_DEADBAND; lesson 08.02 measures it properly).

import time

import config
from motors import Motor

DUTIES = (0.1, 0.2, 0.4, 0.7)           # stays below full speed on purpose
STEP_S = 1.5

left = Motor(config.MOTOR_LEFT_IN1, config.MOTOR_LEFT_IN2, invert=config.MOTOR_LEFT_REVERSED)
right = Motor(config.MOTOR_RIGHT_IN1, config.MOTOR_RIGHT_IN2, invert=config.MOTOR_RIGHT_REVERSED)

print("Wheels OFF the ground? Starting in 3 seconds... (Ctrl-C to abort)")
time.sleep(3)

try:
    for name, motor in (("left", left), ("right", right)):
        for direction in (+1, -1):
            for duty in DUTIES:
                command = direction * duty
                print("%-5s motor, duty %+.1f" % (name, command))
                motor.set_duty(command)             # SAFETY: the wheel turns now
                time.sleep(STEP_S)
            motor.brake()
            time.sleep(0.5)

    print("Coast: left at 0.5, then coast - it spins down slowly")
    left.set_duty(0.5)                              # SAFETY: the wheel turns now
    time.sleep(1.0)
    left.coast()
    time.sleep(1.5)
    print("Brake: left at 0.5, then brake - it stops quickly")
    left.set_duty(0.5)                              # SAFETY: the wheel turns now
    time.sleep(1.0)
    left.brake()
    time.sleep(1.0)
finally:
    left.brake()
    right.brake()
    print("Motors stopped.")
