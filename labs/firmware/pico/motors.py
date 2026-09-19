# motors.py - drive a DC motor through a Pololu DRV8874 carrier in IN/IN mode.
#
# Lesson 01.08 (PWM, direction, deadband). Used by main.py and examples/02_motor_test.py.
#
# SAFETY: anything that calls Motor.set_duty() can make a wheel spin. While you are
# developing, put the robot on a box with its wheels OFF the table, and keep a hand near the
# battery switch.
#
# How the DRV8874 is controlled in IN/IN mode (PMODE pin tied high)
# ---------------------------------------------------------------
# Each carrier is an H-bridge: four switches that can connect each motor terminal to the
# battery (+) or to ground. Two logic inputs choose what the bridge does:
#
#     IN1  IN2   motor terminals            what the motor does
#     ---  ---   ------------------------   -------------------------------------------
#      0    0    both disconnected          COAST: spins freely to a slow stop
#      1    0    OUT1 = +V, OUT2 = GND      drives FORWARD
#      0    1    OUT1 = GND, OUT2 = +V      drives REVERSE
#      1    1    both shorted to ground     BRAKE: the motor's own voltage fights it
#
# To go at, say, 40 % speed we switch between "drive" and one of the stopped states
# thousands of times per second (PWM). There are two ways to do it:
#
#   drive/coast ("fast decay"):  IN1 = PWM 40 %,   IN2 = 0
#   drive/brake ("slow decay"):  IN1 = 1,          IN2 = PWM 60 %  (low 40 % of the time)
#
# Pololu recommends drive/brake: speed is much more linear in duty, which makes the
# velocity controller's job easier. It is the default here; try both in lesson 01.08.
#
# Why 20 kHz: at 1-2 kHz the motor windings whine audibly. 20 kHz is above hearing and the
# motor's inductance smooths the current, so the torque is steady. Much higher frequencies
# waste energy in the driver's switches (the DRV8874 allows up to 100 kHz).

from machine import Pin, PWM

import config

_DUTY_U16_MAX = 65535


def clamp(value, low, high):
    if value < low:
        return low
    if value > high:
        return high
    return value


def compensate_deadband(duty, deadband):
    """Map a requested duty onto the range where the motor actually turns.

    Below about 12 % duty (config.DUTY_DEADBAND) friction wins and the wheel does not move.
    This stretches 0..1 onto deadband..1 so a small command still produces a small motion:

        compensate_deadband(0.0, 0.12)  -> 0.0     (zero stays zero: stopped means stopped)
        compensate_deadband(0.1, 0.12)  -> 0.208
        compensate_deadband(1.0, 0.12)  -> 1.0
    """
    if duty == 0:
        return 0.0
    magnitude = deadband + abs(duty) * (1.0 - deadband)
    return magnitude if duty > 0 else -magnitude


class Motor:
    """One DC motor on one DRV8874 carrier.

        motor = Motor(in1_pin=2, in2_pin=3)
        motor.set_duty(0.4)     # 40 % forward      <- the wheel starts turning!
        motor.brake()
    """

    def __init__(self, in1_pin, in2_pin, invert=False, pwm_freq_hz=config.PWM_FREQ_HZ,
                 decay="slow", deadband=0.0):
        if decay not in ("slow", "fast"):
            raise ValueError("decay must be 'slow' (drive/brake) or 'fast' (drive/coast)")
        self.invert = invert          # True: swap forward and reverse (mirrored motor)
        self.decay = decay
        self.deadband = deadband      # 0.0 = no compensation (raw duty goes to the driver)
        # Start with both inputs low (coast) so the motor cannot jump when the PWM starts.
        self._in1 = PWM(Pin(in1_pin), freq=pwm_freq_hz, duty_u16=0)
        self._in2 = PWM(Pin(in2_pin), freq=pwm_freq_hz, duty_u16=0)
        self.duty = 0.0               # last commanded duty, -1.0 .. 1.0

    def set_duty(self, duty):
        """Signed duty -1.0 (full reverse) .. 1.0 (full forward). 0.0 brakes (slow decay)
        or coasts (fast decay)."""
        duty = clamp(duty, -1.0, 1.0)
        self.duty = duty
        if self.deadband > 0:
            duty = compensate_deadband(duty, self.deadband)
        if self.invert:
            duty = -duty

        level = int(abs(duty) * _DUTY_U16_MAX)
        if self.decay == "slow":
            # Drive/brake: one input held HIGH, the other pulses LOW for `duty` of the period.
            # (duty 0 -> both high = brake.)
            if duty >= 0:
                self._in1.duty_u16(_DUTY_U16_MAX)
                self._in2.duty_u16(_DUTY_U16_MAX - level)
            else:
                self._in1.duty_u16(_DUTY_U16_MAX - level)
                self._in2.duty_u16(_DUTY_U16_MAX)
        else:
            # Drive/coast: one input LOW, the other pulses HIGH for `duty` of the period.
            if duty >= 0:
                self._in1.duty_u16(level)
                self._in2.duty_u16(0)
            else:
                self._in1.duty_u16(0)
                self._in2.duty_u16(level)

    def brake(self):
        """Both outputs to ground: the fastest way to stop. Used by the S command and the
        watchdog."""
        self.duty = 0.0
        self._in1.duty_u16(_DUTY_U16_MAX)
        self._in2.duty_u16(_DUTY_U16_MAX)

    def coast(self):
        """Disconnect the motor: it rolls to a stop. Also the state after power-up/reset."""
        self.duty = 0.0
        self._in1.duty_u16(0)
        self._in2.duty_u16(0)

    def deinit(self):
        self.coast()
        self._in1.deinit()
        self._in2.deinit()


class DriveMotors:
    """The robot's left and right motors, built from config.py."""

    def __init__(self, decay="slow", deadband=0.0):
        self.left = Motor(config.MOTOR_LEFT_IN1, config.MOTOR_LEFT_IN2,
                          invert=config.MOTOR_LEFT_REVERSED, decay=decay, deadband=deadband)
        self.right = Motor(config.MOTOR_RIGHT_IN1, config.MOTOR_RIGHT_IN2,
                           invert=config.MOTOR_RIGHT_REVERSED, decay=decay, deadband=deadband)

    def set_duty(self, left, right):
        # SAFETY: this starts both wheels.
        self.left.set_duty(left)
        self.right.set_duty(right)

    def brake(self):
        self.left.brake()
        self.right.brake()

    def coast(self):
        self.left.coast()
        self.right.coast()
