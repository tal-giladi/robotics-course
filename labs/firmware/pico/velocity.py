# velocity.py - wheel speed from encoder ticks, and the PID that holds a wheel speed.
#
# Lessons: 08.03 (estimating speed), 08.04-08.07 (P, I, D, anti-windup), 08.09 (feedforward),
#          08.12 (why this loop runs on the microcontroller and not on the Pi).
#
# Why run the velocity loop on the Pico?
#   The loop must run at a steady 100 Hz. The Pi runs Linux, Python, ROS and a camera; its loop
#   timing jitters by milliseconds, and the USB link adds more delay. The Pico does nothing
#   else, reads the encoders directly and reacts within one period. The Pi then only sends
#   a setpoint ("left wheel 5 rad/s") a few dozen times per second - that is lesson 08.12.
#
# Pure Python with no hardware imports, so the tests run it under CPython unchanged.

import math

TWO_PI = 2.0 * math.pi


class WheelSpeedEstimator:
    """Wheel angular velocity (rad/s) from a cumulative tick count sampled at a fixed rate.

    speed = (ticks now - ticks last time) / ticks per revolution * 2*pi / dt

    One tick in a 10 ms period is 2*pi / 2464 / 0.01 = 0.255 rad/s, so the raw estimate
    jumps in steps of 0.255 rad/s. A first-order low-pass filter smooths it:

        filtered = filtered + alpha * (raw - filtered)       alpha = 1.0 -> no filtering

    Smaller alpha = smoother but slower to react (more lag in the control loop).
    """

    def __init__(self, ticks_per_rev, alpha=1.0):
        self.ticks_per_rev = ticks_per_rev
        self.alpha = alpha
        self.rad_s = 0.0             # filtered estimate
        self.raw_rad_s = 0.0         # last unfiltered estimate
        self._last_ticks = None

    def update(self, ticks, dt_s):
        """Feed the current cumulative tick count; returns the filtered speed in rad/s."""
        if self._last_ticks is None or dt_s <= 0:
            self._last_ticks = ticks     # first sample: nothing to differentiate yet
            return self.rad_s
        delta = ticks - self._last_ticks
        self._last_ticks = ticks
        self.raw_rad_s = delta * TWO_PI / (self.ticks_per_rev * dt_s)
        self.rad_s += self.alpha * (self.raw_rad_s - self.rad_s)
        return self.rad_s

    def reset(self, ticks=None):
        """Forget history (call after the encoder counts were reset)."""
        self._last_ticks = ticks
        self.rad_s = 0.0
        self.raw_rad_s = 0.0


def feedforward_duty(setpoint_rad_s, gain, max_speed_rad_s, deadband):
    """The duty the motor model says we need for this speed, scaled by `gain` (0..1).

    Measured motor model (lesson 08.02): no motion below `deadband` duty, then speed grows
    linearly up to `max_speed_rad_s` at duty 1.0. Inverting it:

        duty = sign(speed) * (deadband + |speed| / max_speed * (1 - deadband))

    With a good model the PID only has to correct small errors, so it can use small gains.
    The simulator (robotlab.sim) uses the same formula, so gains tuned there carry over.
    """
    if setpoint_rad_s == 0 or gain == 0:
        return 0.0
    fraction = abs(setpoint_rad_s) / max_speed_rad_s
    if fraction > 1.0:
        fraction = 1.0
    duty = deadband + fraction * (1.0 - deadband)
    return gain * (duty if setpoint_rad_s > 0 else -duty)


class PID:
    """A discrete PID controller with feedforward and anti-windup, for one wheel.

        output = feedforward + kp * error + integral + derivative        clamped to [-1, 1]

    * Integral is stored already multiplied by ki (`integral += ki * error * dt`), so changing
      ki while running does not make the output jump.
    * Derivative acts on the MEASUREMENT, not the error: a sudden setpoint change would make
      d(error)/dt huge for one step ("derivative kick"); the measured speed can't jump.
    * Anti-windup (lessons 08.05, 08.07): when the output is already saturated, integrating
      further only stores up error that must later be "unwound", causing overshoot. So while
      the error pushes the output past a limit, the integral may only grow until the output
      sits exactly AT that limit, never beyond (conditional integration that fills the
      remaining headroom), and the integral alone is also clamped to the output range.
      Refusing the whole step instead would strand the wheel below the speed it can reach
      whenever one integration step is bigger than the headroom left.
    """

    def __init__(self, kp, ki, kd, ff=0.0, max_speed_rad_s=1.0, deadband=0.0,
                 output_min=-1.0, output_max=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff
        self.max_speed_rad_s = max_speed_rad_s
        self.deadband = deadband
        self.output_min = output_min
        self.output_max = output_max
        self.integral = 0.0
        self.output = 0.0
        self._last_measured = None

    def reset(self):
        self.integral = 0.0
        self.output = 0.0
        self._last_measured = None

    def update(self, setpoint, measured, dt_s):
        """One control step. Returns the duty to send to the motor (-1.0 .. 1.0)."""
        error = setpoint - measured

        feedforward = feedforward_duty(setpoint, self.ff, self.max_speed_rad_s, self.deadband)
        proportional = self.kp * error

        derivative = 0.0
        if self._last_measured is not None and dt_s > 0:
            derivative = -self.kd * (measured - self._last_measured) / dt_s
        self._last_measured = measured

        # Anti-windup, step 1: try the integral update and see where the output would land.
        candidate = self.integral + self.ki * error * dt_s
        others = feedforward + proportional + derivative
        unclamped = others + candidate
        if unclamped > self.output_max and error > 0:
            # Integrate only up to the limit (fill the headroom); never shrink it against the error.
            self.integral = max(self.integral, self.output_max - others)
        elif unclamped < self.output_min and error < 0:
            self.integral = min(self.integral, self.output_min - others)
        else:
            self.integral = candidate
        # Anti-windup, step 2: the integral term alone may never exceed the output range.
        self.integral = min(max(self.integral, self.output_min), self.output_max)

        output = feedforward + proportional + self.integral + derivative
        self.output = min(max(output, self.output_min), self.output_max)
        return self.output
