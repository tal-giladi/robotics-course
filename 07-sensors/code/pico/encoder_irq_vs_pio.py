# encoder_irq_vs_pio.py - count the SAME encoder with PIO and with pin interrupts, at rising speed.
#
# Lesson 07.02. MicroPython on the Pico 2; uses the course firmware's encoders.py and motors.py:
#
#     cd labs/firmware/pico
#     mpremote mount . run ../../../07-sensors/code/pico/encoder_irq_vs_pio.py
#
# SAFETY - read before running:
#   * The robot must stand on a box or mug with BOTH WHEELS OFF THE GROUND. This drives the left
#     motor up to full duty with no feedback.
#   * Fingers, hair and cables away from the wheels. Hand on the battery switch. Ctrl-C brakes.
#
# Both counters watch pins ENCODER_LEFT_A/B at the same time. PIO is the reference (it cannot miss an
# edge at these rates); the difference is what the interrupt handler lost. See lesson 07.02 for why.

import time

import config
from encoders import QuadratureEncoderIRQ, QuadratureEncoderPIO
from motors import Motor

DUTIES = (0.2, 0.4, 0.6, 0.8, 1.0)
SPIN_UP_MS = 700            # let the motor reach speed before measuring the edge rate
MEASURE_MS = 1500
SPIN_DOWN_MS = 800          # wait until the wheel has stopped before comparing the counts


def lost_percent(reference, other):
    if reference == 0:
        return 0.0
    return 100.0 * (abs(reference) - abs(other)) / abs(reference)


def main():
    irq = QuadratureEncoderIRQ(config.ENCODER_LEFT_A, config.ENCODER_LEFT_B, config.ENCODER_LEFT_REVERSED)
    pio = QuadratureEncoderPIO(4, config.ENCODER_LEFT_A, config.ENCODER_LEFT_REVERSED)
    motor = Motor(config.MOTOR_LEFT_IN1, config.MOTOR_LEFT_IN2, invert=config.MOTOR_LEFT_REVERSED)
    print("LEFT WHEEL OFF THE GROUND? Starting in 3 s (Ctrl-C to abort)")
    time.sleep(3)
    print("duty   edges/s   pio_ticks   irq_ticks   irq_lost")
    try:
        for duty in DUTIES:
            pio.reset()
            irq.reset()
            motor.set_duty(duty)                    # SAFETY: the wheel spins now
            time.sleep_ms(SPIN_UP_MS)
            start = pio.count()
            time.sleep_ms(MEASURE_MS)
            rate = (pio.count() - start) * 1000 / MEASURE_MS
            motor.brake()
            time.sleep_ms(SPIN_DOWN_MS)
            p, i = pio.count(), irq.count()
            print("%.1f  %8.0f  %10d  %10d   %6.2f%%" % (duty, rate, p, i, lost_percent(p, i)))
    finally:
        motor.brake()
        irq.deinit()
        pio.deinit()


if __name__ == "__main__":
    main()
