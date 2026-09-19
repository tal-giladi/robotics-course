# noise_probe.py - measure sensor noise with the motors off, running, and reversing.
#
# Lesson 02.06. Needs the course firmware modules (config, motors, sensors, vl53l1x):
#     cd labs/firmware/pico
#     mpremote mount . run ../../../02-robot-electronics/code/pico/noise_probe.py > noise_before.csv
# Then on your computer:
#     python 02-robot-electronics/code/noise_lab.py --log labs/firmware/pico/noise_before.csv
#
# SAFETY: the motors spin at 50-60 % duty. Robot ON A STAND, wheels in the air, hand on the
# battery switch. Ctrl-C stops the script; the finally block brakes both motors.
#
# Extra wiring for the "ground noise" channel: a 10 k resistor from GP28 (ADC2) to the Pico's
# AGND pin (pin 33). The ADC should read ~0 mV; whatever it reads is noise and ground shift.

from machine import ADC, I2C, Pin

import config
from motors import DriveMotors
from sensors import INA219
from vl53l1x import VL53L1X

try:
    import time
    sleep_ms, ticks_ms, ticks_diff = time.sleep_ms, time.ticks_ms, time.ticks_diff
except AttributeError:                  # CPython test shims
    import utime
    sleep_ms, ticks_ms, ticks_diff = utime.sleep_ms, utime.ticks_ms, utime.ticks_diff

SAMPLES = 60


def sample(condition, i2c, tof, ina, adc, emit):
    """Read every signal once and emit 'condition,signal,value' lines; I2C failures emit 'err'."""
    try:
        mm = tof.read_mm(timeout_ms=150)
        emit("%s,tof_mm,%s" % (condition, "err" if mm is None else mm))
    except OSError:
        emit("%s,tof_mm,err" % condition)
    try:
        emit("%s,bus_v,%.3f" % (condition, ina.bus_voltage_v()))
        emit("%s,current_a,%.3f" % (condition, ina.current_a()))
    except OSError:
        emit("%s,bus_v,err" % condition)
    emit("%s,adc_gnd_mv,%.1f" % (condition, adc.read_u16() * 3300 / 65535))


def run(emit=print, samples=SAMPLES):
    i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ_HZ)
    tof = VL53L1X(i2c)
    ina = INA219(i2c)
    adc = ADC(Pin(28))
    motors = DriveMotors(decay="slow")
    emit("# condition,signal,value")
    try:
        motors.coast()
        sleep_ms(500)
        for _ in range(samples):
            sample("off", i2c, tof, ina, adc, emit)

        motors.set_duty(0.5, 0.5)                   # SAFETY: wheels spin now
        sleep_ms(1000)
        for _ in range(samples):
            sample("run50", i2c, tof, ina, adc, emit)

        direction = 1
        last = ticks_ms()
        for _ in range(samples):
            if ticks_diff(ticks_ms(), last) > 400:  # full reversal every 0.4 s: the worst normal event
                direction = -direction
                motors.set_duty(0.6 * direction, 0.6 * direction)   # SAFETY: wheels spin now
                last = ticks_ms()
            sample("reverse", i2c, tof, ina, adc, emit)
    finally:
        motors.brake()
        sleep_ms(300)
        motors.coast()


if __name__ == "__main__":
    run()
