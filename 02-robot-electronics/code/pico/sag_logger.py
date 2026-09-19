# sag_logger.py - measure the battery sag and the source resistance when one motor starts.
#
# Lesson 02.02. Needs the course firmware modules (config, motors, sensors):
#     cd labs/firmware/pico
#     mpremote mount . run ../../../02-robot-electronics/code/pico/sag_logger.py
#
# SAFETY: robot ON A STAND, wheels in the air, hand on the battery switch. The LEFT motor starts
# at 60 % duty without a ramp (on purpose), runs 0.6 s and brakes. Nothing else moves.
#
# The INA219 (0.1 ohm shunt) saturates at 3.2 A. One motor starting at 60 % stays below that;
# the script marks clipped samples anyway.

from machine import I2C, Pin

import config
from motors import Motor
from sensors import INA219

try:
    import time
    sleep_ms, ticks_ms, ticks_diff = time.sleep_ms, time.ticks_ms, time.ticks_diff
except AttributeError:                  # CPython test shims
    import utime
    sleep_ms, ticks_ms, ticks_diff = utime.sleep_ms, utime.ticks_ms, utime.ticks_diff

CLIP_A = 3.19


def estimate_resistance(samples):
    """samples: list of (ms, volts, amps). Rest = median of the first 50 ms; load = the sample with
    the highest unclipped current. R = (V_rest - V_load) / (I_load - I_rest)."""
    rest = [s for s in samples if s[0] < 50]
    if not rest:
        return None
    rest_v = sorted(s[1] for s in rest)[len(rest) // 2]
    rest_i = sorted(s[2] for s in rest)[len(rest) // 2]
    loaded = [s for s in samples if s[0] >= 50 and abs(s[2]) < CLIP_A]
    if not loaded:
        return None
    peak = max(loaded, key=lambda s: s[2])
    di = peak[2] - rest_i
    if di < 0.5:
        return None                     # too little current change for a meaningful estimate
    return {"rest_v": rest_v, "rest_a": rest_i, "load_v": peak[1], "load_a": peak[2],
            "r_ohm": (rest_v - peak[1]) / di}


def run(duty=0.6, run_ms=600):
    i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ_HZ)
    ina = INA219(i2c)
    left = Motor(config.MOTOR_LEFT_IN1, config.MOTOR_LEFT_IN2, invert=config.MOTOR_LEFT_REVERSED)
    samples = []
    t0 = ticks_ms()
    started = False
    try:
        while True:
            t = ticks_diff(ticks_ms(), t0)
            if t >= 100 and not started:
                left.set_duty(duty)                     # SAFETY: the left wheel starts now
                started = True
            if t >= 100 + run_ms:
                break
            samples.append((t, ina.bus_voltage_v(), ina.current_a()))
    finally:
        left.brake()
        sleep_ms(300)
        left.coast()
    print("ms,bus_v,current_a,clipped")
    for t, v, a in samples:
        print("%d,%.3f,%.3f,%d" % (t, v, a, abs(a) >= CLIP_A))
    result = estimate_resistance(samples)
    if result is None:
        print("# not enough current change; check the INA219 wiring (VIN+ battery side, VIN- load side)")
    else:
        print("# rest %.3f V at %.2f A, loaded %.3f V at %.2f A -> source resistance %.0f mOhm"
              % (result["rest_v"], result["rest_a"], result["load_v"], result["load_a"], 1000 * result["r_ohm"]))
    return result


if __name__ == "__main__":
    run()
