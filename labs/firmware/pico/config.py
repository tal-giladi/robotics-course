# config.py - pins and robot parameters for the karmel firmware (Raspberry Pi Pico 2).
#
# Lessons: 01.08 (motor pins), 01.09 (encoder pins), 01.13 (range sensors), 01.14 (battery),
#          08.05-08.09 (PID defaults).
#
# This file MIRRORS labs/config/karmel.yaml. MicroPython has no YAML parser, so the values are
# copied here by hand. If you change a pin or a wheel size in karmel.yaml, change it here too -
# labs/python/tests/test_firmware_logic.py checks that the two files agree.
#
# Everything is a plain module-level constant so any other module can do `import config`
# and read `config.TICKS_PER_WHEEL_REV`. UPPER_CASE = "set once, never changed at runtime".

FIRMWARE_VERSION = "0.1.0"

# --- Pins (karmel.yaml: pins) ---------------------------------------------------------------
# Two Pololu DRV8874 carriers in IN/IN mode (PMODE tied high, nSLEEP tied high).
# GPIO 2/3 are PWM channels 1A/1B, GPIO 6/7 are 3A/3B: each motor's two inputs share one
# PWM "slice", so they always run at the same frequency.
MOTOR_LEFT_IN1 = 2
MOTOR_LEFT_IN2 = 3
MOTOR_RIGHT_IN1 = 6
MOTOR_RIGHT_IN2 = 7
MOTOR_LEFT_IPROPI_ADC = 27      # optional current sense, not used by main.py

ENCODER_LEFT_A = 10
ENCODER_LEFT_B = 11             # PIO reads A and B as two ADJACENT pins: B must be A + 1
ENCODER_RIGHT_A = 12
ENCODER_RIGHT_B = 13

I2C_ID = 0                      # GPIO 4/5 belong to the I2C0 controller
I2C_SDA = 4
I2C_SCL = 5
I2C_FREQ_HZ = 400_000           # "fast mode"; VL53L1X and INA219 both support it

ULTRASONIC_TRIG = 14            # US-100 in pulse mode (jumper removed), powered at 3.3 V
ULTRASONIC_ECHO = 15
BATTERY_ADC = 26                # ADC0, through a 100k / 22k divider
STATUS_LED = 25                 # on-board LED of the (non-W) Pico 2

# --- Drive (karmel.yaml: drive) ---------------------------------------------------------------
WHEEL_RADIUS_M = 0.045
WHEEL_SEPARATION_M = 0.200
ENCODER_CPR_MOTOR = 11          # hall pulses per motor revolution, one channel
GEAR_RATIO = 56.0
QUADRATURE_MULTIPLIER = 4       # we count both edges of both channels
TICKS_PER_WHEEL_REV = int(ENCODER_CPR_MOTOR * GEAR_RATIO * QUADRATURE_MULTIPLIER)  # 2464
MAX_WHEEL_SPEED_RAD_S = 17.0
DUTY_DEADBAND = 0.12

# The two motors face opposite directions on the chassis, so the same electrical polarity
# turns one wheel forward and the other backward. Flip these (instead of re-wiring) until
# examples/02_motor_test.py moves both wheels FORWARD for a positive duty and
# examples/03_encoder_count.py shows counts going UP when a wheel rolls forward.
MOTOR_LEFT_REVERSED = False
MOTOR_RIGHT_REVERSED = True
ENCODER_LEFT_REVERSED = False
ENCODER_RIGHT_REVERSED = True

# --- Motor driver -------------------------------------------------------------------------------
# 20 kHz PWM: above human hearing (a 1 kHz PWM makes the motors whine), and fast compared with
# the motor's electrical time constant, so the current - and the torque - stays smooth
# instead of pulsing. Much higher frequencies waste more power switching the H-bridge.
PWM_FREQ_HZ = 20_000

# --- Control loop (lessons 08.03-08.09) ---------------------------------------------------------
CONTROL_HZ = 100                # velocity estimate + PID rate
VELOCITY_FILTER_ALPHA = 0.5     # low-pass on the wheel-speed estimate: 1.0 = no filtering

# Default velocity PID gains. Units: duty per (rad/s) of error.
# `ff` scales the model-based feedforward: 1.0 = full inversion of the motor model
# (deadband + max speed), 0.0 = pure PID. Same meaning as the simulator's controller.
VELOCITY_KP = 0.02
VELOCITY_KI = 0.3
VELOCITY_KD = 0.0
VELOCITY_FF = 1.0

# --- Serial protocol (karmel.yaml: serial) ------------------------------------------------------
WATCHDOG_MS = 300               # no M/V command for this long -> motors stop
TELEMETRY_HZ = 50
MAX_LINE_LENGTH = 120           # longer lines are garbage and are discarded

# Hardware watchdog (machine.WDT): resets the whole Pico if the main loop hangs.
# 0 = disabled. Once started it can NOT be stopped - if you press Ctrl-C to get to the REPL,
# the board resets a moment later, which makes copying files with mpremote frustrating.
# Enable it (e.g. 2000) once your firmware is stable. Maximum on the RP2350: 8388 ms.
HARDWARE_WATCHDOG_MS = 0

# --- Sensors (karmel.yaml: sensors, battery) ----------------------------------------------------
SENSOR_HZ = 10                  # range + battery update rate
RANGE_SENSOR = "vl53l1x"        # "vl53l1x", "us100" or "none"
RANGE_MAX_MM = 4000

US100_TIMEOUT_US = 30_000       # ~5 m round trip at 343 m/s; blocks the loop at most this long
SPEED_OF_SOUND_M_S = 343.0      # at 20 C

BATTERY_SENSOR = "ina219"       # "ina219", "adc" or "none"
INA219_ADDRESS = 0x40
INA219_SHUNT_OHMS = 0.1
BATTERY_DIVIDER_TOP_OHMS = 100_000
BATTERY_DIVIDER_BOTTOM_OHMS = 22_000
ADC_REFERENCE_V = 3.3

BATTERY_CELLS = 3
BATTERY_LOW_WARNING_V = 10.5
BATTERY_CUTOFF_V = 9.9
