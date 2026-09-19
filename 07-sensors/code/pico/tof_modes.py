# tof_modes.py - VL53L1X distance mode, timing budget, region of interest (ROI), signal and ambient rates.
#
# Lesson 07.04. MicroPython on the Pico 2. Extends the course driver labs/firmware/pico/vl53l1x.py
# (which only uses the power-on defaults: long mode, ~100 ms budget, full 16x16 SPAD array).
#
# The register addresses and magic values are the ones ST's VL53L1X Ultra Lite Driver (ULD) writes in
# VL53L1X_SetDistanceMode / SetTimingBudgetInMs / SetInterMeasurementInMs / SetROI, as found in
# SparkFun's port of it (https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library,
# src/st_src/vl53l1x_class.cpp; ST's code is BSD-3-Clause). ST publishes no register map, so treat
# these as "copied from the reference driver", not as something to derive.
#
# Run the sweep (from the firmware folder, so `config` and `vl53l1x` import from the mounted folder):
#
#     cd labs/firmware/pico
#     mpremote mount . run ../../../07-sensors/code/pico/tof_modes.py
#
# It prints one summary line per configuration: valid %, mean and sigma of the distance, signal and
# ambient photon rates, and the measured update rate. Aim the sensor at a target at a known distance
# and change the scene between runs (lights on/off, sunlit window, black cloth, glass).
#
# Use the class in your own script:
#     tof = VL53L1XModes(i2c)
#     tof.configure(mode="short", budget_ms=20, roi=(4, 4))
#     reading = tof.read_full(timeout_ms=200)    # (mm or None, status, signal_kcps, ambient_kcps)

try:
    from time import sleep_ms, ticks_diff, ticks_ms
except ImportError:                     # CPython tests
    from utime import sleep_ms, ticks_diff, ticks_ms

from vl53l1x import (REG_INTERRUPT_CLEAR, REG_RESULT_RANGE_STATUS, STATUS_RANGE_VALID,
                     STATUS_RANGE_VALID_MIN_CLIPPED, VL53L1X)

REG_PHASECAL_CONFIG_TIMEOUT_MACROP = 0x004B
REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI = 0x005E
REG_RANGE_CONFIG_VCSEL_PERIOD_A = 0x0060
REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI = 0x0061
REG_RANGE_CONFIG_VCSEL_PERIOD_B = 0x0063
REG_RANGE_CONFIG_VALID_PHASE_HIGH = 0x0069
REG_SYSTEM_INTERMEASUREMENT_PERIOD = 0x006C
REG_SD_CONFIG_WOI_SD0 = 0x0078
REG_SD_CONFIG_INITIAL_PHASE_SD0 = 0x007A
REG_ROI_CONFIG_USER_ROI_CENTRE_SPAD = 0x007F
REG_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE = 0x0080
REG_SYSTEM_MODE_START = 0x0087
REG_RESULT_OSC_CALIBRATE_VAL = 0x00DE

# (phasecal timeout, VCSEL period A, VCSEL period B, valid phase high, WOI SD0, initial phase SD0)
DISTANCE_MODES = {
    "short": (0x14, 0x07, 0x05, 0x38, 0x0705, 0x0606),   # ~1.3 m, most robust in ambient light
    "long": (0x0A, 0x0F, 0x0D, 0xB8, 0x0F0D, 0x0E0E),    # ~4 m in the dark (the power-on default)
}

# timing budget [ms] -> (TIMEOUT_MACROP_A_HI, TIMEOUT_MACROP_B_HI), per distance mode
TIMING_BUDGETS = {
    "short": {15: (0x001D, 0x0027), 20: (0x0051, 0x006E), 33: (0x00D6, 0x006E), 50: (0x01AE, 0x01E8),
              100: (0x02E1, 0x0388), 200: (0x03E1, 0x0496), 500: (0x0591, 0x05C1)},
    "long": {20: (0x001E, 0x0022), 33: (0x0060, 0x006E), 50: (0x00AD, 0x00C6),
             100: (0x01CC, 0x01EA), 200: (0x02D9, 0x02F8), 500: (0x048F, 0x04A4)},
}

OPTICAL_CENTER_SPAD = 199               # the centre of the 16x16 array (ST's default)
MIN_ROI_SPADS = 4                       # ST: ROIs smaller than 4x4 are not supported
VALID_STATUSES = (STATUS_RANGE_VALID, STATUS_RANGE_VALID_MIN_CLIPPED)


def roi_size_byte(width, height):
    """ROI size register: (height - 1) in the high nibble, (width - 1) in the low nibble."""
    return ((height - 1) << 4) | (width - 1)


class VL53L1XModes(VL53L1X):
    """The course VL53L1X driver plus the settings lesson 07.04 experiments with."""

    def _write_u32(self, register, value):
        self.i2c.writeto_mem(self.address, register, bytes([(value >> 24) & 0xFF, (value >> 16) & 0xFF,
                                                             (value >> 8) & 0xFF, value & 0xFF]), addrsize=16)

    def stop_ranging(self):
        self._write_u8(REG_SYSTEM_MODE_START, 0x00)

    def start_ranging(self):
        self._write_u8(REG_INTERRUPT_CLEAR, 0x01)
        self._write_u8(REG_SYSTEM_MODE_START, 0x40)

    def set_distance_mode(self, mode):
        phasecal, vcsel_a, vcsel_b, phase_high, woi, initial_phase = DISTANCE_MODES[mode]
        self._write_u8(REG_PHASECAL_CONFIG_TIMEOUT_MACROP, phasecal)
        self._write_u8(REG_RANGE_CONFIG_VCSEL_PERIOD_A, vcsel_a)
        self._write_u8(REG_RANGE_CONFIG_VCSEL_PERIOD_B, vcsel_b)
        self._write_u8(REG_RANGE_CONFIG_VALID_PHASE_HIGH, phase_high)
        self._write_u16(REG_SD_CONFIG_WOI_SD0, woi)
        self._write_u16(REG_SD_CONFIG_INITIAL_PHASE_SD0, initial_phase)
        self.mode = mode

    def set_timing_budget_ms(self, budget_ms):
        """Allowed: short 15/20/33/50/100/200/500 ms, long 20/33/50/100/200/500 ms. Set the mode first."""
        table = TIMING_BUDGETS[self.mode]
        if budget_ms not in table:
            raise ValueError("timing budget %s ms not allowed in %s mode: %s" % (budget_ms, self.mode, sorted(table)))
        a_hi, b_hi = table[budget_ms]
        self._write_u16(REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, a_hi)
        self._write_u16(REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, b_hi)
        self.budget_ms = budget_ms

    def set_inter_measurement_ms(self, period_ms):
        """Time between the starts of two measurements; must be >= the timing budget."""
        clock_pll = self._read_u16(REG_RESULT_OSC_CALIBRATE_VAL) & 0x3FF
        self._write_u32(REG_SYSTEM_INTERMEASUREMENT_PERIOD, int(clock_pll * period_ms * 1.075))

    def set_roi(self, width, height, center=OPTICAL_CENTER_SPAD):
        """Use only a width x height window of the 16x16 SPAD array: a narrower field of view."""
        width = max(MIN_ROI_SPADS, min(16, width))
        height = max(MIN_ROI_SPADS, min(16, height))
        if width > 10 or height > 10:
            center = OPTICAL_CENTER_SPAD    # what ST's driver does: big ROIs are always centred
        self._write_u8(REG_ROI_CONFIG_USER_ROI_CENTRE_SPAD, center)
        self._write_u8(REG_ROI_CONFIG_USER_ROI_REQUESTED_GLOBAL_XY_SIZE, roi_size_byte(width, height))
        self.roi = (width, height)

    def configure(self, mode="long", budget_ms=100, roi=(16, 16)):
        """Stop, apply all three settings, restart. Returns the new expected update period (ms)."""
        self.stop_ranging()
        self.set_distance_mode(mode)
        self.set_timing_budget_ms(budget_ms)
        self.set_inter_measurement_ms(budget_ms)
        self.set_roi(roi[0], roi[1])
        self.start_ranging()
        sleep_ms(2 * budget_ms)             # throw away the first measurement with the old settings
        self.read_full(timeout_ms=0)
        return budget_ms

    def read_full(self, timeout_ms=0):
        """(distance_mm or None, range_status, signal_kcps, ambient_kcps) of the next measurement.

        signal = photons per second back from the target; ambient = photons from everything else
        (sunlight, lamps). Their ratio decides whether a distance can be measured at all.
        Returns (None, None, 0, 0) if no measurement arrived within ``timeout_ms``.
        """
        start = ticks_ms()
        while not self.data_ready():
            if ticks_diff(ticks_ms(), start) >= timeout_ms:
                return None, None, 0, 0
            sleep_ms(2)
        data = self.i2c.readfrom_mem(self.address, REG_RESULT_RANGE_STATUS, 17, addrsize=16)
        self._write_u8(REG_INTERRUPT_CLEAR, 0x01)
        status = data[0] & 0x1F
        self.last_status = status
        ambient_kcps = ((data[7] << 8) | data[8]) * 8       # RESULT__AMBIENT_COUNT_RATE_MCPS_SD (0x0090)
        distance_mm = (data[13] << 8) | data[14]            # FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0 (0x0096)
        signal_kcps = ((data[15] << 8) | data[16]) * 8      # PEAK_SIGNAL_COUNT_RATE_..._SD0 (0x0098)
        return (distance_mm if status in VALID_STATUSES else None), status, signal_kcps, ambient_kcps


# (distance mode, timing budget ms, ROI) - the sweep lesson 07.04 asks you to run
SWEEP = (
    ("long", 100, (16, 16)),     # power-on default
    ("long", 33, (16, 16)),
    ("long", 200, (16, 16)),
    ("short", 20, (16, 16)),
    ("short", 100, (16, 16)),
    ("long", 100, (4, 4)),       # narrow field of view
)


def summarize(readings):
    """(valid fraction, mean mm, sigma mm, mean signal kcps, mean ambient kcps) of read_full() tuples."""
    valid = [r[0] for r in readings if r[0] is not None]
    n = len(readings)
    signal = sum(r[2] for r in readings) / n if n else 0
    ambient = sum(r[3] for r in readings) / n if n else 0
    if not valid:
        return 0.0, None, None, signal, ambient
    mean = sum(valid) / len(valid)
    var = sum((v - mean) ** 2 for v in valid) / (len(valid) - 1) if len(valid) > 1 else 0.0
    return len(valid) / n, mean, var ** 0.5, signal, ambient


def run_sweep(samples=50):
    import time
    from machine import I2C, Pin
    import config

    i2c = I2C(config.I2C_ID, sda=Pin(config.I2C_SDA), scl=Pin(config.I2C_SCL), freq=config.I2C_FREQ_HZ)
    tof = VL53L1XModes(i2c)
    print("mode   budget  roi     valid   mean_mm  sigma_mm  signal_kcps  ambient_kcps  rate_hz")
    for mode, budget, roi in SWEEP:
        tof.configure(mode, budget, roi)
        t0 = time.ticks_ms()
        readings = [tof.read_full(timeout_ms=2 * budget + 50) for _ in range(samples)]
        seconds = time.ticks_diff(time.ticks_ms(), t0) / 1000
        valid, mean, sigma, signal, ambient = summarize(readings)
        print("%-6s %4d ms  %2dx%-2d  %5.0f%%  %8s  %8s  %11d  %12d  %7.1f" % (
            mode, budget, roi[0], roi[1], valid * 100,
            "-" if mean is None else "%.1f" % mean, "-" if sigma is None else "%.1f" % sigma,
            signal, ambient, samples / seconds))


if __name__ == "__main__":
    run_sweep()
