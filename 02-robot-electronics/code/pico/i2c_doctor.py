# i2c_doctor.py - more than a scan: idle-line check, stuck-bus recovery, and an error-rate
# stress test of every known device at 100 kHz, 400 kHz and (deliberately out of spec) 1 MHz.
#
# Lesson 02.04. MicroPython on the Pico 2 (no other files needed), from the repository root:
#     mpremote run 02-robot-electronics/code/pico/i2c_doctor.py
#
# Motors are not touched. Safe to run with the robot on the bench.

from machine import I2C, Pin

try:
    import time
    ticks_ms, ticks_diff = time.ticks_ms, time.ticks_diff
except AttributeError:                  # CPython test shims
    import utime
    ticks_ms, ticks_diff = utime.ticks_ms, utime.ticks_diff

SDA_PIN = 4
SCL_PIN = 5
I2C_ID = 0

# address: (name, register, n_bytes, addrsize, expected bytes). Registers chosen so that reading
# them has no side effects and returns a known constant.
KNOWN = {
    0x29: ("VL53L1X model id", 0x010F, 2, 16, b"\xea\xcc"),
    0x40: ("INA219 config (power-on 0x399F)", 0x00, 2, 8, b"\x39\x9f"),
}


def classify_idle(sda_level, scl_level):
    """Both I2C lines must idle HIGH (pulled up). Anything else is a hardware problem."""
    if sda_level and scl_level:
        return "ok"
    if not sda_level and not scl_level:
        return "both low: no pull-ups, sensors unpowered, or a short to GND"
    if not sda_level:
        return "SDA stuck low: a device is mid-transfer (try recovery) or SDA is shorted"
    return "SCL stuck low: a device holds the clock or SCL is shorted"


def recover_bus(sda_pin=SDA_PIN, scl_pin=SCL_PIN, pulses=9):
    """A target that was interrupted mid-byte keeps SDA low, waiting for clocks. Give it up to 9
    clock pulses (open-drain style: drive low, or release), then a STOP condition. Returns True
    if SDA is released."""
    sda = Pin(sda_pin, Pin.IN)
    for _ in range(pulses):
        if sda.value():
            break
        Pin(scl_pin, Pin.OUT, value=0)
        Pin(scl_pin, Pin.IN)            # released: the pull-up raises SCL
    # STOP = SDA rising while SCL is high
    Pin(sda_pin, Pin.OUT, value=0)
    Pin(scl_pin, Pin.IN)
    Pin(sda_pin, Pin.IN)
    return bool(Pin(sda_pin, Pin.IN).value())


def stress(i2c, address, register, n_bytes, addrsize, expected, reads=200):
    """Read one constant register many times. Returns (good, oserrors, wrong_data, ms_per_read)."""
    good = errors = wrong = 0
    t0 = ticks_ms()
    for _ in range(reads):
        try:
            data = i2c.readfrom_mem(address, register, n_bytes, addrsize=addrsize)
        except OSError:
            errors += 1
            continue
        if bytes(data) == expected:
            good += 1
        else:
            wrong += 1
    elapsed = ticks_diff(ticks_ms(), t0)
    return good, errors, wrong, elapsed / reads


def run(freqs=(100_000, 400_000, 1_000_000), reads=200):
    idle = classify_idle(Pin(SDA_PIN, Pin.IN).value(), Pin(SCL_PIN, Pin.IN).value())
    print("idle lines:", idle)
    if idle != "ok":
        print("trying bus recovery ->", "released" if recover_bus() else "still stuck: measure SDA/SCL with a meter")
    report = {}
    for freq in freqs:
        i2c = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=freq)
        found = i2c.scan()
        print("\n%7d Hz scan: %s" % (freq, " ".join("0x%02X" % a for a in found) or "nothing"))
        for address in found:
            if address not in KNOWN:
                print("   0x%02X unknown device (look it up before writing to it)" % address)
                continue
            name, register, n_bytes, addrsize, expected = KNOWN[address]
            good, errors, wrong, ms = stress(i2c, address, register, n_bytes, addrsize, expected, reads)
            report[(freq, address)] = (good, errors, wrong)
            print("   0x%02X %-32s good %3d  OSError %3d  wrong data %3d  %.2f ms/read"
                  % (address, name, good, errors, wrong, ms))
        missing = [a for a in KNOWN if a not in found]
        for address in missing:
            print("   0x%02X %s did NOT answer" % (address, KNOWN[address][0]))
    return report


if __name__ == "__main__":
    run()
