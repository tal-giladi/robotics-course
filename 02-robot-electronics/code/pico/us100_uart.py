# us100_uart.py - read the US-100 in UART mode: distance (temperature-compensated by the sensor)
# and its temperature, with timeouts, byte-count checks and error statistics.
#
# Lesson 02.04. MicroPython on the Pico 2, from the repository root:
#     mpremote run 02-robot-electronics/code/pico/us100_uart.py
#
# Wiring for UART mode (JUMPER FITTED on the back of the US-100):
#     US-100 VCC       -> Pico 3V3          US-100 GND -> Pico GND
#     US-100 Trig/TX   -> Pico GP8 (UART1 TX)     <- yes, TX to "TX": the silkscreen names the
#     US-100 Echo/RX   -> Pico GP9 (UART1 RX)        Pico-side function, see lesson 02.04
# GP8/GP9 are free on karmel (karmel.yaml). The pulse-mode wiring on GP14/GP15 is NOT a UART pin pair.

from machine import UART, Pin

try:
    import time
    sleep_ms, ticks_ms, ticks_diff = time.sleep_ms, time.ticks_ms, time.ticks_diff
except AttributeError:                  # CPython test shims
    import utime
    sleep_ms, ticks_ms, ticks_diff = utime.sleep_ms, utime.ticks_ms, utime.ticks_diff

CMD_DISTANCE = 0x55
CMD_TEMPERATURE = 0x50
MIN_MM = 20
MAX_MM = 4500


def parse_distance(data):
    """Two bytes, high byte first, in millimetres. None for a missing, short or out-of-range reply."""
    if data is None or len(data) != 2:
        return None
    mm = (data[0] << 8) | data[1]
    if mm < MIN_MM or mm > MAX_MM:
        return None
    return mm


def parse_temperature(data):
    """One byte; the sensor adds 45 so that negative temperatures fit in a byte."""
    if data is None or len(data) != 1:
        return None
    return data[0] - 45


class US100Uart:
    def __init__(self, uart_id=1, tx=8, rx=9, wait_ms=100):
        # 9600 baud, 8 data bits, no parity, 1 stop bit; wait for the reply at most wait_ms
        self.uart = UART(uart_id, baudrate=9600, bits=8, parity=None, stop=1,
                         tx=Pin(tx), rx=Pin(rx), timeout=wait_ms)
        self.wait_ms = wait_ms

    def _transact(self, command, n_bytes):
        while self.uart.any():          # drop stale bytes (e.g. a late reply to the last command)
            self.uart.read()
        self.uart.write(bytes([command]))
        sleep_ms(self.wait_ms)
        return self.uart.read(n_bytes)

    def distance_mm(self):
        return parse_distance(self._transact(CMD_DISTANCE, 2))

    def temperature_c(self):
        return parse_temperature(self._transact(CMD_TEMPERATURE, 1))


def run(samples=50):
    sensor = US100Uart()
    temp = sensor.temperature_c()
    print("sensor temperature:", "no reply (jumper fitted? TX/RX as in the header?)" if temp is None else "%d C" % temp)
    ok = bad = 0
    readings = []
    t0 = ticks_ms()
    for _ in range(samples):
        mm = sensor.distance_mm()
        if mm is None:
            bad += 1
        else:
            ok += 1
            readings.append(mm)
    seconds = ticks_diff(ticks_ms(), t0) / 1000
    print("%d valid, %d invalid in %.1f s (%.1f Hz)" % (ok, bad, seconds, samples / seconds))
    if readings:
        readings.sort()
        print("min %d  median %d  max %d mm" % (readings[0], readings[len(readings) // 2], readings[-1]))
    return ok, bad


if __name__ == "__main__":
    run()
