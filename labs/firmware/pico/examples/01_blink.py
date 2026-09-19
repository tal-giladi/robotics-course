# 01_blink.py - the "hello world" of microcontrollers: blink the on-board LED.
#
# Lesson 01.05. Run it from your computer or the Pi (no need to copy it to the Pico):
#     mpremote run examples/01_blink.py
# Stop with Ctrl-C.
#
# If the LED blinks, MicroPython is installed, USB works and your tools can reach the board.

import time

from machine import Pin

# GPIO 25 is the on-board LED of the Raspberry Pi Pico 2. (On a Pico 2 W the LED is wired to
# the wireless chip instead; there use Pin("LED", Pin.OUT).)
led = Pin(25, Pin.OUT)

print("Blinking the LED. Press Ctrl-C to stop.")
while True:
    led.toggle()          # on -> off -> on ...
    time.sleep_ms(500)    # 500 ms on + 500 ms off = one blink per second
