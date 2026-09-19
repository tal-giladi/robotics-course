# karmel firmware — Raspberry Pi Pico 2 (MicroPython)

The microcontroller half of the robot. It drives the motors, counts encoder ticks, runs the
wheel-speed PID, reads the range sensor and battery, and talks to the Raspberry Pi over USB
using [protocol v1](../../README.md#the-pi--pico-protocol-v1).

> **Safety.** Anything that drives the motors is marked `SAFETY:` in the code. Run motor
> scripts with the wheels off the ground first, and keep a hand near the battery switch.

## Module map

Each file is small and introduced by one lesson. Read them in this order:

| File | What it does | Lesson |
|---|---|---|
| `examples/01_blink.py` | blink the on-board LED: MicroPython and USB work | 01.05 |
| `config.py` | pins and robot parameters (mirrors `labs/config/karmel.yaml`) | 01.08 |
| `motors.py` | DRV8874 in IN/IN mode: 20 kHz PWM, signed duty, brake vs coast, deadband compensation | 01.08 |
| `examples/02_motor_test.py` | spin each motor both ways (**wheels off the ground**) | 01.08 |
| `encoders.py` | quadrature decoding with a PIO state machine, plus an IRQ version for comparison | 01.09 |
| `examples/03_encoder_count.py` | print ticks while you turn the wheels by hand | 01.09 |
| `protocol.py` | parse and format protocol v1 lines, XOR checksum (runs on MicroPython and CPython) | 01.10 |
| `watchdog.py` | command watchdog (no command for 300 ms -> stop) and `machine.WDT` | 01.10 |
| `sensors.py` | US-100 ultrasonic, INA219 battery monitor, ADC voltage divider | 01.13, 01.14 |
| `vl53l1x.py` | VL53L1X time-of-flight driver (adapted, MIT; see its header) | 01.13 |
| `examples/04_ultrasonic.py` … `07_i2c_scan.py` | one sensor at a time | 01.13, 01.14 |
| `velocity.py` | wheel speed from ticks, PID with anti-windup and feedforward | 08.03-08.09 |
| `main.py` | asyncio main loop: commands, 100 Hz control, 50 Hz telemetry, sensors, status LED | 01.10, 03.04, 08.12 |

`main.py` keeps all decisions in a `Robot` class that takes the time as an argument, so
`labs/python/tests/test_firmware_logic.py` tests the firmware on a PC, with small stand-ins for
`machine`, `rp2` and `utime` in `labs/python/tests/mpshims/`.

## 1. Flash MicroPython (once)

1. Download the latest **Raspberry Pi Pico 2** firmware (`RPI_PICO2-<date>-v<version>.uf2`, the
   Arm build, not RISC-V) from the official page:
   <https://micropython.org/download/RPI_PICO2/>
2. Hold the **BOOTSEL** button while plugging the Pico into USB. A drive called `RP2350` appears.
3. Copy the `.uf2` file onto that drive. The Pico reboots into MicroPython.

(Already running MicroPython? `mpremote bootloader` or `machine.bootloader()` at the REPL
also enters the bootloader.)

## 2. Install `mpremote`

`mpremote` is MicroPython's official command-line tool
([docs](https://docs.micropython.org/en/latest/reference/mpremote.html)). On the Pi or your laptop:

```bash
pipx install mpremote        # or: pip install --user mpremote
mpremote connect list        # the Pico shows up, e.g. /dev/ttyACM0 or COM5
```

With one board attached, `mpremote` connects to it automatically. Otherwise start each
command with `connect /dev/ttyACM0` (Linux) or `connect COM5` (Windows).

## 3. Run and deploy

Run everything from this folder: `cd labs/firmware/pico`.

```bash
# Run a script from your computer without copying it (Ctrl-C stops it):
mpremote run examples/01_blink.py

# Scripts that import our modules: mount this folder on the Pico, then run.
# `mount` makes the Pico see this folder as its current directory, so `import motors` works
# and edits on your computer take effect on the next run - no copying.
mpremote mount . run examples/02_motor_test.py

# Deploy the firmware: copy the modules to the Pico's flash.
mpremote cp config.py protocol.py motors.py encoders.py velocity.py watchdog.py sensors.py vl53l1x.py :
mpremote cp main.py :        # main.py runs automatically at every power-up
mpremote reset               # restart now

# Look at what is on the Pico / remove the auto-start while developing:
mpremote ls
mpremote rm :main.py
```

Once `main.py` runs, the Pico speaks protocol v1 on its USB serial port. Try it from the Pi:

```bash
python labs/robot/battery_monitor.py --once --port /dev/ttyACM0
```

`mpremote` can still connect while `main.py` is running: it interrupts the program with
Ctrl-C first. **If you enabled the hardware watchdog** (`HARDWARE_WATCHDOG_MS` in `config.py`),
the interrupted program stops feeding it and the Pico resets a moment later — set it back
to `0` (or `mpremote rm :main.py`) while you are copying files.

## Without hardware

Every robot script can talk to a software Pico instead:

```bash
python -m robotlab.fake_pico --port 5760                         # emulated firmware + simulator
python labs/robot/drive_square.py --port socket://localhost:5760
python labs/robot/drive_square.py --fake                         # or both in one process
```

## Notes and limits

* **Why PIO for the encoders:** at full speed each wheel produces ~8,000 edges per second; Python
  interrupts can't keep up. The PIO program in `encoders.py` counts in hardware. It needs the
  B pin right after the A pin, and PIO block 1 to itself (state machines 4 and 5).
  MicroPython's `machine.Encoder` class is not available on the rp2 port.
* **US-100:** remove the jumper on the back for pulse (trigger/echo) mode. `time_pulse_us`
  blocks for up to 30 ms, so ranges are read at 10 Hz.
* **Timing:** `asyncio` gives a steady 100 Hz control loop in normal operation, but garbage
  collection and the ultrasonic read can delay a step by a few milliseconds. The PID uses the
  measured `dt`, so this changes accuracy slightly, not stability.
