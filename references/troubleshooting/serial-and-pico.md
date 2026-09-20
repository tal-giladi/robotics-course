# Troubleshooting: the serial link to the Pico

The Pi ↔ Pico link is the one piece of infrastructure every later module sits on. Its failures are
distinctive and its diagnostics are cheap: the protocol has sequence numbers, acknowledgements, a
checksum, a statistics counter and a watchdog, and all five tell you something different. →
[01.10](../../01-first-robot/01.10-pi-pico-protocol.md), [03.03](../../03-robot-software/03.03-robust-serial-communication.md)

```text
  your code ──▶ pyserial ──▶ /dev/karmel (udev) ──▶ USB CDC ──▶ Pico main.py ──▶ motors
      │              │             │                    │            │
   bad_args      bad_lines     not found /         disconnect    watchdog
                 checksum      permission denied                 (300 ms)
```

---

### Symptom: `/dev/ttyACM0` does not exist, or `find_pico.py` finds nothing

1. **`ls /dev/ttyACM*` and `dmesg | tail`** the moment you plug it in. No kernel message at all means
   cable or power, not software — try a different USB cable, because many are charge-only. →
   [01.05](../../01-first-robot/01.05-pico-microcontroller-setup.md)
2. **Check it is not sitting in BOOTSEL mode**: a Pico held in BOOTSEL enumerates as a mass-storage
   drive, not a serial port. Unplug, replug without holding the button. → [01.05](../../01-first-robot/01.05-pico-microcontroller-setup.md)
3. **Check there is firmware on it at all.** A Pico with no `main.py` enumerates but says nothing. →
   [FC.02](../../optional-foundations/cpp-and-embedded/FC.02-micropython-on-pico.md)
4. **In WSL or Docker, the device has to be passed through.** The Pi sees it; your container does
   not. → [FL.02](../../optional-foundations/linux-and-tools/FL.02-installing-ubuntu.md), [FL.12](../../optional-foundations/linux-and-tools/FL.12-docker-for-robotics.md)
5. **If it exists but the number moves between `ttyACM0` and `ttyACM1`**, that is expected — install
   the udev rule and use `/dev/karmel`. → [03.03](../../03-robot-software/03.03-robust-serial-communication.md), [FL.05](../../optional-foundations/linux-and-tools/FL.05-permissions-users-groups.md)

### Symptom: `Permission denied: '/dev/ttyACM0'`

1. `ls -l /dev/ttyACM0` — the group is `dialout`. Add yourself: `sudo usermod -aG dialout $USER`. →
   [FL.05](../../optional-foundations/linux-and-tools/FL.05-permissions-users-groups.md)
2. **Log out and back in.** Group membership is applied at login; `groups` in your current shell will
   still show the old set until you do. This is the step people skip.
3. Do not "fix" it with `sudo` or `chmod 666`: it will come back after every replug, and running ROS 2
   nodes as root creates a new set of problems. → [FL.05](../../optional-foundations/linux-and-tools/FL.05-permissions-users-groups.md)

### Symptom: `AckTimeoutError: no hello reply from /dev/ttyACM0`

The port opened; nobody answered. Bisect at the framing layer.

1. **Is something else holding the port?** `mpremote`, Thonny, a stale node or a previous run. One
   reader at a time. → [01.05](../../01-first-robot/01.05-pico-microcontroller-setup.md)
2. **Is the firmware running?** Open the REPL and look for its banner. A firmware that crashed at
   import is silent but the port still opens. → [FC.02](../../optional-foundations/cpp-and-embedded/FC.02-micropython-on-pico.md)
3. **Is the baud rate the same on both sides?** Wrong baud gives you either silence or plausible
   garbage, never an error. → [FE.09](../../optional-foundations/electronics/FE.09-uart-i2c-spi.md)
4. **Send one command by hand** and read the raw bytes. Seeing bytes arrive but not parse is a framing
   problem; seeing nothing is a firmware problem. → [03.03](../../03-robot-software/03.03-robust-serial-communication.md)
5. **Check the `main.py` boot path.** A Pico that only runs your code from the REPL will not answer
   after a cold plug. → [FC.02](../../optional-foundations/cpp-and-embedded/FC.02-micropython-on-pico.md)

### Symptom: `stats.bad_lines` keeps increasing

Corruption, and the rate tells you the cause.

1. **A few per thousand, rising with motor current** → electrical noise on the USB or ground.
   Shorten and separate the cable, add a ferrite, check the common ground. → [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md), [FE.16](../../optional-foundations/electronics/FE.16-grounding.md)
2. **A burst after every reconnect** → you are reading a partial frame left in the buffer. Flush on
   connect and resynchronise on the delimiter. → [03.03](../../03-robot-software/03.03-robust-serial-communication.md)
3. **Constant, at a steady rate** → a framing or checksum mismatch between the two implementations,
   not noise. Print one rejected line and compare it byte for byte with what the sender produced. →
   [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)
4. **Everything after one bad byte** → you have no framing worth the name. COBS or a length prefix
   makes resynchronisation immediate. → [03.03](../../03-robot-software/03.03-robust-serial-communication.md)

### Symptom: the motors stutter or stop every few hundred milliseconds

The 300 ms watchdog is expiring. Your commands are late, not absent.

1. **Measure your actual command period**, including the tail: log the max and the 99th percentile,
   not the mean. A loop that "runs at 50 Hz" and occasionally takes 400 ms trips the watchdog and
   averages fine. → [03.04](../../03-robot-software/03.04-timing-and-concurrency.md)
2. **Look for the blocking call** in the loop: a synchronous service call, a file write, an image
   decode, a `print` to a slow terminal. → [03.04](../../03-robot-software/03.04-timing-and-concurrency.md), [04.12](../../04-ros2/04.12-executors-and-callbacks.md)
3. **Check the commands are being *accepted***: a rejected command does not feed the watchdog. Watch
   for `ERR` acknowledgements. → [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)
4. **Do not lengthen the watchdog to make the symptom go away.** It is the only thing between a
   crashed script and a robot that keeps driving. → [01.10](../../01-first-robot/01.10-pi-pico-protocol.md), [SAFETY.md](../../SAFETY.md)

### Symptom: `A <seq> ERR bad_args` for a command that looks fine

1. **Print the exact bytes you sent**, not the Python values. Trailing whitespace, a locale decimal
   comma, or a float formatted as `1e-05` are all common. → [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)
2. **Check units and ranges.** The firmware validates; a duty of 50 when it expects 0.5 is rejected
   correctly. → [03.06](../../03-robot-software/03.06-configuration-and-calibration.md)
3. **Check the sequence number is monotonic.** A restarted sender that resets to 1 can be rejected as
   a replay. → [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)

### Symptom: the watchdog never trips, even when you stop sending

1. **Something is still sending.** A leftover process, or a node you thought you stopped. Check with
   `ls /proc/*/fd | grep ttyACM` or simply unplug. → [FL.06](../../optional-foundations/linux-and-tools/FL.06-processes-systemd.md)
2. **The watchdog is being fed by the wrong event** — e.g. reset by *any* received byte, including
   telemetry requests, instead of by a valid motion command. Read the firmware. → [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)
3. Test it deliberately, every time you change the firmware: kill the sender and time how long the
   wheels keep turning. It should be under 300 ms. → [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)

### Symptom: after unplugging, the program hangs instead of reconnecting

1. A blocking read with no timeout will wait for ever. Set one, always. → [03.03](../../03-robot-software/03.03-robust-serial-communication.md)
2. Implement reconnect as a state machine with backoff, not a `while True: try: ...` that spins the
   CPU. → [03.03](../../03-robot-software/03.03-robust-serial-communication.md)
3. Verify by unplugging the cable mid-run as a test, not as an accident. → [03.08](../../03-robot-software/03.08-testing-robot-software.md)

### Symptom: the robot twitches briefly after `stop()`

1. The last command was still in the buffer when you closed the port. Send a stop, wait for its
   acknowledgement, then close. → [03.03](../../03-robot-software/03.03-robust-serial-communication.md)
2. Make the HAL's context manager guarantee it, so the robot stops even when the script raises. →
   [03.05](../../03-robot-software/03.05-hardware-abstraction.md)

### Symptom: `mpremote` hangs, times out, or prints `could not enter raw repl`

1. Another program has the port. Close it. → [01.05](../../01-first-robot/01.05-pico-microcontroller-setup.md)
2. The firmware is in a tight loop with no yield, so it never services the REPL. Reset it with
   BOOTSEL or a hard reset, then fix the loop. → [FC.02](../../optional-foundations/cpp-and-embedded/FC.02-micropython-on-pico.md)
3. `mpremote: command not found` is a PATH/venv problem, not a Pico problem. → [FL.10](../../optional-foundations/linux-and-tools/FL.10-python-environments-ubuntu.md)

## Where to go next

- The protocol design itself (framing, sequence, checksum, watchdog): [01.10](../../01-first-robot/01.10-pi-pico-protocol.md)
- Robust serial in Python, reconnects and udev rules: [03.03](../../03-robot-software/03.03-robust-serial-communication.md)
- Loop timing, jitter and why the tail matters: [03.04](../../03-robot-software/03.04-timing-and-concurrency.md)
- Noise and grounding: [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md), [FE.16](../../optional-foundations/electronics/FE.16-grounding.md)
- The same link exposed as a ROS 2 node: [04.16](../../04-ros2/04.16-your-robot-as-ros2-node.md)
