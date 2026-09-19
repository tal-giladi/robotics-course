# Labs — the runnable code of the course

Lessons explain; `labs/` runs. Everything here is tested in CI without hardware.

```text
labs/
├── requirements.txt          Python deps for the non-ROS labs (numpy, matplotlib, pytest, …)
├── config/
│   └── karmel.yaml           the robot's physical parameters and pin map (single source)
├── python/                   `robotlab` — the course's pure-Python robotics library
│   ├── pyproject.toml
│   ├── robotlab/
│   │   ├── hal.py            hardware abstraction: DifferentialBase protocol + BaseState
│   │   ├── sim/              2D differential-drive simulator (motors, encoders, IMU, range, LiDAR, landmarks)
│   │   ├── protocol.py       Pi ↔ Pico serial protocol (encode/decode, checksum)
│   │   ├── serial_base.py    DifferentialBase over USB serial to the Pico firmware
│   │   ├── geometry.py       angles, SE(2) transforms
│   │   ├── config.py         load labs/config/karmel.yaml
│   │   └── …                 reference implementations added by modules (control, odometry, filters, planning…)
│   └── tests/
├── exercises/<lesson-id>/    auto-graded exercises: `python course.py check 09.04`
├── firmware/pico/            MicroPython firmware for the Raspberry Pi Pico 2
├── robot/                    scripts that run on the Raspberry Pi (teleop, drive square, battery monitor, …)
├── ros2_ws/src/              ROS 2 Jazzy workspace: karmel_description, karmel_gazebo, karmel_bringup, …
└── docker/                   dev containers (ROS 2 Jazzy + Gazebo Harmonic) for Windows/macOS/Linux
```

## Install (non-ROS labs)

```bash
cd labs
python -m venv .venv && source .venv/bin/activate      # Windows: .venv\Scripts\activate
pip install -r requirements.txt
pip install -e python
pytest python/tests
```

## Conventions shared by every lab

### Units and frames
SI units everywhere (m, rad, s, m/s, rad/s, V, A). Angles in radians, wrapped to (-π, π].
Robot frame: x forward, y left, z up (REP-103).

### The hardware abstraction (`robotlab.hal`)

Every robot program talks to a `DifferentialBase`. The simulator, the real robot over serial,
and test fakes all implement it — so the same odometry, PID or teleop code runs everywhere.

```python
@dataclass(frozen=True)
class BaseState:
    t: float                  # seconds (monotonic, robot clock)
    left_ticks: int           # cumulative signed encoder ticks
    right_ticks: int
    left_rad_s: float         # wheel angular velocity estimate
    right_rad_s: float
    battery_v: float | None
    range_m: float | None     # front distance sensor, None if no reading
    flags: int                # bitfield, see protocol

class DifferentialBase(Protocol):
    def set_wheel_duty(self, left: float, right: float) -> None: ...        # open loop, -1.0..1.0
    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None: ...  # closed loop on the MCU
    def stop(self) -> None: ...
    def read(self) -> BaseState: ...
    def close(self) -> None: ...
```

### The Pi ↔ Pico protocol (v1)

Line-based ASCII over USB CDC serial, one message per line, NMEA-style checksum so a human
can read it in a serial terminal and a machine can reject corrupted lines:

```text
<payload>*<hh>\n        hh = two uppercase hex digits = XOR of all payload bytes
```

Host → Pico

| Payload | Meaning |
|---|---|
| `H <seq>` | hello; Pico replies `I …` |
| `M <seq> <left> <right>` | open-loop duty, integers −1000…1000 (per-mille) |
| `V <seq> <left_mrad_s> <right_mrad_s>` | wheel velocity setpoints in milliradians/s (closed loop on the Pico) |
| `S <seq>` | stop (brake) |
| `R <seq>` | reset encoder counts |
| `P <seq> <key> <value>` | set parameter: `kp`, `ki`, `kd`, `ff`, `watchdog_ms`, `telemetry_hz` |

Pico → host

| Payload | Meaning |
|---|---|
| `I <firmware_version> <protocol_version>` | hello reply |
| `A <seq> OK` / `A <seq> ERR <reason>` | acknowledgement of a command |
| `T <ms> <lticks> <rticks> <l_mrad_s> <r_mrad_s> <batt_mV> <range_mm> <flags>` | telemetry at 50 Hz by default; `range_mm` = −1 when invalid |

Details: `seq` is 0…65535 and wraps; `batt_mV` and `range_mm` are −1 when there is no valid
reading; a line with a bad checksum gets no reply; error reasons are `bad_args`, `out_of_range`,
`unknown_command`, `unknown_param`; `ff` is a 0…1 scale on the motor-model feedforward; the
telemetry `ms` counts up from boot; the watchdog flag is set at boot until the first drive command.
The canonical implementation is `labs/python/robotlab/protocol.py`.

Flags: `1` watchdog tripped (no valid M/V command for `watchdog_ms`, default 300 ms → motors
stopped) · `2` low battery · `4` range sensor error · `8` velocity mode active.

Why this design and how to make it more robust (COBS framing, CRC) is taught in
[01.10](../01-first-robot/01.10-pi-pico-protocol.md) and [03.03](../03-robot-software/03.03-robust-serial-communication.md).

### Auto-graded exercises (`labs/exercises/<lesson-id>/`)

```text
labs/exercises/09.04/
├── README.md          what to implement (links back to the lesson)
├── student.py         starter code with `raise NotImplementedError  # TODO(student)`
├── solution.py        reference solution (don't peek until you've tried)
└── test_exercise.py   pytest tests
```

Tests import the implementation through `labs/exercises/conftest.py`:
`impl` fixture → `student.py`, or `solution.py` when `COURSE_USE_SOLUTION=1`
(`python course.py check 09.04 --solution`). Tests must run in seconds, without hardware or ROS.
