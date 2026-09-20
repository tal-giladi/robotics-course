# Troubleshooting: sensors

A sensor problem is always one of five things, and they are cheap to tell apart in this order:
**is it wired · does it answer · is the number physically plausible · is it in the right frame at the
right time · is it good enough for what you are asking of it.** The method is taught in
[07.11](../../07-sensors/07.11-sensor-troubleshooting.md); this page is the cross-lesson index of what you will actually meet.

> [!CAUTION]
> **Safety:** the 2D LiDARs used here are Class 1 eye-safe — say so, and still never disassemble the
> emitter or stare into it up close. → [07.07](../../07-sensors/07.07-2d-lidar.md)

---

## Any sensor on a bus

### Symptom: `i2c.scan()` returns an empty list

1. **Power and ground first.** Measure 3.3 V at the sensor's own pins, and confirm it shares ground
   with the Pico. → [02.04](../../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [FE.16](../../optional-foundations/electronics/FE.16-grounding.md)
2. **Pull-ups.** I²C is open-drain and needs them. Many breakout boards include them; several on one
   bus in parallel can also be too strong. Measure SDA and SCL at idle: both should sit at 3.3 V. →
   [FE.06](../../optional-foundations/electronics/FE.06-pull-up-pull-down.md), [02.04](../../02-robot-electronics/02.04-buses-i2c-spi-uart.md)
3. **SDA/SCL swapped** is the next most common, and it looks identical to "not connected".
4. **Address clash**: two devices at the same address answer over each other. Bring them up one at a
   time. → [02.04](../../02-robot-electronics/02.04-buses-i2c-spi-uart.md)
5. **A held-low bus** (scan works until a power cycle is needed) means a device is stuck mid-transfer.
   That is a wiring/noise problem, not a software one. → [02.04](../../02-robot-electronics/02.04-buses-i2c-spi-uart.md)
6. **Wrong bus number or pins** in the constructor. Check against the board pinout, not a tutorial. →
   [02.01](../../02-robot-electronics/02.01-datasheets-and-pinouts.md)

### Symptom: the scan finds the device, but reads fail sometimes

1. **Bus speed and cable length.** Long jumper leads on a moving robot are an antenna and a
   capacitor; drop to 100 kHz and shorten them. → [02.04](../../02-robot-electronics/02.04-buses-i2c-spi-uart.md), [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)
2. **Only while the motors run** → EMI and grounding, not I²C. → [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md), [FE.16](../../optional-foundations/electronics/FE.16-grounding.md)
3. **After an hour** → thermal or connector. Wiggle-test with the robot powered. → [02.07](../../02-robot-electronics/02.07-wiring-harness.md)

### Symptom: UART gives nothing, or gibberish

1. Nothing: TX/RX swapped, or no common ground. Gibberish: baud rate mismatch. Those two cover almost
   every case. → [FE.09](../../optional-foundations/electronics/FE.09-uart-i2c-spi.md), [02.04](../../02-robot-electronics/02.04-buses-i2c-spi-uart.md)
2. 5 V device driving a 3.3 V pin needs level shifting — in the direction that matters. → [02.05](../../02-robot-electronics/02.05-logic-levels-and-protection.md)

---

## Encoders

### Symptom: the count never changes

1. Power and ground at the encoder, then the signal pin with a meter while you turn the wheel slowly
   by hand — it should toggle. → [01.09](../../01-first-robot/01.09-reading-encoders.md)
2. If the pin toggles and the count does not, the counting code is not running or the pins are wrong.
   PIO on the Pico requires channel B on the pin immediately after A. → [01.09](../../01-first-robot/01.09-reading-encoders.md)
3. One wheel counting and not the other is almost always wiring; swap the two connectors and see
   whether the fault follows. → [07.02](../../07-sensors/07.02-wheel-encoders-in-depth.md)

### Symptom: the count drifts while the wheels are stationary, or jumps by hundreds

1. Drifting at rest = a floating input. Enable the pull-up. → [FE.06](../../optional-foundations/electronics/FE.06-pull-up-pull-down.md)
2. Jumping by hundreds while motors run = EMI on the encoder wires. Separate them from the motor
   leads, twist them, add the decoupling the lesson specifies. → [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)

### Symptom: counts are right at low speed and too low at high speed

You are missing edges.

1. **Interrupt-based counting cannot keep up.** Compute the edge rate: karmel at full speed is
   2464 counts per wheel revolution × several revolutions per second. → [01.09](../../01-first-robot/01.09-reading-encoders.md)
2. **Move the counting into PIO** (or hardware) instead of trying to make the ISR faster. →
   [01.09](../../01-first-robot/01.09-reading-encoders.md), [FC.05](../../optional-foundations/cpp-and-embedded/FC.05-interrupts-timers-pio.md)
3. Verify by driving a known distance at two speeds and comparing the totals. → [07.02](../../07-sensors/07.02-wheel-encoders-in-depth.md)

---

## Ultrasonic and ToF

### Symptom: the ultrasonic sensor always reports maximum range (or "no echo")

1. **Timing and wiring first**: a 5 V HC-SR04's echo pin must be divided down for a 3.3 V input, and
   the trigger pulse has a minimum width. → [01.13](../../01-first-robot/01.13-distance-sensors-stop.md), [02.05](../../02-robot-electronics/02.05-logic-levels-and-protection.md)
2. **Then physics.** A smooth surface at an angle reflects the ping away (specular reflection) and the
   sensor hears nothing. Test against a cardboard box square-on before blaming the code. →
   [07.03](../../07-sensors/07.03-ultrasonic-sensors.md)
3. **Phantom fixed-distance readings** are usually the sensor hearing the robot's own chassis: check
   the mounting and the beam cone. → [07.03](../../07-sensors/07.03-ultrasonic-sensors.md)
4. **Two sensors seeing things that are not there** is crosstalk: stagger the triggers. → [07.03](../../07-sensors/07.03-ultrasonic-sensors.md)

### Symptom: the ToF sensor reads, but the numbers jump around, or collapse in one room

1. **Ambient light** — especially sunlight through a window — raises the noise floor and shortens the
   range dramatically. Compare the same target in two rooms and you have your answer. → [07.04](../../07-sensors/07.04-tof-sensors.md)
2. **Target reflectivity**: a matte black object can be invisible to a ToF sensor that an ultrasonic
   sees easily. The two sensors fail on opposite surfaces, which is why karmel has both. →
   [07.04](../../07-sensors/07.04-tof-sensors.md), [07.03](../../07-sensors/07.03-ultrasonic-sensors.md)
3. **Cover glass** in front of the sensor reflects straight back; it needs a proper aperture. →
   [07.04](../../07-sensors/07.04-tof-sensors.md)
4. **Flicker between two values** points at the ROI/timing budget settings, not at the target. →
   [07.04](../../07-sensors/07.04-tof-sensors.md)
5. Before filtering, characterise: 200 samples at a fixed distance, then report mean and standard
   deviation. Filter to a specification, not to taste. → [07.01](../../07-sensors/07.01-sensor-fundamentals.md)

---

## IMU

### Symptom: heading drifts even after bias correction

1. **Re-measure the bias with the robot completely still**, for at least 10 s, and check it is stable
   between runs. If the bias itself moves run to run (temperature), you need to estimate it online. →
   [07.05](../../07-sensors/07.05-imu-fundamentals.md)
2. **Integrating a rate always drifts.** The fix is not a better gyro, it is an absolute reference:
   fuse with odometry and a localizer. → [10.07](../../10-localization/10.07-fusing-imu-and-odometry.md), [10.09](../../10-localization/10.09-amcl-in-ros2.md)
3. **The EKF making heading worse than raw odometry** is nearly always a covariance problem: the IMU's
   declared covariance is too optimistic, or its frame is wrong. → [07.06](../../07-sensors/07.06-imu-in-ros2.md), [10.07](../../10-localization/10.07-fusing-imu-and-odometry.md)

### Symptom: the accelerometer says the robot is tilted when it is flat

1. Check the mounting orientation against REP-103 before calibrating anything. → [07.06](../../07-sensors/07.06-imu-in-ros2.md),
   [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)
2. Then calibrate the accelerometer offsets; a few degrees of static error is normal uncalibrated. →
   [07.05](../../07-sensors/07.05-imu-fundamentals.md)
3. Roll and pitch that look right at rest and swing while driving is expected: the accelerometer
   cannot distinguish gravity from acceleration. That is what the complementary/Madgwick filter is
   for. → [07.06](../../07-sensors/07.06-imu-in-ros2.md), [10.07](../../10-localization/10.07-fusing-imu-and-odometry.md)

### Symptom: the compass heading swings wildly when the robot drives

The motors. Magnetometers on a robot with unshielded motor currents are close to useless without
hard/soft-iron calibration done *in place*, and often afterwards too. → [07.05](../../07-sensors/07.05-imu-fundamentals.md)

---

## LiDAR

### Symptom: the driver starts and immediately exits

1. Serial permissions and the port name, exactly as for the Pico. → [FL.05](../../optional-foundations/linux-and-tools/FL.05-permissions-users-groups.md), [07.07](../../07-sensors/07.07-2d-lidar.md)
2. Baud rate and model variant — the same product family ships several. → [07.07](../../07-sensors/07.07-2d-lidar.md)
3. Power: a LiDAR's motor draws more than a USB port may supply. → [02.02](../../02-robot-electronics/02.02-power-budget.md)

### Symptom: `/scan` publishes and RViz shows nothing

1. **"No transform from [laser]"** → the frame is not in the tree. → [TF and frames](tf-and-frames.md)
2. **QoS**: `/scan` is best-effort sensor data; a reliable subscriber will see nothing. → [04.11](../../04-ros2/04.11-qos.md)
3. **Fixed frame** in RViz set to something that does not exist. → [05.09](../../05-frames-and-transforms/05.09-rviz.md)

### Symptom: obstacles appear on top of the robot, or Nav2 refuses to move

1. The LiDAR sees parts of the robot itself. Set the driver's angular exclusion, or raise the mount. →
   [07.07](../../07-sensors/07.07-2d-lidar.md), [12.04](../../12-navigation/12.04-costmaps.md)
2. Check the `laser → base_link` transform: a sign error puts every obstacle behind you. →
   [05.06](../../05-frames-and-transforms/05.06-frame-conventions-rep103-rep105.md)

### Symptom: the LiDAR misses a chair, a table or a glass door

1. **A 2D scan is a plane.** A table top above the plane and legs that are thin between samples are
   both invisible. This is a sensor limitation, not a fault — it is why voxel layers and extra range
   sensors exist. → [07.07](../../07-sensors/07.07-2d-lidar.md), [11.01](../../11-slam/11.01-map-representations.md), [12.04](../../12-navigation/12.04-costmaps.md)
2. **Glass and mirrors** are specular: the beam does not come back. Mark them as keepout zones. →
   [07.07](../../07-sensors/07.07-2d-lidar.md), [12.10](../../12-navigation/12.10-recovery-and-navigation-safety.md)

---

## Cameras

### Symptom: the node starts but no images appear

1. `v4l2-ctl --list-devices` and check the device index; a USB camera that moved to `/dev/video2` will
   silently produce nothing. → [07.08](../../07-sensors/07.08-rgb-cameras.md)
2. In a container, the device must be passed through. → [FL.12](../../optional-foundations/linux-and-tools/FL.12-docker-for-robotics.md)
3. Check the requested resolution and format are supported by that camera. → [07.08](../../07-sensors/07.08-rgb-cameras.md)

### Symptom: images arrive but the viewer shows nothing

1. Compressed vs raw transport: `rqt_image_view` on `/image_raw` when only `/image_raw/compressed` is
   being published shows an empty pane. → [07.08](../../07-sensors/07.08-rgb-cameras.md)
2. QoS, again: sensor data profile. → [04.11](../../04-ros2/04.11-qos.md)

### Symptom: the image is dark, noisy, banded, or detections got worse today

1. **Exposure and gain are the variable you forgot.** Auto-exposure changes with the room, and with
   it, every threshold you tuned. Log the exposure with the image. → [07.08](../../07-sensors/07.08-rgb-cameras.md), [FCV.01](../../optional-foundations/computer-vision/FCV.01-how-cameras-form-images.md)
2. **Banding** is the rolling shutter beating against mains-frequency lighting (50 Hz in Israel). Set
   the anti-flicker/power-line frequency. → [07.08](../../07-sensors/07.08-rgb-cameras.md)
3. **Motion blur** at low light: shorten exposure and accept the noise, or add light. → [07.08](../../07-sensors/07.08-rgb-cameras.md)

### Symptom: detections are in the wrong place in 3D

1. `CameraInfo` missing or wrong (the wrong calibration for this camera). → [07.08](../../07-sensors/07.08-rgb-cameras.md), [13.06](../../13-computer-vision/13.06-lens-distortion-calibration.md)
2. Optical frame convention. → [TF and frames](tf-and-frames.md), [13.05](../../13-computer-vision/13.05-pinhole-camera-model.md)
3. The detection's stamp used with a transform at `now()`. → [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md)

### Symptom: the Pi's CPU is pinned and the control loop stutters

Image work at full resolution and frame rate will do this. Downscale, drop the rate, use compressed
transport, and keep the control loop in a different process from the perception. → [07.08](../../07-sensors/07.08-rgb-cameras.md),
[04.12](../../04-ros2/04.12-executors-and-callbacks.md), [13.15](../../13-computer-vision/13.15-vision-in-ros2.md)

---

## Depth cameras

### Symptom: a dense blob at the camera origin, or big holes in the depth image

1. **Blob at the origin** = invalid depths being treated as 0 instead of dropped. Filter NaN/0 before
   converting to a cloud. → [07.09](../../07-sensors/07.09-depth-cameras-and-point-clouds.md), [13.08](../../13-computer-vision/13.08-depth-and-3d-vision.md)
2. **Holes** are the physics: shiny, transparent, very dark, or out-of-range surfaces, plus stereo
   shadows at depth discontinuities. Expect them, and do not design a grasp pipeline that assumes a
   complete surface. → [07.09](../../07-sensors/07.09-depth-cameras-and-point-clouds.md)
3. **Sunlight destroys structured light** and degrades most ToF cameras. → [07.09](../../07-sensors/07.09-depth-cameras-and-point-clouds.md)
4. **Depth and colour not aligned** → the aligned stream is not enabled, or you are using the wrong
   extrinsics. → [07.09](../../07-sensors/07.09-depth-cameras-and-point-clouds.md)

---

## Timing and synchronisation

### Symptom: "the topic is publishing and my callback never fires"

QoS or executor, not the sensor. → [ROS 2 discovery, QoS and timing](ros2-discovery-and-qos.md)

### Symptom: stamp age is in the billions of milliseconds

Sim time vs wall clock. Set `use_sim_time` consistently. → [06.04](../../06-simulation/06.04-bridging-gazebo-ros2.md), [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md)

### Symptom: two sensors "disagree", and the error grows with speed

That is latency, not disagreement. Two measurements from different moments, compared as if
simultaneous, diverge in proportion to how fast you are moving. Synchronise on header stamps with
`message_filters`, and measure each sensor's actual latency. → [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md), [07.01](../../07-sensors/07.01-sensor-fundamentals.md)

## Where to go next

- The systematic sensor-debugging method, end to end: [07.11](../../07-sensors/07.11-sensor-troubleshooting.md)
- Characterising a sensor (bias, noise, latency, resolution) before trusting it: [07.01](../../07-sensors/07.01-sensor-fundamentals.md)
- Timestamps, synchronisation and covariance conventions: [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md)
- Noise, grounding and motor EMI: [02.06](../../02-robot-electronics/02.06-noise-grounding-emi.md)
- When the data is fine and the estimate is not: [SLAM and localization](slam-localization.md)
