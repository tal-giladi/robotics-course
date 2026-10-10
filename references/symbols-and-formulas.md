# Symbols and formulas

The letters and formulas you use again and again on karmel, the course robot, all on one page: wheels, encoders, driving, odometry, motors, PID, battery and stopping. Each row links to the lesson that derives the formula and tries it with real numbers. The karmel column uses the values from [`labs/config/karmel.yaml`](../labs/config/karmel.yaml).

- **The same letter can mean different things.** $k$ is a gain in one lesson and a time step in another; $\tau$ is a motor time constant here and a torque in physics. This page gives the meaning used for karmel. The lesson always says which one it means.
- **Two names for the same thing.** The distance between the wheels is $b$ in modules 01 and 08 and $L$ in module 09. Ticks per wheel revolution are written $N$, $N_{ticks}$ or $N_{rev}$. They are the same number.
- **Units are SI**: metres, radians, seconds, volts, amps. "—" means no unit.
- **Words, not letters?** Terms like *duty cycle* or *deadband* are in the [Glossary](glossary.md).

> [!TIP]
> **Ask your teacher:** "Take karmel driving at 0.3 m/s for 2 seconds and walk me through every formula on this page that applies, with numbers."

**Jump to:** [Symbols](#symbols) · [Encoders and wheel speed](#encoders-and-wheel-speed) · [Driving](#driving-differential-drive) · [Odometry](#odometry) · [Motor and PWM](#motor-and-pwm) · [PID control](#pid-control) · [Battery and power](#battery-and-power) · [Stopping and safety](#stopping-and-safety) · [Distance sensors](#distance-sensors)

---

## Symbols

| Symbol | Meaning | Unit | karmel | Lesson |
|---|---|---|---|---|
| $r$ | wheel radius | m | 0.045 | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $b$ (also $L$) | track width: distance between the two wheel centres | m | 0.200 | [01.11](../01-first-robot/01.11-drive-and-rotate.md), [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $N$ ($N_{ticks}$, $N_{rev}$) | encoder ticks per wheel revolution | ticks | 2464 | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $N_{cpr}$ (PPR) | encoder pulses per *motor* revolution, one channel | pulses | 11 | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $G$ | gear ratio: motor turns per wheel turn | — | 56 | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $n$ | tick count read from the encoder | ticks | | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $\Delta n$ | ticks counted since the last reading | ticks | | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $d_{tick}$ | distance the wheel rolls per tick | m | 0.000115 | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $\phi$ | angle the wheel has turned | rad | | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $\omega$ (wheel), $\omega_L$, $\omega_R$ | wheel angular speed: how fast a wheel turns | rad/s | max 17 | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $\omega$ (robot), $\omega_z$ | robot turn rate (yaw rate) | rad/s | limit 2.5 | [01.11](../01-first-robot/01.11-drive-and-rotate.md), [08.10](../08-control/08.10-straight-line-heading-control.md) |
| $\Delta\omega$ | speed resolution: the smallest speed change the encoder can see in one window | rad/s | 0.13 at 50 Hz | [01.09](../01-first-robot/01.09-reading-encoders.md), [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\hat\omega$ | *estimated* wheel speed (the hat means "estimate") | rad/s | | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $v$ | robot forward speed | m/s | limit 0.5 | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $v_L$, $v_R$ | left / right wheel rim speed | m/s | max 0.77 | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $d_L$, $d_R$ | distance rolled by the left / right wheel | m | | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $s$, $\Delta s$ | distance travelled by the robot centre (in one step) | m | | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| $R$ | turning radius: radius of the circle the robot drives | m | | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $x$, $y$ | robot position in the world | m | | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\theta$ | robot heading: which way the nose points, from the $x$ axis | rad | | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\Delta\theta$ | heading change (in one step, or over a turn) | rad | | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $\Delta t$, $T$ | time step / loop period | s | 0.02 (50 Hz) | [01.09](../01-first-robot/01.09-reading-encoders.md), [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $f$ | loop rate or frequency | Hz | 50 | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $k$ (index) | step number in a loop: $x_k$ is the value at step $k$ | — | | [08.05](../08-control/08.05-integral-control.md) |
| $u$ | command to the motor: PWM duty, −1…1 | — | | [08.02](../08-control/08.02-motor-step-response.md) |
| $D$ | PWM duty cycle: fraction of each period the switch is on | — | | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $u_{db}$ | deadband: below this duty the motor does not turn | — | 0.12 | [08.02](../08-control/08.02-motor-step-response.md) |
| $K$ | motor gain: steady wheel speed per unit of duty above the deadband | rad/s | | [08.02](../08-control/08.02-motor-step-response.md) |
| $\tau$ | motor time constant: time to reach 63 % of the final speed | s | 0.08 | [08.02](../08-control/08.02-motor-step-response.md) |
| $r_{sp}$ (setpoint) | the value the controller should reach (in control lessons $r$ means setpoint, not radius) | as controlled | | [08.04](../08-control/08.04-proportional-control.md) |
| $y$ | the measured value the controller compares with the setpoint | as controlled | | [08.04](../08-control/08.04-proportional-control.md) |
| $e$ | error: setpoint minus measurement | as controlled | | [08.04](../08-control/08.04-proportional-control.md) |
| $k_p$, $k_i$, $k_d$ ($K_p$…) | proportional, integral and derivative gains | duty per unit of error (×s, /s) | | [08.04](../08-control/08.04-proportional-control.md), [08.07](../08-control/08.07-pid-mathematics.md) |
| $I$ | integral term: accumulated error × $k_i$ | — (duty) | | [08.05](../08-control/08.05-integral-control.md) |
| $u_{ff}$ | feedforward: the duty the motor model predicts, before feedback | — | | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $\alpha$ | smoothing factor of a low-pass filter, 0…1 | — | | [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $V$, $V_{bat}$ | voltage; battery voltage | V | 9.9–12.6 | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $I$ (current) | electric current | A | | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md) |
| $P$ | power | W | | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md) |
| $C$ | battery capacity | Ah | 3.5 | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $\eta$ | efficiency, 0…1 | — | | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $a$ | acceleration (braking) | m/s² | limit 1.0 | [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $L$ (latency) | total delay from sensing to the motors reacting | s | | [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $c$ | speed of sound | m/s | 343 at 20 °C | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $\sigma$ | standard deviation: typical size of the noise | as measured | | [08.03](../08-control/08.03-wheel-speed-estimation.md) |

---

## Encoders and wheel speed

| Formula | What it gives | karmel | Lesson |
|---|---|---|---|
| $N = N_{cpr} \cdot G \cdot 4$ | ticks per wheel revolution (×4: both edges of both channels) | 11·56·4 = 2464 | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $d_{tick} = \dfrac{2\pi r}{N}$ | distance per tick | 0.283 / 2464 = 0.115 mm | [01.09](../01-first-robot/01.09-reading-encoders.md) |
| $d = \dfrac{2\pi r}{N}\,\Delta n$ | distance a wheel rolled from its ticks | | [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $\omega = \dfrac{2\pi}{N} \cdot \dfrac{\Delta n}{\Delta t}$ | wheel speed from ticks counted in a window | | [01.09](../01-first-robot/01.09-reading-encoders.md), [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $\Delta\omega = \dfrac{2\pi}{N\,\Delta t}$ | speed resolution: one tick more or less in the window | 0.13 rad/s at 50 Hz | [01.09](../01-first-robot/01.09-reading-encoders.md), [08.03](../08-control/08.03-wheel-speed-estimation.md) |
| $v = \omega\,r$ | rim speed from wheel speed | 17 · 0.045 = 0.77 m/s | [03.06](../03-robot-software/03.06-configuration-and-calibration.md) |
| $\omega = 2\pi \cdot \dfrac{\text{rpm}}{60}$ | rpm to rad/s | 205 rpm = 21.5 rad/s | [FP.01](../optional-foundations/physics/FP.01-kinematics-units.md) |
| $y_k = y_{k-1} + \alpha\,(x_k - y_{k-1})$ | low-pass filter (EMA) to smooth a noisy speed | | [08.03](../08-control/08.03-wheel-speed-estimation.md) |

## Driving (differential drive)

| Formula | What it gives | karmel | Lesson |
|---|---|---|---|
| $v = \dfrac{v_L + v_R}{2}$ | robot speed from the two wheel speeds | | [01.11](../01-first-robot/01.11-drive-and-rotate.md), [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\omega = \dfrac{v_R - v_L}{b}$ | robot turn rate from the two wheel speeds | | [01.11](../01-first-robot/01.11-drive-and-rotate.md), [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\omega_L = \dfrac{v - \omega b/2}{r}, \quad \omega_R = \dfrac{v + \omega b/2}{r}$ | inverse: wheel speeds for a wanted $v$ and $\omega$ (`cmd_vel`) | | [09.02](../09-odometry/09.02-inverse-kinematics-cmd-vel.md) |
| $R = v/\omega$ | turning radius | | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $\omega = 2 v_R / b$ (with $v_L = -v_R$) | turn rate when rotating in place | max 2·0.77/0.2 = 7.7 rad/s | [01.11](../01-first-robot/01.11-drive-and-rotate.md) |
| $d_L = d_R = \theta\,b/2$ | arc each wheel rolls to turn $\theta$ in place | 90°: 0.157 m | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $\Delta\theta = \dfrac{d_R - d_L}{b}$ | heading change from the two wheel distances | | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $b_{true} = b \cdot 90° / \theta_{measured}$ | calibrate the track width from a commanded 90° turn | | [01.12](../01-first-robot/01.12-encoder-distance-and-square.md) |
| $r = D/(10 \cdot 2\pi)$ | measure the real wheel radius: roll 10 turns, measure distance $D$ | | [01.06](../01-first-robot/01.06-mechanical-assembly.md) |
| $e = \operatorname{wrap}(\theta_{target} - \theta) \in (-\pi, \pi]$ | heading error, always the short way round | | [08.11](../08-control/08.11-rotate-exactly-90.md) |

## Odometry

| Formula | What it gives | Lesson |
|---|---|---|
| $\Delta s = \dfrac{d_R + d_L}{2}, \quad \Delta\theta = \dfrac{d_R - d_L}{b}$ | how far the centre moved and how much it turned in one step | [09.03](../09-odometry/09.03-dead-reckoning-integration.md), [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| $\dot x = v\cos\theta, \quad \dot y = v\sin\theta, \quad \dot\theta = \omega$ | how the pose changes over time (unicycle model) | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |
| $x \mathrel{+}= \Delta s\cos(\theta + \tfrac{\Delta\theta}{2}), \quad y \mathrel{+}= \Delta s\sin(\theta + \tfrac{\Delta\theta}{2}), \quad \theta \mathrel{+}= \Delta\theta$ | dead reckoning: update the pose each step (midpoint method) | [09.03](../09-odometry/09.03-dead-reckoning-integration.md) |
| $\theta \leftarrow \operatorname{atan2}(\sin\theta, \cos\theta)$ | keep the heading in $(-\pi, \pi]$ | [09.04](../09-odometry/09.04-odometry-from-scratch.md) |
| lateral error $\approx d\,\delta\theta$ | a small heading error $\delta\theta$ turns into sideways error after $d$ metres | [09.06](../09-odometry/09.06-accumulated-error.md) |
| $\dfrac{\delta v}{v} = \dfrac{\delta r}{r}, \quad \dfrac{\delta\omega}{\omega} = \dfrac{\delta r}{r} - \dfrac{\delta b}{b}$ | how a wrong radius or track width scales the errors | [09.01](../09-odometry/09.01-diff-drive-kinematics.md) |

## Motor and PWM

| Formula | What it gives | karmel | Lesson |
|---|---|---|---|
| $T = 1/f$, $\; D = t_{on}/T$ | PWM period and duty cycle | | [FE.07](../optional-foundations/electronics/FE.07-pwm.md) |
| $\bar V = D \cdot V_{bat}$ | average voltage the motor sees | $D$ = 0.5 at 10.8 V → 5.4 V | [01.08](../01-first-robot/01.08-first-motor-spin.md) |
| $\omega_{ss} = K\,(u - u_{db})$ for $u > u_{db}$, else 0 | steady wheel speed for a duty (static motor model) | $u_{db}$ = 0.12 | [08.02](../08-control/08.02-motor-step-response.md) |
| $\tau\,\dfrac{d\omega}{dt} = -\omega + K\,(u - u_{db})$ | how the speed changes over time (first-order motor) | $\tau$ = 0.08 s | [08.02](../08-control/08.02-motor-step-response.md) |
| $\omega(t) = \omega_f\left(1 - e^{-t/\tau}\right)$ | speed after a duty step: 63 % at $\tau$, 98 % at $4\tau$ | 98 % after 0.32 s | [08.02](../08-control/08.02-motor-step-response.md) |
| $V = I R + k\,\omega$ | DC motor: supply voltage = resistance drop + back-EMF | | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $I_{stall} = V/R$ | current when the motor is blocked (the worst case) | | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |
| $\omega_{out} = \omega_{motor}/G, \quad \tau_{out} = \tau_{motor} \cdot G \cdot \eta$ | gearbox: slower and stronger | $G$ = 56 | [FE.10](../optional-foundations/electronics/FE.10-dc-motors.md) |

## PID control

| Formula | What it gives | Lesson |
|---|---|---|
| $e = r_{sp} - y$ | error: what you want minus what you measure | [08.04](../08-control/08.04-proportional-control.md) |
| $u = k_p\,e$ | P controller | [08.04](../08-control/08.04-proportional-control.md) |
| $u = k_p\,e + k_i \displaystyle\int e\,dt$ | PI controller: the integral removes the steady error | [08.05](../08-control/08.05-integral-control.md) |
| $u = k_p\,e + k_i \displaystyle\int e\,dt + k_d\,\dfrac{de}{dt}$ | PID controller | [08.06](../08-control/08.06-derivative-control.md), [08.07](../08-control/08.07-pid-mathematics.md) |
| $I_k = I_{k-1} + k_i\,e_k\,\Delta t, \quad u_k = \operatorname{clamp}(k_p e_k + I_k,\ -1,\ 1)$ | PI in code, one line per loop step | [08.05](../08-control/08.05-integral-control.md) |
| $D_k = -k_d\,\dfrac{y_k - y_{k-1}}{\Delta t}$ | D term on the measurement (no kick when the setpoint jumps) | [08.06](../08-control/08.06-derivative-control.md) |
| $u_{ff} = \operatorname{sign}(\omega)\left(u_{db} + \dfrac{\lvert\omega\rvert}{\omega_{top}}(1 - u_{db})\right)$ | feedforward: the duty the motor model says you need | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $u = u_{ff} + k_p\,e + I$ | feedforward plus PI correction (karmel's wheel loop) | [08.09](../08-control/08.09-feedforward-exact-speed.md) |
| $k_p = \dfrac{\tau}{K\,\tau_{cl}}, \quad k_i = \dfrac{k_p}{\tau}$ | lambda tuning: pick the closed-loop time constant $\tau_{cl}$, get the gains | [08.07](../08-control/08.07-pid-mathematics.md) |
| $f_{loop} \ge 10/\tau$ | the loop must run at least 10× faster than what it controls | [00.02](../00-orientation/00.02-the-robotics-stack.md) |

## Battery and power

| Formula | What it gives | karmel | Lesson |
|---|---|---|---|
| $V = I R$, $\; P = V I$ | Ohm's law and power | | [FE.01](../optional-foundations/electronics/FE.01-voltage-current-resistance-power.md) |
| $V_{pack} = S \cdot V_{cell}$ | pack voltage from cells in series | 3 · 3.6 = 10.8 V | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $E = V_{nom} \cdot C$ | battery energy | 10.8 · 3.5 = 37.8 Wh | [FE.13](../optional-foundations/electronics/FE.13-batteries.md) |
| $t \approx \dfrac{0.8\,E}{P_{avg}}$ | runtime, using 80 % of the energy | | [02.02](../02-robot-electronics/02.02-power-budget.md) |
| $V_{terminal} = V_{open} - I\,R_{internal}$ | voltage sag when the motors pull current | | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $V_{pin} = V_{bat}\,\dfrac{R_b}{R_t + R_b}$ | battery voltage divider into the ADC | 100k/22k: 12.6 V → 2.27 V | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $V_{pin} = \dfrac{\text{raw}}{65535} \cdot 3.3$ | Pico ADC reading to volts | | [01.14](../01-first-robot/01.14-battery-monitoring.md) |
| $I_{bat} = \dfrac{V_{out}\,I_{out}}{\eta\,V_{bat}}$ | battery current drawn by the 5 V regulator | | [02.02](../02-robot-electronics/02.02-power-budget.md) |

## Stopping and safety

| Formula | What it gives | karmel | Lesson |
|---|---|---|---|
| $d = v\,T = v/f$ | distance driven between two loop decisions | 0.5 m/s at 50 Hz → 1 cm | [00.01](../00-orientation/00.01-what-is-a-robot.md) |
| $d_{stop} = v\,L + \dfrac{v^2}{2a}$ | stopping distance: travel during the delay + braking | | [00.02](../00-orientation/00.02-the-robotics-stack.md) |
| $d_{coast} = v \cdot t_{watchdog}$ | how far it rolls before the watchdog cuts the motors | 0.5 · 0.3 = 15 cm | [01.10](../01-first-robot/01.10-pi-pico-protocol.md) |
| $E_k = \tfrac12 m v^2$ | kinetic energy: what hits the wall | 1.6 kg at 0.5 m/s → 0.2 J | [00.06](../00-orientation/00.06-safety-mindset.md) |

## Distance sensors

| Formula | What it gives | Lesson |
|---|---|---|
| $d = \dfrac{c \cdot t}{2}$ | distance from the echo round-trip time (ultrasonic, ToF) | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |
| $c = 331.3 + 0.606\,T$ | speed of sound at air temperature $T$ (°C) | [01.13](../01-first-robot/01.13-distance-sensors-stop.md) |

---

## See also

- [Glossary](glossary.md): every term in words.
- [`labs/config/karmel.yaml`](../labs/config/karmel.yaml): the source of every karmel number on this page.
