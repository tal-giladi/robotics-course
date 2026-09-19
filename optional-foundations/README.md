# Optional foundations

> *You don't need to know this before starting. You can learn it exactly when you need it.*

The main path of the course is practical robotics. Whenever a main lesson uses something that
may be new — vectors, PWM, Gaussians, quaternions, backpropagation — it links the foundation
lesson here with a one-line note. If you already know the topic, skip it
(`python course.py skip FM.05 --reason "…"`). If you struggle with a main lesson twice,
`course.py` and your AI teacher will point you back here to the specific lesson that fills the gap.

Foundation lessons are short, self-contained, and every concept ends with a robotics example.
They are not a degree: they teach exactly what the main path uses.

| Track | Lessons | What it covers | Main modules that use it |
|---|---|---|---|
| [Mathematics (FM)](mathematics/) | 21 | algebra, angles, trig/atan2, coordinates, vectors, dot/cross, matrices, transformations, homogeneous coordinates, rotation matrices, Euler angles, quaternions, probability, Gaussians, covariance, Bayes, derivatives, integration, least squares, optimization, statistics | 05, 07, 09, 10, 11, 14, 17 |
| [Electronics (FE)](electronics/) | 19 | voltage/current/power, Ohm's law, multimeter, breadboards, GPIO, pull-ups, PWM, ADC, UART/I²C/SPI, DC motors, servos & steppers, H-bridges, batteries, Li-ion safety, regulators, grounding, soldering, capacitors & diodes, CAN | 01, 02, 14 |
| [Physics (FP)](physics/) | 7 | kinematics, forces, friction, torque & gears, energy & runtime, inertia, center of mass & tipping | 01, 06, 09, 15, 20 |
| [Linux and tools (FL)](linux-and-tools/) | 13 | Ubuntu, shell, filesystem, permissions, systemd, apt, SSH, networking, Python environments, Git, Docker, tmux | 01, 03, 04 |
| [Python for C# developers (FPY)](python/) | 6 | idioms, numpy, matplotlib, typing & packaging, asyncio, performance | 01, 03, 13 |
| [Embedded systems and C/C++ (FC)](cpp-and-embedded/) | 8 | microcontrollers, MicroPython, C++ for C# devs, CMake, interrupts & PIO, Pico SDK, real-time, micro-ROS | 01, 03, 08 |
| [Control theory (FCT)](control-theory/) | 6 | feedback, first/second-order systems, ODEs, stability, sampling, state space/LQR/MPC | 08, 12 |
| [Machine learning (FML)](machine-learning/) | 12 | ML basics, supervised learning, splits, losses, gradient descent, neural networks, PyTorch, CNNs, embeddings, transformers, unsupervised learning, diffusion | 13, 16, 17, 18 |
| [Computer vision (FCV)](computer-vision/) | 6 | image formation, convolution, color spaces, projective geometry, stereo, detection metrics | 07, 13 |
| [3D printing and CAD (F3D)](3d-printing-and-cad/) | 7 | when to print, CAD basics, file formats, tolerances, brackets, gears & inserts, materials and Israeli print services | 20 |

Every lesson is listed with time and prerequisites in [COURSE_MAP.md](../COURSE_MAP.md#optional-foundations--every-lesson).
