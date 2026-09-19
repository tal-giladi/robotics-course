I want you to build a complete, self-contained robotics learning course for me as a GitHub repository.

## 1. Goal

Create a **from-zero-to-advanced practical robotics curriculum** designed primarily for an experienced software/AI engineer who wants to learn how to build real physical robots.

The course should eventually take me from:

> "I know software, but I know little or nothing about robotics"

to:

> "I can design, assemble, program, control, and extend an autonomous robot, understand the major robotics technologies, and integrate modern AI/LLMs/vision with physical robots."

The course should be practical, technical, and hands-on.

The final project should be approximately:

> A small autonomous mobile robot that can navigate an environment, build/use a map, understand its surroundings through sensors and cameras, accept natural-language instructions, and eventually interact with simple physical objects using a robotic arm.

Do NOT assume that I already know robotics, electronics, mechanical engineering, control theory, mathematics, computer vision, machine learning, ROS, or embedded programming.

At the same time, I am an experienced software developer/CTO with strong C#/.NET/software architecture experience and substantial interest and experience in AI/LLMs.

Therefore:

- Do not waste time teaching basic programming.
- Do explain unfamiliar concepts from first principles.
- When something is potentially prerequisite knowledge, provide an optional prerequisite lesson rather than blocking the main curriculum.
- I should be able to skip material I already know.

The course should be usable by someone who knows essentially nothing about the relevant technical subjects.

---

# 2. Teaching philosophy

The curriculum should follow this rule:

**Main path = practical robotics.**

**Side paths = optional prerequisite/technical foundations.**

For example, if the main lesson requires vectors:

MAIN:

> "Robotics uses a vector to represent position..."

Then provide:

> OPTIONAL: Vectors from zero

with enough material to learn vectors from scratch.

If I already understand vectors, I skip it.

Do the same for:

- matrices
- linear algebra
- trigonometry
- probability
- calculus where needed
- physics
- electronics
- electricity
- mechanics
- control theory
- PID
- coordinate systems
- rotations
- quaternions
- statistics
- machine learning
- neural networks
- computer vision
- optimization
- Python
- Linux
- networking
- C/C++
- embedded systems

The optional material should be genuinely useful, not a giant academic detour.

The goal is:

> "You don't need to know this before starting. You can learn it exactly when you need it."

---

# 3. My existing background

Assume the student has:

- strong C#/.NET experience
- professional software engineering experience
- Git/GitHub experience
- SQL experience
- JavaScript experience
- software architecture experience
- experience with APIs and distributed systems
- experience with AI/LLMs
- some Python experience
- experience with machine learning concepts, but do NOT assume mastery
- interest in AI agents, tool calling, computer vision, LLMs, RAG, and modern AI

However, the curriculum must remain understandable even if I encounter a subject I have never studied.

Do not repeatedly explain basic software concepts that an experienced developer would already know.

---

# 4. Course progression

Build the curriculum around the following progression.

## Phase 0 — What is robotics?

Explain:

- robot
- actuator
- sensor
- controller
- embedded computer
- microcontroller
- SBC
- motor
- servo
- encoder
- IMU
- camera
- LiDAR
- ToF
- ultrasonic sensor
- motor controller
- battery
- ROS 2
- robot middleware
- perception
- planning
- control

Give me a conceptual map of the entire robotics stack.

Show how:

```text
AI / application
       ↓
high-level planning
       ↓
ROS 2
       ↓
perception / localization / planning
       ↓
control
       ↓
motor drivers
       ↓
motors
       ↓
physical robot
```

works.

Include diagrams wherever useful.

---

# 5. Phase 1 — First physical robot

This must be extremely practical.

Select a realistic first robot platform that I can actually buy and assemble in Israel.

IMPORTANT:

I live in Israel.

Every hardware recommendation must therefore consider:

- availability in Israel
- Israeli retailers/distributors where possible
- shipping to Israel
- voltage/power compatibility
- import considerations where relevant
- realistic local prices
- replacement parts
- availability of batteries and chargers
- availability of spare motors/sensors/etc.

Do not design a course around hardware that is easy to buy only in the US.

For every required hardware item, provide:

- exact component
- alternative component
- why it is needed
- approximate Israeli price
- where it can be purchased in Israel
- links
- compatibility
- required cables/connectors
- what additional tools are needed

If possible, design the course so that the initial robot costs roughly a few hundred dollars rather than thousands.

The first robot should ideally be a small differential-drive robot.

Teach me to:

1. assemble it
2. power it safely
3. control motors
4. read sensors
5. use encoders
6. move forward
7. move backward
8. rotate
9. drive a square
10. stop based on sensor input
11. measure and report battery state

This should be the first major hands-on milestone.

---

# 6. Electronics foundation

Create optional beginner modules covering:

- voltage
- current
- resistance
- power
- Ohm's law
- DC motors
- brushless vs brushed motors
- servo motors
- stepper motors
- PWM
- GPIO
- ADC
- digital vs analog signals
- I²C
- SPI
- UART
- CAN if relevant
- motor drivers
- batteries
- Li-ion/LiPo safety
- voltage regulators
- grounding
- pull-up resistors
- breadboards
- connectors
- multimeters
- soldering

Do not turn this into an electrical engineering degree.

Teach only enough theory to understand and build robots.

Include practical experiments.

---

# 7. Programming for robotics

Use Python where appropriate because it is dominant in modern robotics and AI.

Explain:

- Python robotics ecosystem
- serial communication
- GPIO
- sensor libraries
- asynchronous programming where relevant
- processes
- networking
- logging
- configuration
- testing

Explain where C/C++ is used and why.

Since I know C#, explain concepts by analogy when useful, but do not force C# into places where Python/C++ is the standard.

---

# 8. ROS 2

Make ROS 2 a major part of the curriculum.

Teach from zero:

- what ROS is
- why middleware is useful
- ROS 2 architecture
- nodes
- topics
- publishers
- subscribers
- services
- actions
- parameters
- launch files
- packages
- messages
- custom messages
- TF2
- URDF
- RViz
- rosbag
- lifecycle nodes where relevant
- debugging
- logging
- QoS

Every concept should have a practical exercise.

Build a progressively more sophisticated ROS 2 robot.

Explain the architecture in terms an experienced distributed-systems developer will understand.

---

# 9. Linux and development environment

Provide an optional but complete beginner track for:

- Linux basics
- Ubuntu
- shell
- filesystem
- permissions
- processes
- SSH
- networking
- package management
- Python environments
- Git
- Docker where useful

The main curriculum should assume Linux eventually becomes the primary robotics environment.

If ROS 2 requires a particular Ubuntu version, verify the currently supported combination rather than assuming.

Because this course will evolve, use current official documentation.

---

# 10. Simulation

Introduce simulation before the physical robot becomes complicated.

Use an appropriate modern robotics simulator, such as Gazebo if appropriate at the time the course is created.

Teach:

- simulated robot
- simulated world
- physics
- sensors
- motors
- camera
- LiDAR
- noise
- ROS integration
- robot models

Create exercises where the exact same ROS concepts are used in simulation and on the physical robot.

Explain sim-to-real.

---

# 11. Mathematics foundation

Create optional modules for:

### Basic mathematics

- algebra
- trigonometry
- angles
- radians
- coordinate systems

### Linear algebra

- scalars
- vectors
- matrices
- matrix multiplication
- transformations
- dot product
- cross product
- basis vectors

### Geometry

- Cartesian coordinates
- polar coordinates
- 2D/3D transformations

### Rotations

- rotation matrices
- Euler angles
- quaternions

### Probability

- probability distributions
- uncertainty
- Gaussian distributions
- noise

### Calculus

Only teach what robotics actually needs.

### Optimization

Teach enough to understand robotics/ML applications.

Every mathematical concept should ideally include a robotics example.

For example:

> "A vector isn't just an abstract mathematical object. In robotics it can represent the robot's position, velocity, direction, or force."

---

# 12. Coordinate frames and transformations

This is extremely important.

Give this subject substantial treatment.

Teach:

- world frame
- robot frame
- sensor frame
- camera frame
- end-effector frame
- transformations
- homogeneous transformations
- TF2
- translation
- rotation
- coordinate conversion

Create visual exercises.

I should eventually be able to understand:

> "The camera sees an object at this coordinate. Where is that object relative to the robot?"

---

# 13. Sensors

Teach progressively:

- wheel encoders
- IMU
- ultrasonic
- ToF
- LiDAR
- RGB camera
- depth camera

For each sensor:

- what it measures
- physical principle
- strengths
- weaknesses
- noise
- failure modes
- sampling rate
- latency
- practical robotics applications

Include real experiments.

---

# 14. Control systems

Teach:

- open-loop control
- closed-loop control
- feedback
- error
- setpoint
- proportional control
- PID
- tuning
- stability
- oscillation
- overshoot
- response time

Build practical examples.

For example:

> "Make the robot drive at exactly 0.5 m/s."

Then:

> "Make the robot maintain a straight line."

Then:

> "Make the robot rotate exactly 90 degrees."

Explain PID mathematically but also intuitively.

---

# 15. Odometry

Teach:

- wheel encoders
- differential drive
- wheel distance
- wheel radius
- wheelbase
- robot pose
- x/y/theta
- integration
- accumulated error

Build an odometry system from scratch.

Then compare it to ROS 2 implementations.

---

# 16. Localization

Teach:

- why odometry drifts
- landmarks
- sensor fusion
- IMU + encoders
- probabilistic localization
- Kalman filter
- Extended Kalman Filter
- particle filters where relevant

The optional math track should explain the probability/math needed to understand these.

---

# 17. Mapping and SLAM

This should be a major milestone.

Teach:

- mapping
- occupancy grids
- localization
- SLAM
- LiDAR SLAM
- visual SLAM
- loop closure
- map representation

Eventually make the robot:

> explore a room → build a map → save it → localize itself on that map.

---

# 18. Navigation

Teach:

- goals
- waypoints
- path planning
- global planning
- local planning
- obstacle avoidance
- costmaps
- navigation stacks
- recovery behaviors

The robot should eventually be able to:

> "Go to location X."

and navigate there autonomously.

---

# 19. Computer vision

Build a complete practical computer vision track.

Start from zero if necessary.

Teach:

- pixels
- images
- RGB
- grayscale
- camera geometry
- lenses
- calibration
- distortion
- OpenCV
- image preprocessing
- feature detection
- object detection
- segmentation
- tracking
- depth
- 3D vision

Then modern AI vision:

- CNNs
- embeddings
- object detection models
- segmentation models
- vision-language models

Do not spend excessive time training models from scratch unless it is educationally useful.

The goal is robotics.

---

# 20. Robotic arm

After the mobile robot and navigation foundations, introduce a robotic arm.

Recommend a realistic arm that can be purchased in Israel or shipped there.

Teach:

- joints
- degrees of freedom
- links
- end effector
- gripper
- forward kinematics
- inverse kinematics
- Jacobian concept
- joint limits
- trajectories
- motion planning
- collision checking

Create exercises progressively.

Eventually:

> Move the gripper to a specified position.

Then:

> Detect an object with the camera and move the gripper toward it.

Then:

> Pick it up.

---

# 21. Manipulation

Teach:

- grasping
- grasp planning
- object pose
- force
- friction
- grippers
- collision avoidance
- visual servoing

Build a simple pick-and-place system.

---

# 22. Machine learning for robotics

Create an optional ML foundation track.

Assume the student may know absolutely nothing.

Teach:

- what ML is
- supervised learning
- unsupervised learning
- training
- validation
- test sets
- loss functions
- gradient descent
- neural networks
- CNNs
- embeddings
- transformers
- reinforcement learning

Then explain where each is actually useful in robotics.

Do not turn this into a generic ML course.

Keep asking:

> "Why would a robot need this?"

---

# 23. Reinforcement learning

Introduce:

- environment
- state
- action
- reward
- policy
- value
- exploration
- exploitation
- Q-learning
- policy gradients
- actor-critic

Use simulation.

Do not start by training a real robot.

Explain sim-to-real problems.

---

# 24. Modern embodied AI

Create an advanced section covering current approaches such as:

- imitation learning
- behavior cloning
- diffusion policies
- vision-language-action models
- robot foundation models
- multimodal policies
- learned manipulation
- sim-to-real
- foundation models for robotics

Because this area changes rapidly, the course must use current papers and documentation and clearly identify which technologies are mature versus research-oriented.

---

# 25. LLMs and robot agents

This should connect directly to my existing AI interests.

Teach how an LLM can operate a robot safely through tools.

Example:

```text
User:
"Find the bottle and put it on the table."

LLM
 ↓
planner
 ↓
navigate()
 ↓
detect_objects()
 ↓
move_arm()
 ↓
grasp()
 ↓
navigate()
 ↓
release()
```

Explain why the LLM should generally NOT directly control motors.

Teach:

- tool calling
- planners
- state machines
- behavior trees
- agent loops
- memory
- perception/action loops
- task decomposition
- safety boundaries
- deterministic low-level controllers

This should integrate naturally with modern agent architecture.

---

# 26. Safety

Include practical robotics safety throughout the course.

Teach:

- electrical safety
- batteries
- moving mechanisms
- emergency stops
- software limits
- motor limits
- torque limits
- collision detection
- watchdogs
- safe shutdown
- physical workspace safety
- protecting people and pets
- safe testing procedures

Do not leave safety as one chapter at the end.

---

# 27. Course projects

Create increasingly difficult projects.

At minimum:

### Project 1
Robot drives manually.

### Project 2
Obstacle avoidance.

### Project 3
Encoder-based movement.

### Project 4
PID-controlled movement.

### Project 5
Autonomous square/path.

### Project 6
ROS 2 robot.

### Project 7
Simulated robot.

### Project 8
Odometry.

### Project 9
Mapping.

### Project 10
Localization.

### Project 11
Autonomous navigation.

### Project 12
Camera object detection.

### Project 13
Robot + vision.

### Project 14
Robotic arm.

### Project 15
Vision-guided arm.

### Project 16
Pick-and-place.

### Project 17
LLM-controlled high-level robot.

### Final project
Autonomous AI robot combining:

- ROS 2
- sensors
- camera
- localization
- mapping
- navigation
- computer vision
- robotic arm
- manipulation
- high-level AI/LLM
- safety mechanisms

---

# 28. Every lesson should have a consistent structure

Each lesson should contain:

1. What you will learn
2. Why it matters
3. Prerequisites
4. Optional prerequisite links
5. Conceptual explanation
6. Technical explanation
7. Diagram
8. Code
9. Hardware exercise if applicable
10. Expected result
11. Troubleshooting
12. Common mistakes
13. Knowledge check
14. Practical challenge
15. "You can skip this if..." section
16. Links to deeper material
17. Progress checkpoint

Do not merely dump information into Markdown files.

Make it feel like an interactive course.

---

# 29. Interactive learning

The course should assume that I will interact with a coding/AI agent while studying.

The agent should be able to answer questions such as:

> "I don't understand why this matrix multiplication works."

> "Explain PID without mathematics."

> "Explain PID again mathematically."

> "I don't understand TF2."

> "Show me a concrete numerical example."

> "I know Kalman filters already. Let me skip this."

> "I tried the exercise and got this error."

> "Look up the current ROS documentation."

The curriculum should explicitly encourage this.

When a concept has multiple levels, provide:

- intuitive explanation
- practical explanation
- mathematical explanation
- implementation explanation

I can choose how deep to go.

---

# 30. Internet research

For every important subject, provide authoritative resources.

Prefer:

1. official documentation
2. original research papers
3. university courses
4. high-quality technical tutorials
5. reputable books

Avoid relying primarily on random blogs or YouTube.

For fast-changing technologies such as:

- ROS 2
- Gazebo
- NVIDIA robotics tools
- foundation models
- VLA models
- AI frameworks

the course should explicitly instruct the agent to verify current documentation before giving instructions.

Every external resource should have a complete clickable URL.

Do not write vague things such as:

> "Search for ROS 2 tutorials."

Provide the actual resource.

---

# 31. Papers

For advanced subjects, provide a curated paper list.

Do not just list famous papers.

Explain:

- why the paper matters
- what prerequisite knowledge it requires
- what I should understand from it
- whether I need to read the entire paper
- which sections matter

The paper progression should eventually connect classical robotics to modern embodied AI.

---

# 32. GitHub repository architecture

Create a clean repository structure.

For example:

```text
robotics-course/
│
├── README.md
├── COURSE_MAP.md
├── PROGRESS.md
├── HARDWARE.md
├── SAFETY.md
│
├── 00-orientation/
├── 01-first-robot/
├── 02-electronics/
├── 03-python-linux/
├── 04-ros2/
├── 05-simulation/
├── 06-math/
├── 07-coordinate-systems/
├── 08-sensors/
├── 09-control/
├── 10-odometry/
├── 11-localization/
├── 12-slam/
├── 13-navigation/
├── 14-computer-vision/
├── 15-robotic-arm/
├── 16-manipulation/
├── 17-machine-learning/
├── 18-reinforcement-learning/
├── 19-embodied-ai/
├── 20-llm-robot-agents/
│
├── projects/
│
├── labs/
│
├── optional-foundations/
│   ├── mathematics/
│   ├── electronics/
│   ├── physics/
│   ├── machine-learning/
│   ├── computer-vision/
│   └── control-theory/
│
├── hardware/
│
├── references/
│
└── tools/
```

Improve this structure if you have a better design.

---

# 33. Progress tracking

This is important.

I want to be able to track exactly where I am in the course.

Create a progress system using GitHub-friendly files.

For example:

```text
PROGRESS.md
```

with:

```text
[x] 01.1 What is a robot?
[x] 01.2 Sensors
[ ] 01.3 Actuators
[ ] 01.4 First motor experiment
```

But make it more useful than a simple checklist.

Track:

- lesson status
- exercises completed
- projects completed
- optional modules skipped
- concepts I struggled with
- hardware purchased
- hardware assembled
- skills mastered
- current project
- next recommended lesson

Consider a machine-readable progress file such as:

```text
progress.yaml
```

or JSON, in addition to the human-readable Markdown file.

Design it so an AI coding agent can read it and answer:

> "Where am I in the course?"

and:

> "What should I study next?"

It should also be possible to update progress through a simple command or script.

For example:

```text
python course.py status
python course.py complete 04.3
python course.py next
```

Design an appropriate mechanism.

---

# 34. Exercises and verification

Don't let the course become passive reading.

Each important concept should have a way to verify understanding.

Use:

- quizzes
- coding exercises
- numerical exercises
- debugging exercises
- hardware experiments
- "predict what will happen" questions
- mini-projects

The course should distinguish:

> "I read this"

from:

> "I can actually do this."

---

# 35. Hardware should evolve

Do NOT require me to buy all hardware at the beginning.

Design the curriculum so that I can start cheaply.

At each stage tell me:

### Buy now

### Optional

### Buy later

For example:

```text
Stage 1:
- robot chassis
- motors
- motor controller
- Raspberry Pi
- encoder

Stage 2:
- IMU
- camera

Stage 3:
- LiDAR

Stage 4:
- better compute

Stage 5:
- robotic arm
```

Give alternatives when possible.

---

# 36. Israel-specific hardware research

Because I live in Israel, hardware research is part of the course.

Search current Israeli suppliers and marketplaces when preparing the course.

Consider sources such as:

- Israeli electronics suppliers
- Israeli robotics suppliers
- Israeli maker stores
- local distributors
- Amazon Israel / international shipping where practical
- AliExpress when appropriate
- Mouser/DigiKey if practical
- local 3D-printing services

Do not assume that an item is available in Israel merely because it exists on Amazon US.

Verify current availability when possible.

Prices should be clearly labeled as approximate and dated.

---

# 37. 3D printing

Include an optional module covering:

- why 3D printing is useful in robotics
- basic CAD
- STL/3MF
- tolerances
- mounting brackets
- gears
- robot chassis parts
- replacement parts

I don't necessarily want to buy a 3D printer immediately.

Explain when it becomes worthwhile and alternatives such as ordering prints locally.

---

# 38. Software engineering quality

Because I am an experienced software engineer, the course should teach robotics software professionally.

Include:

- architecture
- modularity
- interfaces
- testing
- logging
- telemetry
- configuration
- simulation testing
- hardware abstraction
- CI where practical
- Git workflows
- reproducibility
- version pinning
- dependency management

Don't create fragile "copy these 30 commands and hope it works" tutorials.

---

# 39. Troubleshooting

Every hardware/software stage should include troubleshooting.

Examples:

> Motor doesn't move.

Possible causes:

- power
- wiring
- GPIO
- driver
- PWM
- software
- battery
- ground

Teach systematic debugging rather than just listing fixes.

Do the same for:

- ROS problems
- network problems
- sensor problems
- calibration
- localization
- SLAM
- camera
- motors
- arm movement

---

# 40. Avoid outdated instructions

This course will be maintained over time.

Clearly identify:

- version-sensitive instructions
- hardware-specific instructions
- deprecated ROS packages
- deprecated APIs
- old tutorials

Prefer current official documentation.

Whenever a technology changes significantly, the repository should make it easy to update that section without rewriting the entire course.

---

# 41. Difficulty labeling

Every lesson should have:

- difficulty
- estimated time
- prerequisites
- optional prerequisites
- hardware required
- software required

Use a simple system such as:

```text
Difficulty: Beginner
Time: 45 minutes
Hardware: None
Prerequisites: None
Optional prerequisites: Basic vectors
```

---

# 42. Course roadmap

Create a visual roadmap showing:

```text
Programming
     │
     ├── Electronics
     │
     ├── Mathematics
     │
     ↓
First Robot
     ↓
ROS 2
     ↓
Simulation
     ↓
Sensors
     ↓
Control
     ↓
Odometry
     ↓
Localization
     ↓
SLAM
     ↓
Navigation
     ↓
Computer Vision
     ↓
Robotic Arm
     ↓
Manipulation
     ↓
Machine Learning
     ↓
Embodied AI
     ↓
LLM Robot Agent
     ↓
FINAL ROBOT
```

Also show which optional prerequisite tracks feed into each stage.

---

# 43. Do not make the course artificially academic

The purpose is not to give me a university degree.

If something can be understood through a practical experiment, prefer that.

For example:

Instead of:

> 30 pages about PID theory

do:

> Make a motor maintain a target speed.

Then:

> Observe the error.

Then:

> Add proportional control.

Then:

> Observe what happens.

Then:

> Add integral control.

Then:

> Add derivative control.

Then explain the mathematics.

The same principle should apply throughout the course.

---

# 44. Depth

The course should be comprehensive.

Do not create a 20-page "learn robotics in 7 days" tutorial.

I expect this to become a substantial long-term repository.

However, each individual lesson should remain digestible.

Think:

> many small lessons + many experiments + optional deep dives

rather than:

> enormous textbook chapters.

---

# 45. Final curriculum deliverables

When you finish preparing the repository, it should contain:

1. Complete curriculum map
2. Detailed lessons
3. Optional prerequisite tracks
4. Hardware list
5. Israel-specific purchasing information
6. Setup instructions
7. Software installation instructions
8. Robotics projects
9. Exercises
10. Quizzes
11. Troubleshooting guides
12. Reference material
13. Paper list
14. Official documentation links
15. Simulation environment
16. Progress tracking
17. Final project specification
18. Architecture diagrams
19. Glossary
20. Course maintenance/update instructions

---

# 46. Important: build the course, don't just describe it

I don't want you to respond with:

> "Here is a suggested robotics curriculum..."

I want you to **actually create the repository structure and content**.

Start by creating:

- README.md
- COURSE_MAP.md
- PROGRESS.md
- HARDWARE.md
- SAFETY.md
- the complete directory structure
- the first lessons
- optional foundations
- progress tracking mechanism

Then progressively fill in the curriculum.

If the repository is too large to create in one operation, create it in logical stages and keep track of what has already been completed.

Do not silently omit sections.

---

# 47. Agent behavior while I study

After the repository exists, I want to be able to use you as my interactive teacher.

When I tell you:

> "I'm starting lesson 07.3"

you should read the relevant lesson and teach it interactively.

When I say:

> "I don't understand this"

explain it rather than simply pointing me to another page.

When appropriate, use:

- simpler explanations
- analogies
- diagrams
- numerical examples
- code
- exercises

If I ask a question that requires current information, you should research the Internet and cite the sources.

If I already understand something, let me skip it.

If I struggle repeatedly with a prerequisite, identify the relevant optional foundation lesson and direct me there.

---

# 48. Curriculum intelligence

The course should have enough metadata that an AI agent can reason about dependencies.

For example:

```text
SLAM
requires:
  - coordinate systems
  - transformations
  - sensors
  - probability
  - odometry

optional:
  - Kalman filtering
```

and:

```text
Vision-guided manipulation
requires:
  - computer vision
  - coordinate frames
  - robotic arm
  - inverse kinematics

optional:
  - Jacobians
  - optimization
```

Create this dependency graph.

This should allow the agent to determine what I need to learn when I encounter a difficult subject.

---

# 49. Quality standard

Before considering the course complete, audit it for:

- missing prerequisites
- broken links
- outdated software versions
- unavailable hardware
- unexplained terminology
- exercises with no expected result
- exercises requiring hardware not listed
- mathematical concepts used without explanation
- missing safety information
- circular prerequisites
- duplicate material

The most important criterion is:

> A motivated software engineer starting with essentially zero robotics knowledge should be able to follow the course all the way to the final project without getting stuck simply because the curriculum assumed knowledge that it never taught.

At the same time:

> An experienced engineer should not be forced to spend weeks learning material they already know.

The solution to both requirements is the **main practical path + optional prerequisite/deep-dive paths** architecture.

Build the course around that principle.