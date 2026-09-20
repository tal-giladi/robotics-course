# FC.08 — Does ROS 2 fit on this microcontroller?

Lesson: [FC.08 micro-ROS: ROS 2 on microcontrollers](../../../optional-foundations/cpp-and-embedded/FC.08-micro-ros.md)

Three questions to answer before you put micro-ROS on a robot, turned into code: does the message
fit through the link, do the entities fit in the pool, and what does the robot do when the agent
disappears.

## What to implement (`student.py`)

| Name | Does |
|---|---|
| `align`, `cdr_string_size` | CDR alignment and strings — where the padding bytes come from |
| `joint_state_body_bytes` | the size of a `sensor_msgs/msg/JointState` on the wire |
| `xrce_frame_bytes` | plus the XRCE submessage, session and serial framing |
| `link_utilisation`, `max_rate_hz` | what fraction of a serial link that rate needs, and the rate that fits |
| `check_design`, `executor_handles` | the build-time `RMW_UXRCE_MAX_*` pool, and what an rclc executor must be sized for |
| `AgentLink` | the WAITING_AGENT → AGENT_CONNECTED → AGENT_DISCONNECTED state machine, with the brake |
| `blind_travel_m` | how far the robot goes before a dead agent is noticed |

## Check

```bash
python course.py check FC.08              # your code
python course.py check FC.08 --solution   # the reference
```

The tests use karmel's real message (`left_wheel_joint` / `right_wheel_joint`, `frame_id`
`base_link` — **124 bytes** of CDR body), the Pico port's default entity pool, and a timeline in
which the agent appears at t = 1,000 ms and dies at t = 1,400 ms.

## Hints

* CDR aligns each primitive to its own size, counted from the start of the body — that is why
  adding an empty `effort` sequence costs 4 bytes and a two-element one costs 20, not 16.
* A string's length field includes the terminating NUL.
* `check_design` returns its strings in the order of `FIELDS`, formatted exactly as the docstring
  shows.
* `AgentLink.update` performs **at most one transition per call** and returns what it did. The
  brake comes before the teardown, and the tests check the order.
