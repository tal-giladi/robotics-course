# Troubleshooting: ROS 2 discovery, QoS and timing

Three unrelated mechanisms produce the same symptom — "the topic is there and nothing arrives" — and
telling them apart is most of ROS 2 debugging:

| The topic is… | Cause | Check |
|---|---|---|
| not listed at all | **discovery**: domain id, network, multicast, container boundary | `ros2 node list` from both machines |
| listed, no messages | **QoS incompatibility** | `ros2 topic info -v` and compare the profiles |
| listed, messages flowing, callback silent | **executor**: blocked, starved, or no `spin` | `ros2 topic hz` vs your own log line |

Work top to bottom. → [04.13](../../04-ros2/04.13-debugging-and-introspection.md), [04.11](../../04-ros2/04.11-qos.md), [04.12](../../04-ros2/04.12-executors-and-callbacks.md)

> [!TIP]
> **Ask your teacher:** "Walk me through `ros2 topic info -v` on this topic and tell me which field
> proves it is a QoS problem rather than a discovery problem."

---

### Symptom: nodes on the laptop and the Pi do not see each other

1. **`ROS_DOMAIN_ID` must match** on both, in the shell that actually launched each node — including
   over SSH and inside systemd units, which do not read your `.bashrc`. → [04.02](../../04-ros2/04.02-installing-ros2.md), [03.02](../../03-robot-software/03.02-environments-and-pinning.md)
2. **`ros2 doctor --report`** on both sides; it catches most environment mistakes in one command. →
   [04.13](../../04-ros2/04.13-debugging-and-introspection.md)
3. **Can they reach each other at all?** `ping` first. If `ping karmel.local` fails but the IP works,
   it is mDNS, not ROS. → [FL.09](../../optional-foundations/linux-and-tools/FL.09-networking-basics.md), [03.10](../../03-robot-software/03.10-networking-for-robots.md)
4. **Multicast** is what discovery uses, and guest/corporate Wi-Fi commonly blocks it. Test it
   explicitly (`ros2 multicast receive` / `send`). If it is blocked, you need a discovery server or a
   different network. → [04.01](../../04-ros2/04.01-why-middleware.md), [03.10](../../03-robot-software/03.10-networking-for-robots.md)
5. **Docker and WSL add a network boundary.** Host networking, or an explicit discovery
   configuration, is required. → [FL.12](../../optional-foundations/linux-and-tools/FL.12-docker-for-robotics.md)
6. **Both machines must run compatible RMW implementations.** Mixed vendors mostly work and sometimes
   do not; make them the same while debugging. → [04.01](../../04-ros2/04.01-why-middleware.md)

### Symptom: "ghost" nodes or topics you did not start, or the robot responded to somebody else's command

1. You are sharing a domain id with another machine — a classmate, another container, or your own
   forgotten session. Change `ROS_DOMAIN_ID`. → [04.02](../../04-ros2/04.02-installing-ros2.md)
2. On a shared network this is a safety issue, not an annoyance: an unexpected `cmd_vel` publisher
   will drive your robot. → [04.16](../../04-ros2/04.16-your-robot-as-ros2-node.md), [03.10](../../03-robot-software/03.10-networking-for-robots.md)
3. Stale entries that persist after you stopped a node are discovery cache; they clear. A node that
   keeps *republishing* is still alive somewhere. → [04.03](../../04-ros2/04.03-nodes-and-cli.md)

### Symptom: the topic is listed, has a publisher and a subscriber, and no messages arrive

This is QoS until proven otherwise.

1. **`ros2 topic info -v <topic>`** and compare reliability and durability on both ends. A
   `RELIABLE` subscriber will not match a `BEST_EFFORT` publisher — no error, just silence. →
   [04.11](../../04-ros2/04.11-qos.md)
2. **Sensor data** (`/scan`, images, IMU) is published best-effort with a small depth: your subscriber
   must use the sensor-data profile. → [04.11](../../04-ros2/04.11-qos.md), [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md)
3. **Late-joining and getting nothing** (map, robot description, a spec published once) is durability:
   the publisher needs `TRANSIENT_LOCAL` and so does the subscriber. → [04.11](../../04-ros2/04.11-qos.md)
4. **`ros2 topic echo` works and your node does not** — then the topic is fine and the problem is in
   your node's QoS or its executor. → [04.13](../../04-ros2/04.13-debugging-and-introspection.md)

### Symptom: data arrives, but the controller acts on stale values

1. **Check the history depth.** A large queue with a slow callback delivers old messages on time. Use
   depth 1 for "latest value wins" data. → [04.11](../../04-ros2/04.11-qos.md)
2. **Check the header stamp, not the arrival time.** Log `now − stamp`; if it grows, you have a
   pipeline backing up. → [07.10](../../07-sensors/07.10-sensor-data-in-ros2.md)
3. **Over Wi-Fi, reliable delivery of large messages (images, clouds) causes retransmission storms**
   that look like stutter. Switch to best-effort, or compress. → [04.11](../../04-ros2/04.11-qos.md), [03.10](../../03-robot-software/03.10-networking-for-robots.md)

### Symptom: the node logs one line and then nothing, and Ctrl-C does not stop it

Classic executor deadlock.

1. **Did you call a service synchronously from inside a callback?** With a single-threaded executor
   that is a guaranteed deadlock: the callback cannot return, so the response can never be processed.
   → [04.07](../../04-ros2/04.07-services.md), [04.12](../../04-ros2/04.12-executors-and-callbacks.md)
2. **Fix it properly**: use the async client and handle the future, or move the call into a
   `ReentrantCallbackGroup` with a `MultiThreadedExecutor`. → [04.12](../../04-ros2/04.12-executors-and-callbacks.md)
3. **Do not reach for `MultiThreadedExecutor` first.** It converts a reliable deadlock into an
   intermittent race, which is worse. Understand the callback groups. → [04.12](../../04-ros2/04.12-executors-and-callbacks.md)

### Symptom: the control loop rate is lower than the timer period says

1. **Measure it**: log the actual period's mean, max and 99th percentile. Timers do not catch up after
   a long callback; they skip. → [04.12](../../04-ros2/04.12-executors-and-callbacks.md), [03.04](../../03-robot-software/03.04-timing-and-concurrency.md)
2. **Find the callback that takes too long.** Everything in a single-threaded executor shares one
   thread, so one slow image callback starves your 50 Hz controller. → [04.12](../../04-ros2/04.12-executors-and-callbacks.md)
3. **Separate rates into separate callback groups or separate nodes** rather than trying to make the
   slow thing fast. → [04.12](../../04-ros2/04.12-executors-and-callbacks.md), [08.12](../../08-control/08.12-control-in-ros2.md)
4. Remember the GIL: threads will not give you CPU parallelism for Python work. → [03.04](../../03-robot-software/03.04-timing-and-concurrency.md)

### Symptom: my YAML parameter value is ignored and the node runs with the default

1. **The parameter must be declared** by the node. Undeclared keys in YAML are silently dropped. →
   [04.09](../../04-ros2/04.09-parameters.md)
2. **The YAML nesting must match**: `node_name: ros__parameters: key: value`, and `node_name` must be
   the node's actual name after remapping. A wildcard `/**` is the quick test. → [04.09](../../04-ros2/04.09-parameters.md)
3. **Type must match exactly**: `1` is an integer and `1.0` is a double, and
   `Wrong parameter type, expected 'Type.DOUBLE'` means exactly that. → [04.09](../../04-ros2/04.09-parameters.md)
4. **Verify at runtime** with `ros2 param get`, which tells you what the node believes, not what you
   wrote. → [04.09](../../04-ros2/04.09-parameters.md)
5. If the value *is* right and behaviour did not change, the node read it once at startup and has no
   change callback. → [04.09](../../04-ros2/04.09-parameters.md)

### Symptom: every command against one node times out (`ros2 param`, services, lifecycle)

1. The node's executor is blocked — see the deadlock entry above. Its topics may still publish if the
   publishing happens on another thread, which makes it look alive. → [04.12](../../04-ros2/04.12-executors-and-callbacks.md)
2. Check CPU: a Pi pinned at 100 % starves everything. → [04.13](../../04-ros2/04.13-debugging-and-introspection.md)
3. If the node is a lifecycle node and is `unconfigured`/`inactive`, it is supposed to do nothing.
   That is not a bug. → [04.15](../../04-ros2/04.15-lifecycle-nodes.md)

### Symptom: the node runs, publishes nothing, and logs no error

1. **Is it a lifecycle node?** Check `ros2 lifecycle get`. Nav2 servers stay inactive until the
   lifecycle manager activates them, and a failed transition is often logged only at debug level. →
   [04.15](../../04-ros2/04.15-lifecycle-nodes.md)
2. **Is `spin` being called at all?** A node constructed and never spun is a very quiet bug. →
   [04.04](../../04-ros2/04.04-topics-publishers-subscribers.md)
3. **Raise the log level** (`--ros-args --log-level debug`) before inventing theories. → [04.13](../../04-ros2/04.13-debugging-and-introspection.md)

### Symptom: the fix worked, and the problem came back after a restart

1. You fixed it in one shell. The environment (`ROS_DOMAIN_ID`, sourced workspace, `PYTHONPATH`) is
   not the same over SSH, in a systemd unit, or in a launch file. Put it where it persists. →
   [03.02](../../03-robot-software/03.02-environments-and-pinning.md), [FL.06](../../optional-foundations/linux-and-tools/FL.06-processes-systemd.md)
2. Overlay order matters: a stale `install/` shadowing your source is the other half of this. →
   [Build and toolchain](build-and-toolchain.md)

## Where to go next

- The systematic ROS 2 debugging method: [04.13](../../04-ros2/04.13-debugging-and-introspection.md)
- QoS profiles in detail, with the compatibility table: [04.11](../../04-ros2/04.11-qos.md)
- Executors, callback groups and timers: [04.12](../../04-ros2/04.12-executors-and-callbacks.md)
- Networking between the laptop and the robot: [03.10](../../03-robot-software/03.10-networking-for-robots.md), [FL.09](../../optional-foundations/linux-and-tools/FL.09-networking-basics.md)
- Recording and replaying to debug offline: [04.14](../../04-ros2/04.14-rosbag2.md)
