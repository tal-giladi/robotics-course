"""bag_replay_check — replay a recorded bag through TODAY's battery logic and compare (lesson 04.14).

    ros2 run course_bag_examples bag_replay_check battery_run                 # exit 0 = logic reproduces the recording
    ros2 run course_bag_examples bag_replay_check battery_run --alpha 0.1     # "what if?" -> mismatches, exit 1

The bag must contain both the INPUT (battery/voltage, std_msgs/Float32) and the OUTPUT
(battery/health, karmel_tutorial_interfaces/BatteryHealth) of battery_health.

How it works (event sourcing, in robot clothes):
  1. snapshot  the first recorded BatteryHealth gives the filter value, state and threshold
  2. replay    every later voltage message goes through LowPassFilter + BatteryHealthTracker
  3. compare   each recorded BatteryHealth is paired with the replayed result for the same raw
               sample (health.raw_voltage_v == voltage.data) and state + filtered voltage are checked
No ROS graph is involved: the bag is read directly with rosbag2_py, so this runs in CI.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import sys

from karmel_tutorial.battery_logic import BatteryHealthTracker, HealthState, LowPassFilter
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message


@dataclass
class Replayed:
    raw_v: float
    filtered_v: float
    state: HealthState


def read_bag(uri: str, topics: list[str]):
    """Yield (topic, message, receive_time_ns) for the given topics, in recorded order."""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id=''),  # '' = detect mcap / sqlite3 from metadata
                ConverterOptions(input_serialization_format='', output_serialization_format=''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    missing = [t for t in topics if t not in types]
    if missing:
        raise SystemExit(f'bag {uri} has no topic(s) {missing}; it has {sorted(types)}')
    reader.set_filter(StorageFilter(topics=topics))
    classes = {t: get_message(types[t]) for t in topics}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        yield topic, deserialize_message(data, classes[topic]), t_ns


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('bag', help='bag directory (the folder ros2 bag record -o created)')
    parser.add_argument('--voltage-topic', default='/battery/voltage')
    parser.add_argument('--health-topic', default='/battery/health')
    parser.add_argument('--alpha', type=float, default=0.3, help='filter alpha to replay with')
    parser.add_argument('--critical', type=float, default=9.9)
    parser.add_argument('--hysteresis', type=float, default=0.2)
    parser.add_argument('--tolerance', type=float, default=1e-3, help='filtered-voltage tolerance [V]')
    args = parser.parse_args(argv)

    lpf = LowPassFilter(alpha=args.alpha)
    tracker = BatteryHealthTracker(critical_v=args.critical, hysteresis_v=args.hysteresis)
    pending: deque[Replayed] = deque()
    snapshot_taken = False
    t0_ns = None
    matched = mismatched = unpaired = 0
    replay_transitions: list[str] = []
    recorded_transitions: list[str] = []
    last_recorded_state = None

    for topic, msg, t_ns in read_bag(args.bag, [args.voltage_topic, args.health_topic]):
        t0_ns = t_ns if t0_ns is None else t0_ns
        t_s = (t_ns - t0_ns) * 1e-9

        if topic == args.health_topic:
            recorded = HealthState(msg.state)
            if last_recorded_state is not None and recorded != last_recorded_state:
                recorded_transitions.append(f'{last_recorded_state.name}->{recorded.name} @ {t_s:.1f} s')
            last_recorded_state = recorded
            if not snapshot_taken:  # 1. snapshot
                lpf.value = msg.voltage_v
                tracker.state = recorded
                tracker.low_threshold_v = msg.low_threshold_v
                snapshot_taken = True
                continue
            # 3. compare with the replayed result for the same raw sample
            while pending and pending[0].raw_v != msg.raw_voltage_v:
                pending.popleft()
                unpaired += 1
            if not pending:
                unpaired += 1
                continue
            expected = pending.popleft()
            if expected.state == recorded and abs(expected.filtered_v - msg.voltage_v) <= args.tolerance:
                matched += 1
            else:
                mismatched += 1
                if mismatched <= 5:
                    print(f'  MISMATCH @ {t_s:6.1f} s: recorded {recorded.name} {msg.voltage_v:.3f} V, '
                          f'replayed {expected.state.name} {expected.filtered_v:.3f} V')
            tracker.low_threshold_v = msg.low_threshold_v  # runtime parameter changes are in the record too

        elif snapshot_taken:  # 2. replay an input
            previous = tracker.state
            state = tracker.update(lpf.update(msg.data))
            if state != previous:
                replay_transitions.append(f'{previous.name}->{state.name} @ {t_s:.1f} s')
            pending.append(Replayed(raw_v=msg.data, filtered_v=lpf.value, state=state))

    print(f'recorded transitions: {recorded_transitions}')
    print(f'replayed transitions: {replay_transitions}')
    print(f'compared {matched + mismatched} health messages: {matched} match, {mismatched} mismatch, '
          f'{unpaired} unpaired')
    if not snapshot_taken or matched + mismatched == 0:
        print('FAIL: nothing to compare (did you record both topics?)')
        return 1
    print('PASS' if mismatched == 0 else 'FAIL')
    return 0 if mismatched == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
