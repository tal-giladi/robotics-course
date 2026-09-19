"""Small helpers shared by the QoS demo nodes (lesson 04.11)."""
from rclpy.qos import QoSProfile


def describe(qos: QoSProfile) -> str:
    """One-line human summary, e.g. 'BEST_EFFORT / VOLATILE / KEEP_LAST(5)'."""
    return f'{qos.reliability.name} / {qos.durability.name} / {qos.history.name}({qos.depth})'
