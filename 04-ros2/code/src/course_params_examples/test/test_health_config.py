"""Plain pytest, no ROS needed: python -m pytest test/test_health_config.py."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from course_params_examples.health_config import HealthConfig, voltage_range  # noqa: E402


def test_defaults_are_valid():
    assert HealthConfig().problems() == []


def test_low_threshold_too_close_to_critical_is_rejected():
    cfg = HealthConfig().with_changes({'low_threshold_v': 10.0})
    assert len(cfg.problems()) == 1
    assert 'critical_v + hysteresis_v' in cfg.problems()[0]


def test_low_threshold_above_full_is_rejected():
    assert HealthConfig().with_changes({'low_threshold_v': 12.6}).problems()


def test_batch_change_is_checked_as_a_whole():
    # lowering critical_v first makes a lower low_threshold_v legal in the same batch
    cfg = HealthConfig().with_changes({'critical_v': 9.5, 'low_threshold_v': 10.0})
    assert cfg.problems() == []


def test_integer_values_are_converted():
    assert HealthConfig().with_changes({'full_v': 13}).full_v == 13.0


def test_voltage_range_3s():
    lo, hi = voltage_range(3)
    assert (round(lo, 2), round(hi, 2)) == (9.0, 12.75)
