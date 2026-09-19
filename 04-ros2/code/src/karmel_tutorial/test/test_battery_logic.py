"""Unit tests for battery_logic: no ROS needed (`pytest test/` or `colcon test`)."""
from karmel_tutorial.battery_logic import (BatteryHealthTracker, estimate_percent,
                                           HealthState, LowPassFilter)
import pytest


def test_percent_is_linear_and_clamped():
    assert estimate_percent(12.6) == pytest.approx(100.0)
    assert estimate_percent(9.9) == pytest.approx(0.0)
    assert estimate_percent(11.4) == pytest.approx(55.56, abs=0.01)
    assert estimate_percent(13.0) == 100.0
    assert estimate_percent(8.0) == 0.0


def test_low_pass_filter_first_sample_then_ema():
    f = LowPassFilter(alpha=0.3)
    assert f.update(12.0) == pytest.approx(12.0)
    assert f.update(11.0) == pytest.approx(11.7)
    assert f.update(11.0) == pytest.approx(11.49)


def test_state_gets_worse_immediately():
    t = BatteryHealthTracker()
    assert t.update(11.0) == HealthState.OK
    assert t.update(10.49) == HealthState.LOW
    assert t.update(9.8) == HealthState.CRITICAL


def test_hysteresis_prevents_flapping():
    t = BatteryHealthTracker()
    t.update(10.45)
    assert t.state == HealthState.LOW
    assert t.update(10.55) == HealthState.LOW     # above 10.5 but not above 10.7
    assert t.update(10.69) == HealthState.LOW
    assert t.update(10.71) == HealthState.OK


def test_recover_from_critical_to_low():
    t = BatteryHealthTracker()
    t.update(9.8)
    assert t.update(10.0) == HealthState.CRITICAL  # needs > 10.1
    assert t.update(10.2) == HealthState.LOW


@pytest.mark.parametrize('value, ok', [(10.1, True), (11.0, True), (10.0, False),
                                       (12.6, False), (15.0, False)])
def test_threshold_validation(value, ok):
    assert (BatteryHealthTracker().check_threshold(value) is None) == ok
