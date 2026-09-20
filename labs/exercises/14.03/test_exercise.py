"""Checker for 14.03 — safe bus-servo control.

Run: ``python course.py check 14.03`` (or ``--solution`` to see the reference pass).

Everything runs against ``FakeArmBus``, so no hardware and no risk. Several tests check the
*order* of bus operations rather than the final pose: on a real arm the order is the difference
between a quiet start-up and a joint snapping 40 degrees at full speed.
"""

from __future__ import annotations

import pytest


def fresh(impl, **kwargs):
    """A simulated arm plus the default safety config."""
    return impl.FakeArmBus(**kwargs), impl.SafetyConfig()


def index_of(log, needle):
    return next(i for i, line in enumerate(log) if line == needle)


# --- register conversions --------------------------------------------------------------------------
def test_speed_register(impl):
    assert impl.speed_register(30.0) == 341          # 30 / 0.0879 deg per step
    assert impl.speed_register(12.0) == 137          # 14.03-E3 (a)
    assert impl.speed_register(1000 * impl.DEG_PER_STEP) == 1000     # 14.03-E3 (b), the other way


def test_speed_register_never_returns_zero(impl):
    """0 means "no limit". Rounding a small request down to 0 uncaps the servo."""
    assert impl.speed_register(0.001) == 1
    for bad in (0.0, -5.0):
        with pytest.raises(ValueError):
            impl.speed_register(bad)


def test_acceleration_register(impl):
    assert impl.acceleration_register(180.0) == 20   # 14.03-E3 (c): 2048 steps/s^2 / 100
    assert impl.acceleration_register(60.0) == 7


def test_acceleration_register_stays_in_range(impl):
    assert impl.acceleration_register(1e-6) == 1     # never 0, which would mean "maximum"
    assert impl.acceleration_register(1e9) == 254
    with pytest.raises(ValueError):
        impl.acceleration_register(0.0)


def test_torque_register(impl):
    assert impl.torque_register(30.0) == 300
    assert impl.torque_register(0.0) == 0
    assert impl.torque_register(100.0) == 1000
    for bad in (-1.0, 100.1):
        with pytest.raises(ValueError):
            impl.torque_register(bad)


def test_decode_sign_magnitude(impl):
    assert impl.decode_sign_magnitude(1036, 10) == -12   # 14.03-E3 (f): 1024 + 12, not +103.6 %
    assert impl.decode_sign_magnitude(12, 10) == 12
    assert impl.decode_sign_magnitude(0, 10) == 0
    assert impl.decode_sign_magnitude(1024, 10) == 0     # "negative zero" is still zero
    assert impl.decode_sign_magnitude((1 << 15) + 341, 15) == -341


# --- clamp_goals ------------------------------------------------------------------------------------
def test_clamp_goals_clamps_and_reports(impl):
    limits = impl.first_run_limits()
    clamped, notes = impl.clamp_goals({"wrist_flex": 45.0, "elbow_flex": -80.0}, limits)
    assert clamped == {"wrist_flex": 30.0, "elbow_flex": -30.0}
    assert sorted(notes) == ["elbow_flex: -80.0 clamped to -30.0", "wrist_flex: 45.0 clamped to 30.0"]


def test_clamp_goals_is_silent_when_nothing_changes(impl):
    limits = impl.first_run_limits()
    clamped, notes = impl.clamp_goals({"wrist_flex": 12.5, "gripper": 40.0}, limits)
    assert clamped == {"wrist_flex": 12.5, "gripper": 40.0}
    assert notes == []


def test_clamp_goals_refuses_a_joint_with_no_limits(impl):
    """Passing an unlimited joint through is how arms meet tables. It has to be an error."""
    limits = {k: v for k, v in impl.first_run_limits().items() if k != "elbow_flex"}
    with pytest.raises(impl.SafetyError):
        impl.clamp_goals({"elbow_flex": 5.0}, limits)


# --- health_problems --------------------------------------------------------------------------------
def test_a_healthy_arm_has_no_problems(impl):
    bus, cfg = fresh(impl)
    assert impl.health_problems(impl.read_telemetry(bus), cfg) == []


def test_a_twelve_volt_servo_on_the_five_volt_profile_is_caught(impl):
    bus, cfg = fresh(impl, voltage_v=12.1)
    problems = impl.health_problems(impl.read_telemetry(bus), cfg)
    assert len(problems) == len(bus.joint_names)
    assert "12.1" in problems[0] and problems[0].startswith(bus.joint_names[0])


def test_an_overheated_joint_is_caught(impl):
    bus, cfg = fresh(impl)
    bus.servos["elbow_flex"].temperature_c = 61
    problems = impl.health_problems(impl.read_telemetry(bus), cfg)
    assert len(problems) == 1 and problems[0].startswith("elbow_flex") and "61" in problems[0]


def test_an_overloaded_joint_is_caught(impl):
    bus, cfg = fresh(impl)
    impl.safe_enable_torque(bus, cfg)
    bus.servos["wrist_flex"].blocked_at = bus.servos["wrist_flex"].position
    bus.servos["wrist_flex"].goal = 25.0                       # pushing into the obstacle
    problems = impl.health_problems(impl.read_telemetry(bus), cfg)
    assert len(problems) == 1 and problems[0].startswith("wrist_flex")


# --- safe_enable_torque -----------------------------------------------------------------------------
def test_safe_enable_does_not_let_the_arm_jump(impl):
    """The failure this whole lesson exists to prevent: a stale Goal_Position register."""
    bus, cfg = fresh(impl, goal_register={"shoulder_lift": 25.0, "elbow_flex": -28.0})
    held = impl.safe_enable_torque(bus, cfg)
    assert bus.max_jump_deg == pytest.approx(0.0)
    assert held == {j: 0.0 for j in bus.joint_names} | {"gripper": 30.0}
    assert bus.read_positions() == held


def test_the_naive_order_really_does_jump(impl):
    """A control: enabling torque first throws the joint 25 degrees. The test above is not vacuous."""
    bus = impl.FakeArmBus(goal_register={"shoulder_lift": 25.0})
    bus.enable_torque()
    assert bus.max_jump_deg == pytest.approx(25.0)


def test_the_goal_is_written_before_torque_is_enabled(impl):
    bus, cfg = fresh(impl, goal_register={"shoulder_lift": 25.0})
    impl.safe_enable_torque(bus, cfg)
    goal_writes = [i for i, line in enumerate(bus.log) if line.startswith("goals ")]
    assert goal_writes, "safe_enable_torque must write Goal_Position before enabling torque"
    assert max(goal_writes) < index_of(bus.log, "torque on")


def test_torque_is_disabled_before_anything_else(impl):
    bus, cfg = fresh(impl)
    impl.safe_enable_torque(bus, cfg)
    assert bus.log[0] == "torque off"


def test_the_servo_limits_are_written_before_torque(impl):
    bus, cfg = fresh(impl)
    impl.safe_enable_torque(bus, cfg)
    on = index_of(bus.log, "torque on")
    for j in bus.joint_names:
        for register, value in (("Torque_Limit", impl.torque_register(cfg.torque_percent)),
                                ("Acceleration", impl.acceleration_register(cfg.accel_deg_s2)),
                                ("Goal_Velocity", impl.speed_register(cfg.speed_deg_s))):
            assert index_of(bus.log, f"{j}.{register}={value}") < on
    assert bus.servos["wrist_flex"].velocity == impl.speed_register(cfg.speed_deg_s)


def test_safe_enable_refuses_a_joint_outside_its_limits(impl):
    """A person moves it back by hand. Driving there would be the arm's first act of the session."""
    bus, cfg = fresh(impl, positions={"elbow_flex": 55.0})
    with pytest.raises(impl.SafetyError, match="elbow_flex"):
        impl.safe_enable_torque(bus, cfg)
    assert "torque on" not in bus.log


def test_safe_enable_refuses_an_unhealthy_arm(impl):
    bus, cfg = fresh(impl, voltage_v=12.1)
    with pytest.raises(impl.SafetyError):
        impl.safe_enable_torque(bus, cfg)
    assert "torque on" not in bus.log


# --- move_smoothly -----------------------------------------------------------------------------------
def test_move_smoothly_reaches_the_goal(impl):
    bus, cfg = fresh(impl)
    impl.safe_enable_torque(bus, cfg)
    final = impl.move_smoothly(bus, {"wrist_flex": 20.0}, cfg, sleep=bus.advance)
    assert final["wrist_flex"] == pytest.approx(20.0, abs=cfg.max_step_deg)
    assert final["shoulder_pan"] == pytest.approx(0.0, abs=1e-9), "untouched joints must not move"


def test_move_smoothly_respects_the_per_tick_step(impl):
    bus, cfg = fresh(impl)
    impl.safe_enable_torque(bus, cfg)
    commands: list[float] = []
    raw_write = bus.write_goal_positions

    def spy(goals):
        if "wrist_flex" in goals:
            commands.append(float(goals["wrist_flex"]))
        raw_write(goals)

    bus.write_goal_positions = spy
    impl.move_smoothly(bus, {"wrist_flex": 20.0}, cfg, sleep=bus.advance)
    assert len(commands) >= 20, "20 degrees at 1 degree per tick cannot take fewer than 20 ticks"
    steps = [b - a for a, b in zip(commands, commands[1:])]
    assert max(abs(s) for s in steps) <= cfg.max_step_deg + 1e-9


def test_move_smoothly_clamps_the_target_and_says_so(impl, capsys):
    bus, cfg = fresh(impl)
    impl.safe_enable_torque(bus, cfg)
    final = impl.move_smoothly(bus, {"wrist_flex": 90.0}, cfg, sleep=bus.advance)
    assert final["wrist_flex"] == pytest.approx(30.0, abs=cfg.max_step_deg)
    assert "wrist_flex: 90.0 clamped to 30.0" in capsys.readouterr().out


def test_move_smoothly_stops_when_a_joint_is_blocked(impl):
    bus, cfg = fresh(impl)
    impl.safe_enable_torque(bus, cfg)
    bus.servos["elbow_flex"].blocked_at = 10.0
    with pytest.raises(impl.SafetyError, match="elbow_flex"):
        impl.move_smoothly(bus, {"elbow_flex": 25.0}, cfg, sleep=bus.advance)
    assert bus.servos["elbow_flex"].position == pytest.approx(10.0)
    assert bus.servos["elbow_flex"].goal == pytest.approx(10.0), "stop pushing: goal = present"


def test_move_smoothly_gives_up_instead_of_looping_for_ever(impl):
    bus, cfg = fresh(impl)
    impl.safe_enable_torque(bus, cfg)
    with pytest.raises(impl.SafetyError, match="did not finish"):
        impl.move_smoothly(bus, {"wrist_flex": 30.0}, cfg, timeout_s=0.2, sleep=bus.advance)


def test_move_smoothly_refuses_a_joint_with_no_limits(impl):
    bus, cfg = fresh(impl)
    impl.safe_enable_torque(bus, cfg)
    narrow = impl.SafetyConfig(limits={k: v for k, v in impl.first_run_limits().items() if k != "wrist_flex"})
    with pytest.raises(impl.SafetyError):
        impl.move_smoothly(bus, {"wrist_flex": 10.0}, narrow, sleep=bus.advance)


# --- the whole start-up sequence, as 14.03-E5 describes it -------------------------------------------
def test_the_full_session_from_a_stale_register(impl):
    bus, cfg = fresh(impl, goal_register={"shoulder_lift": 25.0, "elbow_flex": -28.0})
    impl.safe_enable_torque(bus, cfg)
    impl.move_smoothly(bus, {"shoulder_lift": 15.0, "gripper": 50.0}, cfg, sleep=bus.advance)
    assert bus.max_jump_deg == pytest.approx(0.0)
    assert bus.servos["shoulder_lift"].position == pytest.approx(15.0, abs=cfg.max_step_deg)
    assert bus.servos["gripper"].position == pytest.approx(50.0, abs=cfg.max_step_deg)
    assert impl.health_problems(impl.read_telemetry(bus), cfg) == []
