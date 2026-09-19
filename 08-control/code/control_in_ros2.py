"""08.12 — Where control loops run, how fast, and what ROS 2 does about limits and timeouts.

    python 08-control/code/control_in_ros2.py --plot ros2_control.png
    python 08-control/code/control_in_ros2.py --port /dev/ttyACM0   # real robot, WHEELS IN THE AIR

No ROS 2 is needed to run this: it reproduces, on the course simulator, the three things the
ros2_control stack does to your commands, so that the YAML in
`labs/ros2_ws/src/karmel_bringup/config/controllers.yaml` stops being magic.

Part 1  The same 0 -> 8 rad/s wheel-speed step closed at three places: on the Pico (velocity mode,
        100 Hz), on the Pi at 50 Hz, and in a "ROS 2 node" at 20 Hz with 30 ms of extra latency.
Part 2  The latency budget of one command, term by term.
Part 3  diff_drive_controller's SpeedLimiter: what linear.x.max_acceleration actually does to a
        cmd_vel step, and what the wheels are asked for with and without it.
Part 4  The publisher dies. cmd_vel_timeout (0.5 s) vs the Pico's watchdog (300 ms).
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np

from control_lab import (
    Target, add_target_args, is_sim, let_wheels_stop, oscillation_std, plt_headless, run_loop, settling_time,
    steady_value,
)
from pid_lab import MotorModel, PositionalPID, wheel_speeds_for
from robotlab.config import load_config

SETPOINT, T_STEP = 8.0, 0.2
KP, KI = 0.084, 1.05  # the lambda = 50 ms design of 08.07, on the deadband-compensated plant


# --- what diff_drive_controller does to cmd_vel ------------------------------------------------------
@dataclass
class SpeedLimiter:
    """The velocity and acceleration limiter of ros2_controllers' diff_drive_controller.

    Field names mirror the Jazzy parameters ``linear.x.*`` / ``angular.z.*``; as in the C++ header,
    ``max_deceleration`` is normally NEGATIVE (m/s^2). ``limit`` is called once per
    controller_manager cycle with the period of that cycle, and clamps the command so that neither
    the speed nor its rate of change exceeds the configuration. (Jerk limits exist too; the same
    idea one derivative further out.)
    """

    max_velocity: float
    min_velocity: float
    max_acceleration: float  # >= 0
    max_deceleration: float  # <= 0
    previous: float = 0.0

    def limit(self, command: float, dt: float) -> float:
        wanted = min(max(command, self.min_velocity), self.max_velocity)
        change = wanted - self.previous
        self.previous += min(max(change, self.max_deceleration * dt), self.max_acceleration * dt)
        return self.previous

    def reset(self, value: float = 0.0) -> None:
        self.previous = value


# --- Part 1 ------------------------------------------------------------------------------------------
def firmware_loop(base, seconds: float):
    """Velocity mode: the Pico runs the PID at 100 Hz, we only send a setpoint."""
    def step(t, dt, state, extra):
        target = SETPOINT if t >= T_STEP - 1e-9 else 0.0
        extra["setpoint"] = target
        return target, 0.0

    log = run_loop(base, step, seconds, rate_hz=50.0, command="velocity")
    let_wheels_stop(base)
    return log


def host_loop(base, model, rate_hz: float, delay_steps: int, seconds: float):
    """The same PI, but running up here: slower, and with extra latency in the way."""
    controller = PositionalPID(KP, KI)
    controller.reset()

    def step(t, dt, state, extra):
        target = SETPOINT if t >= T_STEP - 1e-9 else 0.0
        u = controller.update(target, state.left_rad_s, dt)
        extra.update(setpoint=target, u=u)
        return model.compensate_deadband(u), 0.0

    log = run_loop(base, step, seconds, rate_hz=rate_hz, delay_steps=delay_steps)
    let_wheels_stop(base)
    return log


def metrics(log) -> dict[str, float]:
    t, w = log["t"], log["left_rad_s"]
    reached = [time for time, speed in zip(t, w) if time >= T_STEP and speed >= 0.95 * SETPOINT]
    return {"settled": steady_value(t, w, 0.4),
            "to_95": (reached[0] - T_STEP) if reached else math.inf,
            "overshoot": float(max(0.0, (np.max(w) - SETPOINT) / SETPOINT * 100.0)),
            "settling": settling_time(t, w, SETPOINT, 0.03 * SETPOINT, t_start=T_STEP),
            "wobble": oscillation_std(t, w, last_s=0.6)}


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_target_args(ap)
    args = ap.parse_args(argv)

    cfg = load_config()
    model = MotorModel.from_config(cfg)
    results: dict = {}
    plots: dict = {}
    seconds = 2.0

    with Target(args) as base:
        print("Part 1 - the same wheel-speed step, closed in three different places")
        print(f"  {'where the loop runs':34s} {'settled':>8s} {'to 95 %':>8s} {'overshoot':>10s} "
              f"{'settling':>9s} {'wobble':>7s}")
        runs = [("Pico, 100 Hz (velocity mode)", lambda: firmware_loop(base, seconds)),
                ("Pi, 50 Hz", lambda: host_loop(base, model, 50.0, 0, seconds)),
                ("ROS 2 node, 20 Hz + 30 ms latency", lambda: host_loop(base, model, 20.0, 1, seconds))]
        for name, run in runs:
            if is_sim(base):
                base.sim.reset((0.0, 0.0, 0.0))
            log = run()
            m = metrics(log)
            results[name] = m
            plots[name] = log
            print(f"  {name:34s} {m['settled']:8.2f} {m['to_95'] * 1000:6.0f}ms {m['overshoot']:9.1f}% "
                  f"{m['settling']:8.2f}s {m['wobble']:7.2f}")
        print("  (the Pico row's overshoot is its feedforward, which does not know the battery voltage —")
        print("   lesson 08.09, Part 2. Row 3 is the point of this lesson: same design, wrong place.)")

        # --- Part 2 -------------------------------------------------------------------------
        print("\nPart 2 - latency budget of one command, from 'Nav2 decided' to 'the motor changed'")
        budget = [("Nav2 controller period (20 Hz)", 50.0),
                  ("controller_manager cycle (update_rate 50 Hz)", 20.0),
                  ("serial frame, 115200 baud, ~40 bytes", 3.5),
                  ("Pico control period (100 Hz)", 10.0),
                  ("PWM period (20 kHz)", 0.05),
                  ("motor time constant (63 %)", model.tau_s * 1000.0),
                  ("speed estimate filter (alpha 0.5 at 100 Hz)", 10.0),
                  ("telemetry period (50 Hz)", 20.0)]
        for name, ms in budget:
            print(f"  {name:46s} {ms:7.2f} ms")
        command_path = sum(ms for name, ms in budget[:5])
        round_trip = command_path + sum(ms for name, ms in budget[5:])
        results["latency"] = {"command_path_ms": command_path, "round_trip_ms": round_trip}
        print(f"  {'command out (sum of the first five)':46s} {command_path:7.2f} ms")
        print(f"  {'until the Pi can SEE the result':46s} {round_trip:7.2f} ms")
        print("  A loop is stable only if its own period is well inside its dead time budget:")
        print(f"  the wheel-speed loop needs ~10 ms, so it lives on the Pico; a 0.5 m/s driving loop "
              f"needs ~100 ms, so it can live in ROS 2.")

        # --- Part 3 -------------------------------------------------------------------------
        print("\nPart 3 - diff_drive_controller limits: cmd_vel jumps 0 -> 0.5 m/s at 50 Hz")
        limiter = SpeedLimiter(cfg.drive.max_linear_speed_m_s, -cfg.drive.max_linear_speed_m_s,
                               cfg.drive.max_linear_accel_m_s2, -cfg.drive.max_linear_accel_m_s2)
        dt = 1.0 / 50.0
        limited = [limiter.limit(0.5, dt) for _ in range(30)]
        unlimited = [0.5] * 30
        radius, separation = cfg.drive.wheel_radius_m, cfg.drive.wheel_separation_m
        first_wheel_jump = wheel_speeds_for(0.5 / radius, 0.0, radius, separation)[1]
        results["limits"] = {"reach_time_s": (len([v for v in limited if v < 0.5 - 1e-9]) + 1) * dt,
                             "first_step_m_s": limited[0], "wheel_jump_rad_s": first_wheel_jump}
        print(f"  max_acceleration {cfg.drive.max_linear_accel_m_s2} m/s^2 -> the command may change by "
              f"{cfg.drive.max_linear_accel_m_s2 * dt:.3f} m/s per 20 ms cycle")
        print(f"  limited:   " + " ".join(f"{v:.2f}" for v in limited[:12]) + " ...")
        print(f"  unlimited: " + " ".join(f"{v:.2f}" for v in unlimited[:12]) + " ...")
        print(f"  reaching 0.5 m/s takes {results['limits']['reach_time_s']:.2f} s; without the limit the "
              f"wheels are asked to jump to {first_wheel_jump:.2f} rad/s in one cycle")

        # --- Part 4 -------------------------------------------------------------------------
        print("\nPart 4 - the publisher dies after 1 s: two timeouts, and which one saves you")
        timeout = 0.5  # controllers.yaml: cmd_vel_timeout
        watchdog = cfg.serial.watchdog_ms / 1000.0
        stop_at = {"cmd_vel_timeout (diff_drive_controller)": 1.0 + timeout,
                   "Pico watchdog (firmware)": 1.0 + watchdog}
        for name, when in sorted(stop_at.items(), key=lambda kv: kv[1]):
            print(f"  {name:42s} stops the wheels at t = {when:.2f} s")
        results["timeouts"] = stop_at
        if is_sim(base):
            # Drive the simulator directly (run_loop always stops the motors at the end, which is
            # exactly what we must NOT do here): 1 s of commands, then total silence.
            base.sim.reset((0.0, 0.0, 0.0))
            for _ in range(round(1.0 / base.dt)):
                base.set_wheel_velocity(SETPOINT, SETPOINT)
                base.read()
            last_command_t = base.sim.t
            trace = []
            for _ in range(round(1.0 / base.dt)):  # nobody sends anything any more
                state = base.read()
                trace.append((base.sim.t - last_command_t, float(base.sim.wheel_rad_s[0]), state.flags))
            flagged = next((t for t, _, flags in trace if flags & 1), math.nan)
            half = next((t for t, w, _ in trace if w < 0.5 * SETPOINT), math.nan)
            stopped = next((t for t, w, _ in trace if w < 0.05 * SETPOINT), math.nan)
            results["watchdog"] = {"flag_s": flagged, "half_speed_s": half, "stopped_s": stopped}
            print(f"  measured on the simulator: FLAG_WATCHDOG at {flagged:.2f} s after the last command, "
                  f"below half speed at {half:.2f} s, effectively stopped at {stopped:.2f} s")
            plots["watchdog"] = trace
            base.stop()

    if args.plot and plots:
        plt = plt_headless()
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        ax = axes[0]
        for name in [r[0] for r in runs]:
            ax.plot(plots[name]["t"], plots[name]["left_rad_s"], lw=1.3, label=name)
        ax.axhline(SETPOINT, color="k", ls="--", lw=0.8)
        ax.set(xlabel="time [s]", ylabel="left wheel [rad/s]", title="the same loop, three places")
        ax.grid(True)
        ax.legend(fontsize=8, loc="lower right")
        ax = axes[1]
        t = np.arange(len(limited)) * dt
        ax.step(t, unlimited, where="post", lw=1.2, label="cmd_vel as published")
        ax.step(t, limited, where="post", lw=1.6, label="after SpeedLimiter (1.0 m/s^2)")
        ax.set(xlabel="time [s]", ylabel="linear.x [m/s]", title="what the acceleration limit does")
        ax.grid(True)
        ax.legend(fontsize=8, loc="lower right")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=90)
        print(f"saved {args.plot}")
    return results


if __name__ == "__main__":
    main()
