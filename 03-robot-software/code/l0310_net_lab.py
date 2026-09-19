"""Lesson 03.10 — measure a link, then measure what that link does to a control loop.

Two experiments, both runnable with no hardware and no second machine:

    python 03-robot-software/code/l0310_net_lab.py link                    # loopback baseline
    python 03-robot-software/code/l0310_net_lab.py serve --port 5799       # on the Pi
    python 03-robot-software/code/l0310_net_lab.py link --host karmel.local --port 5799
    python 03-robot-software/code/l0310_net_lab.py control                 # latency vs stopping distance
    python 03-robot-software/code/l0310_net_lab.py control --partition 0.5

**link** — a UDP echo round trip, reported as mean / p50 / p90 / p99 / max, plus loss. UDP because
that is what DDS (and therefore every ROS 2 topic) rides on, and because TCP's retransmissions hide
exactly the tail you want to see. The number that matters for a robot is the MAX, not the mean.

**control** — the same 4 rad/s wall approach as lesson 03.05, run through ``DelayedBase``: a
``DifferentialBase`` decorator that holds every sensor reading and every command for N lockstep
steps, drops a fraction of commands, and can black out the link entirely for a while. Because the
simulator is lockstep, one step is exactly 20 ms, so the experiment is deterministic and the answer
is arithmetic you can check by hand.

Nothing here needs hardware or a network (``link`` defaults to loopback). Tests: test_l0310_network.py
"""

from __future__ import annotations

import argparse
import math
import socket
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "labs" / "python"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from robotlab.hal import FLAG_WATCHDOG, BaseState, DifferentialBase  # noqa: E402

PERCENTILES = (50, 90, 99)


# --- link measurement -------------------------------------------------------------------------------
@dataclass(frozen=True)
class LinkStats:
    name: str
    sent: int
    received: int
    rtt_ms: list[float]

    @property
    def loss_fraction(self) -> float:
        return 0.0 if self.sent == 0 else 1.0 - self.received / self.sent

    def percentile(self, pct: float) -> float:
        """Nearest-rank percentile — no numpy needed, and exact for small samples."""
        if not self.rtt_ms:
            return math.nan
        ordered = sorted(self.rtt_ms)
        rank = max(1, min(len(ordered), round(pct / 100.0 * len(ordered))))
        return ordered[rank - 1]

    def line(self) -> str:
        if not self.rtt_ms:
            return f"{self.name:<24} n=0  every packet lost"
        tail = "  ".join(f"p{p} {self.percentile(p):6.2f}" for p in PERCENTILES)
        return (f"{self.name:<24} n={self.received:4d}/{self.sent:<4d} "
                f"loss {100 * self.loss_fraction:4.1f}%  mean {statistics.mean(self.rtt_ms):6.2f} ms  "
                f"{tail}  max {max(self.rtt_ms):7.2f} ms")


class UdpEchoServer:
    """Echo every datagram straight back. 15 lines, and it is the whole 'other end'."""

    def __init__(self, host: str = "127.0.0.1", port: int = 0) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.2)
        self.host, self.port = self._sock.getsockname()[:2]
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="UdpEchoServer", daemon=True)

    def start(self) -> UdpEchoServer:
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._sock.close()

    def __enter__(self) -> UdpEchoServer:
        return self.start()

    def __exit__(self, *exc: object) -> None:
        self.stop()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                data, address = self._sock.recvfrom(2048)
            except (TimeoutError, OSError):
                continue
            try:
                self._sock.sendto(data, address)
            except OSError:
                pass


def measure_link(host: str, port: int, count: int = 200, payload_bytes: int = 64,
                 timeout_s: float = 0.5, interval_s: float = 0.005, name: str = "udp round trip") -> LinkStats:
    """Send `count` datagrams, time each round trip. A lost or late packet is counted, not retried."""
    payload = b"k" * max(8, payload_bytes - 8)
    rtts: list[float] = []
    received = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(timeout_s)
        for sequence in range(count):
            message = sequence.to_bytes(8, "big") + payload
            start = time.perf_counter()
            try:
                sock.sendto(message, (host, port))
                while True:
                    data, _ = sock.recvfrom(2048)
                    if data[:8] == message[:8]:       # ignore a late reply to an earlier packet
                        break
            except (TimeoutError, OSError):
                time.sleep(interval_s)
                continue
            # Stop the clock BEFORE the pacing sleep. Putting `time.sleep` in a `finally` and
            # measuring afterwards would add the whole interval to every sample — a real mistake,
            # and one that makes loopback look like 6 ms.
            rtts.append((time.perf_counter() - start) * 1000.0)
            received += 1
            time.sleep(interval_s)                    # pace the probes; do not flood the link
    return LinkStats(name, count, received, rtts)


# --- a link between your code and the robot ------------------------------------------------------------
@dataclass
class DelayedBase:
    """A ``DifferentialBase`` with a network in the middle: latency, loss, and blackouts.

    ``delay_steps`` holds both directions for N lockstep steps (20 ms each with the default
    ``SimBase``): ``read()`` returns a state from N steps ago, and a command sent now reaches the
    robot N steps from now. ``loss`` drops that fraction of commands. ``blackout`` is a window of
    simulated time in which nothing gets through in either direction — a Wi-Fi roam, a microwave
    oven, a wall.
    """

    inner: DifferentialBase
    delay_steps: int = 0
    loss: float = 0.0
    blackout: tuple[float, float] | None = None   # (start_t, end_t) in robot time
    seed: int = 0
    commands_sent: int = 0
    commands_delivered: int = 0
    watchdog_samples: int = 0                     # how often the ROBOT reported FLAG_WATCHDOG
    blind_samples: int = 0                        # reads that returned stale data (blackout)
    _outbound: list[tuple[str, float, float]] = field(default_factory=list, init=False)
    _inbound: list[BaseState] = field(default_factory=list, init=False)
    _last_delivered: BaseState | None = field(default=None, init=False)
    _t: float = field(default=0.0, init=False)

    def __post_init__(self) -> None:
        import random

        self._rng = random.Random(self.seed)

    # --- the network ------------------------------------------------------------------------------
    def _blacked_out(self) -> bool:
        return self.blackout is not None and self.blackout[0] <= self._t < self.blackout[1]

    def _queue(self, kind: str, left: float, right: float) -> None:
        self.commands_sent += 1
        if self._blacked_out() or self._rng.random() < self.loss:
            return                                   # the packet never arrives; nothing is retried
        self._outbound.append((kind, left, right))

    def _deliver(self) -> None:
        while len(self._outbound) > self.delay_steps:
            kind, left, right = self._outbound.pop(0)
            self.commands_delivered += 1
            if kind == "velocity":
                self.inner.set_wheel_velocity(left, right)
            elif kind == "duty":
                self.inner.set_wheel_duty(left, right)
            else:
                self.inner.stop()

    # --- DifferentialBase -------------------------------------------------------------------------
    def set_wheel_duty(self, left: float, right: float) -> None:
        self._queue("duty", left, right)

    def set_wheel_velocity(self, left_rad_s: float, right_rad_s: float) -> None:
        self._queue("velocity", left_rad_s, right_rad_s)

    def stop(self) -> None:
        self.inner.stop()                            # SAFETY: a stop is never delayed or dropped

    def read(self) -> BaseState:
        self._deliver()
        state = self.inner.read()
        self._t = state.t
        if state.flags & FLAG_WATCHDOG:
            self.watchdog_samples += 1
        if self._blacked_out() and self._last_delivered is not None:
            # Nothing arrives, so your code keeps seeing the LAST packet it got. This is the
            # dangerous failure: not an error, just numbers that stopped changing. SerialBase
            # guards against it with max_telemetry_age_s; a decorator like this one cannot.
            self.blind_samples += 1
            return self._last_delivered
        self._inbound.append(state)
        while len(self._inbound) > self.delay_steps + 1:
            self._inbound.pop(0)
        self._last_delivered = self._inbound[0]
        return self._last_delivered

    def close(self) -> None:
        self.inner.close()

    def __getattr__(self, name: str):
        return getattr(self.inner, name)


# --- the control experiment ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ApproachResult:
    delay_steps: int
    latency_ms: float
    stopped_at_m: float | None
    reason: str
    overshoot_m: float | None          # how much closer to the wall than the no-latency run
    watchdog_samples: int              # samples in which the robot reported FLAG_WATCHDOG
    blind_samples: int                 # reads that returned stale data
    commands_undelivered: int          # dropped, plus whatever was still in flight at the end


def run_approach(delay_steps: int = 0, loss: float = 0.0, blackout: tuple[float, float] | None = None,
                 stop_at_m: float = 0.5, speed_rad_s: float = 4.0, seed: int = 0,
                 baseline_m: float | None = None) -> ApproachResult:
    """Drive at a wall through a lossy, delayed link and report where the robot actually stopped."""
    from l0305_hal_demo import approach_wall
    from robotlab.fake_pico import default_sim
    from robotlab.sim import SimBase

    base = SimBase(default_sim("room", seed=seed), dt=0.02, watchdog_s=0.3)
    link = DelayedBase(base, delay_steps=delay_steps, loss=loss, blackout=blackout, seed=seed)
    try:
        result = approach_wall(link, stop_at_m, speed_rad_s)
        for _ in range(25):                          # 0.5 s of settling, so the wheels spin down
            base.read()
        true_range = base.sim.front_range()
    finally:
        base.close()
    stopped = true_range if true_range is not None else result.stopped_at_m
    overshoot = None if baseline_m is None or stopped is None else baseline_m - stopped
    return ApproachResult(delay_steps, delay_steps * 20.0, stopped, result.reason, overshoot,
                          link.watchdog_samples, link.blind_samples,
                          link.commands_sent - link.commands_delivered)


def latency_sweep(delays: list[int], loss: float = 0.0, seed: int = 0) -> list[ApproachResult]:
    baseline = run_approach(0, seed=seed).stopped_at_m
    return [run_approach(d, loss=loss, seed=seed, baseline_m=baseline) for d in delays]


# --- command line -----------------------------------------------------------------------------------
def cmd_link(args: argparse.Namespace) -> int:
    if args.host is None:
        with UdpEchoServer() as server:
            stats = measure_link(server.host, server.port, args.count, args.bytes,
                                 name="loopback (this machine)")
        print(stats.line())
        print("\nThis is your floor: the OS, the interpreter and nothing else. Every real link is\n"
              "this plus physics. Run `serve` on the Pi and measure again over Wi-Fi.")
        return 0
    stats = measure_link(args.host, args.port, args.count, args.bytes, name=f"{args.host}:{args.port}")
    print(stats.line())
    if stats.rtt_ms:
        print(f"\none-way estimate (half of p99): {stats.percentile(99) / 2:.2f} ms")
        print(f"at 0.5 m/s the robot travels {0.5 * stats.percentile(99) / 1000.0 * 100:.2f} cm "
              f"in one worst-case round trip")
    return 0 if stats.loss_fraction < 0.05 else 1


def cmd_serve(args: argparse.Namespace) -> int:
    with UdpEchoServer(args.bind, args.port) as server:
        print(f"UDP echo on {server.host}:{server.port} — Ctrl-C to stop")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("bye")
    return 0


def cmd_control(args: argparse.Namespace) -> int:
    if args.partition:
        baseline = run_approach(0).stopped_at_m
        start = args.partition_at
        result = run_approach(args.delay, loss=args.loss, blackout=(start, start + args.partition),
                              baseline_m=baseline)
        print(f"blackout {args.partition:.2f} s of robot time, starting at t = {start:.2f} s "
              f"(the un-blacked-out run reaches the wall at ~12.9 s)")
        delta = 100 * (result.overshoot_m or 0.0)
        print(f"  reason={result.reason}  stopped at {result.stopped_at_m:.3f} m  "
              f"({abs(delta):.1f} cm {'closer to' if delta > 0 else 'further from'} the wall than the clean run)")
        print(f"  blind reads (stale data)={result.blind_samples}  "
              f"watchdog samples={result.watchdog_samples}  undelivered commands={result.commands_undelivered}")
        print("\nTwo separate failures, and they do not start together: your code goes blind at once\n"
              "(the numbers simply stop changing), while the robot keeps driving on the last command\n"
              "until the firmware's 300 ms watchdog stops it. Past that point the watchdog wins, so a\n"
              "long blackout stops the robot EARLY, not late — and it does not resume by itself when\n"
              "the link comes back.")
        return 0

    delays = [int(d) for d in args.delays.split(",")]
    print(f"approach a wall at {args.speed:g} rad/s, stop at {args.stop_at:g} m, "
          f"through a link with added latency (loss {100 * args.loss:.0f} %)\n")
    print(f"{'delay':>6} {'latency':>9} {'stopped at':>11} {'overshoot':>10} {'undeliv':>8} {'wdog':>5}  reason")
    for result in latency_sweep(delays, loss=args.loss):
        overshoot = "-" if result.overshoot_m is None else f"{result.overshoot_m * 100:8.1f} cm"
        stopped = "-" if result.stopped_at_m is None else f"{result.stopped_at_m:9.3f} m"
        print(f"{result.delay_steps:>6} {result.latency_ms:>7.0f} ms {stopped} {overshoot} "
              f"{result.commands_undelivered:>8} {result.watchdog_samples:>5}  {result.reason}")
    print(f"\nPredicted overshoot per step of delay: speed x wheel radius x dt = "
          f"{args.speed:g} x 0.045 x 0.02 = {args.speed * 0.045 * 0.02 * 1000:.1f} mm")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    link = sub.add_parser("link", help="UDP round-trip latency, loopback or to a host")
    link.add_argument("--host", help="run against a remote echo server (default: loopback)")
    link.add_argument("--port", type=int, default=5799)
    link.add_argument("--count", type=int, default=200)
    link.add_argument("--bytes", type=int, default=64, help="payload size (a telemetry line is ~42 B)")
    link.set_defaults(func=cmd_link)

    serve = sub.add_parser("serve", help="run the UDP echo server (start this on the Pi)")
    serve.add_argument("--bind", default="0.0.0.0")
    serve.add_argument("--port", type=int, default=5799)
    serve.set_defaults(func=cmd_serve)

    control = sub.add_parser("control", help="what latency and loss do to a control loop")
    control.add_argument("--delays", default="0,1,2,5,10", help="latency in 20 ms steps, comma separated")
    control.add_argument("--loss", type=float, default=0.0, help="fraction of commands dropped")
    control.add_argument("--delay", type=int, default=0, help="delay to use with --partition")
    control.add_argument("--partition", type=float, help="blackout duration in seconds")
    control.add_argument("--partition-at", type=float, default=12.6,
                         help="robot time the blackout starts (default: just before the stop decision)")
    control.add_argument("--stop-at", type=float, default=0.5)
    control.add_argument("--speed", type=float, default=4.0)
    control.set_defaults(func=cmd_control)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
