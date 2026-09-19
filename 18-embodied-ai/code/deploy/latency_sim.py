"""latency_sim.py - what network and inference latency do to action-chunk execution (lessons 18.08, 18.10).

A robot loop ticks at `fps`. A policy server (a GPU machine on the LAN, or a cloud GPU) turns an
observation into a chunk of `chunk` future actions, one per tick. The simulator replays the robot
loop tick by tick and reports what the arm actually executed.

Standard library only.

  py latency_sim.py compare                     the lesson's table: local GPU, LAN, Wi-Fi, cloud, CPU

All latencies in the scenarios are ASSUMPTIONS for teaching. Measure yours (lesson 18.08) and pass
them with --infer-ms / --rtt-ms / --obs-kb / --uplink-mbps.
  py latency_sim.py run --mode async --fps 30 --chunk 50 --infer-ms 120 --rtt-ms 40 --threshold 0.5
  py latency_sim.py sweep --infer-ms 120 --rtt-ms 40     async threshold sweep
  py latency_sim.py budget --fps 30 --infer-ms 120 --rtt-ms 40 --obs-kb 1800 --uplink-mbps 40

The task: the gripper must follow a target that moves smoothly (a slowly swinging object, in
metres) and that jumps 5 cm at t = 3 s (someone nudges it). The "policy" is perfect: it knows
the smooth swing exactly (as if it had learned it from demonstrations) and it sees the nudge in
any observation taken after t = 3 s. So every error you see is caused by latency, blocked ticks
and late or dropped actions - not by learning.

Modes
  sync   the control loop calls the policy when its action queue is empty and BLOCKS until the
         chunk returns; the returned actions are then played one per tick (late).
  async  the loop never blocks. When the queue falls to `threshold * chunk` actions, it sends a
         new observation in the background. Returned actions are aligned by timestamp: actions
         for ticks that have already passed are discarded; overlaps with the old queue are
         replaced ("latest") or averaged ("average"). Simplification: at most one request is in
         flight at a time, so a chunk must hold more than 2 * latency * fps actions or the arm starves.
"""
from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass, field, replace


@dataclass(frozen=True)
class Scenario:
    name: str = "custom"
    mode: str = "async"            # "sync" | "async"
    fps: float = 30.0              # robot control rate, Hz
    chunk: int = 50                # actions per chunk
    threshold: float = 0.5         # async: request a new chunk when queue <= threshold * chunk
    infer_ms: float = 100.0        # policy forward pass on the server, ms
    rtt_ms: float = 5.0            # network round trip without payload, ms
    jitter_ms: float = 2.0         # mean of an exponential extra delay per request, ms
    spike_prob: float = 0.0        # probability that a request hits a Wi-Fi stall
    spike_ms: float = 250.0        # length of that stall, ms
    obs_kb: float = 100.0          # observation payload size (images + state), kB
    uplink_mbps: float = 1000.0    # uplink bandwidth for the observation, Mbit/s
    aggregate: str = "latest"      # "latest" | "average"
    duration_s: float = 20.0
    nudge_t: float = 3.0           # when the target is nudged, s
    seed: int = 1


def request_latency_s(sc: Scenario, rng: random.Random) -> float:
    """Observation capture -> chunk available on the robot, seconds."""
    upload_s = sc.obs_kb * 8.0 / 1000.0 / sc.uplink_mbps          # kB -> Mbit, / Mbit/s
    extra = rng.expovariate(1.0 / sc.jitter_ms) if sc.jitter_ms > 0 else 0.0
    spike = sc.spike_ms if rng.random() < sc.spike_prob else 0.0
    return upload_s + (sc.infer_ms + sc.rtt_ms + extra + spike) / 1000.0


NUDGE_M = 0.05


def swing(t: float) -> tuple[float, float]:
    """The smooth, predictable part of the target: position (m) and velocity (m/s)."""
    w = 2 * math.pi * 0.25
    return 0.08 * math.sin(w * t), 0.08 * w * math.cos(w * t)


def nudge(t: float, nudge_t: float) -> float:
    """The unpredictable part: a 5 cm step at t = nudge_t."""
    return NUDGE_M if t >= nudge_t else 0.0


def target(t: float, nudge_t: float) -> tuple[float, float]:
    """Target position (m) and velocity (m/s)."""
    x, v = swing(t)
    return x + nudge(t, nudge_t), v


@dataclass
class Result:
    scenario: Scenario
    ticks: int = 0
    idle_ticks: int = 0
    requests: int = 0
    stale_ms: list[float] = field(default_factory=list)   # age of the observation behind each executed action
    err_m: list[float] = field(default_factory=list)
    max_jump_m: float = 0.0
    reaction_s: float = float("nan")

    @property
    def idle_pct(self) -> float:
        return 100.0 * self.idle_ticks / max(1, self.ticks)

    @property
    def rms_err_mm(self) -> float:
        return 1000.0 * math.sqrt(sum(e * e for e in self.err_m) / max(1, len(self.err_m)))

    def stale_p(self, q: float) -> float:
        if not self.stale_ms:
            return float("nan")
        s = sorted(self.stale_ms)
        return s[min(len(s) - 1, int(q * len(s)))]


def simulate(sc: Scenario) -> Result:
    rng = random.Random(sc.seed)
    dt = 1.0 / sc.fps
    n_ticks = int(sc.duration_s * sc.fps)
    res = Result(sc)

    queue: dict[int, tuple[float, float]] = {}   # tick -> (action, observation time)
    pending: list[tuple[float, int, list[float]]] = []  # (arrival time, obs tick, actions)
    cmd = target(0.0, sc.nudge_t)[0]
    prev_cmd = cmd
    cmd_obs_t = 0.0                              # observation time behind the current command
    blocked_until = -1.0
    sync_play: list[tuple[float, float]] = []    # sync mode: actions still to play, with obs time

    def predict(obs_tick: int) -> list[float]:
        t_obs = obs_tick * dt
        return [swing(t_obs + j * dt)[0] + nudge(t_obs, sc.nudge_t) for j in range(sc.chunk)]

    for k in range(n_ticks):
        t = k * dt
        res.ticks += 1
        executed = False

        if sc.mode == "sync":
            if t < blocked_until:
                pass                                          # loop is blocked inside the policy call
            elif sync_play:
                cmd, t_obs = sync_play.pop(0)
                cmd_obs_t = t_obs
                res.stale_ms.append(1000.0 * (t - t_obs))
                executed = True
            else:
                lat = request_latency_s(sc, rng)
                res.requests += 1
                blocked_until = t + lat
                sync_play = [(a, t) for a in predict(k)]
                if lat < dt:                                  # fast enough: act in this tick
                    cmd, t_obs = sync_play.pop(0)
                    cmd_obs_t = t_obs
                    res.stale_ms.append(1000.0 * (t - t_obs))
                    executed = True
        else:
            for item in [p for p in pending if p[0] <= t]:    # deliver chunks that have arrived
                pending.remove(item)
                _, obs_tick, actions = item
                for j, a in enumerate(actions):
                    tick = obs_tick + j
                    if tick < k:
                        continue                              # already in the past: drop
                    if sc.aggregate == "average" and tick in queue:
                        a = 0.5 * (a + queue[tick][0])
                    queue[tick] = (a, obs_tick * dt)
            for old in [i for i in queue if i < k]:
                del queue[old]
            remaining = len(queue)
            if not pending and remaining <= sc.threshold * sc.chunk:
                lat = request_latency_s(sc, rng)
                res.requests += 1
                pending.append((t + lat, k, predict(k)))
            if k in queue:
                cmd, t_obs = queue.pop(k)
                cmd_obs_t = t_obs
                res.stale_ms.append(1000.0 * (t - t_obs))
                executed = True

        if not executed:
            res.idle_ticks += 1                               # the arm holds its last command
        x_true, v_true = target(t, sc.nudge_t)
        res.err_m.append(cmd - x_true)
        # jump = change of command beyond what the target moved in one tick; the 5 cm nudge
        # itself is removed so only chunk-switch discontinuities and idle catch-ups count
        smooth_cmd = cmd - nudge(cmd_obs_t, sc.nudge_t)
        if k > 0:
            res.max_jump_m = max(res.max_jump_m, abs((smooth_cmd - prev_cmd) - v_true * dt))
        prev_cmd = smooth_cmd
        if cmd_obs_t >= sc.nudge_t and math.isnan(res.reaction_s) and abs(cmd - x_true) < 0.01:
            res.reaction_s = t - sc.nudge_t
    return res


def mean_reaction_s(sc: Scenario, trials: int = 20) -> float:
    """Average time to react to the nudge, over nudges at different phases of the request cycle.
    NaN if the arm never reacted in at least one trial."""
    times = [simulate(replace(sc, nudge_t=3.0 + 2.0 * i / trials, seed=sc.seed + i)).reaction_s for i in range(trials)]
    return float("nan") if any(math.isnan(x) for x in times) else sum(times) / len(times)


def fmt_row(r: Result) -> str:
    sc = r.scenario
    return (f"{sc.name:<28} {sc.mode:<5} {r.idle_pct:6.1f} {r.stale_p(0.5):8.0f} {r.stale_p(0.95):8.0f} "
            f"{r.rms_err_mm:8.1f} {1000 * r.max_jump_m:8.1f} {1000 * mean_reaction_s(sc):9.0f} {r.requests:5d}")


HEADER = (f"{'scenario':<28} {'mode':<5} {'idle%':>6} {'age p50':>8} {'age p95':>8} "
          f"{'RMS mm':>8} {'jump mm':>8} {'react ms':>9} {'reqs':>5}")


def scenarios() -> list[Scenario]:
    base = Scenario(fps=30, chunk=50, threshold=0.5)
    return [
        replace(base, name="GPU in the robot/laptop", infer_ms=60, rtt_ms=0.5, jitter_ms=0.5, obs_kb=0),
        replace(base, name="desktop GPU, Ethernet LAN", infer_ms=60, rtt_ms=1, jitter_ms=1, obs_kb=1800, uplink_mbps=1000),
        replace(base, name="desktop GPU, Wi-Fi, raw 2xVGA", infer_ms=60, rtt_ms=8, jitter_ms=10, spike_prob=0.03,
                obs_kb=1800, uplink_mbps=40),
        replace(base, name="desktop GPU, Wi-Fi, 100 kB", infer_ms=60, rtt_ms=8, jitter_ms=10, spike_prob=0.03,
                obs_kb=100, uplink_mbps=40),
        replace(base, name="cloud GPU (Europe)", infer_ms=60, rtt_ms=90, jitter_ms=20, spike_prob=0.01,
                obs_kb=100, uplink_mbps=20),
        replace(base, name="on the Pi 5 CPU (2.5 s)", infer_ms=2500, rtt_ms=0, jitter_ms=100, obs_kb=0),
    ]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("compare")
    for name in ("run", "sweep", "budget"):
        s = sub.add_parser(name)
        s.add_argument("--mode", default="async", choices=["sync", "async"])
        s.add_argument("--fps", type=float, default=30.0)
        s.add_argument("--chunk", type=int, default=50)
        s.add_argument("--threshold", type=float, default=0.5)
        s.add_argument("--infer-ms", type=float, default=100.0)
        s.add_argument("--rtt-ms", type=float, default=5.0)
        s.add_argument("--jitter-ms", type=float, default=2.0)
        s.add_argument("--spike-prob", type=float, default=0.0)
        s.add_argument("--obs-kb", type=float, default=100.0)
        s.add_argument("--uplink-mbps", type=float, default=1000.0)
        s.add_argument("--aggregate", default="latest", choices=["latest", "average"])
        s.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    if a.cmd == "compare":
        print("fps=30, chunk=50, threshold=0.5; 'age' = observation age of the executed action (ms)\n")
        print(HEADER)
        for sc in scenarios():
            for mode in ("sync", "async"):
                print(fmt_row(simulate(replace(sc, mode=mode))))
        return

    sc = Scenario(mode=a.mode, fps=a.fps, chunk=a.chunk, threshold=a.threshold, infer_ms=a.infer_ms,
                  rtt_ms=a.rtt_ms, jitter_ms=a.jitter_ms, spike_prob=a.spike_prob, obs_kb=a.obs_kb,
                  uplink_mbps=a.uplink_mbps, aggregate=a.aggregate, seed=a.seed)
    if a.cmd == "run":
        print(HEADER)
        print(fmt_row(simulate(sc)))
    elif a.cmd == "sweep":
        print(HEADER)
        for th in (0.0, 0.1, 0.2, 0.3, 0.5, 0.7, 0.9):
            print(fmt_row(simulate(replace(sc, name=f"threshold {th:.1f}", threshold=th))))
    else:
        upload_ms = a.obs_kb * 8.0 / a.uplink_mbps
        total_ms = upload_ms + a.infer_ms + a.rtt_ms + a.jitter_ms
        ticks = total_ms / 1000.0 * a.fps
        print(f"upload {upload_ms:.0f} ms + inference {a.infer_ms:.0f} ms + network {a.rtt_ms:.0f} ms "
              f"+ mean jitter {a.jitter_ms:.0f} ms = {total_ms:.0f} ms")
        print(f"at {a.fps:.0f} Hz that is {ticks:.1f} ticks: the queue must still hold >= {math.ceil(ticks)} actions "
              f"when you ask -> threshold >= {math.ceil(ticks) / a.chunk:.2f} for a {a.chunk}-action chunk")
        print(f"the first useful action of each chunk is already {total_ms:.0f} ms old; "
              f"a {a.chunk}-action chunk covers {a.chunk / a.fps:.2f} s")


if __name__ == "__main__":
    main()
