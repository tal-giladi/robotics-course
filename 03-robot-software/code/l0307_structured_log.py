"""Lesson 03.07 — two log streams from one run: readable lines for you, JSON lines for a machine.

    python 03-robot-software/code/l0307_structured_log.py --output-dir runs/
    python 03-robot-software/code/l0307_structured_log.py --seconds 5 --level DEBUG

What it demonstrates, on a 4-second simulated run:

* **A run identity.** Every artifact of one run -- the events, the telemetry CSV, the plot, the
  bag -- shares a ``run_id`` and a metadata sidecar that records the git commit, the config file
  and its hash, the robot name and the software versions. A log you cannot tie to a code version
  and a calibration is an anecdote.
* **Two sinks, one call.** ``log.info("stopped", extra={"reason": ...})`` prints a line you can
  read over SSH and appends a JSON object a script can filter. You never write the same fact twice.
* **Rate limiting.** A 50 Hz loop must not log 50 lines per second. ``Throttle`` logs the first
  event immediately and then at most once per interval, reporting how many it suppressed.
* **Levels that mean something on a robot.** DEBUG = per-sample detail, INFO = state changes,
  WARNING = degraded but driving, ERROR = the robot stopped. Not "how interesting is this".

The run writes ``<run_id>.events.jsonl``, ``<run_id>.telemetry.csv`` and ``<run_id>.meta.json``.
With no ``--output-dir`` they go to a temporary directory that is printed and left behind.

Nothing here needs hardware. Tests: test_l0307_logging.py
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import logging
import platform
import subprocess
import sys
import tempfile
import time
import uuid
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, TextIO

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "labs" / "python"))

from robotlab.config import KarmelConfig, find_config_path, load_config  # noqa: E402
from robotlab.hal import FLAG_LOW_BATTERY, FLAG_WATCHDOG, BaseState  # noqa: E402

log = logging.getLogger("karmel")

TELEMETRY_COLUMNS = ["host_time_s", "t_s", "left_ticks", "right_ticks", "left_rad_s", "right_rad_s",
                     "battery_v", "range_m", "flags", "left_cmd", "right_cmd"]

# Fields LogRecord always has; anything else came from `extra=` and belongs in the JSON object.
_STANDARD_RECORD_FIELDS = set(logging.LogRecord("", 0, "", 0, "", None, None).__dict__) | {
    "message", "asctime", "taskName",
}


# --- run identity -----------------------------------------------------------------------------------
@dataclass(frozen=True)
class RunContext:
    """Who, what, when and from which code — written once per run, next to the data."""

    run_id: str
    started_at: str
    robot: str
    config_source: str
    config_sha256: str
    git_commit: str
    git_dirty: bool
    python: str
    platform: str
    notes: str = ""
    extra: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def git_commit(root: Path = REPO_ROOT) -> tuple[str, bool]:
    """(short commit, dirty). ('unknown', False) outside a git checkout — never raises."""
    def run(*args: str) -> str | None:
        try:
            out = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.SubprocessError):
            return None
        return out.stdout.strip() if out.returncode == 0 else None

    commit = run("rev-parse", "--short", "HEAD")
    if commit is None:
        return "unknown", False
    return commit, bool(run("status", "--porcelain"))


def file_sha256(path: Path) -> str:
    """The first 16 hex digits of the file's SHA-256 — enough to detect 'the config changed'."""
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()[:16]
    except OSError:
        return "unreadable"


def make_run_context(cfg: KarmelConfig, notes: str = "", **extra: Any) -> RunContext:
    commit, dirty = git_commit()
    source = cfg.source or find_config_path(None)
    return RunContext(
        run_id=f"{datetime.now(timezone.utc):%Y%m%dT%H%M%SZ}-{uuid.uuid4().hex[:6]}",
        started_at=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        robot=cfg.robot.name,
        config_source=str(source),
        config_sha256=file_sha256(Path(source)),
        git_commit=commit,
        git_dirty=dirty,
        python=platform.python_version(),
        platform=f"{platform.system()} {platform.machine()}",
        notes=notes,
        extra=dict(extra),
    )


# --- logging ----------------------------------------------------------------------------------------
def record_extras(record: logging.LogRecord) -> dict[str, Any]:
    """Everything the caller passed via ``extra=`` (plus anything a record factory added)."""
    return {k: v for k, v in record.__dict__.items()
            if k not in _STANDARD_RECORD_FIELDS and not k.startswith("_")}


class HumanFormatter(logging.Formatter):
    """``13:36:26 WARNING karmel  flags changed  t=0.32 flags=1 active=['watchdog']``.

    The same facts as the JSON line. Structured logging is not "log less for humans": it is
    "write the fact once, render it twice".
    """

    def __init__(self) -> None:
        super().__init__("%(asctime)s %(levelname)-7s %(name)s  %(message)s", "%H:%M:%S")

    def format(self, record: logging.LogRecord) -> str:
        line = super().format(record)
        extras = {k: v for k, v in record_extras(record).items() if k != "run_id"}
        if extras:
            line += "  " + " ".join(f"{k}={_short(v)}" for k, v in extras.items())
        return line


def _short(value: Any) -> str:
    return f"{value:.3f}" if isinstance(value, float) else str(value)


class JsonLinesFormatter(logging.Formatter):
    """One JSON object per line: the standard fields plus everything passed via ``extra=``."""

    def format(self, record: logging.LogRecord) -> str:
        payload: dict[str, Any] = {
            "ts": datetime.fromtimestamp(record.created, timezone.utc).isoformat(timespec="milliseconds"),
            "mono": round(time.monotonic(), 6),
            "level": record.levelname,
            "logger": record.name,
            "event": record.getMessage(),
        }
        payload.update(record_extras(record))
        if record.exc_info:
            payload["exception"] = self.formatException(record.exc_info)
        return json.dumps(payload, default=str)


def configure_logging(level: str = "INFO", jsonl_path: Path | None = None) -> logging.Handler | None:
    """Console (human) + optional JSON Lines file (machine). Returns the file handler, if any."""
    root = logging.getLogger()
    for handler in list(root.handlers):
        root.removeHandler(handler)
    root.setLevel(logging.DEBUG)

    console = logging.StreamHandler()
    console.setLevel(getattr(logging, level.upper()))
    console.setFormatter(HumanFormatter())
    root.addHandler(console)

    if jsonl_path is None:
        return None
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    file_handler = logging.FileHandler(jsonl_path, encoding="utf-8")
    file_handler.setLevel(logging.DEBUG)  # the file keeps everything; the console shows a summary
    file_handler.setFormatter(JsonLinesFormatter())
    root.addHandler(file_handler)
    return file_handler


class Throttle:
    """Let the first call through, then at most one per ``interval_s`` — and count the rest.

    >>> t = Throttle(1.0)
    >>> t.ready(0.0), t.ready(0.5), t.ready(1.1)
    (True, False, True)
    >>> t.suppressed                       # how many were dropped since the last allowed call
    0
    """

    def __init__(self, interval_s: float) -> None:
        self.interval_s = interval_s
        self._last: float | None = None
        self._suppressed = 0
        self.suppressed = 0  # suppressed count belonging to the most recent allowed call

    def ready(self, now: float) -> bool:
        if self._last is None or now - self._last >= self.interval_s:
            self._last = now
            self.suppressed, self._suppressed = self._suppressed, 0
            return True
        self._suppressed += 1
        return False


# --- telemetry sink ------------------------------------------------------------------------------------
class TelemetryWriter:
    """Append ``BaseState`` rows to a CSV with the same columns as labs/robot/log_telemetry.py."""

    def __init__(self, stream: TextIO) -> None:
        self._writer = csv.writer(stream)
        self._writer.writerow(TELEMETRY_COLUMNS)
        self._start = time.monotonic()
        self.rows = 0

    def write(self, state: BaseState, command: tuple[float, float] | None = None) -> None:
        self._writer.writerow([
            f"{time.monotonic() - self._start:.4f}", f"{state.t:.3f}",
            state.left_ticks, state.right_ticks,
            f"{state.left_rad_s:.3f}", f"{state.right_rad_s:.3f}",
            "" if state.battery_v is None else f"{state.battery_v:.3f}",
            "" if state.range_m is None else f"{state.range_m:.3f}",
            state.flags,
            "" if command is None else command[0], "" if command is None else command[1],
        ])
        self.rows += 1


def flag_names(flags: int) -> list[str]:
    """3 -> ['watchdog', 'low_battery'] — logs say what happened, not which bits were set."""
    names = {FLAG_WATCHDOG: "watchdog", FLAG_LOW_BATTERY: "low_battery", 4: "range_error", 8: "velocity_mode"}
    return [name for bit, name in sorted(names.items()) if flags & bit]


# --- the demo run -----------------------------------------------------------------------------------
def record_run(output_dir: Path, seconds: float = 4.0, level: str = "INFO",
               notes: str = "lesson 03.07 demo") -> dict[str, Any]:
    """Drive the simulator, log events to two sinks, write telemetry + metadata. Returns the paths."""
    from robotlab.sim import DiffDriveParams, DiffDriveSim, SensorParams, SimBase, World

    cfg = load_config()
    context = make_run_context(cfg, notes=notes, seconds=seconds)
    output_dir.mkdir(parents=True, exist_ok=True)
    events_path = output_dir / f"{context.run_id}.events.jsonl"
    telemetry_path = output_dir / f"{context.run_id}.telemetry.csv"
    meta_path = output_dir / f"{context.run_id}.meta.json"

    handler = configure_logging(level, events_path)
    # Every JSON line carries the run_id, so logs from several runs can be concatenated safely.
    old_factory = logging.getLogRecordFactory()

    def factory(*args: Any, **kwargs: Any) -> logging.LogRecord:
        record = old_factory(*args, **kwargs)
        record.run_id = context.run_id
        return record

    logging.setLogRecordFactory(factory)
    try:
        meta_path.write_text(json.dumps(context.as_dict(), indent=2), encoding="utf-8")
        # NB: no "run_id" here — the record factory already adds it, and logging refuses to let
        # `extra=` overwrite a field that is already on the record.
        log.info("run started", extra={"git": context.git_commit, "dirty": context.git_dirty,
                                       "config_sha256": context.config_sha256})

        sim = DiffDriveSim(World.rectangle_room(6.0, 3.0), DiffDriveParams.ideal(cfg),
                           SensorParams.ideal(cfg), pose=(1.0, 1.5, 0.0), seed=11)
        base = SimBase(sim, dt=0.02, watchdog_s=cfg.serial.watchdog_ms / 1000.0)
        heartbeat = Throttle(1.0)
        previous_flags = -1
        steps = int(seconds / 0.02)
        with telemetry_path.open("w", newline="", encoding="utf-8") as f:
            writer = TelemetryWriter(f)
            for step in range(steps):
                t = step * 0.02
                command = (8.0, 8.0) if 0.5 <= t < seconds - 1.0 else None
                if command is not None:
                    base.set_wheel_velocity(*command)
                state = base.read()
                writer.write(state, command)

                # DEBUG: every sample. The console never shows it; the JSONL file keeps it.
                log.debug("telemetry", extra={"t": state.t, "left_ticks": state.left_ticks,
                                              "right_ticks": state.right_ticks, "flags": state.flags})
                # INFO: only when something CHANGES. This is the line you read at 3 a.m.
                if state.flags != previous_flags:
                    changed = flag_names(state.flags)
                    level_for = log.warning if state.flags & (FLAG_WATCHDOG | FLAG_LOW_BATTERY) else log.info
                    level_for("flags changed", extra={"t": state.t, "flags": state.flags, "active": changed})
                    previous_flags = state.flags
                # Rate-limited progress, with the count of what was suppressed.
                if heartbeat.ready(state.t):
                    log.info("driving", extra={"t": state.t, "left_rad_s": round(state.left_rad_s, 3),
                                               "suppressed": heartbeat.suppressed})
            base.stop()
            base.close()
        log.info("run finished", extra={"rows": writer.rows, "sim_x": round(sim.pose.x, 4),
                                        "telemetry": telemetry_path.name})
    finally:
        logging.setLogRecordFactory(old_factory)
        if handler is not None:
            handler.close()
            logging.getLogger().removeHandler(handler)

    return {"run_id": context.run_id, "events": events_path, "telemetry": telemetry_path, "meta": meta_path}


def read_events(path: Path) -> list[dict[str, Any]]:
    """Parse a JSON Lines event file — what a log analysis script does."""
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output-dir", type=Path, help="where to write the run (default: a temp dir)")
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--level", default="INFO", choices=("DEBUG", "INFO", "WARNING", "ERROR"))
    args = parser.parse_args(argv)

    output = args.output_dir or Path(tempfile.mkdtemp(prefix="karmel-run-"))
    paths = record_run(output, seconds=args.seconds, level=args.level)
    events = read_events(paths["events"])
    counts: dict[str, int] = {}
    for event in events:
        counts[event["level"]] = counts.get(event["level"], 0) + 1
    print(f"\nrun {paths['run_id']}")
    print(f"  events    {paths['events']}  ({len(events)} lines: {counts})")
    print(f"  telemetry {paths['telemetry']}")
    print(f"  metadata  {paths['meta']}")
    print(f"  plot it:  python labs/robot/plot_telemetry.py {paths['telemetry']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
