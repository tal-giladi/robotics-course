"""Tests for the lesson 03.07 structured-logging demo — no hardware, no network.

    python -m pytest 03-robot-software/code/test_l0307_logging.py
"""

from __future__ import annotations

import json
import logging
from pathlib import Path
from typing import Iterator

import pytest

import l0307_structured_log as slog
from robotlab.config import load_config
from robotlab.hal import FLAG_LOW_BATTERY, FLAG_VELOCITY_MODE, FLAG_WATCHDOG


@pytest.fixture(autouse=True)
def restore_root_logger() -> Iterator[None]:
    """configure_logging() replaces the root handlers; put pytest's back afterwards."""
    root = logging.getLogger()
    handlers, level = list(root.handlers), root.level
    factory = logging.getLogRecordFactory()
    yield
    for handler in list(root.handlers):
        root.removeHandler(handler)
    for handler in handlers:
        root.addHandler(handler)
    root.setLevel(level)
    logging.setLogRecordFactory(factory)


def record(**extra: object) -> logging.LogRecord:
    r = logging.LogRecord("karmel", logging.WARNING, __file__, 1, "flags changed", None, None)
    r.__dict__.update(extra)
    return r


# --- throttling ------------------------------------------------------------------------------------
def test_throttle_passes_the_first_call_then_one_per_interval() -> None:
    t = slog.Throttle(1.0)
    assert t.ready(0.0) is True
    assert [t.ready(x) for x in (0.2, 0.4, 0.6, 0.8)] == [False] * 4
    assert t.ready(1.0) is True
    assert t.suppressed == 4, "the allowed call reports how many were dropped before it"
    assert t.ready(1.5) is False
    assert t.ready(2.0) is True and t.suppressed == 1


def test_throttle_at_50_hz_keeps_one_line_per_second() -> None:
    t = slog.Throttle(1.0)
    allowed = [i for i in range(250) if t.ready(i * 0.02)]  # 5 s at 50 Hz
    assert len(allowed) == 5, "250 samples must not become 250 log lines"


# --- formatters -------------------------------------------------------------------------------------
def test_json_formatter_emits_one_object_with_the_extras() -> None:
    line = slog.JsonLinesFormatter().format(record(t=0.32, flags=1, active=["watchdog"]))
    payload = json.loads(line)
    assert payload["event"] == "flags changed" and payload["level"] == "WARNING"
    assert (payload["t"], payload["flags"], payload["active"]) == (0.32, 1, ["watchdog"])
    assert payload["ts"].endswith("+00:00"), "timestamps are UTC and explicit about it"
    assert "\n" not in line, "JSON Lines: exactly one line per event"


def test_json_formatter_never_raises_on_an_unserializable_extra() -> None:
    payload = json.loads(slog.JsonLinesFormatter().format(record(base=object())))
    assert payload["base"].startswith("<object")


def test_human_formatter_shows_the_same_facts() -> None:
    line = slog.HumanFormatter().format(record(t=0.32, flags=1, active=["watchdog"], run_id="abc"))
    assert "flags changed" in line and "t=0.320" in line and "active=['watchdog']" in line
    assert "run_id" not in line, "the run id is in the file name and the JSON, not in every console line"


def test_configure_logging_writes_json_lines(tmp_path: Path) -> None:
    path = tmp_path / "events.jsonl"
    handler = slog.configure_logging("WARNING", path)
    logging.getLogger("karmel").debug("telemetry", extra={"t": 1.0})
    logging.getLogger("karmel").error("stopped", extra={"reason": "obstacle"})
    handler.close()
    events = slog.read_events(path)
    assert [e["event"] for e in events] == ["telemetry", "stopped"], "the FILE keeps DEBUG too"
    assert events[-1]["reason"] == "obstacle"


# --- run identity -------------------------------------------------------------------------------------
def test_flag_names_decodes_the_bitfield() -> None:
    assert slog.flag_names(0) == []
    assert slog.flag_names(FLAG_WATCHDOG) == ["watchdog"]
    assert slog.flag_names(FLAG_WATCHDOG | FLAG_LOW_BATTERY) == ["watchdog", "low_battery"]
    assert slog.flag_names(FLAG_VELOCITY_MODE) == ["velocity_mode"]


def test_git_commit_never_raises() -> None:
    commit, dirty = slog.git_commit()
    assert isinstance(commit, str) and commit
    assert isinstance(dirty, bool)


def test_git_commit_outside_a_checkout_is_unknown(tmp_path: Path) -> None:
    assert slog.git_commit(tmp_path)[0] in {"unknown", slog.git_commit()[0]}  # nested repos exist


def test_file_sha256_changes_with_the_content(tmp_path: Path) -> None:
    a, b = tmp_path / "a.yaml", tmp_path / "b.yaml"
    a.write_text("wheel_radius_m: 0.045\n", encoding="utf-8")
    b.write_text("wheel_radius_m: 0.0446\n", encoding="utf-8")
    assert len(slog.file_sha256(a)) == 16
    assert slog.file_sha256(a) != slog.file_sha256(b)
    assert slog.file_sha256(tmp_path / "missing.yaml") == "unreadable"


def test_run_context_records_the_provenance() -> None:
    context = slog.make_run_context(load_config(), notes="unit test", seconds=1.0)
    assert context.robot == "karmel"
    assert context.config_source.endswith("karmel.yaml")
    assert context.extra == {"seconds": 1.0}
    assert len(context.run_id) > 15 and context.run_id.endswith(context.run_id[-6:])
    assert set(context.as_dict()) >= {"run_id", "git_commit", "config_sha256", "started_at"}


# --- the whole run ---------------------------------------------------------------------------------------
def test_record_run_writes_three_artifacts_that_agree(tmp_path: Path) -> None:
    paths = slog.record_run(tmp_path, seconds=2.0, level="WARNING")
    for key in ("events", "telemetry", "meta"):
        assert paths[key].is_file(), key
        assert paths[key].name.startswith(paths["run_id"]), "one run id ties the artifacts together"

    meta = json.loads(paths["meta"].read_text(encoding="utf-8"))
    assert meta["run_id"] == paths["run_id"] and meta["robot"] == "karmel"

    events = slog.read_events(paths["events"])
    assert all(e["run_id"] == paths["run_id"] for e in events)
    names = [e["event"] for e in events]
    assert names[0] == "run started" and names[-1] == "run finished"
    assert names.count("telemetry") == 100, "2 s at 50 Hz, one DEBUG record per sample"

    # The watchdog must trip twice: before the first command, and after the last one.
    watchdogs = [e for e in events if e["event"] == "flags changed" and "watchdog" in e.get("active", [])]
    assert len(watchdogs) == 2, [e["t"] for e in watchdogs]
    assert watchdogs[0]["t"] == pytest.approx(0.32, abs=0.02)

    rows = paths["telemetry"].read_text(encoding="utf-8").splitlines()
    assert rows[0].split(",") == slog.TELEMETRY_COLUMNS
    assert len(rows) == 101, "header + 100 samples"


def test_main_runs_and_reports(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert slog.main(["--output-dir", str(tmp_path), "--seconds", "1", "--level", "ERROR"]) == 0
    out = capsys.readouterr().out
    assert "events" in out and "telemetry" in out and "plot it" in out
