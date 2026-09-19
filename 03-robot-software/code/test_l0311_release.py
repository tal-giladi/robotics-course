"""Tests for the lesson 03.11 release recorder — no hardware, no network.

    python -m pytest 03-robot-software/code/test_l0311_release.py

Every test that touches git builds its own throwaway repository in ``tmp_path``, so the suite
does not depend on the state of this checkout (which is dirty while you are writing a lesson).
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import pytest

import l0311_release as rel

HAS_GIT = rel.git("rev-parse", "HEAD") is not None


def make_repo(tmp_path: Path) -> Path:
    root = tmp_path / "robot"
    (root / "labs" / "config").mkdir(parents=True)
    (root / "labs" / "config" / "karmel.yaml").write_text("wheel_radius_m: 0.045\n", encoding="utf-8")
    (root / "labs" / "requirements.txt").write_text("numpy>=1.26\n", encoding="utf-8")
    for args in (("init", "-b", "main"), ("config", "user.email", "t@example.invalid"),
                 ("config", "user.name", "Test"), ("add", "-A"), ("commit", "-m", "initial")):
        subprocess.run(["git", "-C", str(root), *args], capture_output=True, check=True)
    return root


FILES = ("labs/config/karmel.yaml", "labs/requirements.txt")

pytestmark = pytest.mark.skipif(not HAS_GIT, reason="git is not available")


# --- hashing ------------------------------------------------------------------------------------
def test_file_sha256_is_stable_and_content_dependent(tmp_path: Path) -> None:
    a, b = tmp_path / "a", tmp_path / "b"
    a.write_text("wheel_radius_m: 0.045\n", encoding="utf-8")
    b.write_text("wheel_radius_m: 0.0446\n", encoding="utf-8")
    assert rel.file_sha256(a) == rel.file_sha256(a)
    assert rel.file_sha256(a) != rel.file_sha256(b)
    assert len(rel.file_sha256(a)) == 64


def test_a_missing_file_is_recorded_not_raised(tmp_path: Path) -> None:
    assert rel.file_sha256(tmp_path / "nope.yaml") == "missing"
    assert rel.hash_files(("nope.yaml",), root=tmp_path) == {"nope.yaml": "missing"}


# --- git state ----------------------------------------------------------------------------------
def test_a_fresh_commit_is_clean(tmp_path: Path) -> None:
    state = rel.git_state(make_repo(tmp_path))
    assert state.commit and len(state.commit) == 40
    assert state.short and state.branch == "main"
    assert state.dirty is False and state.dirty_files == []
    assert state.tag is None


def test_an_edited_file_makes_the_tree_dirty(tmp_path: Path) -> None:
    root = make_repo(tmp_path)
    (root / "labs" / "config" / "karmel.yaml").write_text("wheel_radius_m: 0.0446\n", encoding="utf-8")
    state = rel.git_state(root)
    assert state.dirty is True
    assert "labs/config/karmel.yaml" in state.dirty_files
    assert state.describe and state.describe.endswith("-dirty")


def test_a_tag_is_picked_up(tmp_path: Path) -> None:
    root = make_repo(tmp_path)
    subprocess.run(["git", "-C", str(root), "tag", "v0.3.0"], capture_output=True, check=True)
    state = rel.git_state(root)
    assert state.tag == "v0.3.0"
    assert state.describe == "v0.3.0"


def test_git_state_outside_a_checkout_does_not_raise(tmp_path: Path) -> None:
    state = rel.git_state(tmp_path)              # a bare directory, no repository
    assert state.commit is None and state.dirty is False


# --- the record ---------------------------------------------------------------------------------
def test_make_release_captures_code_environment_and_robot(tmp_path: Path) -> None:
    root = make_repo(tmp_path)
    release = rel.make_release(label="demo", notes="unit test", root=root, files=FILES)
    assert release.label == "demo"
    assert release.git["branch"] == "main" and release.git["dirty"] is False
    assert set(release.files) == set(FILES)
    assert all(len(digest) == 64 for digest in release.files.values())
    assert release.python and release.platform
    assert release.as_dict()["notes"] == "unit test"


def test_the_label_defaults_to_the_tag(tmp_path: Path) -> None:
    root = make_repo(tmp_path)
    subprocess.run(["git", "-C", str(root), "tag", "v1.2.3"], capture_output=True, check=True)
    assert rel.make_release(root=root, files=FILES).label == "v1.2.3"


def test_a_calibration_overlay_is_hashed_separately(tmp_path: Path) -> None:
    root = make_repo(tmp_path)
    calibration = tmp_path / "karmel-001.yaml"
    calibration.write_text("drive:\n  wheel_radius_m: 0.0446\n", encoding="utf-8")
    release = rel.make_release(calibration=calibration, root=root, files=FILES)
    assert release.calibration[str(calibration)] == rel.file_sha256(calibration)


# --- comparison ----------------------------------------------------------------------------------
def test_an_unchanged_tree_matches(tmp_path: Path) -> None:
    root = make_repo(tmp_path)
    recorded = rel.make_release(root=root, files=FILES).as_dict()
    assert rel.compare(recorded, root=root) == []


def test_a_changed_config_is_reported_by_name(tmp_path: Path) -> None:
    root = make_repo(tmp_path)
    recorded = rel.make_release(root=root, files=FILES).as_dict()
    (root / "labs" / "config" / "karmel.yaml").write_text("wheel_radius_m: 0.0446\n", encoding="utf-8")
    differences = rel.compare(recorded, root=root)
    names = {d.what for d in differences}
    assert "labs/config/karmel.yaml" in names
    assert "git.dirty" in names, "the tree is dirty now and was not when recorded"


def test_a_new_commit_is_reported(tmp_path: Path) -> None:
    root = make_repo(tmp_path)
    recorded = rel.make_release(root=root, files=FILES).as_dict()
    (root / "labs" / "requirements.txt").write_text("numpy>=2.0\n", encoding="utf-8")
    subprocess.run(["git", "-C", str(root), "commit", "-am", "bump"], capture_output=True, check=True)
    differences = {d.what for d in rel.compare(recorded, root=root)}
    assert "git.commit" in differences and "labs/requirements.txt" in differences


def test_difference_prints_both_sides() -> None:
    text = str(rel.Difference("labs/config/karmel.yaml", "aaaa", "bbbb"))
    assert "karmel.yaml" in text and "aaaa" in text and "bbbb" in text


# --- the command line ------------------------------------------------------------------------------
def test_record_then_verify_round_trips(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "release.json"
    assert rel.main(["record", "--out", str(out), "--label", "demo",
                     "--tests", '{"labs": "262 passed"}']) == 0
    payload = json.loads(out.read_text(encoding="utf-8"))
    assert payload["label"] == "demo" and payload["tests"] == {"labs": "262 passed"}
    assert rel.main(["verify", str(out)]) == 0
    assert "MATCH" in capsys.readouterr().out


def test_verify_fails_when_a_tracked_file_changed(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "release.json"
    rel.main(["record", "--out", str(out)])
    payload = json.loads(out.read_text(encoding="utf-8"))
    payload["files"]["labs/config/karmel.yaml"] = "0" * 64      # pretend it was different
    out.write_text(json.dumps(payload), encoding="utf-8")
    assert rel.main(["verify", str(out)]) == 1
    assert "labs/config/karmel.yaml" in capsys.readouterr().out


def test_require_clean_refuses_a_dirty_tree(tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
                                            capsys: pytest.CaptureFixture[str]) -> None:
    root = make_repo(tmp_path)
    (root / "labs" / "requirements.txt").write_text("numpy>=2.0\n", encoding="utf-8")
    monkeypatch.setattr(rel, "REPO_ROOT", root)
    monkeypatch.setattr(rel, "TRACKED_FILES", FILES)
    out = tmp_path / "release.json"
    assert rel.main(["record", "--out", str(out), "--require-clean"]) == 1
    assert "REFUSING" in capsys.readouterr().out
    assert not out.exists(), "nothing is written when the release is refused"


def test_show_prints_a_summary(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    out = tmp_path / "release.json"
    rel.main(["record", "--out", str(out), "--label", "v9.9.9"])
    capsys.readouterr()
    assert rel.main(["show", str(out)]) == 0
    printed = capsys.readouterr().out
    assert "v9.9.9" in printed and "labs/config/karmel.yaml" in printed
