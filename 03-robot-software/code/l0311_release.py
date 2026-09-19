"""Lesson 03.11 — record what a robot release IS, and prove later that it still is.

    python 03-robot-software/code/l0311_release.py record --out release.json
    python 03-robot-software/code/l0311_release.py record --out release.json --require-clean
    python 03-robot-software/code/l0311_release.py verify release.json
    python 03-robot-software/code/l0311_release.py show release.json

"The robot drove well on Tuesday" is only useful if you can rebuild Tuesday. That needs four
things, and a git commit is only the first:

    code          commit, branch, tag, and whether the tree was dirty
    environment   the dependency files that pin what gets installed (03.02)
    robot         the config and the calibration overlay it was running (03.06)
    evidence      which tests passed, recorded at the same moment

``record`` writes all four into one small JSON file that travels with the release (and into the
run metadata of lesson 03.07). ``verify`` recomputes them and tells you exactly what has changed
since — which is the question you actually ask six weeks later, usually in a hurry.

Deliberately NOT a build system, a packager or a deployment tool. It is the small, boring artifact
that makes the other three answerable.

Nothing here needs hardware or a network. Tests: test_l0311_release.py
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import sys
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]

# The files that decide what the robot actually is. Everything else is either derived from the
# commit or does not affect behaviour.
TRACKED_FILES = (
    "labs/config/karmel.yaml",          # the robot's physical parameters (03.06)
    "labs/requirements.txt",            # what gets installed (03.02)
    "labs/python/pyproject.toml",       # the robotlab package definition
    ".github/workflows/ci.yml",         # what "the tests passed" means
)


# --- git, without ever raising ------------------------------------------------------------------
def git(*args: str, root: Path = REPO_ROOT) -> str | None:
    """Run a git command; None if git is missing, this is not a checkout, or the command fails."""
    try:
        result = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.SubprocessError):
        return None
    # rstrip only: `git status --porcelain` puts the status in the FIRST TWO COLUMNS, so a
    # leading space is data (" M path" = modified, not staged). A .strip() here eats it and
    # every path you parse afterwards is missing its first character.
    return result.stdout.rstrip() if result.returncode == 0 else None


@dataclass(frozen=True)
class GitState:
    commit: str | None
    short: str | None
    branch: str | None
    tag: str | None                       # the exact tag on this commit, if any
    describe: str | None                  # nearest tag + distance, e.g. "v0.3.0-4-gabc1234"
    dirty: bool
    dirty_files: list[str] = field(default_factory=list)


def git_state(root: Path = REPO_ROOT, max_dirty_listed: int = 20) -> GitState:
    status = git("status", "--porcelain", root=root)
    dirty_files = [line[3:] for line in (status or "").splitlines() if line.strip()]
    return GitState(
        commit=git("rev-parse", "HEAD", root=root),
        short=git("rev-parse", "--short", "HEAD", root=root),
        branch=git("rev-parse", "--abbrev-ref", "HEAD", root=root),
        tag=git("tag", "--points-at", "HEAD", root=root) or None,
        describe=git("describe", "--tags", "--always", "--dirty", root=root),
        dirty=bool(dirty_files),
        dirty_files=sorted(dirty_files)[:max_dirty_listed],
    )


# --- hashing ------------------------------------------------------------------------------------
def file_sha256(path: Path) -> str:
    """Full SHA-256, or a marker. Never raises: a release record must always be writable."""
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return "missing"


def hash_files(paths: tuple[str, ...], root: Path = REPO_ROOT) -> dict[str, str]:
    return {name: file_sha256(root / name) for name in paths}


# --- the record ---------------------------------------------------------------------------------
@dataclass(frozen=True)
class Release:
    recorded_at: str
    label: str
    git: dict[str, Any]
    files: dict[str, str]                 # path -> sha256
    calibration: dict[str, str] = field(default_factory=dict)
    python: str = ""
    platform: str = ""
    tests: dict[str, Any] = field(default_factory=dict)
    notes: str = ""

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def make_release(label: str = "", calibration: Path | None = None, notes: str = "",
                 tests: dict[str, Any] | None = None, root: Path = REPO_ROOT,
                 files: tuple[str, ...] = TRACKED_FILES) -> Release:
    state = git_state(root)
    return Release(
        recorded_at=datetime.now(timezone.utc).isoformat(timespec="seconds"),
        label=label or (state.tag or state.describe or "untagged"),
        git=asdict(state),
        files=hash_files(files, root),
        calibration={str(calibration): file_sha256(calibration)} if calibration else {},
        python=platform.python_version(),
        platform=f"{platform.system()} {platform.machine()}",
        tests=tests or {},
        notes=notes,
    )


@dataclass(frozen=True)
class Difference:
    what: str
    recorded: Any
    now: Any

    def __str__(self) -> str:
        return f"{self.what:<34} recorded {self.recorded!r}  now {self.now!r}"


def compare(recorded: dict[str, Any], root: Path = REPO_ROOT) -> list[Difference]:
    """What changed between a recorded release and the tree in front of you, most important first."""
    differences: list[Difference] = []
    state = asdict(git_state(root))
    for key in ("commit", "branch", "tag"):
        if recorded.get("git", {}).get(key) != state.get(key):
            differences.append(Difference(f"git.{key}", recorded.get("git", {}).get(key), state.get(key)))
    if state["dirty"] and not recorded.get("git", {}).get("dirty"):
        differences.append(Difference("git.dirty", False, f"True ({len(state['dirty_files'])} files)"))

    for name, digest in recorded.get("files", {}).items():
        current = file_sha256(root / name)
        if current != digest:
            differences.append(Difference(name, digest[:12], current[:12]))
    for name, digest in recorded.get("calibration", {}).items():
        current = file_sha256(Path(name))
        if current != digest:
            differences.append(Difference(f"calibration {name}", digest[:12], current[:12]))
    return differences


# --- reporting ----------------------------------------------------------------------------------
def summary_lines(release: dict[str, Any]) -> list[str]:
    state = release.get("git", {})
    lines = [
        f"label        {release.get('label')}",
        f"recorded_at  {release.get('recorded_at')}",
        f"commit       {state.get('short')} on {state.get('branch')}"
        + (f"  tag {state['tag']}" if state.get("tag") else "")
        + ("  DIRTY" if state.get("dirty") else ""),
        f"describe     {state.get('describe')}",
        f"python       {release.get('python')} on {release.get('platform')}",
    ]
    for name, digest in release.get("files", {}).items():
        lines.append(f"  {name:<34} {digest[:12]}")
    for name, digest in release.get("calibration", {}).items():
        lines.append(f"  calibration {name:<22} {digest[:12]}")
    if release.get("tests"):
        lines.append(f"tests        {release['tests']}")
    if state.get("dirty_files"):
        lines.append(f"dirty files  {', '.join(state['dirty_files'][:5])}"
                     + (" …" if len(state["dirty_files"]) > 5 else ""))
    if release.get("notes"):
        lines.append(f"notes        {release['notes']}")
    return lines


def cmd_record(args: argparse.Namespace) -> int:
    tests = json.loads(args.tests) if args.tests else {}
    release = make_release(args.label, args.calibration, args.notes, tests)
    if args.require_clean and release.git["dirty"]:
        print("REFUSING to record a release from a dirty working tree:")
        for name in release.git["dirty_files"]:
            print(f"  {name}")
        print("Commit or stash first — a release you cannot rebuild is not a release.")
        return 1
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(release.as_dict(), indent=2) + "\n", encoding="utf-8")
    print("\n".join(summary_lines(release.as_dict())))
    print(f"\nwrote {args.out}")
    return 0


def cmd_verify(args: argparse.Namespace) -> int:
    recorded = json.loads(args.release.read_text(encoding="utf-8"))
    differences = compare(recorded)
    print("\n".join(summary_lines(recorded)))
    if not differences:
        print("\nMATCH: this tree is the release that was recorded.")
        return 0
    print(f"\n{len(differences)} difference(s) from the recorded release:")
    for difference in differences:
        print(f"  {difference}")
    print("\nA run made now would NOT be the recorded release. Check out the commit, or record a new one.")
    return 1


def cmd_show(args: argparse.Namespace) -> int:
    print("\n".join(summary_lines(json.loads(args.release.read_text(encoding="utf-8")))))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    record = sub.add_parser("record", help="write a release record for this tree")
    record.add_argument("--out", type=Path, default=Path("release.json"))
    record.add_argument("--label", default="", help="default: the tag on HEAD, or git describe")
    record.add_argument("--calibration", type=Path, help="the calibration overlay this release runs with")
    record.add_argument("--notes", default="")
    record.add_argument("--tests", help='JSON, e.g. \'{"labs": "262 passed", "exercises": "536 passed"}\'')
    record.add_argument("--require-clean", action="store_true", help="refuse to record from a dirty tree")
    record.set_defaults(func=cmd_record)

    verify = sub.add_parser("verify", help="compare a release record with the current tree")
    verify.add_argument("release", type=Path)
    verify.set_defaults(func=cmd_verify)

    show = sub.add_parser("show", help="print a release record")
    show.add_argument("release", type=Path)
    show.set_defaults(func=cmd_show)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
