"""Tests for the lesson 03.02 environment report — no hardware, no network.

    python -m pytest 03-robot-software/code/test_l0302_env.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

import l0302_env_report as env


def test_parses_the_courses_own_requirements_file() -> None:
    reqs = env.parse_requirements(env.DEFAULT_REQUIREMENTS)
    names = {r.name for r in reqs}
    assert {"numpy", "scipy", "pyyaml", "pyserial", "pytest"} <= names
    assert all(r.operator == ">=" for r in reqs), "labs/requirements.txt pins minimums, not exact versions"


def test_parse_requirements_ignores_comments_options_and_blanks(tmp_path: Path) -> None:
    f = tmp_path / "requirements.txt"
    f.write_text("# a comment\n\nnumpy>=1.26  # inline\n--index-url https://example.invalid\npyserial\n", encoding="utf-8")
    reqs = env.parse_requirements(f)
    assert [r.text for r in reqs] == ["numpy>=1.26", "pyserial"]


def test_parse_requirements_rejects_what_it_cannot_understand(tmp_path: Path) -> None:
    f = tmp_path / "requirements.txt"
    f.write_text("numpy; python_version < '3.12'\n", encoding="utf-8")
    with pytest.raises(ValueError):
        env.parse_requirements(f)


@pytest.mark.parametrize(
    ("version", "expected"),
    [("1.26", (1, 26)), ("4.9.0.80", (4, 9, 0, 80)), ("2.0rc1", (2, 0)), ("3", (3,))],
)
def test_version_tuple(version: str, expected: tuple[int, ...]) -> None:
    assert env.version_tuple(version) == expected


@pytest.mark.parametrize(
    ("installed", "spec", "ok"),
    [
        ("1.26.4", "numpy>=1.26", True),
        ("1.25.0", "numpy>=1.26", False),
        ("7.4.4", "pytest>=8.0", False),   # apt's python3-pytest on Ubuntu 24.04
        ("9.1.1", "pytest>=8.0", True),
        ("6.0.1", "pyyaml==6.0", True),    # "==6.0" matches any 6.0.x prefix here
        ("6.1.0", "pyyaml==6.0", False),
        ("1.27.0", "numpy~=1.26", True),
        ("2.0.0", "numpy~=1.26", False),
        ("3.5", "pyserial", True),         # no operator: anything satisfies it
    ],
)
def test_satisfies(installed: str, spec: str, ok: bool) -> None:
    (req,) = env.parse_requirements_text(spec)
    assert env.satisfies(installed, req) is ok


def test_import_name_mapping_is_used() -> None:
    assert env.installed_version("pyyaml") is not None, "pyyaml is imported as 'yaml'"
    assert env.installed_version("definitely-not-a-real-package-9999") is None


def test_this_interpreter_passes_the_report(capsys: pytest.CaptureFixture[str]) -> None:
    code = env.main([])
    out = capsys.readouterr().out
    assert sys.executable in out
    assert "requirement" in out and "numpy" in out
    assert code == 0, "the labs' requirements must be satisfied in the environment that runs the tests"


def test_quiet_mode_prints_nothing_when_everything_is_fine(capsys: pytest.CaptureFixture[str]) -> None:
    assert env.main(["--quiet"]) == 0
    assert capsys.readouterr().out == ""


def test_missing_requirements_are_reported_and_exit_nonzero(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    f = tmp_path / "requirements.txt"
    f.write_text("numpy>=1.26\nno-such-robot-package>=1.0\n", encoding="utf-8")
    assert env.main(["--requirements", str(f), "--quiet"]) == 1
    assert "no-such-robot-package" in capsys.readouterr().out


def test_environment_probes_are_consistent() -> None:
    marker = env.externally_managed_marker()
    assert marker is None or marker.name == "EXTERNALLY-MANAGED"
    if not env.in_virtualenv():
        assert env.venv_sees_system_packages() is None
