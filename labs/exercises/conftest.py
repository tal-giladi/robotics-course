"""Shared pytest plumbing for the auto-graded exercises in ``labs/exercises/<lesson-id>/``.

* ``impl`` fixture — the module under test: ``student.py`` next to the requesting test file, or
  ``solution.py`` when ``COURSE_USE_SOLUTION=1`` (``python course.py check 09.04 --solution``).
* A test that hits a ``NotImplementedError`` from student code is reported as *skipped* with
  "not implemented yet — edit student.py", so the repository-wide test run stays green.
* When a run covers only exercise tests (``python course.py check 09.04``), any such skip makes
  the run fail: an exercise isn't passed until every test actually passes.
"""

from __future__ import annotations

import importlib.util
import os
import re
import sys
from collections.abc import Callable, Generator
from pathlib import Path
from types import ModuleType
from typing import Any, TypeVar

import pytest

SOLUTION_ENV = "COURSE_USE_SOLUTION"
NOT_IMPLEMENTED_MESSAGE = "not implemented yet — edit student.py"
_EXERCISES_DIR = Path(__file__).resolve().parent

collect_ignore = ["_template"]

T = TypeVar("T")


def using_solution() -> bool:
    """True when tests should import ``solution.py`` instead of ``student.py``."""
    return os.environ.get(SOLUTION_ENV, "").strip().lower() in {"1", "true", "yes"}


def load_exercise_module(exercise_dir: Path, use_solution: bool | None = None) -> ModuleType:
    """Import ``student.py`` / ``solution.py`` from ``exercise_dir`` under a unique module name."""
    use_solution = using_solution() if use_solution is None else use_solution
    kind = "solution" if use_solution else "student"
    path = exercise_dir / f"{kind}.py"
    if not path.is_file():
        raise FileNotFoundError(f"{path} does not exist")
    safe_dir = re.sub(r"\W", "_", exercise_dir.name)  # "09.04" -> "09_04"
    name = f"course_exercise_{safe_dir}_{kind}"
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module  # dataclasses and pickling look modules up by name
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def impl(request: pytest.FixtureRequest) -> ModuleType:
    """The implementation under test (student or reference solution)."""
    return load_exercise_module(Path(request.path).resolve().parent)


def skip_if_not_implemented(func: Callable[..., T], *args: Any, **kwargs: Any) -> T:
    """Call ``func``; skip the test with a clear message if it raises ``NotImplementedError``.

    Tests rarely need this (the hooks below do it automatically); it is handy when a test wants
    to skip *early*, before doing expensive work, e.g. ``skip_not_implemented(impl.Odometry, ...)``.
    """
    try:
        return func(*args, **kwargs)
    except NotImplementedError as exc:
        if using_solution():
            raise
        pytest.skip(_skip_reason(exc))


@pytest.fixture
def skip_not_implemented() -> Callable[..., Any]:
    """Fixture giving tests :func:`skip_if_not_implemented` (conftest modules aren't importable)."""
    return skip_if_not_implemented


def _skip_reason(exc: NotImplementedError) -> str:
    detail = f" ({exc})" if str(exc) else ""
    return f"{NOT_IMPLEMENTED_MESSAGE}{detail}"


def _is_exercise_item(item: pytest.Item) -> bool:
    return _EXERCISES_DIR in Path(item.path).resolve().parents


@pytest.hookimpl(wrapper=True)
def pytest_runtest_setup(item: pytest.Item) -> Generator[None, None, None]:
    return (yield from _convert_not_implemented(item))


@pytest.hookimpl(wrapper=True)
def pytest_runtest_call(item: pytest.Item) -> Generator[None, None, None]:
    return (yield from _convert_not_implemented(item))


def _convert_not_implemented(item: pytest.Item) -> Generator[None, None, None]:
    try:
        return (yield)
    except NotImplementedError as exc:
        if using_solution() or not _is_exercise_item(item):
            raise
        pytest.skip(_skip_reason(exc))


def _not_implemented_skips(config: pytest.Config) -> int:
    """How many tests were skipped because student code is unfinished (from the terminal stats)."""
    reporter = config.pluginmanager.get_plugin("terminalreporter")
    if reporter is None:
        return 0
    return sum(NOT_IMPLEMENTED_MESSAGE in str(r.longrepr) for r in reporter.stats.get("skipped", []))


def pytest_sessionfinish(session: pytest.Session, exitstatus: int) -> None:
    """Fail an exercise-only run (``course.py check``) that still has unimplemented parts."""
    only_exercises = bool(session.items) and all(_is_exercise_item(i) for i in session.items)
    if only_exercises and exitstatus == pytest.ExitCode.OK and _not_implemented_skips(session.config):
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


def pytest_terminal_summary(terminalreporter: Any, exitstatus: int, config: pytest.Config) -> None:
    skipped = _not_implemented_skips(config)
    if skipped:
        terminalreporter.write_line(
            f"{skipped} exercise test(s) skipped: {NOT_IMPLEMENTED_MESSAGE}. "
            "An exercise passes only when none are skipped.",
            yellow=True,
        )
