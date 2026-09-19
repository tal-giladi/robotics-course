"""Lesson 03.02 — answer "which Python is this, and does it have what the labs need?"

    python 03-robot-software/code/l0302_env_report.py
    python 03-robot-software/code/l0302_env_report.py --requirements labs/requirements.txt --quiet

Four questions, in the order you should ask them when an import fails on a robot:

1. **Which interpreter?**   `sys.executable`, version, and whether it is a virtual environment.
2. **Is it managed?**       PEP 668: Ubuntu 24.04 marks the system Python ``EXTERNALLY-MANAGED``,
                            so ``pip install`` outside a venv is refused. The marker is a file.
3. **What can it import?**  every package in ``labs/requirements.txt``, with the version actually
                            importable and whether it satisfies the requirement.
4. **Is ROS 2 in play?**    ``ROS_DISTRO``/``AMENT_PREFIX_PATH``, because ROS binaries run the
                            *system* interpreter, which is usually not the venv you are in.

Exit code 0 when every requirement is satisfied, 1 otherwise — so it doubles as a health check
on the Pi (``python3 l0302_env_report.py --quiet || echo "fix the environment"``).

Nothing here needs hardware, a network or ROS. Tests: test_l0302_env.py
"""

from __future__ import annotations

import argparse
import importlib.metadata as md
import os
import re
import sys
import sysconfig
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REQUIREMENTS = REPO_ROOT / "labs" / "requirements.txt"

# "opencv-python-headless>=4.9" -> name, operator, version. Enough for this repo's files;
# the full grammar (extras, markers, URLs) lives in PEP 508 and the `packaging` library.
REQUIREMENT_RE = re.compile(r"^\s*([A-Za-z0-9._-]+)\s*(==|>=|~=|>)?\s*([0-9][0-9A-Za-z.*+!-]*)?\s*$")

# Distribution name on PyPI -> the name you actually `import`.
IMPORT_NAMES = {
    "opencv-python-headless": "cv2",
    "opencv-python": "cv2",
    "pyyaml": "yaml",
    "pyserial": "serial",
    "pillow": "PIL",
}


@dataclass(frozen=True)
class Requirement:
    name: str
    operator: str | None
    version: str | None

    @property
    def text(self) -> str:
        return f"{self.name}{self.operator or ''}{self.version or ''}"


@dataclass(frozen=True)
class Check:
    requirement: Requirement
    installed: str | None  # None = not installed
    ok: bool

    @property
    def status(self) -> str:
        if self.installed is None:
            return "MISSING"
        return "ok" if self.ok else "TOO OLD"


def parse_requirements_text(text: str, source: str = "<text>") -> list[Requirement]:
    """Parse requirements.txt content, ignoring comments, blank lines and pip options."""
    out: list[Requirement] = []
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line or line.startswith("-"):
            continue
        m = REQUIREMENT_RE.match(line)
        if m is None:
            raise ValueError(f"{source}: cannot parse requirement {line!r}")
        out.append(Requirement(m.group(1), m.group(2), m.group(3)))
    return out


def parse_requirements(path: Path) -> list[Requirement]:
    """Read and parse a requirements.txt file."""
    return parse_requirements_text(path.read_text(encoding="utf-8"), str(path))


def version_tuple(version: str) -> tuple[int, ...]:
    """'4.9.0.80' -> (4, 9, 0, 80); non-numeric parts stop the comparison ('2.0rc1' -> (2, 0))."""
    parts: list[int] = []
    for piece in version.split("."):
        digits = re.match(r"\d+", piece)
        if digits is None:
            break
        parts.append(int(digits.group()))
    return tuple(parts)


def satisfies(installed: str, requirement: Requirement) -> bool:
    if requirement.operator is None or requirement.version is None:
        return True
    have, want = version_tuple(installed), version_tuple(requirement.version)
    if requirement.operator == "==":
        return have[: len(want)] == want
    if requirement.operator == ">":
        return have > want
    if requirement.operator == "~=":  # compatible release: same prefix, not older
        return have >= want and have[: len(want) - 1] == want[: len(want) - 1]
    return have >= want  # ">="


def installed_version(name: str) -> str | None:
    """Version of a distribution, falling back to the imported module's ``__version__``."""
    try:
        return md.version(name)
    except md.PackageNotFoundError:
        pass
    module_name = IMPORT_NAMES.get(name.lower(), name.replace("-", "_"))
    try:
        module = __import__(module_name)
    except Exception:  # noqa: BLE001 — a broken install must read as "not installed"
        return None
    return str(getattr(module, "__version__", "unknown"))


def check_requirements(requirements: list[Requirement]) -> list[Check]:
    checks = []
    for req in requirements:
        found = installed_version(req.name)
        checks.append(Check(req, found, found is not None and satisfies(found, req)))
    return checks


def in_virtualenv() -> bool:
    return sys.prefix != sys.base_prefix


def venv_sees_system_packages() -> bool | None:
    """From pyvenv.cfg: did this venv get --system-site-packages? None = not a venv."""
    if not in_virtualenv():
        return None
    cfg = Path(sys.prefix) / "pyvenv.cfg"
    if not cfg.is_file():
        return None
    for line in cfg.read_text(encoding="utf-8").splitlines():
        key, _, value = line.partition("=")
        if key.strip() == "include-system-site-packages":
            return value.strip().lower() == "true"
    return False


def externally_managed_marker() -> Path | None:
    """The PEP 668 marker file, if this interpreter has one (Ubuntu 24.04's system Python does)."""
    marker = Path(sysconfig.get_path("stdlib")) / "EXTERNALLY-MANAGED"
    return marker if marker.is_file() else None


def robotlab_location() -> str:
    try:
        import robotlab
    except ImportError:
        return "not importable (pip install -e labs/python, or run pytest from the repo root)"
    path = getattr(robotlab, "__file__", None)
    return str(Path(path).resolve().parent) if path else "importable (no __file__)"


def ros_environment() -> dict[str, str]:
    return {k: os.environ[k] for k in ("ROS_DISTRO", "ROS_DOMAIN_ID", "AMENT_PREFIX_PATH") if k in os.environ}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--requirements", type=Path, default=DEFAULT_REQUIREMENTS)
    parser.add_argument("--quiet", action="store_true", help="only print problems")
    args = parser.parse_args(argv)

    checks = check_requirements(parse_requirements(args.requirements))
    failures = [c for c in checks if not c.ok]

    if not args.quiet:
        marker = externally_managed_marker()
        system = venv_sees_system_packages()
        print(f"interpreter   {sys.executable}")
        print(f"version       {sys.version.split()[0]} on {sys.platform}")
        print(f"virtualenv    {'yes, ' + sys.prefix if in_virtualenv() else 'no (this is the base interpreter)'}")
        if system is not None:
            print(f"  system site-packages visible: {'yes (--system-site-packages)' if system else 'no'}")
        if marker is None:
            print("PEP 668       not marked: pip may install into this interpreter")
        elif in_virtualenv():
            print(f"PEP 668       marker exists ({marker}) but does NOT apply: you are in a venv")
        else:
            print(f"PEP 668       EXTERNALLY-MANAGED: {marker} — pip install will be refused here")
        print(f"robotlab      {robotlab_location()}")
        ros = ros_environment()
        print(f"ROS 2         {ros if ros else 'no ROS environment sourced'}")
        print(f"\n{'requirement':<34} {'installed':<14} status")
        for c in checks:
            print(f"{c.requirement.text:<34} {c.installed or '-':<14} {c.status}")

    if failures:
        print(f"\n{len(failures)} unsatisfied requirement(s): " + ", ".join(c.requirement.text for c in failures))
        print(f"fix: {sys.executable} -m pip install -r {args.requirements}")
    elif not args.quiet:
        print("\nall requirements satisfied")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
