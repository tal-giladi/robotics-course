"""Shared helpers for the module-12 navigation scripts.

* Makes ``robotlab`` importable without ``pip install -e labs/python``.
* The canonical "go to the kitchen" scenario used by lessons 12.01-12.05.
* Headless-safe figure saving into ``nav_out/`` (override with the ``NAV_OUT`` environment variable).
"""

from __future__ import annotations

import importlib.util
import os
import re
import sys
from pathlib import Path
from types import ModuleType

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
if str(ROOT / "labs" / "python") not in sys.path:
    sys.path.insert(0, str(ROOT / "labs" / "python"))

import matplotlib  # noqa: E402

if "MPLBACKEND" not in os.environ:
    matplotlib.use("Agg")  # scripts write PNGs; no display needed

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

# The running example: karmel starts in the living room and is asked to go to the kitchen.
START_XY = (1.0, 1.3)
START_POSE = (1.0, 1.3, 0.0)
KITCHEN_XY = (5.0, 2.3)
STUDY_XY = (0.8, 3.8)
BEDROOM_XY = (5.0, 3.2)
GRID_RESOLUTION = 0.05

OUT_DIR = Path(os.environ.get("NAV_OUT", "nav_out"))


def save(fig: Figure, name: str, out_dir: Path | str | None = None) -> Path:
    """Save ``fig`` as ``<out_dir>/<name>.png``, close it and print the path."""
    folder = Path(out_dir) if out_dir is not None else OUT_DIR
    folder.mkdir(parents=True, exist_ok=True)
    path = folder / f"{name}.png"
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print(f"wrote {path}")
    return path


def load_exercise(lesson_id: str, solution: bool = False) -> ModuleType:
    """Import ``labs/exercises/<lesson_id>/student.py`` (or ``solution.py``)."""
    kind = "solution" if solution else "student"
    path = ROOT / "labs" / "exercises" / lesson_id / f"{kind}.py"
    safe_id = re.sub(r"\W", "_", lesson_id)
    name = f"module12_{safe_id}_{kind}"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ImportError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module
