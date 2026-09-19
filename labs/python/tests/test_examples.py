"""Keep labs/examples runnable."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


def test_sim_quickstart_writes_png(tmp_path: Path):
    output = tmp_path / "square.png"
    result = subprocess.run(
        [sys.executable, str(EXAMPLES / "sim_quickstart.py"), str(output)],
        capture_output=True, text=True, timeout=120, cwd=tmp_path,
    )
    assert result.returncode == 0, result.stderr
    assert output.stat().st_size > 10_000
