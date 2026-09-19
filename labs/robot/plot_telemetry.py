"""Plot a telemetry CSV from log_telemetry.py into a PNG: wheel speeds, ticks, battery, range.

Lessons 03.07 (plotting what the robot did) and 08.02 / 08.08 (step responses, tuning by logging).

    python labs/robot/plot_telemetry.py run.csv                   # writes run.png
    python labs/robot/plot_telemetry.py step.csv --output step_response.png

Runs anywhere (no robot, no display needed): copy the CSV from the Pi to your laptop, or run it
on the Pi and copy the PNG back.

One measure per panel, all sharing the time axis - never two different units on one y-axis.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

LEFT_COLOR = "#2a78d6"  # blue
RIGHT_COLOR = "#eb6834"  # orange
SINGLE_COLOR = "#2a78d6"
COMMAND_COLOR = "#52514e"  # gray: the command is context, the measurement is the data
SURFACE = "#fcfcfb"
TEXT = "#0b0b0b"
GRID = "#e4e3df"


def read_csv(path: Path) -> dict[str, list[float | None]]:
    """Columns -> lists of floats; empty cells become None."""
    with path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path} has no samples")
    return {key: [float(r[key]) if r[key] != "" else None for r in rows] for key in rows[0]}


def plot(data: dict[str, list[float | None]], output: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")  # file output only, works without a display (e.g. over SSH)
    import matplotlib.pyplot as plt

    t = data["t_s"]
    t0 = t[0] or 0.0
    time_s = [(v or 0.0) - t0 for v in t]

    fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True, facecolor=SURFACE)
    panels = [
        ("wheel speed (rad/s)", [("left", "left_rad_s", LEFT_COLOR), ("right", "right_rad_s", RIGHT_COLOR)]),
        ("encoder ticks", [("left", "left_ticks", LEFT_COLOR), ("right", "right_ticks", RIGHT_COLOR)]),
        ("battery (V)", [("battery", "battery_v", SINGLE_COLOR)]),
        ("front range (m)", [("range", "range_m", SINGLE_COLOR)]),
    ]
    for ax, (label, series) in zip(axes, panels):
        ax.set_facecolor(SURFACE)
        for name, column, color in series:
            values = [v if v is not None else float("nan") for v in data[column]]
            # The right wheel is dashed so it stays visible when both wheels overlap exactly.
            style = "--" if name == "right" else "-"
            ax.plot(time_s, values, color=color, linewidth=2, linestyle=style, label=name)
        ax.ticklabel_format(axis="y", useOffset=False, style="plain")
        finite = [v for _, column, _ in series for v in data[column] if v is not None]
        if finite and max(finite) - min(finite) < 0.1:  # a flat signal: don't zoom into noise
            middle = (max(finite) + min(finite)) / 2
            ax.set_ylim(middle - 0.5, middle + 0.5)
        if len(series) > 1:
            ax.legend(loc="upper left", frameon=False, labelcolor=TEXT)
        ax.set_ylabel(label, color=TEXT)
        ax.grid(True, color=GRID, linewidth=0.8)
        ax.tick_params(colors=TEXT)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)

    # Commands (duty or rad/s) get their own panel when present, so units never mix.
    commanded = [v for v in data.get("left_cmd", []) if v is not None]
    if commanded:
        axes[0].set_title("command active from the gray dashed line", color=COMMAND_COLOR, fontsize=9, loc="right")
        first = next(i for i, v in enumerate(data["left_cmd"]) if v is not None)
        for ax in axes:
            ax.axvline(time_s[first], color=COMMAND_COLOR, linestyle="--", linewidth=1)

    axes[-1].set_xlabel("time (s)", color=TEXT)
    fig.suptitle(title, color=TEXT)
    fig.tight_layout()
    fig.savefig(output, dpi=120, facecolor=SURFACE)
    plt.close(fig)


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", type=Path, help="CSV written by log_telemetry.py")
    parser.add_argument("--output", type=Path, help="PNG file (default: the CSV name with .png)")
    parser.add_argument("--title", help="figure title (default: the CSV file name)")
    args = parser.parse_args(argv)
    output = args.output or args.csv.with_suffix(".png")
    plot(read_csv(args.csv), output, args.title or args.csv.name)
    print(f"wrote {output}")


if __name__ == "__main__":
    main()
