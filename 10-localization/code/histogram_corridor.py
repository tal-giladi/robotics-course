"""10.03 — The histogram (grid Bayes) filter in a corridor with doors.

    python 10-localization/code/histogram_corridor.py              # YOUR labs/exercises/10.03/student.py
    python 10-localization/code/histogram_corridor.py --solution   # the reference

1. Global localization: karmel (realistic simulator) starts somewhere in a 10 m corridor with a
   uniform belief, drives 5 m, and the belief collapses onto the true cell.
2. Kidnapped robot: after 20 cells it is carried back to the start of the corridor. A pure Bayes
   filter never recovers; mixing a little uniform probability into every prediction lets it.
3. Motion-model experiment: too-sharp and too-blurry motion kernels, with and without one cell
   that odometry missed.
"""

from __future__ import annotations

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import loc_common as lc  # noqa: E402

KERNEL = [0.05, 0.9, 0.05]
P_HIT, P_FALSE = 0.9, 0.1


def filter_with_mixing(ex, cs, moves, readings, kernel, epsilon: float) -> list[np.ndarray]:
    """run_filter plus 'injected uniform probability': belief = (1 - eps) belief + eps / n after predict."""
    doors = cs.door_map()
    belief = ex.normalize(np.ones(cs.N_CELLS))
    history = []
    for move, z in zip(moves, readings):
        belief = ex.predict(belief, move, kernel, wrap=False)
        belief = (1.0 - epsilon) * belief + epsilon / belief.size
        if z is not None:
            belief = ex.update(belief, ex.door_likelihood(doors, z, P_HIT, P_FALSE))
        history.append(belief)
    return history


def main(argv: list[str] | None = None) -> dict:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--solution", action="store_true", help="use the reference solution")
    ap.add_argument("--out", default="localization_out")
    ap.add_argument("--seed", type=int, default=2)
    ap.add_argument("--start-cell", type=int, default=5)
    args = ap.parse_args(argv)
    out = lc.out_dir(args.out)
    ex = lc.load_exercise("10.03", solution=args.solution)
    cs = lc.load_provided("10.03", "corridor_sim")
    doors = cs.door_map()
    results: dict = {}

    # 1. global localization --------------------------------------------------------------------
    run = cs.drive_corridor(args.start_cell, 20, seed=args.seed)
    history = ex.run_filter(np.ones(cs.N_CELLS), doors, run.moves, run.readings, KERNEL, P_HIT, P_FALSE, wrap=False)
    print(f"1) Global localization, start cell {args.start_cell} (unknown to the filter), seed {args.seed}")
    print(f"{'step':>4} {'door?':>6} {'true':>5} {'peak':>5} {'P(peak)':>8} {'P(true)':>8}")
    for k, (z, cell, b) in enumerate(zip(run.readings, run.true_cells, history), start=1):
        print(f"{k:4d} {str(z):>6} {cell:5d} {int(np.argmax(b)):5d} {b.max():8.3f} {b[cell]:8.3f}")
    results["global"] = {"true": run.true_cells, "peak": [int(np.argmax(b)) for b in history],
                         "p_true_final": float(history[-1][run.true_cells[-1]])}

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), gridspec_kw={"height_ratios": [3, 1]})
    axes[0].imshow(np.array(history), aspect="auto", origin="lower", cmap="viridis",
                   extent=(-0.5, cs.N_CELLS - 0.5, 0.5, len(history) + 0.5))
    axes[0].plot(run.true_cells, np.arange(1, len(history) + 1), "r.", label="true cell")
    axes[0].set_ylabel("step (one cell of travel)")
    axes[0].set_title("Belief over cells (bright = probable)")
    axes[0].legend(loc="upper left")
    axes[1].bar(np.arange(cs.N_CELLS), doors.astype(float), color="tab:brown")
    axes[1].set_xlabel("cell (0.25 m each)")
    axes[1].set_ylabel("door")
    fig.tight_layout()
    fig.savefig(out / "histogram_global.png", dpi=120)
    plt.close(fig)

    # 2. kidnapped robot ------------------------------------------------------------------------
    first = cs.drive_corridor(12, 20, seed=11)   # cells 13..32
    second = cs.drive_corridor(1, 24, seed=12)   # carried back to cell 1, then cells 2..25
    moves, readings = first.moves + second.moves, first.readings + second.readings
    truth = first.true_cells + second.true_cells
    n_before = len(first.moves)
    print(f"\n2) Kidnapped after {n_before} cells (carried from cell {first.true_cells[-1]} back to cell 1)")
    results["kidnap"] = {}
    for eps in (0.0, 0.01, 0.05, 0.2):
        hist = filter_with_mixing(ex, cs, moves, readings, KERNEL, eps)
        errors = [abs(int(np.argmax(b)) - c) for b, c in zip(hist, truth)]
        recovered = next((k + 1 for k in range(n_before, len(hist)) if all(e <= 1 for e in errors[k:])), None)
        p_before, p_end = float(hist[n_before - 1][truth[n_before - 1]]), float(hist[-1][truth[-1]])
        print(f"   epsilon={eps:.2f}: P(true) before kidnap {p_before:.3f}, at the end {p_end:.3f}; "
              f"{'never recovers' if recovered is None else f'peak correct again from step {recovered} of {len(hist)}'}")
        results["kidnap"][eps] = {"p_before": p_before, "p_end": p_end, "recovered_step": recovered}

    # 3. motion model too sharp / too blurry ----------------------------------------------------
    # Same drive, but odometry misses one cell at step 9 (a wheel slipped): drop that step.
    m2 = run.moves[:8] + run.moves[9:]
    r2 = run.readings[:8] + run.readings[9:]
    t2 = run.true_cells[:8] + run.true_cells[9:]
    print("\n3) Motion kernels. Clean drive, and the same drive with one cell missed by odometry")
    results["kernels"] = {}
    for name, kernel in (("sharp [0, 1, 0]", [0.0, 1.0, 0.0]), ("matched [.05, .9, .05]", KERNEL),
                         ("blurry [.25, .5, .25]", [0.25, 0.5, 0.25])):
        clean = ex.run_filter(np.ones(cs.N_CELLS), doors, run.moves, run.readings, kernel, P_HIT, P_FALSE, wrap=False)
        missed = ex.run_filter(np.ones(cs.N_CELLS), doors, m2, r2, kernel, P_HIT, P_FALSE, wrap=False)
        errors = [abs(int(np.argmax(b)) - c) for b, c in zip(missed, t2)]
        settled = next((k + 1 for k in range(len(errors)) if all(e <= 1 for e in errors[k:])), None)
        p_clean, p_missed = float(clean[-1][run.true_cells[-1]]), float(missed[-1][t2[-1]])
        print(f"   {name:>24}: P(true) clean {p_clean:.3f} | missed cell: {p_missed:.3f}, "
              f"{'peak never settles' if settled is None else f'peak correct from step {settled} of {len(errors)}'}")
        results["kernels"][name] = {"p_clean": p_clean, "p_missed": p_missed, "settled_step": settled}
    print(f"wrote {out / 'histogram_global.png'}")
    return results


if __name__ == "__main__":
    main()
