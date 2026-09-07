"""Find starting geometries for a design profile, then optimise the best of them.

    python3 tools/seed_search.py --design long_ad --finger middle

Adam is a local method: it follows the slope under its feet and never crosses a
ridge. On a changed mount the shipped nominal lengths often sit in a basin with no
feasible design in it, so the search has to be handed a better starting point.

Two lessons from earlier runs are baked in:

* Rank candidates with `evaluate()`, never with a geometric proxy. Scoring "how
  near does H's path pass to each R4 sample" lets R4's first pose pair with H's
  fortieth; closure pairs them in step. That proxy once picked seeds scoring 1.2 mm
  that carried 38 mm of real closure error.
* Save every result, not just the feasible ones. A 2/3 candidate that misses by
  half a millimetre is the most valuable starting point for the next round.
"""
from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np

from _common import (bounds, fgh_angle, load_design, passes, report_row,
                     seeded_problem, variable_ids, violation)


def build_screen(module, problem):
    """Cheap reject: must assemble across the sweep and stay clear of the palm."""
    tilt = problem.ad_tilt_deg
    q_grid = np.linspace(0, 90, 46)
    hand = problem.nominal_design["human_hand_model"]
    palm_width = float(hand["palm"]["width_mm"])
    palm_length = float(hand["palm"]["length_mm"])
    palm_y = -problem.dorsal_clearance_mm - palm_width / 2.0
    keep_out = palm_width / 2.0 + 3.5

    def screen(values) -> bool:
        lengths = module._candidate_lengths(problem, values)
        poses = module._mechanism_workspace_poses(lengths, q_grid, tilt)
        if len(poses) < len(q_grid):
            return False
        for _, positions in poses:
            for node_id, point in positions.items():
                if node_id == "d":      # the intentional mount, already excluded
                    continue
                x = min(max(point[0], -palm_length), 0.0)
                if math.hypot(point[0] - x, point[1] - palm_y) < keep_out:
                    return False
        return True

    return screen


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--design", default="long_ad")
    parser.add_argument("--finger", default="middle")
    parser.add_argument("--samples", type=int, default=600_000)
    parser.add_argument("--evaluate", type=int, default=1200,
                        help="survivors scored with the real evaluator")
    parser.add_argument("--seeds", type=int, default=6, help="best seeds to optimise")
    parser.add_argument("--iterations", type=int, default=300)
    parser.add_argument("--learning-rate", type=float, default=0.05)
    parser.add_argument("--w-closure", type=float, default=5.0)
    parser.add_argument("--w-collision", type=float, default=20.0)
    parser.add_argument("--w-rod", type=float, default=20.0)
    parser.add_argument("--rng-seed", type=int, default=11)
    parser.add_argument("--out", default="runs/seed_search",
                        help="directory for the saved design vectors")
    args = parser.parse_args()

    module, problem = load_design(args.design, args.finger, weights={
        "closure": args.w_closure,
        "collision": args.w_collision,
        "rod": args.w_rod,
    })
    ids = variable_ids(problem)
    low, high = bounds(problem)
    screen = build_screen(module, problem)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"design {args.design}  finger {args.finger}  "
          f"weights closure={args.w_closure} collision={args.w_collision} rod={args.w_rod}")

    rng = np.random.default_rng(args.rng_seed)
    started = time.time()
    survivors = []
    for _ in range(args.samples):
        values = low + rng.random(len(low)) * (high - low)
        if screen(values):
            survivors.append(values.copy())
    print(f"assembly screen: {len(survivors)} of {args.samples} survived "
          f"in {time.time()-started:.0f}s")
    if not survivors:
        raise SystemExit("nothing assembles; widen the bounds or revisit the fixed mount")

    started = time.time()
    scored = []
    for values in survivors[:args.evaluate]:
        evaluation = module.evaluate(problem, values)
        scored.append((-passes(evaluation), violation(evaluation), values))
    scored.sort(key=lambda row: (row[0], row[1]))
    print(f"scored {len(scored)} with the real evaluator in {time.time()-started:.0f}s; "
          f"best violations {[round(row[1], 2) for row in scored[:6]]}")

    print(f"\noptimising the {args.seeds} best seeds for {args.iterations} iterations")
    results = []
    for index, (_, start_violation, values) in enumerate(scored[:args.seeds], 1):
        problem_seeded = seeded_problem(problem, values)
        started = time.time()
        result = module.optimize(problem_seeded, iterations=args.iterations,
                                 learning_rate=args.learning_rate, progress_every=0)
        evaluation = result.evaluation
        lengths = module._candidate_lengths(problem, result.values)
        path = out_dir / f"seed_{index:02d}.npy"
        np.save(path, result.values)          # save every result, feasible or not
        results.append((passes(evaluation), evaluation, lengths, path))
        print(f"{report_row(f'seed {index} ({start_violation:.2f})', evaluation, lengths)}"
              f"  angle_FGH={fgh_angle(lengths):5.1f}  ({time.time()-started:.0f}s)")

    results.sort(key=lambda row: (-row[0], row[1].total_loss))
    best_passes, best_eval, best_lengths, best_path = results[0]
    print(f"\nbest {best_passes}/3, rod {best_lengths['L_tip_rod']:.2f} mm -> {best_path}")
    if best_passes < 3:
        print("No feasible design yet. What has worked from here:")
        print("  * seed from an infeasible SHORT rod and let closure guidance pull it in "
              "(a feasible design is a trap: any rod-shortening step drops it to 2/3 and "
              "the ranking rule then rejects it)")
        print("  * re-weight toward whichever constraint is actually binding; an "
              "over-satisfied one keeps stealing the gradient")


if __name__ == "__main__":
    main()
