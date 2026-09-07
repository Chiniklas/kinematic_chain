"""Turn a saved design vector into a candidate directory, and optionally animate it.

    python3 tools/materialize.py --design short_ad --finger index \
        --vector runs/seed_search/seed_01.npy --out runs/my_candidate --animate

The optimiser writes candidate directories itself, but the tools here hand back raw
`.npy` vectors. This reuses `write_outputs` so a vector becomes a real candidate
with a self-contained mechanism.yaml that the analysis and animation scripts accept.

Note that `run_optimization.sh` runs the analysis but not the animation; pass
--animate here (or call src/animate_design.py yourself) to get the GIF.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

from _common import ROOT, fgh_angle, load_design, mark, passes, seeded_problem


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--design", default="short_ad")
    parser.add_argument("--finger", default="index")
    parser.add_argument("--vector", required=True, help="path to a saved .npy vector")
    parser.add_argument("--out", required=True, help="candidate directory to write")
    parser.add_argument("--animate", action="store_true", help="also render the GIF")
    parser.add_argument("--w-closure", type=float, default=None)
    parser.add_argument("--w-collision", type=float, default=None)
    parser.add_argument("--w-rod", type=float, default=None)
    args = parser.parse_args()

    # Record the weights that produced the vector, so the candidate file is honest
    # about the configuration it came from.
    weights = {name: value for name, value in (
        ("closure", args.w_closure),
        ("collision", args.w_collision),
        ("rod", args.w_rod),
    ) if value is not None}

    module, problem = load_design(args.design, args.finger, weights=weights or None)
    values = np.load(args.vector)
    if values.shape != (len(problem.variables),):
        raise SystemExit(f"vector has {values.shape} entries, expected "
                         f"{len(problem.variables)}")

    # Raise any bound the vector already exceeds, otherwise the candidate would
    # record itself as out of range.
    caps = {var.id: max(var.maximum, float(value))
            for var, value in zip(problem.variables, values)}
    problem = seeded_problem(problem, values, caps)

    evaluation = module.evaluate(problem, values)
    result = module.OptimizationResult(
        values, evaluation, evaluation, 0, False, (), "torch.optim.Adam", "cpu")
    out_dir = Path(args.out)
    module.write_outputs(out_dir, problem, result)

    lengths = module._candidate_lengths(problem, values)
    print(f"{passes(evaluation)}/3  rod={lengths['L_tip_rod']:.3f} mm  "
          f"angle_FGH={fgh_angle(lengths):.2f} deg")
    print(f"  collision   {mark(evaluation.collision_free):>4}  "
          f"{evaluation.minimum_clearance_mm:+.3f} mm")
    print(f"  rod closure {mark(evaluation.rod_closure_feasible):>4}  "
          f"{evaluation.maximum_rod_closure_error_mm:.4f} mm")
    print(f"  curl        {mark(evaluation.hand_motion_feasible):>4}")
    candidate_dir = out_dir / "candidate_0001"
    print(f"written to {candidate_dir}")

    if args.animate:
        command = [sys.executable, str(ROOT / "src" / "animate_design.py"),
                   str(candidate_dir), "--finger", args.finger]
        completed = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
        if completed.returncode != 0:
            print(completed.stderr.strip()[-800:], file=sys.stderr)
            raise SystemExit("animation failed")
        for line in completed.stdout.splitlines():
            if "animation:" in line:
                print(line.strip())


if __name__ == "__main__":
    main()
