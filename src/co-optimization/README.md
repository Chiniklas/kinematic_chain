# Co-optimization skeleton

Run the initial multi-objective Adam scaffold through the canonical top-level entry:

```bash
./run_optimization.sh
```

The wrapper forwards all options to `src/co-optimization/run_adam.py`.

It reads `config/objectives.yaml`, the four per-finger objective YAMLs, and
`config/optimizable_variables.yaml`. The default output contract is:

```text
runs/co-optimization_<UTC timestamp>/
├── history.csv
└── candidate_0001/
    ├── candidate.yaml
    ├── mechanism.yaml
    └── analysis_<UTC timestamp>/
        ├── mechanism/
        └── combined/
            └── fingers/{index,middle,ring,little}/
```

The algorithm remains Adam internally, but algorithm names are not part of the output
path. `mechanism.yaml` is materialized with the candidate dimensions and a reconstructed
closed initial pose. The nested analysis directory groups mechanism-only workspace and
torque artifacts under `mechanism/`, aggregate coupled artifacts under `combined/`, and
workspace-style per-finger full-assembly sweep reports under
`combined/fingers/<finger>/`. Each overlays the moving mechanism, palm, three rounded
phalanges, output rod, and RP4 slider from horizontal pose to that finger's supplied
maximum curl. Future retained candidates must follow the same numbered-directory
contract and each receive their own analysis.

The variables are the twelve mechanism link lengths from `L_ab` through `L_fh`, plus
the external `L_tip_rod` length from TCP `H` to the lower distal `RP4` pin-in-slot
pair. The active objective analytically tracks the candidate linkage and minimizes
rod-closure error over the finite slot in the horizontal and maximum-curled poses for
all four fingers. Consequently, all 13 variables can influence the active loss.

The exact formulation and its limitations are documented in
[`objective_math.md`](objective_math.md). Every other planned objective and constraint
is disabled until this primary reachability model is validated.
