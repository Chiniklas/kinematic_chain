# Multi-finger exoskeleton co-optimization

This repository is developing a hand-mounted exoskeleton from the planar
`mechanism_2` linkage. The four long fingers have different phalanx lengths and
intended curl ranges, so the design problem is treated as a multi-objective
co-optimization rather than a fit to one representative finger.

The project currently contains:

- a validated body–joint abstraction with remeasured nominal link lengths;
- a combined mildly curled nominal index-finger abstraction, with the mechanism on
  the dorsal side of a horizontal 90 × 25 mm palm, horizontal member
  `A–D`, a manually defined dorsal clearance between `D` and indexed MCP joint `J1`,
  and a rod from TCP `H` to an `RP4` rotating pin that translates along the lower
  distal-phalanx surface;
- mechanism-only workspace and nominal quasi-static torque analysis, plus a
  synchronized mechanism–hand sweep for all four long fingers;
- pixel-derived index, middle, ring, and little finger targets in millimetres and
  degrees;
- a bounded, non-destructive Adam optimization skeleton over all 12 mechanism link
  lengths plus the `H–RP4` output-rod length; and
- an active two-pose task-space reachability objective; all energy, torque, path,
  regularization, collision, singularity, and joint-limit terms are currently disabled.

The optimizer analytically tracks the candidate linkage and minimizes the `H–RP4` rod
closure error over the finite distal slot at each finger's horizontal and
maximum-curled target. It remains an
early design tool: full joint-pose tracking, force, energy, collision, singularity,
and safety metrics are disabled, so candidates are not yet physically validated.

See [handover.md](handover.md) for the complete mental model, current contracts,
limitations, and recommended next steps.

## Quick starts

Create the environment:

```bash
conda env create -f environment.yml
conda activate kinematic-chain
```

If it already exists:

```bash
conda env update --name kinematic-chain --file environment.yml
conda activate kinematic-chain
```

Validate and analyze the nominal mechanism:

```bash
./run_analysis.sh
```

Each standalone run creates `runs/analysis_<UTC timestamp>/`, grouped into
`mechanism/` and `combined/`. The aggregate four-finger abstraction and sweep are in
`combined/`. Each of `combined/fingers/{index,middle,ring,little}/` contains a
workspace-style `combined_workspace_report.png`, its individual abstraction, sample
CSV, and YAML summary. The report overlays representative poses of the complete palm,
three phalanges, mechanism, output rod, and RP4 slider from horizontal extension to
the supplied maximum MCP/PIP/DIP curl; supporting panels report RP4 translation and
rod-closure error.

Run the current multi-objective Adam skeleton:

```bash
./run_optimization.sh
```

Each run creates `runs/co-optimization_<UTC timestamp>/`; there is no algorithm-named
`adam/` output layer. The retained result is stored under `candidate_0001/` as both a
candidate record and a complete mechanism YAML. A timestamped `analysis_*` directory
inside that candidate contains the same full analysis suite produced by
`run_analysis.sh`.

Override optimizer settings:

```bash
./run_optimization.sh \
  --iterations 500 \
  --learning-rate 0.2 \
  --output-dir /tmp/co-optimization
```

Run all tests:

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=src/analysis \
MPLCONFIGDIR=/tmp/kinematic-chain-matplotlib \
python3 -m unittest discover -s tests -v
```

Primary inputs are [the nominal mechanism](designs/mechanism_2/nominal/mechanism.yaml),
[the objective manifest](src/co-optimization/config/objectives.yaml), and
[the link-length variables](src/co-optimization/config/optimizable_variables.yaml).
The active loss is specified in
[objective_math.md](src/co-optimization/objective_math.md).
The nominal [four-finger combined abstraction](designs/mechanism_2/nominal/combined_abstraction.png)
and its adjacent finger-specific images show the current attachment assumptions.
