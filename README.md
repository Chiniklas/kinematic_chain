# Multi-finger exoskeleton co-optimization

This repository is developing a hand-mounted exoskeleton from the planar
`mechanism_2` linkage. The index, middle, ring, and little fingers are four independent
design problems: each starts from the same nominal mechanism, then optimizes its own
copy of every mechanism and attachment length against that finger's geometry.

The project currently contains:

- a validated body–joint abstraction with remeasured nominal link lengths;
- a combined mildly curled nominal index-finger abstraction, with the mechanism on
  the dorsal side of a horizontal 90 × 25 mm palm, horizontal member
  `A–D`, a manually defined dorsal clearance between `D` and indexed MCP joint `J1`,
  and an ideal rod from TCP `H` to fixed revolute `R4` at the upper distal-phalanx
  midpoint;
- mechanism-only workspace and nominal quasi-static torque analysis, plus a coupled
  mechanism–hand sweep for all four long fingers;
- pixel-derived index, middle, ring, and little finger targets in millimetres and
  degrees;
- four bounded, non-destructive Adam optimization jobs, each with its own copy of 11
  mechanism link lengths plus the `H–R4` output-rod length; the 54 mm grounded `AD`
  baseline remains fixed; and
- one design objective, `H–R4` rod perpendicularity, plus hard full-workspace rod
  closure, downward-only hand motion, and sampled hand–mechanism non-collision
  constraints.

Each optimizer treats crank angle `q` as the sole prescribed coordinate. The current
ideal compliant hand has one passive curl coordinate constrained to the measured
horizontal-to-maximum-curl path; closure solves this response without a hand/crank
ratio, and a hard rule forbids dorsal bending or mid-motion reversal. The fixed
`H–R4` rod must close across all 31 poses.
Every pose must close within `0.1 mm`; smooth closure and clearance penalties guide
Adam toward the hard-feasible region but are not design objectives. It remains an
early design tool: collision is sampled rather than continuously certified, and full
joint-pose tracking, force, energy, singularity, and other safety metrics are still
incomplete, so candidates are not yet physically validated.

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

This reads the self-contained nominal `mechanism.yaml` and refreshes its adjacent
`designs/mechanism_2/nominal/artifacts/`, grouped into `mechanism/` and `combined/`.
The aggregate four-finger abstraction and sweep are in `combined/`. Each of
`combined/fingers/{index,middle,ring,little}/` contains a
workspace-style `combined_workspace_report.png`, its individual abstraction, sample
CSV, and YAML summary. The report overlays representative poses of the complete palm,
three phalanges, mechanism, output rod, and fixed R4 contact from horizontal extension
to the supplied maximum MCP/PIP/DIP curl; supporting panels report fixed-contact rod
closure and perpendicular deviation.

Analyze one other design, several YAMLs, or a directory tree containing designs:

```bash
./run_analysis.sh path/to/design/mechanism.yaml
./run_analysis.sh path/to/a/mechanism.yaml path/to/b/mechanism.yaml
./run_analysis.sh path/to/designs/
```

By default, every input writes to an `artifacts/` folder beside its own
`mechanism.yaml`. For a single input, `--output-dir /tmp/artifacts` redirects that
artifact tree. Analysis does not read co-optimization configuration; hand targets and
attachment assumptions required by a design are stored in that design's YAML.

Run the current multi-objective Adam skeleton:

```bash
./run_optimization.sh
```

Each run creates `runs/co-optimization_<UTC timestamp>/fingers/<finger>/`. Every finger
has its own `candidate_0001/`, `history.csv`, TensorBoard stream, optimized mechanism
YAML, and finger-specific full analysis. The four candidate vectors share no variables
or Adam state; only their initial values come from the same nominal YAML. Run-level
aggregate material is comparison-only and never enters a finger's loss.

After updating the environment, inspect a run with:

```bash
tensorboard --logdir runs/co-optimization_<UTC timestamp>/fingers
```

Override optimizer settings:

```bash
./run_optimization.sh \
  --iterations 500 \
  --learning-rate 0.2 \
  --output-dir /tmp/co-optimization
```

Run only one independent problem while developing its evaluator:

```bash
./run_optimization.sh --finger index --iterations 50
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
The nominal [four-finger combined abstraction](designs/mechanism_2/nominal/artifacts/combined/combined_abstraction.png)
and its grouped [finger-specific artifacts](designs/mechanism_2/nominal/artifacts/combined/fingers/)
show the current attachment assumptions. See the
[nominal design layout](designs/mechanism_2/nominal/README.md) for the source/artifact
contract.
