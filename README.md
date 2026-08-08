# Kinematic-chain abstraction pipeline

This repository turns a real planar mechanism into a validated, reusable body–joint
abstraction. The current target is **mechanism 2**. Its topology, nominal geometry,
joint incidence, actuation, and model-readiness state live in one YAML file:

```text
sources/mechanism_2/mechanism.yaml
```

The tooling is mechanism-independent. Any planar mechanism represented with the same
tables can use the same validation, reporting, and plotting pipeline without adding
mechanism-specific joint names to the Python code.

## Quick start

From the repository root, create the environment and run the current analysis:

```bash
conda env create -f environment.yml
conda activate kinematic-chain
./src/run_analysis.sh
```

If the environment already exists, update it instead:

```bash
conda env update --name kinematic-chain --file environment.yml
conda activate kinematic-chain
./src/run_analysis.sh
```

The analysis writes:

```text
runs/mechanism_2/
├── abstraction.png       # topology with every declared nominal dimension
├── link_lengths.csv      # machine-readable dimension table
├── mechanism_tables.md   # complete human-readable abstraction report
├── workspace_report.png  # swept poses, output path, and closure residuals
├── workspace_samples.csv # solved node coordinates for every input sample
├── torque_report.png     # nominal quasi-static gravity torque
└── torque_samples.csv    # total and component torque samples
```

Open [the generated abstraction plot](runs/mechanism_2/abstraction.png) to inspect the
current mechanism.

Run the test suite with:

```bash
PYTHONPATH=src MPLCONFIGDIR=/tmp/kinematic-chain-matplotlib \
  python3 -m unittest discover -s tests -v
```

Preview the reserved optimization entry point without treating its expected placeholder
status as a shell failure:

```bash
./src/run_optimization.sh || [[ $? -eq 2 ]]
```

This validates the current abstraction and demonstrates the future override contract;
it does not optimize or modify anything yet.

## Current mechanism

Mechanism 2 currently has the following abstract topology:

| Body | Type | Nodes | Purpose |
|---|---|---|---|
| `ground` | fixed rigid body | A–D–E | Base frame |
| `input_crank` | binary link | A–B | Actuated input |
| `loop_1_coupler` | binary link | B–C | First-loop coupler |
| `central_body` | rigid body | C–D–F | Central ternary member |
| `link_eh` | binary link | E–H | Lower distal coupler |
| `distal_body` | rigid body | F–H–I | Output member and fingertip |

Source labels **F and G refer to the same physical revolute joint**. The abstraction
canonicalizes that joint as node `f`; there is no separate node `g`, FG link, or
`L_fg` dimension. Original source points H and I remain nodes `h` and `i`.

The two closed paths are A–B–C–D–A and D–F–H–E–D. Six bodies and seven equivalent
lower pairs give planar Grübler mobility 1, matching the single rotary actuator at A.
Positive input is clockwise closing motion. The configured workspace request is
0–90°; with the present photo-derived lengths, the nominal distance constraints close
only through approximately 66°. The workspace report marks 66–90° as infeasible
instead of extrapolating nonexistent poses.

The 12 current dimensions are nominal estimates derived from the source photograph.
They include uncertainty and drive the generic distance-constraint workspace solver.
The torque stage uses an explicitly declared nominal placeholder mass model. These
results support pipeline development, but not manufacturing, rated-load, or actuator
sizing decisions. Diagram coordinates under `nodes[*].layout` control only the plot
layout and are never interpreted as physical lengths.

## Pipeline contracts

The repository deliberately separates inspection from mutation:

| Pipeline | Entry point | Input mutation | Current status |
|---|---|---|---|
| Analysis | `src/run_analysis.sh` | Never | Operational |
| Optimization | `src/run_optimization.sh` | Reserved for validated atomic replacement | Placeholder |

### Analysis pipeline

Use analysis to validate and render the current mechanism without modifying its YAML:

```bash
./src/run_analysis.sh [abstraction.yaml] [output-directory]
```

With no arguments it reads `sources/mechanism_2/mechanism.yaml` and writes to
`runs/mechanism_2`. A custom, read-only run looks like:

```bash
./src/run_analysis.sh \
  sources/mechanism_2/mechanism.yaml \
  /tmp/mechanism_2-analysis
```

The pipeline performs these operations:

1. Load and validate the YAML schema and all cross-table references.
2. Calculate body count, equivalent lower pairs, independent loops, and planar mobility.
3. Export the complete mechanism report as Markdown.
4. Export the dimension table as CSV.
5. Render the topology and annotate every declared nominal length and unit.
6. Sweep the rotary input, solve the declared distance constraints, and report the
   output workspace and closure residual at every feasible pose.
7. Apply the YAML mass model to the solved poses and calculate nominal quasi-static
   gravitational input torque by virtual work.

The abstraction YAML is read-only throughout this flow. Invalid topology, incidence,
dimensions, loops, actuators, or outputs stop the pipeline; artifacts completed by an
earlier stage may remain in the selected output directory.

### Optimization pipeline

The optimization entry point reserves the future mutation workflow:

```bash
./src/run_optimization.sh [abstraction.yaml] [candidate-work-directory]
```

It currently validates the input, prints the intended workflow, leaves the YAML
unchanged, and exits with status `2`. That exit status is expected while optimization
is a placeholder.

The implementation contract is:

1. Load and validate the current abstraction.
2. Copy parameters into an isolated candidate workspace.
3. Optimize the candidate against an explicit objective and constraints.
4. Validate the complete candidate with the same generic schema and kinematic checks.
5. Atomically replace the current YAML only when every check succeeds.
6. Regenerate the analysis report, dimension CSV, and abstraction plot.

Optimization must not overwrite the current abstraction partially or replace it with
an invalid candidate. A working optimizer still requires validated kinematics, design
bounds, an objective function, and an acceptance criterion.

## Editing the abstraction

Edit only the YAML source of truth:

```text
sources/mechanism_2/mechanism.yaml
```

Its main tables are:

| YAML section | Meaning |
|---|---|
| `mechanism` | Identity, units, coordinate convention, and readiness status |
| `sources` | Photographs or other evidence used for the abstraction |
| `photo_calibration` | Pixel scale and uncertainty for nominal photo geometry |
| `nodes` | Revolute joints and reference/output points |
| `bodies` | Binary links, multipoint rigid bodies, and ground |
| `joints` | Joint-to-body incidence |
| `dimensions` | Body-owned centre-to-centre distances |
| `loops` | Closed node and body cycles |
| `actuators` | Driven joints and positive direction |
| `outputs` | Tracked points and their owning bodies |
| `analysis` | Sweep range, sampling, and numerical solver settings |
| `mass_model` | Body masses, point masses, and centre-of-mass node weights |
| `model_readiness` | Availability of downstream analyses |

After changing the YAML, regenerate and verify all derived files:

```bash
./src/run_analysis.sh
PYTHONPATH=src MPLCONFIGDIR=/tmp/kinematic-chain-matplotlib \
  python3 -m unittest discover -s tests -v
```

Do not independently edit `runs/mechanism_2/mechanism_tables.md` or
`runs/mechanism_2/link_lengths.csv`; they are generated views and will be overwritten.

## Validation rules

The generic validator currently enforces that:

1. Exactly one ground body exists.
2. Every body references known, non-repeated nodes.
3. A binary link contains exactly two nodes.
4. Every revolute node has one joint-incidence row.
5. Joint incidence exactly matches body membership.
6. Every dimension belongs to a body containing both endpoint nodes.
7. Each binary link has a physical dimension.
8. Multipoint rigid bodies have enough dimensions to define their planar shape.
9. Photo-derived values agree with calibrated pixel distances and carry uncertainty.
10. Every loop side belongs to its declared body.
11. Actuators reference existing revolute joints and bodies, and output nodes belong to
    their declared bodies.
12. Planar mobility is derived from the validated body and lower-pair counts.
13. Workspace ranges, tolerances, and iteration limits are numerically valid.
14. Mass rows reference existing bodies/nodes and use valid centre weights.

These checks validate the abstraction’s structure. They do not yet prove that every
input angle has a feasible assembly configuration or that the mechanism avoids branch
changes, singularities, interference, or excessive loads.

## Direct tools

The two shell entry points are the normal interface. The underlying tools are also
available for focused work.

Validate and print a summary:

```bash
python3 src/mechanism_table.py sources/mechanism_2/mechanism.yaml
```

Export selected tables:

```bash
python3 src/mechanism_table.py sources/mechanism_2/mechanism.yaml \
  --markdown /tmp/mechanism_tables.md \
  --csv /tmp/link_lengths.csv
```

Render a plot at a custom location:

```bash
MPLCONFIGDIR=/tmp/kinematic-chain-matplotlib \
  python3 src/plot_linkage.py sources/mechanism_2/mechanism.yaml \
  --output /tmp/abstraction.png
```

Add `--show` to the plot command when an interactive graphical session is available.

Run only the workspace sweep:

```bash
MPLCONFIGDIR=/tmp/kinematic-chain-matplotlib \
  python3 src/workspace_sweep.py sources/mechanism_2/mechanism.yaml \
  --output /tmp/workspace_report.png \
  --csv /tmp/workspace_samples.csv
```

Run only the nominal torque analysis:

```bash
MPLCONFIGDIR=/tmp/kinematic-chain-matplotlib \
  python3 src/torque_analysis.py sources/mechanism_2/mechanism.yaml \
  --output /tmp/torque_report.png \
  --csv /tmp/torque_samples.csv
```

Both tools accept `--q-min`, `--q-max`, and `--steps` overrides. Without overrides,
they use `analysis.workspace_sweep` from the YAML.

## Repository layout

```text
environment.yml                         Conda environment
sources/mechanism_2/
  mechanism.yaml                        Editable abstraction source of truth
  1224a3cf4f13d2e78e428296289e2e0c.jpg Source assembly photograph
  mechanism_abstract.md                 Reference table snapshot
  link_lengths.csv                      Reference dimension snapshot
src/
  mechanism_schema.py                   Generic loader, validator, and mobility summary
  mechanism_table.py                    Markdown and CSV exporter
  plot_linkage.py                       Dimensioned abstraction renderer
  workspace_sweep.py                    Generic distance-constraint workspace solver
  torque_analysis.py                    Generic quasi-static gravity analysis
  run_analysis.sh                       Read-only analysis entry point
  run_optimization.sh                   Reserved optimization/override entry point
tests/test_mechanism_schema.py           Schema, workspace, torque, and plot tests
runs/mechanism_2/                        Generated analysis artifacts
```

## Preparing for numerical optimization

Before activating the optimization pipeline, replace or validate:

- the photo-derived joint-centre dimensions with measurements or CAD geometry;
- the current continuity-based assembly-mode selection with design-specific rules;
- permitted input range and design-variable bounds;
- the required output trajectory or workspace;
- an objective function and candidate acceptance thresholds;
- collision, singularity, packaging, and transmission constraints;
- the nominal mass placeholders with measured masses and centres of mass;
- external loads and drive limits if torque is part of the objective.

Those additions should consume the same YAML abstraction. Mechanism-specific solvers
or parameters should extend the schema explicitly rather than reintroducing hard-coded
dependencies on a particular mechanism.
