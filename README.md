# Biomimetic finger-linkage analysis

This project reconstructs the planar finger linkage from the images in
`sources/`, evaluates its workspace, and optimizes its passive-loop pivots for
monotonic finger closure.

The mechanism has one active input: rotation `q` at `R01`. The two passive rods
close the loops `P0-B2` and `A1-C3`. The anatomical phalange lengths are always:

- `R01-R12 = 40 mm`
- `R12-R23 = 24 mm`
- `R23-T3 = 20 mm`

The optimizer does not change these three lengths.

## Repository layout

```text
.
├── environment.yml          # reproducible Conda environment
├── README.md
├── sources/                 # CAD views, explanation, and topology rules
├── src/                     # model, sweep, optimization, and torque scripts
└── runs/
    ├── nominal/             # reports from the original geometry
    ├── opt_YYYYMMDD_HHMMSS/ # automatically stamped optimization runs
    └── rejected_*/          # retained invalid runs, never design inputs
```

Source code and input references stay separate from generated results. All
default output paths are under `runs/`.

## Environment

Create the reproducible Conda environment:

```bash
conda env create -f environment.yml
conda activate kinematic-chain
```

The environment contains Python 3.11, NumPy, SciPy, and Matplotlib. The system
Python in the original workspace has incompatible NumPy and SciPy versions, so
the Conda environment should be used for optimization.

## Scripts

### Draw the nominal mechanism

```bash
python src/plot_linkage.py
```

The initial configuration is the horizontal, fully open pose. Geometry and plot
settings are parameterized by `MechanismParameters` and `PlotParameters`.

### Analyze the nominal workspace

```bash
python src/workspace_sweep.py
```

This solves both passive loop closures while rotating only `R01`. It writes
`runs/nominal/workspace_report.png` and opens the report interactively.

### Optimize biomimetic closure

Run the deterministic global and local search:

```bash
python src/optimize_linkage.py
```

Default synthesis target over `q = 0...90 deg`:

```text
PIP flexion = 1.0 * q
DIP flexion = 0.70 * PIP flexion
```

The optimization variables are the local coordinates of:

- `P0` on Base
- `A1` on link 1
- `B2` on link 2
- `C3` on link 3

The passive-rod lengths are calculated from these pivots in the open pose and
then remain constant throughout the sweep.

The objective includes:

- PIP and DIP target-trajectory error
- a strong penalty for non-monotonic flexion
- positive anatomical flexion direction and joint-angle limits
- minimum clearance between nonadjacent phalanges
- failed loop closure and assembly-branch loss
- poor transmission angles near toggle singularities
- weak geometry regularization and excessive rod-length penalties

The default run writes:

- `runs/opt_YYYYMMDD_HHMMSS/parameters.json` -- parameters and metrics
- `runs/opt_YYYYMMDD_HHMMSS/optimization_report.png` -- flexion report
- `runs/opt_YYYYMMDD_HHMMSS/workspace_report.png` -- optimized workspace
- `runs/opt_YYYYMMDD_HHMMSS/torque_report.png` -- input holding torque
- `runs/opt_YYYYMMDD_HHMMSS/torque_samples.csv` -- numerical torque samples

The timestamp uses local time at the start of the optimization. It is also saved
as ISO 8601 metadata inside `parameters.json`. Use `--run-dir` only when an
explicit directory name is required.

It also opens the generated reports interactively unless `--no-show` is supplied.

### Analyze unloaded input torque

The torque model is quasi-static. It computes the R01 actuator holding torque
from gravitational virtual work, `tau = dU/dq`. The default TCP payload is zero.

```bash
python src/torque_analysis.py \
  --params runs/opt_YYYYMMDD_HHMMSS/parameters.json
```

Default lumped masses are 5/4/3 g for links 1/2/3, 3 g for each moving-joint
assembly, and 1 g for each passive rod. Override them with measured values:

```bash
python src/torque_analysis.py \
  --params runs/opt_YYYYMMDD_HHMMSS/parameters.json \
  --link-masses-g 5 4 3 \
  --joint-mass-g 3 \
  --rod-masses-g 1 1 \
  --tcp-payload-g 0
```

The report uses mN·m; its numerical value is identical in N·mm. Negative signed
torque means gravity assists positive-q closure. This model excludes friction,
joint preload, actuator/gear inertia, acceleration torque, cable forces, and
contact loads, so it is not an actuator-sizing result by itself.

### Draw the SyLink mechanism abstraction

The supplied SyLink paper is stored at `sources/sylink/2606.14250v1.pdf`.
Figure 4(c,f) is abstracted into the same mechanical-graph convention used by
this project:

```bash
python src/plot_sylink_mechanisms.py
```

The result is written to
`runs/nominal/sylink/mechanism_abstraction.png`. It shows one crossed four-bar
coupling PIP–DIP for each ordinary finger and two stacked crossed four-bars
coupling CMC–MCP–IP for the thumb. Because the paper provides symbolic topology
but no complete dimensional table, this diagram is explicitly not a scale
reconstruction.

For a faster development run:

```bash
python src/optimize_linkage.py \
  --run-dir runs/quick_test \
  --maxiter 30 --popsize 8 --samples 13
```

Change the synthesis target if measured Inspire Hand or human-finger data are
available:

```bash
python src/optimize_linkage.py \
  --q-max 90 \
  --pip-ratio 0.9 \
  --dip-ratio 0.7 \
  --monotonic-weight 100
```

## Reusing an optimized design

Draw the optimized open pose:

```bash
python src/plot_linkage.py \
  --params runs/opt_YYYYMMDD_HHMMSS/parameters.json \
  -o runs/opt_YYYYMMDD_HHMMSS/open_pose.png
```

Run a new workspace sweep with the optimized parameters:

```bash
python src/workspace_sweep.py \
  --params runs/opt_YYYYMMDD_HHMMSS/parameters.json \
  --q-min -5 --q-max 95 --steps 401 \
  -o runs/opt_YYYYMMDD_HHMMSS/extended_sweep.png
```

## Coordinate conventions

- World `+x` points right and `+y` points upward.
- Link-local `+x` points from the proximal joint toward the distal joint.
- Angles are counter-clockwise in the model.
- Positive input `q` closes the finger from the horizontal open pose.
- Reported PIP and DIP values are relative joint flexions, not absolute link
  orientations.

## Current optimization result

The corrected verified run is `runs/opt_20260705_191401`. It preserves all
closures from `q=0` through `q=90 deg`, uses the anatomical flexion direction,
and has zero PIP/DIP reopening samples. Its end flexions are approximately
`PIP=78.6 deg` and `DIP=51.8 deg`. The minimum distance between link 1 and link 3
centerlines is `24.0 mm` over a 901-pose validation sweep. Both relative joint
angles remain strictly monotonic on that denser grid.

Under the documented unloaded mass assumptions, the peak quasi-static R01
holding torque is approximately `17.52 mN·m` (`17.52 N·mm`) at the open pose.
Hardware sizing must add measured friction, dynamic loads, contact force, drive
efficiency, and an appropriate safety factor.

This is a substantial improvement over the original geometry, but it does not
fully reach the aggressive `90/63 deg` target. Several pivot coordinates approach
their allowed bounds. That result indicates either the physical pivot bounds
must be reconsidered or the topology needs another design variable.

`runs/rejected_wrong_flexion_sign` is retained only for traceability. That run
used the wrong relative-angle sign and must not be used as a mechanical design.

## Engineering limitations

This is a planar kinematic synthesis tool, not a manufacturing validator. Before
building hardware, add checks for:

- link and pin collision/clearance
- plate boundaries around optimized pivots
- bearing, pin, and rod strength
- actuator torque and fingertip force
- backlash and compliance
- singularity margin on a finer motion grid
- alignment with anatomical joint centers for exoskeleton use
