# Mechanism 2 exoskeleton co-optimization handover

## Product mental model

We are designing a planar, hand-mounted exoskeleton based on `mechanism_2`. One
actuated linkage topology should support the index, middle, ring, and little fingers.
Those fingers differ in phalanx lengths and intended curl, so success is not a single
best-fit pose: it is a constrained compromise across four human geometries, motion
trajectories, device packaging, and actuator performance.

The present optimizer assumes one shared mechanism link-length vector for all four
fingers. That is only the first co-design interpretation. A likely better product
architecture is a shared topology and shared dimensionless ratios with a small set of
finger-specific scale, attachment, or mounting variables. This shared-versus-local
variable split has not yet been decided.

The intended information flow is:

```text
12 mechanism link lengths + tip-rod length
                           |
                           v
             candidate geometry and assembly branch
                           |
                           v
       per-finger kinematics, fingertip path, forces, energy
                           |
                           v
        objective vector + feasibility/safety constraints
                           |
                           v
              scalarization/Pareto strategy + Adam
                           |
                           v
            validated candidate, never automatic adoption
```

The repository has the inputs and optimizer plumbing around this flow. The missing
core is the candidate-to-per-finger kinematic and load predictor.

The nominal assembly uses a mildly curled index-finger pose, a horizontal placeholder
palm (`90 × 25 mm`, rounded rectangle), and places the complete mechanism on its dorsal
side. It makes two interface assumptions explicit:

- mechanism node `D` is separated dorsally from indexed hand joint `J1` (`hand_mcp`)
  by `dorsal_clearance_mm`, a manually controlled upstream design parameter currently
  set to 1 mm; ground member `A–D` remains horizontal, and unequal phalanx widths
  extend toward the palmar side; and
- output TCP `H` connects through a nominal 28 mm rod to compound pair `RP4` on the
  distal phalanx's lower surface. The pin rotates freely and translates through
  `0…L_distal`; it is not a fixed distal revolute joint.

The output rod is fixed-length within one mechanism motion, but its length is a design
variable between candidates. The present 28 mm baseline is the rounded minimum-RMS
choice from a rod-only scan of the four synchronized nominal sweeps; the earlier 15 mm
assumption is retained as `previous_assumption_mm` in the nominal YAML for provenance.

These interfaces are external metadata and do not add unconstrained human joints to
the one-DOF mechanism solver.

## Nominal mechanism

The source of truth is:

```text
designs/mechanism_2/nominal/mechanism.yaml
```

Node order follows the dimensioned source exactly: `a, b, c, d, e, f, g, h`.
`f` and `g` are distinct revolute joints; `h` is the fingertip/output reference.

| Body | Type | Nodes |
|---|---|---|
| `ground` | fixed rigid body | A–D–E |
| `input_crank` | binary link | A–B |
| `loop_1_coupler` | binary link | B–C |
| `central_body` | rigid body | C–D–G |
| `link_ef` | binary link | E–F |
| `distal_body` | rigid body | F–G–H |

The closed loops are `A-B-C-D-A` and `D-G-F-E-D`. Six bodies and seven equivalent
lower pairs give planar mobility 1, matching the rotary actuator at A. Positive input
is clockwise closing motion.

Current remeasured dimensions are:

| Dimension | Value (mm) | Dimension | Value (mm) |
|---|---:|---|---:|
| `L_ab` | 31 | `L_bc` | 54 |
| `L_cd` | 28 | `L_ad` | 54 |
| `L_ae` | 66 | `L_de` | 14 |
| `L_cg` | 50 | `L_dg` | 57 |
| `L_ef` | 30 | `L_fg` | 28 |
| `L_gh` | 50 | `L_fh` | 57 |

`L_ad=54 mm` is rounded from the supplied plot value `53.845 mm`; it was absent from
the typed measurement list. Explicit initial coordinates were reconstructed so the
rounded dimensions close at the nominal pose.

The nominal analysis requests 0–90° input motion, but the current assembly branch only
closes through 65.5°. The analysis correctly stops there rather than extrapolating.

## Human-finger targets

The source drawings and YAML abstractions live in:

```text
src/co-optimization/config/objectives/
```

Each drawing contains a horizontal nominal pose and a maximum intended curled pose.
Handwritten centimetre lengths were converted to millimetres. Curl angles were
estimated from the drawn pixel polylines with an approximate ±5° uncertainty.

| Finger | Proximal / middle / distal (mm) | MCP range | PIP range | DIP range |
|---|---:|---:|---:|---:|
| Index | 40 / 25 / 25 | 0–46° | 0–67° | 0–27° |
| Middle | 49 / 27 / 27 | 0–37° | 0–79° | 0–27° |
| Ring | 44 / 26 / 26 | 0–39° | 0–52° | 0–50° |
| Little | 34 / 23 / 23 | 0–65° | 0–44° | 0–34° |

These are early design targets, not clinical range-of-motion limits. The coordinate
registration between the human MCP/PIP/DIP joints and mechanism nodes has not been
validated.

The combined suite draws a horizontal placeholder palm and three slightly curled
rounded phalanges for every long finger. Lengths come from each finger objective; the
provisional widths `20/15/10 mm` and `90 × 25 mm` palm are shared. The four-panel view
is `designs/mechanism_2/nominal/combined_abstraction.png`, with individual files named
`combined_abstraction_<finger>.png`. Anatomical revolutes are indexed `J1–J3`; the
lower-surface rotating/translating output pair is labeled `RP4`.
Mechanism placement enforces horizontal `A–D`, the upstream `D–J1` clearance, and
nominal `H–RP4` slot/rod closure, but remains an abstraction rather than a validated wearable
fit.

## Analysis pipeline

Canonical entry point:

```bash
./run_analysis.sh
```

It validates the YAML body–joint graph, exports tables, draws the abstraction, sweeps
the distance-constrained mechanism workspace, runs a coupled mechanism–hand sweep for
all four long fingers, and calculates nominal quasi-static gravity torque. Standalone
outputs go to `runs/analysis_<UTC timestamp>/`.

The combined sweep synchronizes normalized actuator progress with each finger's
horizontal-to-maximum-curl target. At every sample it reports TCP `H`, the finite
lower-distal RP4 slot, the closest valid slider coordinate, output-rod closure error,
and mechanism loop residual. It also searches the feasible actuator sweep for the
independently best match to each static curled target. The generated artifacts are:

```text
runs/analysis_<UTC timestamp>/
├── mechanism/
│   ├── abstraction.png
│   ├── workspace/
│   └── torque/
└── combined/
    ├── combined_abstraction.png
    ├── combined_workspace_report.png
    ├── combined_workspace_samples.csv
    ├── combined_workspace_summary.yaml
    └── fingers/
        └── <finger>/
            ├── combined_abstraction.png
            ├── combined_workspace_report.png
            ├── combined_workspace_samples.csv
            └── combined_workspace_summary.yaml
```

This is a prescribed-motion compatibility analysis, not yet a coupled dynamics or
contact solve: the hand angles are interpolated from measured target ranges rather
than predicted from mechanism forces and attachment constraints.
Each per-finger workspace report nevertheless renders the whole declared assembly at
representative synchronized poses: fixed palm and D/J1 mounting clearance, all three
rounded phalanges from horizontal through the supplied maximum MCP/PIP/DIP curl,
every mechanism body, TCP `H`, output rod, and the translating RP4 pin.

What is trustworthy today:

- topology, joint incidence, loop declaration, and mobility checks;
- the remeasured nominal link table;
- closure residuals for the nominal assembly branch;
- repeatable four-finger RP4 translation and rod-closure metrics under the declared
  synchronized motion assumption;
- identification of infeasible requested input poses; and
- repeatable generated reports.

What remains provisional:

- photo-derived node evidence and the rounded `L_ad` assumption;
- assembly-mode selection based only on continuity from the initial pose;
- the placeholder mass and center-of-mass model;
- torque without finger contact loads, friction, springs, or transmission losses;
- collision, packaging, singularity, and structural checks; and
- any manufacturing or actuator-sizing conclusion.

## Co-optimization pipeline

Canonical entry point:

```bash
./run_optimization.sh
```

The shell wrapper delegates numerical optimization to
`src/co-optimization/run_adam.py`, materializes every retained candidate as a complete
mechanism YAML, and invokes the full analysis entry point for that candidate. The
implementation loads YAML configuration, performs bounded Adam updates with central
finite-difference gradients, retains individual per-finger losses, scalarizes them
with configurable weights, and writes:

```text
runs/co-optimization_<UTC timestamp>/
├── history.csv
└── candidate_0001/
    ├── candidate.yaml
    ├── mechanism.yaml
    └── analysis_<UTC timestamp>/
        ├── mechanism/
        └── combined/
            ├── combined_workspace_report.png
            └── fingers/<finger>/combined_workspace_report.png
```

The `candidate_0001` numbering is the retained-candidate contract, not an assumption
that optimization will always produce only one candidate. Future Pareto or checkpoint
candidates should be numbered alongside it and must each receive a full analysis.
There is deliberately no `adam/` path component: algorithms are implementation
metadata. The pipeline is non-destructive and never modifies the nominal design.

The first result produced under this contract illustrates why the nested analysis is
mandatory: although its two-pose optimization loss decreased, its reconstructed
assembly branch completed only `0…39°` of the requested `0…90°` sweep and accumulated
large synchronized middle/ring-finger errors. It is therefore an analyzed but rejected
candidate, not a design improvement. Feasible sweep extent must become an explicit
optimization constraint before candidates can be promoted.

### Design variables

`src/co-optimization/config/optimizable_variables.yaml` declares all 12 mechanism link
lengths plus the external `H–RP4` output rod (`L_tip_rod`) as bounded variables. Their
initial values match the nominal mechanism and its 28 mm attachment baseline.
The bounds are broad scaffolding limits and have not been checked against packaging,
anatomy, triangle inequalities, or assembly feasibility.

### Active objective behavior

The sole active objective is two-pose task-space reachability of lower distal slot
`RP4`. For each finger, forward kinematics of its measured phalanx lengths and curl
angles produces horizontal and maximum-curled slot poses. An analytic linkage solver
tracks TCP `H`; the loss minimizes rod-closure residual over translation
`0…L_distal`. Horizontal is evaluated at zero actuator input and curl uses a smooth
minimum over the feasible 0–90° sweep. All 13 design lengths can affect this loss. The complete derivation is in
`src/co-optimization/objective_math.md`.

Reaching the RP4 slot does not prove that the intermediate anatomical joints reproduce the full
drawn hand pose. The generated candidate remains a reachability study, not a feasible
mechanism recommendation.

### Declared but unevaluated placeholders

`config/objectives.yaml` records the intended objective vocabulary:

- pose tracking;
- fingertip-path tracking;
- actuation energy;
- peak motor torque;
- cross-finger error or robustness;
- link-length regularization;
- compactness; and
- motion smoothness.

It also declares closure, singularity, joint-limit, and collision penalties. Every one
of these placeholder objectives and penalties is currently disabled; they are planning
metadata, not active safeguards.

## Important modeling decisions still open

1. **Shared versus finger-specific geometry.** Decide whether all fingers use one
   identical link vector, scaled copies, shared ratios with local attachment offsets,
   or independently sized mechanisms with shared actuator/manufacturing constraints.
2. **Human-device registration.** Define which mechanism points attach to each phalanx
   and how device motion maps to MCP/PIP/DIP flexion without forcing joint-axis
   coincidence.
3. **Motion parameterization.** Decide whether all fingers share actuator angle,
   normalized curl progress, cable displacement, or force as the comparison variable.
4. **Objective aggregation.** A weighted mean can sacrifice one finger. Decide whether
   the primary robustness term is worst-case loss, a smooth maximum, explicit per-finger
   limits, or a Pareto-front selection.
5. **Feasibility treatment.** Decide which requirements are hard constraints and which
   are differentiable penalties. Closure, assembly, collision, and safe joint limits
   should generally not be traded away for better tracking.

## Recommended implementation sequence

### 1. Freeze the human-device mapping

Extend the existing nominal hand/attachment schema to every finger. Validate the
dorsal MCP mount at node D, manually defined clearance, lower-surface distal slot,
offset, and the mapping from mechanism motion to phalanges before adding more
objectives.

### 2. Build candidate geometry robustly

The active evaluator now reconstructs candidate geometry with analytic circle
intersections and continuity-based assembly-branch tracking. The next step is to expose
explicit triangle-inequality margins for `ADE`, `CDG`, and `FGH`, distinguish truncated
workspaces from complete sweeps, and report why non-assemblable candidates failed.

### 3. Connect a per-finger kinematic predictor

The current predictor returns mechanism TCP `H` and `H–RP4` slot reachability. Extend it to
solve the complete coupled hand trajectory and return:

- mechanism node coordinates;
- predicted human MCP/PIP/DIP angles;
- fingertip position and orientation;
- closure residual and feasible motion extent;
- singularity or Jacobian metrics; and
- attachment misalignment or sliding demand.

This is required before treating a reachable slot as proof of the full hand pose.

### 4. Activate hard feasibility checks first

Before tuning soft objective weights, make candidate rejection reliable for assembly,
loop closure, joint limits, ground/hand packaging, link collision, finger collision,
and a minimum singularity/transmission margin. Add tests containing deliberately invalid
link vectors.

### 5. Activate pose and path objectives

Compare predicted trajectories with the full nominal-to-curled target, not only endpoint
angles. Normalize errors by the measured uncertainty and finger size. Add a worst-finger
metric so a low mean error cannot conceal an unusable little or middle finger.

### 6. Establish the physical load model

Measure or estimate link masses, centers of mass, actuator/cable routing, friction,
return springs, desired fingertip force, and external finger resistance. Then activate
energy, peak torque, transmission, and thermal/duty-cycle objectives. The current
gravity-only torque report is insufficient for actuator selection.

### 7. Choose the co-design variable hierarchy

A practical starting point is shared dimensionless linkage ratios plus one scale and a
few attachment offsets per finger. This captures anatomical variation without creating
four unrelated mechanisms. Manufacturing objectives can then reward common parts and a
small number of discrete link sizes.

### 8. Improve optimization only after the evaluator is stable

Central finite differences are appropriate for plumbing tests but will become expensive
and noisy once each evaluation contains four full sweeps and collision/load analysis.
First validate the evaluator against hand calculations and known poses. Then consider
automatic differentiation, analytic sensitivities, parallel finite differences, or a
hybrid global/local search. Adam should not be the only algorithm used to establish
design quality.

### 9. Add candidate acceptance and provenance

Keep optimization non-destructive. A future promotion command should save the complete
configuration, objective vector, constraint margins, solver version, and plots; validate
the candidate with the analysis pipeline; compare it against the nominal baseline and
per-finger thresholds; and only then create a new named design directory. Never overwrite
`designs/mechanism_2/nominal/` automatically.

## Immediate definition of done for the next milestone

The next milestone should produce one candidate link vector for which all four fingers
can be simulated from horizontal to intended curl, with explicit attachment mapping,
zero accepted closure violations, recorded joint-limit and singularity margins, and
per-finger pose/path errors. Energy and torque can remain secondary until this
kinematic-feasibility milestone is trustworthy.

## Key paths

| Purpose | Path |
|---|---|
| Nominal mechanism | `designs/mechanism_2/nominal/mechanism.yaml` |
| Nominal dimensioned graph | `designs/mechanism_2/nominal/mechanism_graph.png` |
| Combined hand/device abstraction | `designs/mechanism_2/nominal/combined_abstraction.png` |
| Finger objective manifest | `src/co-optimization/config/objectives.yaml` |
| Per-finger targets | `src/co-optimization/config/objectives/*.yaml` |
| Active objective derivation | `src/co-optimization/objective_math.md` |
| Link-length variables | `src/co-optimization/config/optimizable_variables.yaml` |
| Adam implementation | `src/co-optimization/run_adam.py` |
| Analysis implementation | `src/analysis/` |
| Combined mechanism–hand analyzer | `src/analysis/combined_analysis.py` |
| Analysis entry point | `run_analysis.sh` |
| Optimization entry point | `run_optimization.sh` |
| Tests | `tests/` |

At handover time, all 18 tests pass. The tests cover topology/schema consistency,
workspace closure for the nominal design, plotting, nominal torque plumbing, exact
alignment between optimizer variable initials and nominal dimensions, and end-to-end
execution through the top-level Adam wrapper. They also verify that task-space
reachability is the only enabled optimization objective and validate the nominal hand
dimensions, attachment interfaces, combined abstraction renderer, and four-finger
combined sweep bounds.
