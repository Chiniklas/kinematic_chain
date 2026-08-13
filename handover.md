# Mechanism 2 exoskeleton co-optimization handover

## Product mental model

We are designing four planar, hand-mounted exoskeleton mechanisms based on the same
`mechanism_2` topology: one distinctive design for each of index, middle, ring, and
little. The fingers differ in phalanx lengths and intended curl, so each is an
independent optimization problem. All four jobs start from the same nominal mechanism
dimensions, then own separate copies of all design variables and separate Adam state.
There is no cross-finger scalarization or shared optimized link vector.

The intended information flow is:

```text
                 shared nominal Mechanism 2
                            |
             copy the same 12 optimizable initial lengths
          / index / middle / ring / little /
         v        v        v       v
  four independent candidate geometries and Adam states
         |        |        |       |
  finger-local kinematics, collision, contact and objectives
         |        |        |       |
  four validated candidates, never automatic nominal adoption
```

The repository has the inputs and independent optimizer plumbing around this flow.
It now includes a first crank-driven passive-hand coupling abstraction. The missing
core is continuous-time certification and a physically calibrated multi-DOF
hand/contact/load equilibrium model within each finger problem.

The nominal assembly uses a mildly curled index-finger pose, a horizontal placeholder
palm (`90 × 25 mm`, rounded rectangle), and places the complete mechanism on its dorsal
side. It makes two interface assumptions explicit:

- mechanism node `D` is separated dorsally from indexed hand joint `J1` (`hand_mcp`)
  by `dorsal_clearance_mm`, a manually controlled upstream design parameter currently
  set to 1 mm; ground member `A–D` remains horizontal, and unequal phalanx widths
  extend toward the palmar side; and
- output TCP `H` connects through a nominal 28 mm ideal rigid rod to fixed revolute
  `R4` at the longitudinal midpoint of the distal phalanx's upper surface. There is no
  translational pair or sliding contact.

The output rod is fixed-length within one mechanism motion, but its length is a design
variable between candidates. The present 28 mm value predates fixed R4 and is retained
only as the optimization initialization; it is not claimed to close the new nominal
assembly. The earlier 15 mm assumption remains in the nominal YAML for provenance.

This first interface layer is deliberately ideal: rigid distal phalanx, exact planar
revolute, and massless two-force rod. Cuff compliance, skin motion, attachment
pressure, backlash, and out-of-plane freedom are deferred.

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

### Selected input actuator

Each finger mechanism is intended to use one Dynamixel **XC330-M288-T** as its input
actuator. It commands the sole mechanism input coordinate `q` by driving the crank at
joint `A`; the human finger remains the passive side of the current coupling model.
This actuator choice is an upstream hardware requirement and is not an optimization
variable.

The repository does not yet model the XC330-M288-T motor curve, controller mode,
current/torque calibration, horn and mounting geometry, gear compliance, backlash,
communication latency, thermal limits, or duty cycle. Consequently, the present
gravity-only torque report does not validate this actuator for the intended
force-feedback load. Those properties must enter the quasi-static load model and later
prototype tests before a candidate can be considered actuator-feasible.

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
rounded phalanges for every long finger. Analysis lengths and ranges are embedded in
the design's `mechanism.yaml`; the co-optimization objective files are not analysis
dependencies. The
provisional widths `20/15/10 mm` and `90 × 25 mm` palm are shared. The four-panel view
is `designs/mechanism_2/nominal/artifacts/combined/combined_abstraction.png`, with
individual files under
`artifacts/combined/fingers/<finger>/combined_abstraction.png`. Anatomical revolutes are indexed
`J1–J3`; the fixed upper-distal attachment revolute is labeled `R4`.
Mechanism placement enforces horizontal `A–D`, the upstream `D–J1` clearance, and
nominal `H–R4` rod closure, but remains an abstraction rather than a validated wearable
fit.

## Analysis pipeline

Canonical entry point:

```bash
./run_analysis.sh
```

It accepts one `mechanism.yaml`, several YAMLs, or directories searched recursively
for `mechanism.yaml`. It validates each self-contained body–joint graph, exports tables, draws the abstraction, sweeps
the distance-constrained mechanism workspace, runs a coupled mechanism–hand sweep for
its declared finger targets, and calculates nominal quasi-static gravity torque. By
default, each input writes beside itself under `artifacts/`; `--output-dir` redirects
one selected design.

For standalone nominal analysis, the combined sweep still uses normalized actuator
progress as a visualization assumption. Candidate analysis instead replays the
optimizer's recorded crank-driven passive hand response. At every sample it reports
TCP `H`, fixed upper
distal contact `R4`, output-rod closure error, perpendicular deviation, and mechanism
loop residual. It also searches the feasible actuator sweep for the
independently best match to each static curled target. The generated artifacts are:

```text
<design>/artifacts/
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

Standalone nominal output is a prescribed-motion compatibility analysis. Optimized
candidate output uses fixed-contact closure to solve one passive curl coordinate, but
it is still kinematic: it has no calibrated joint stiffness, user effort, or contact
force equilibrium.
Each per-finger workspace report nevertheless renders the whole declared assembly at
representative coupled poses: fixed palm and D/J1 mounting clearance, all three
rounded phalanges from horizontal through the supplied maximum MCP/PIP/DIP curl,
every mechanism body, TCP `H`, output rod, and fixed R4 joint.

What is trustworthy today:

- topology, joint incidence, loop declaration, and mobility checks;
- the remeasured nominal link table;
- closure residuals for the nominal assembly branch;
- repeatable four-finger fixed-contact closure and perpendicularity metrics under the
  declared motion model;
- identification of infeasible requested input poses; and
- repeatable generated reports.

What remains provisional:

- photo-derived node evidence and the rounded `L_ad` assumption;
- assembly-mode selection based only on continuity from the initial pose;
- the placeholder mass and center-of-mass model;
- torque without finger contact loads, friction, springs, or transmission losses;
- continuously certified collision, packaging, singularity, and structural checks; and
- whether the selected Dynamixel XC330-M288-T satisfies torque, speed, bandwidth,
  thermal, and packaging requirements; and
- any manufacturing or actuator-sizing conclusion.

## Co-optimization pipeline

Canonical entry point:

```bash
./run_optimization.sh
```

The shell wrapper delegates numerical optimization to
`src/co-optimization/run_adam.py`. It instantiates four independent problems from the
finger manifest, gives every problem a fresh copy of the same nominal 12-value vector,
runs separate bounded Adam updates, materializes a distinct mechanism YAML, and invokes
the full analysis for only that candidate's finger. It writes:

```text
runs/co-optimization_<UTC timestamp>/
├── run_manifest.yaml
└── fingers/
    └── {index,middle,ring,little}/
        ├── history.csv
        ├── tensorboard/events.out.tfevents.*
        └── candidate_0001/
            ├── candidate.yaml
            ├── mechanism.yaml
            └── artifacts/
                ├── mechanism/
                └── combined/fingers/<that-finger>/combined_workspace_report.png
```

The `candidate_0001` numbering is the retained-candidate contract, not an assumption
that optimization will always produce only one candidate. Future Pareto or checkpoint
candidates should be numbered alongside it and must each receive a full analysis.
There is deliberately no `adam/` path component: algorithms are implementation
metadata. The pipeline is non-destructive and never modifies the nominal design.

Each finger owns a TensorBoard stream that logs its total and component losses,
optimizer gradient/step norms, each finite-difference gradient, and its 12 local
design-variable values and bounds. View all four with
`tensorboard --logdir runs/co-optimization_<timestamp>/fingers`.

A legacy result produced before the independent-finger refactor used one shared vector;
it is conceptually invalid and must not be used as a starting design. It also showed why
nested analysis remains mandatory: its assembly branch completed only `0…39°` of the
requested `0…90°` sweep. Feasible sweep extent must become an explicit constraint in
each finger problem before candidates can be promoted.

### Design variables

`src/co-optimization/config/optimizable_variables.yaml` declares 11 mechanism link
lengths plus the external `H–R4` output rod (`L_tip_rod`) as bounded variables. Their
initial values match the nominal mechanism and its 28 mm attachment baseline. `L_ad`
is excluded from Adam and remains the fixed 54 mm horizontal grounded baseline.
The bounds are broad scaffolding limits and have not been checked against packaging,
anatomy, triangle inequalities, or assembly feasibility.

### Active objective behavior

Output-rod perpendicularity is the sole design objective. Full-workspace fixed
`H–R4` closure, monotone downward hand curl, and sampled hand–mechanism non-collision
are hard constraints. The crank `q` is the sole prescribed input. A curled endpoint
search selects terminal `q_f,*`; closure then solves the passive one-DOF hand curl
coordinate `s` at 31 increasing crank poses. The rule `0 <= s <= 1`,
`s[k+1] >= s[k]`, `s[0]=0`, and terminal `s=1` forbids dorsal bending and reversal.
There is no fixed hand/crank ratio. Every pose must close within `0.1 mm`, and any
sampled unintended penetration rejects the candidate.
All 12 optimizable lengths can affect closure, clearance, and rod angle; fixed `L_ad`
still participates in every kinematic evaluation. The complete ideal-interface
derivation is in `src/co-optimization/objective_math.md`.

Candidate combined reports replay the recorded prescribed crank samples and passive
hand-progress samples. They do not synthesize a new linear hand/crank mapping;
mechanism-only reports still show the broader diagnostic sweep.

The passive response remains idealized: it restricts MCP/PIP/DIP motion to a measured
one-DOF synergy and assumes zero hand stiffness. It therefore proves geometric
compatibility only, not feedback force or comfort. Collision between the 31 samples is
also not yet continuously certified.

### Declared but unevaluated placeholders

`config/objectives.yaml` records the intended objective vocabulary:

- pose tracking;
- fingertip-path tracking;
- actuation energy;
- peak motor torque;
- legacy cross-finger error (disabled and incompatible with the independent contract);
- link-length regularization;
- compactness; and
- motion smoothness.

It also declares closure, singularity, joint-limit, and a legacy collision penalty.
Those generic placeholders remain disabled; the dedicated
`hand_mechanism_non_collision` component is the active collision safeguard.

## Important modeling decisions still open

1. **Manufacturing coordination after optimization.** The geometry decision is fixed:
   four independent link vectors. Later comparison may identify reusable parts, but
   commonality must remain a postprocessing/manufacturing decision unless explicitly
   introduced as a new outer-level problem.
2. **Human-device registration.** Define which mechanism points attach to each phalanx
   and how device motion maps to MCP/PIP/DIP flexion without forcing joint-axis
   coincidence.
3. **Motion parameterization.** The current decision is that crank angle is the sole
   prescribed input and hand curl is passive. Next decide how joint stiffness, user
   effort, and feedback torque determine the passive multi-joint equilibrium.
4. **Objective aggregation within each finger.** Decide how that finger's reachability,
   clearance, contact travel, and perpendicularity terms are scalarized or represented
   on a Pareto front. Never aggregate losses across fingers.
5. **Feasibility treatment.** Decide which requirements are hard constraints and which
   are differentiable penalties. Closure, assembly, collision, and safe joint limits
   should generally not be traded away for better tracking.

## Recommended implementation sequence

### 1. Freeze the human-device mapping

Extend the existing nominal hand/attachment schema to every finger. Validate the
dorsal MCP mount at node D, manually defined clearance, fixed upper-distal R4,
and the mapping from mechanism motion to phalanges before adding more
objectives.

### 2. Build candidate geometry robustly

The active evaluator now reconstructs candidate geometry with analytic circle
intersections and continuity-based assembly-branch tracking. The next step is to expose
explicit triangle-inequality margins for `ADE`, `CDG`, and `FGH`, distinguish truncated
workspaces from complete sweeps, and report why non-assemblable candidates failed.

### 3. Connect a per-finger kinematic predictor

The current predictor returns mechanism TCP `H` and a passive one-DOF hand-synergy
response from fixed `H–R4` closure. Extend it to solve a calibrated multi-DOF static or
quasi-static hand equilibrium and return:

- mechanism node coordinates;
- predicted human MCP/PIP/DIP angles;
- fingertip position and orientation;
- closure residual and feasible motion extent;
- singularity or Jacobian metrics; and
- attachment misalignment and force-transmission quality.

This is required before using the geometry to claim force-feedback quality or comfort.

### 4. Activate hard feasibility checks first

Before tuning soft objective weights, make candidate rejection reliable for assembly,
loop closure, joint limits, ground/hand packaging, link collision, finger collision,
and a minimum singularity/transmission margin. Add tests containing deliberately invalid
link vectors.

### 5. Activate pose and path objectives

Compare predicted trajectories with the full nominal-to-curled target, not only endpoint
angles. Normalize errors by the measured uncertainty and finger size. Apply explicit
acceptance thresholds separately to every finger candidate.

### 6. Establish the physical load model

Measure or estimate link masses, centers of mass, actuator/cable routing, friction,
return springs, desired fingertip force, and external finger resistance. Then activate
energy, peak torque, transmission, and thermal/duty-cycle objectives. Add the selected
Dynamixel XC330-M288-T torque–speed/current limits and operating duty cycle to this
model. The current gravity-only torque report is insufficient to validate the selected
actuator.

### 7. Compare the four independent designs

After all four candidates pass their own feasibility checks, generate a read-only
comparison of link sizes, objective vectors, packaging envelopes, and potential common
parts. This comparison must not silently feed back into the independent losses.

### 8. Improve optimization only after the evaluator is stable

Central finite differences are appropriate for plumbing tests but will become expensive
and noisy once each evaluation contains a full collision/load sweep.
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

The next milestone should produce four candidate link vectors, each initialized from
the same nominal design and each able to simulate its assigned finger from horizontal
to intended curl, with explicit attachment mapping, zero accepted closure/collision
violations, recorded joint-limit and singularity margins, and pose/path errors. Energy
and torque can remain secondary until this kinematic-feasibility milestone is trustworthy.

## Key paths

| Purpose | Path |
|---|---|
| Nominal mechanism | `designs/mechanism_2/nominal/mechanism.yaml` |
| Nominal design layout | `designs/mechanism_2/nominal/README.md` |
| Nominal dimensioned graph | `designs/mechanism_2/nominal/artifacts/mechanism/abstraction.png` |
| Combined hand/device abstraction | `designs/mechanism_2/nominal/artifacts/combined/combined_abstraction.png` |
| Finger objective manifest | `src/co-optimization/config/objectives.yaml` |
| Embedded nominal analysis targets | `designs/mechanism_2/nominal/mechanism.yaml` |
| Optimization training targets | `src/co-optimization/config/objectives/*.yaml` |
| Active objective derivation | `src/co-optimization/objective_math.md` |
| Link-length variables | `src/co-optimization/config/optimizable_variables.yaml` |
| Adam implementation | `src/co-optimization/run_adam.py` |
| Analysis implementation | `src/analysis/` |
| Combined mechanism–hand analyzer | `src/analysis/combined_analysis.py` |
| Analysis entry point | `run_analysis.sh` |
| Optimization entry point | `run_optimization.sh` |
| Tests | `tests/` |

At handover time, all 19 tests pass. The tests cover topology/schema consistency,
workspace closure for the nominal design, plotting, nominal torque plumbing, exact
alignment between optimizer variable initials and nominal dimensions, and end-to-end
execution through the top-level Adam wrapper. They also enforce four isolated finger
candidate trees, shared nominal initialization, distinct finger mechanism identities,
finger-specific analysis routing, reachability plus whole-workspace collision as the
enabled optimization components, and the nominal hand/attachment and sweep bounds.
