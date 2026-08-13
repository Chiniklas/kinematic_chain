# Independent per-finger co-optimization

Run the initial multi-objective Adam scaffold through the canonical top-level entry:

```bash
./run_optimization.sh
```

The wrapper forwards all options to `src/co-optimization/run_adam.py`.

Use `--finger index` (or another long finger) to run only one independent job during
development; omit it to run all four.

It reads `config/objectives.yaml`, the four finger target YAMLs, and
`config/optimizable_variables.yaml`. It creates four independent optimization
problems. Each receives a fresh copy of the same nominal 12-value initialization;
there is no cross-finger loss, shared candidate vector, or shared Adam state.

The default output contract is:

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
                └── combined/fingers/<that-finger>/
```

The algorithm remains Adam internally, but algorithm names are not part of the output
path. Each `mechanism.yaml` has a distinct ID such as `mechanism_2_index`, materializes
that finger's hand dimensions and analysis target, and records the shared nominal
parent. It is therefore independently analyzable without reading optimization
configuration. Its nested
analysis contains only that candidate's target finger plus the mechanism-only reports.
The combined report replays exactly the optimizer's crank-driven intended workspace,
including the recorded passive hand response from `q=0` to that candidate's terminal
`q*`; it no longer imposes a linear hand/crank ratio.
Future retained candidates must follow the same numbered-directory contract.

Each finger's TensorBoard stream records `loss/total`, that finger's component losses, every
numeric component metric, gradient and step norms, individual finite-difference
gradients, and all 12 optimizable link-length trajectories and bounds. Launch it with:

```bash
tensorboard --logdir runs/co-optimization_<UTC timestamp>/fingers
```

The event writer has no training-time dependency on TensorFlow or PyTorch.
`environment.yml` includes TensorBoard for reading and visualizing the logs.

For each finger, the variables are eleven mechanism link lengths plus the external
`L_tip_rod` length from TCP `H` to fixed revolute `R4` at the upper distal midpoint.
Every sampled pose, including initial and terminal poses, must close within `0.1 mm`
or the candidate is rejected. All 12 local variables can influence closure and rod
angle. `L_ad` remains fixed at the nominal 54 mm because it is the horizontal grounded
mounting baseline.

Crank angle is the sole prescribed input. The initial ideal compliant-hand model has
one passive coordinate on the measured curl path. Closure solves that coordinate at
each increasing crank sample, while a hard constraint enforces `0 <= s <= 1`, monotone
downward curl, no reversal, and arrival at maximum intended curl. There is no fixed
hand/crank ratio. A future stiffness/equilibrium model is still required to predict
force feedback.

The exact ideal-interface formulation is documented in
[`objective_math.md`](objective_math.md). `H–R4` perpendicularity is the sole design
objective. Full-workspace fixed-contact closure and sampled non-collision are hard
constraints with smooth Adam guidance terms. Between-sample continuous collision
certification and a multi-DOF stiffness/equilibrium hand model remain future work.
