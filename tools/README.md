# Analysis tools

Diagnostics and search helpers that sit beside `run_optimization.sh`. The shell
entry points run the standard pipeline; these answer the questions that come up
when it does not converge — where a design collides, which loop fails to assemble,
whether a mount revision even has a feasible region.

All of them take `--design` (a profile directory under
`src/co-optimization/config/`) and `--finger`. Run them from the repository root
with the `kinematic-chain` environment active.

## `design_probe.py` — what does this design look like, and why does it fail?

```bash
python3 tools/design_probe.py --design short_ad --finger index
```

Link lengths with their bounds (flagging any pinned at a limit, which usually means
the optimiser wants to go further), rigid-body interior angles, node coordinates at
q=0, how much of the crank sweep assembles, and the current constraint status. When
the sweep breaks it names the loop and prints the triangle inequality that failed —
for example `|EG|=66.00 but EF+FG=65.97`, which says the E-F and F-G links are
0.03 mm too short to keep reaching as the crank turns.

Run this first. It is fast and it usually explains the failure outright.

## `collision_report.py` — which part hits which, and when?

```bash
python3 tools/collision_report.py --design short_ad --finger index --top 15
```

`evaluate()` reports only the worst signed clearance. This walks the same primitive
pairs the optimiser uses but keeps the labels, so you learn that the worst contact
is, say, the output rod grazing the distal phalanx at q=0 rather than a link
brushing the palm.

A clearance of exactly `-(link radius + hand-part radius)` — such as `-12.000` for
the 2 mm links against the 20 mm-wide proximal phalanx — means the phalanx passes
clean through a rigid body. That is a gross failure, not a near miss, and it is
worth distinguishing before spending an optimisation run on it.

`--vector path.npy` inspects a saved design vector instead of the profile's nominal.

## `seed_search.py` — find a starting point on a changed mount

```bash
python3 tools/seed_search.py --design long_ad --finger middle
```

Samples the bounded space, rejects anything that will not assemble or that pushes
into the palm, scores the survivors with the real evaluator, then optimises the
best few and saves every result to `--out`.

Adam follows the slope under its feet and never crosses a ridge, so when the mount
changes the shipped lengths often sit in a basin containing no feasible design.
That is what this is for.

Two things it does deliberately:

- **Ranks with `evaluate()`, not a geometric proxy.** An earlier version scored
  "how near does H's path pass to each R4 sample". That lets R4's first pose pair
  with H's fortieth, while closure pairs them in step — it selected seeds scoring
  1.2 mm that carried 38 mm of real closure error.
- **Saves every result, feasible or not.** A 2/3 candidate missing by half a
  millimetre is the best starting point for the next round, and one was lost early
  on to a `if passes == 3` guard around the save.

Weights are exposed as `--w-closure`, `--w-collision`, `--w-rod`, because the
binding constraint moves as a design improves and the weights have to follow.

## `materialize.py` — vector to candidate, and to a GIF

```bash
python3 tools/materialize.py --design short_ad --finger index \
    --vector runs/seed_search/seed_01.npy --out runs/my_candidate --animate
```

Writes a real candidate directory with a self-contained `mechanism.yaml`, which
`run_analysis.sh` and `src/animate_design.py` both accept. Pass `--animate` for the
GIF — note that `run_optimization.sh` runs the analysis but not the animation, so
its output directories contain no GIF unless you render one.

## `stack_gifs.py` — compare two designs side by side

```bash
python3 tools/stack_gifs.py --out runs/compare.gif \
    --gif runs/a/design_animation.gif --label "short_ad" \
    --gif runs/b/design_animation.gif --label "long_ad"
```

Pairs frames by index and stacks them vertically. Inputs are centred rather than
scaled, so both mechanisms keep the same millimetre scale on screen and the
comparison is not visually misleading.

## Two behaviours worth knowing before you tune anything

**A feasible design is a trap.** Candidate ranking prefers "most constraints
passed", so from a 3/3 design every rod-shortening step is discarded the moment it
dips to 2/3. The search cannot tunnel through infeasible ground. Shortening a rod
worked by seeding from an infeasible *short* rod and letting the closure guidance
pull it back, not by squeezing a feasible long one.

**Whatever is not in the loss gets sacrificed.** Rod length sat outside the
objective entirely, so the optimiser let it grow to 45 mm without ever considering
it; adding `output_rod_length` as a weighted objective is what made it negotiable.
The same still applies to actuation energy, peak torque and compactness, which
remain disabled placeholders in `objectives.yaml`.
