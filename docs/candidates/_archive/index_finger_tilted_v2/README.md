# Index finger, tilted mount — 22.54 mm rod candidate

Mount revision of 2026-09: `L_ad` 54 → **21.112 mm**, sloping **37.054°** downward
from A to D, and `dorsal_clearance_mm` 2.0 → **7.0**. E was held at its previous
physical location, which makes `L_ae` 66 → 28.765 mm.

`runs/` is gitignored, so this folder is the permanent record.

## Constraint status — all three pass

| Constraint | Result | Value |
|---|---|---|
| Collision (hand–mechanism) | PASS | min clearance **+0.197 mm** |
| Rod closure (31 poses) | PASS | max error **0.0590 mm** (tol 0.1) |
| Downward curl | PASS | terminal s = 1.0000, monotone |

Crank travel 0° → 37.892°. Rod-normal deviation **64.90°** — see the caveat below.

## Link lengths (mm)

| Variable | Value | | Variable | Value |
|---|---:|---|---|---:|
| L_ab | 19.248 | | L_dg | 74.485 |
| L_bc | 17.285 | | L_ef | 28.045 |
| L_cd | 13.798 | | L_fg | 37.927 |
| L_ad *(fixed)* | 21.112 | | L_gh | 68.788 |
| L_ae | 34.766 | | L_fh | 54.368 |
| L_de | 20.574 | | **L_tip_rod** | **22.539** |
| L_cg | 62.700 | | | |

Distal-body interior angles: ∠FGH **51.97°**, ∠GFH 94.69°, ∠FHG 33.33°.

## How the rod got from 45.6 mm to 22.5 mm

| Step | Rod | What changed |
|---|---:|---|
| First feasible design | 45.61 mm | lifted the `L_tip_rod` ceiling it was pinned against |
| Angle FGH reduced | 35.90 mm | swung GH down (76° → 61°), bringing H toward the finger |
| Started from a short rod | 22.54 mm | seeded from infeasible short-rod points instead of shrinking a feasible long-rod one |

Two findings drove this:

- **The guidance weights were inverted.** Closure — the binding constraint — carried
  weight 1.0 while collision, already over-satisfied, carried 5.0. Adam spent its
  effort on a solved problem. Rebalancing cut closure error ~3x at identical seeds.
- **A feasible point is a trap.** Candidate ranking prefers "most constraints
  passed", so from a 3/3 design every rod-shortening step is rejected the moment it
  dips to 2/3. The search cannot tunnel through infeasible ground; it has to start
  on the far side. This is why the 35.90 mm design refused to move at all.

Weights that produced this candidate: closure 5.0, collision 20.0, rod length 20.0,
perpendicularity 1.0. Note these differ from the checked-in `objectives.yaml`
(closure 30, collision 1), which produced the 35.90 mm alternative kept here as
`mechanism_rod36_alt.yaml`.

## Caveats

- **Perpendicularity regressed badly**: 44.96° → 48.30° → **64.90°** across the
  three designs. It is the project's original sole design objective, and it was
  diluted to make room for the rod-length term. The rod now pulls well off the
  distal surface normal, so transmission efficiency and cuff slip need a look.
- **Clearance margin is thin** (+0.197 mm) — likely inside manufacturing tolerance.
- Collision is sampled at 31 poses, not continuously certified.
- Mass/torque model is still a placeholder; the actuator is not validated.
- Index finger only.
