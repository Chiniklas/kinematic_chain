# Middle finger, long tilted mount — 31.63 mm rod candidate

The `long_ad` profile after the 2026-09 mount revision: `L_ad` 54 → **71.112 mm**,
sloping **10.305°** downward from A to D, `dorsal_clearance_mm` **7.0**. This is the
`short_ad` mount extended by 50 mm with A held at the same height, so the two
variants align in CAD. Target is the **middle finger** (49/27/27), where `short_ad`
targets the index.

`runs/` is gitignored, so this folder is the permanent record.

## Constraint status — all three pass

| Constraint | Result | Value |
|---|---|---|
| Collision (hand–mechanism) | PASS | min clearance **+0.798 mm** |
| Rod closure (31 poses) | PASS | max error **0.0774 mm** (tol 0.1) |
| Downward curl | PASS | terminal s = 1.0000, monotone |

The clearance margin is four times `short_ad`'s +0.198 mm, so this one has more
room for manufacturing tolerance.

## Link lengths (mm), against the previous 54 mm design

| Variable | 54 mm design | This design | Change |
|---|---:|---:|---:|
| L_ad *(fixed)* | 54.00 | **71.112** | +17.11 |
| L_ab | 31.57 | 35.170 | +3.60 |
| L_bc | 51.99 | 57.085 | +5.09 |
| L_cd | 27.07 | 34.059 | +6.99 |
| L_ae | 64.80 | **83.656** | +18.86 |
| L_de | 15.18 | 17.628 | +2.45 |
| L_cg | 47.88 | 52.690 | +4.81 |
| L_dg | 58.33 | 59.862 | +1.53 |
| L_ef | 28.76 | 35.127 | +6.37 |
| L_fg | 26.68 | **18.000** | -8.68 |
| L_gh | 51.08 | **65.903** | +14.82 |
| L_fh | 55.87 | 59.536 | +3.67 |
| L_tip_rod | 29.30 | 31.634 | +2.33 |

`L_ae` grows almost exactly as much as `L_ad` — the ground triangle scales up with
the frame, E travelling back with A.

`L_fg` shortens while `L_gh` lengthens. That is the same move that shortened the
`short_ad` rod: pulling the FGH apex angle in swings H down toward the finger, and
extending GH throws it back out along the new direction. The optimiser found it
independently here.

## Rigid-body interior angles (deg), at q=0

| Body | Vertices |
|---|---|
| ground A-D-E | A 9.21, D 130.57, E 40.22 |
| central C-D-G | C 84.36, D 61.16, G 34.49 |
| distal F-G-H | F 102.79, **G 61.76**, H 15.45 |

The distal body is obtuse at F, with only 15.45° at H — a long thin triangle,
noticeably different in shape from the earlier designs.

## Node coordinates at q=0 (mm, D at origin, tilt applied)

| Node | x | y | | Node | x | y |
|---|---:|---:|---|---|---:|---:|
| A | -69.965 | 12.721 | | E | 13.676 | 11.123 |
| B | -63.674 | 47.323 | | F | 46.666 | 23.188 |
| C | -8.414 | 33.003 | | G | 43.675 | 40.938 |
| D | 0.000 | 0.000 | | H | 106.108 | 19.833 |

## How it was found

The 54 mm design's proportions assemble on the 71 mm frame (25 of 91 poses) but
their H path is nowhere near parallel to the middle finger's R4 path, so no fixed
rod length closes it — three rounds of angle and weight sweeps all stalled at 1/3
with about 5 mm of closure error.

`tools/seed_search.py` then sampled the bounded space, screened for assembly and
palm clearance, ranked survivors with the real evaluator and optimised the best
six. One reached 3/3. Notably it was the seed with the *worst* starting violation
(26.95 mm); the seeds that started closest to feasible (1.20, 1.33 mm) finished at
1/3 and 2/3. How near a start looks says little about where it converges.

Weights: closure 5.0, collision 20.0, rod length 20.0, perpendicularity 1.0.

## Open items

- **`L_fg` is pinned at its 18 mm lower bound**, which usually means the optimiser
  wants to go further. Relaxing that bound is a specific, evidence-backed next step
  rather than a guess.
- Seed 4 of the same search reached 2/3 with a **10.64 mm** rod (clearance +2.466,
  closure 0.0805) and failed only the curl constraint — worth pursuing if a short
  rod matters more than the extra margin here.
- Perpendicularity is not reported above because it is no longer the binding
  concern, but it remains the project's original design objective and has been
  degrading as rod length was prioritised.
- Collision is sampled at 31 poses, not continuously certified; the mass and torque
  model is still a placeholder.
