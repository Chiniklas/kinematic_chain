# Middle finger, long tilted mount — 35.49 mm rod

The adopted `long_ad` candidate. `runs/` is gitignored, so this folder is the
permanent record.

## The mount, and what "50 mm longer" actually means

The two variants differ by **50 mm horizontally**, not in A-D length. An earlier
revision of this design assumed the A-D member was horizontal and set
`L_ad = 21.112 + 50`; that was wrong and has been corrected.

| | short_ad | long_ad |
|---|---:|---:|
| horizontal component | 16.849 mm | **66.849 mm** (+50) |
| vertical component (A height) | 12.721 mm | 12.721 mm (unchanged) |
| **L_ad** | 21.112 mm | **68.0485 mm** |
| **tilt** | 37.054° | **10.7746°** |

The A-D lengths therefore differ by 46.94 mm, not 50. Both mounts hold A at the same
height, which is what makes them align in CAD.

That 3 mm correction was not cosmetic: the design built on the wrong 71.112 mm frame
dropped from 3/3 to 1/3 when re-evaluated at 68.049 mm (clearance -1.529 mm, closure
5.67 mm). The feasible region here is narrow enough that the frame length has to be
right before anything else is tuned.

## Constraint status — all three pass

| Constraint | Result | Value |
|---|---|---|
| Collision (hand–mechanism) | PASS | min clearance **+0.803 mm** |
| Rod closure (31 poses) | PASS | max error **0.0954 mm** (tol 0.1) |
| Downward curl | PASS | terminal s = 1.0000, monotone |

Crank travel 0° → 54.026°. Maximum rod-normal deviation 54.85°.

## Link lengths (mm)

| Variable | Nodes | Length |
|---|---|---:|
| L_ab | A–B (crank) | 34.531 |
| L_bc | B–C | 54.193 |
| L_cd | C–D | 26.287 |
| L_ad | A–D (**fixed** frame) | 68.049 |
| L_ae | A–E | 84.148 |
| L_de | D–E | 16.821 |
| L_cg | C–G | 55.415 |
| L_dg | D–G | 65.532 |
| L_ef | E–F | 36.549 |
| L_fg | F–G | 26.983 |
| L_gh | G–H | 58.493 |
| L_fh | F–H | 44.523 |
| **L_tip_rod** | H–R4 (output rod) | **35.494** |

Fixed upstream: tilt 10.7746°, dorsal clearance 7.0 mm, middle finger 49/27/27.

## Rigid-body interior angles (deg), at q=0

| Body | Angles |
|---|---|
| ground A–D–E | A 3.69 / **D 161.21** / E 15.10 |
| central C–D–G | C 100.54 / D 56.24 / G 23.23 |
| distal F–G–H | **F 46.64** / G 107.21 / H 26.15 |

**The ground triangle is very flat** — 161.21° at D means A, D and E are nearly
collinear, so the E pivot sits close to the A-D line. Worth checking in CAD whether
that bracket is stiff enough; the optimiser has no structural model and will not
have accounted for it.

## Node coordinates at q=0 (mm, D at origin, tilt applied)

| Node | x | y | | Node | x | y |
|---|---:|---:|---|---|---:|---:|
| A | -66.849 | 12.721 | | E | 16.657 | 2.346 |
| B | -60.393 | 46.643 | | F | 40.380 | 30.150 |
| C | -11.275 | 23.747 | | G | 33.594 | 56.266 |
| D | 0.000 | 0.000 | | H | 84.855 | 28.093 |

## The alternative that was not adopted

`tools/seed_search.py` returned two feasible designs with near-identical numbers but
very different shapes. `alternative_seed_01.npy` holds the other one; restore it with
`tools/materialize.py --design long_ad --finger middle --vector <file> --out <dir>`.

| | adopted (B) | alternative (A) |
|---|---:|---:|
| rod | 35.494 | 35.405 |
| clearance | +0.803 | +0.722 |
| closure | 0.0954 | 0.0882 |
| rod-normal deviation | 54.85° | 56.26° |
| crank travel | 54.03° | 59.42° |
| **angle FGH** | **46.64°** | **90.90°** |
| L_fh | 44.523 | 69.536 |

B was adopted for the more compact distal body (L_fh 25 mm shorter) and the smaller
crank travel. A's near-right-angle distal body may be easier to machine, so it is
kept here rather than discarded.

Their 44° difference in FGH says something about the search space: the feasible
region is not one connected pocket but several separated ones, which is why runs
from different random starts land on such different shapes.

## Worth knowing about how this was found

The best seeds were the ones that *started* furthest from feasible. Ranked by
starting violation, the two that converged to 3/3 scored 22.54 and 32.52 mm, while
those starting at 0.11 mm finished at 1/3. The same pattern appeared in the previous
search. How close a start looks says little about where it converges, so the
violation-ranked ordering in `seed_search.py` is not worth much — random selection
among assembling candidates would likely do as well.

Weights: closure 5.0, collision 20.0, rod length 20.0, perpendicularity 1.0.

## Open items

- Perpendicularity sits at 54.85°, against an ideal of 0°. It is the project's
  original design objective and has been losing ground to rod length; the rod pulls
  well off the distal surface normal, which affects transmission efficiency and cuff
  slip.
- The flat ground triangle noted above.
- Collision is sampled at 31 poses, not continuously certified; the mass and torque
  model remains a placeholder and the actuator is unvalidated.
