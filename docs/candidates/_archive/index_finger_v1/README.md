# Index finger candidate v1 (hard-constraint-feasible)

Source run: `runs/co-optimization_20260823T141647Z/fingers/index/candidate_0001/`
(not tracked in git; `runs/` is gitignored — this folder is the permanent record).

Optimizer settings: `--finger index --iterations 300 --learning-rate 0.05`,
after raising `dorsal_clearance_mm` from `1.0` to `2.0` in the nominal design to
resolve a fixed 1 mm penetration between the input crank's ground pivot (node A)
and the palm capsule.

## Constraint status

| Constraint | Result | Value |
|---|---|---|
| Collision (hand–mechanism) | PASS | minimum clearance 0.000 mm |
| Rod closure (H–R4, all 31 poses) | PASS | max error 0.0977 mm (tolerance 0.1 mm) |
| Downward-curl (monotone, no reversal) | PASS | — |

Both collision and closure pass right at their tolerance — no safety margin.
Treat this as a proof-of-feasibility candidate, not a final manufacturing spec.

## Link lengths (mm)

| Variable | Nominal initial | Optimized | Change |
|---|---:|---:|---:|
| L_ab | 31.00 | 31.57 | +0.57 |
| L_bc | 54.00 | 51.99 | -2.01 |
| L_cd | 28.00 | 27.07 | -0.93 |
| L_ad *(fixed)* | 54.00 | 54.00 | 0 |
| L_ae | 66.00 | 64.80 | -1.20 |
| L_de | 14.00 | 15.18 | +1.18 |
| L_cg | 50.00 | 47.88 | -2.12 |
| L_dg | 57.00 | 58.33 | +1.33 |
| L_ef | 30.00 | 28.76 | -1.24 |
| L_fg | 28.00 | 26.68 | -1.32 |
| L_gh | 50.00 | 51.08 | +1.08 |
| L_fh | 57.00 | 55.87 | -1.13 |
| L_tip_rod (H–R4 output rod) | 28.00 | **29.30** | +1.30 |

Fixed upstream parameters (not optimized): `dorsal_clearance_mm = 2.0`,
`distal_phalanx_width_mm = 10.0`.

## Node positions at q=0° (straight pose, mm, origin at ground node D)

| Node | x | y |
|---|---:|---:|
| A | -54.000 | 0.000 |
| B | -54.000 | 31.574 |
| C | -2.210 | 26.978 |
| D | 0.000 | 0.000 |
| E | 9.746 | 11.643 |
| F | 38.502 | 11.775 |
| G | 44.436 | 37.781 |
| H | 93.274 | 22.806 |

## Rigid-body interior angles at q=0° (deg)

Each rigid body's shape is fully determined by its own side lengths (SSS); these
angles change as the mechanism moves and are reported here only for the q=0°
reference pose.

| Body | Corner | Angle |
|---|---|---:|
| ground (A-D-E) | at A (∠DAE) | 10.35 |
| ground (A-D-E) | at D (∠ADE) | 129.93 |
| ground (A-D-E) | at E (∠AED) | 39.72 |
| central_body (C-D-G) | at C (∠DCG) | 98.36 |
| central_body (C-D-G) | at D (∠CDG) | 54.31 |
| central_body (C-D-G) | at G (∠CGD) | 27.33 |
| distal_body (F-G-H) | at F (∠GFH) | 65.76 |
| distal_body (F-G-H) | at G (∠FGH) | 85.81 |
| distal_body (F-G-H) | at H (∠FHG) | 28.43 |

## Motion summary

```
crank range: 0.000° -> 47.794°
hand curl:   0.0000 -> 1.0000 (monotone)
output rod:  29.299 mm
max rod-closure error: 0.0977 mm
```

## Diagrams

`abstraction.png` — mechanism-only topology diagram.

`combined_workspace_report.png` — full workspace overlay (hand + mechanism +
rod across representative poses) with closure/perpendicularity panels.

`design_animation.gif` — animated motion from horizontal extension to maximum
curl (`python3 src/animate_design.py <candidate dir> --finger index`).

`mechanism.yaml` — the self-contained candidate design file (copied from the
run's `candidate_0001/mechanism.yaml`).

## Known limitations (carried over from the project-wide caveats)

- Collision is sampled at 31 poses, not continuously certified between samples.
- Torque/mass model is a nominal placeholder; not validated for the XC330-M288-T actuator.
- No safety margin on collision or closure — both pass exactly at their tolerance.
- This is the `index` finger only; `middle`/`ring`/`little` have not been optimized
  with this `dorsal_clearance_mm=2.0` setting yet.
