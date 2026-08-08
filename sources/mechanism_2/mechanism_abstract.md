# Mechanism 2 source abstraction

Source of truth: `mechanism.yaml` (schema version 1).

## Validation summary

- mechanism: mechanism_2
- nodes: 8 (7 revolute + 1 reference)
- bodies including ground: 6
- equivalent lower pairs: 7
- independent loops: 2
- planar Gruebler mobility: 1
- warning: 12 dimensions are nominal photo estimates, not measured values
- warning: mechanism status is nominal_photo_geometry
- warning: mass model status is nominal_placeholder; torque results are nominal

## Photo calibration

| Field | Value |
|---|---|
| image | 1224a3cf4f13d2e78e428296289e2e0c.jpg |
| image_orientation | exif_autorotated_landscape |
| image_size_pixels | 4032, 3024 |
| pixels_per_mm | 24.3 |
| relative_uncertainty | 0.08 |
| method | XL330 case and mounting geometry visible in the photograph |
| reference_component | DYNAMIXEL XL330-M288-T |
| reference_dimensions_mm | 20.0, 34.0, 26.0 |
| reference_url | https://emanual.robotis.com/docs/en/dxl/x/xl330-m288/ |

## Nodes

| Node | Source | Kind | Role | Fixed | Confidence |
|---|---|---|---|---|---|
| a | A | revolute | actuated base pivot | yes | high |
| b | B | revolute | input crank endpoint | no | high |
| c | C | revolute | loop 1 / loop 2 junction | no | high |
| d | D | revolute | ADE-to-CDF revolute connection | yes | high |
| e | E | revolute | lower coupler ground pivot | yes | high |
| f | F/G | revolute | CDF-to-FHI revolute connection; merged source F/G point | no | high |
| h | H | revolute | lower coupler / distal-body joint | no | high |
| i | I/T | reference | fingertip/output point | no | high |

## Bodies and members

| Body | Kind | Nodes | Role | Confidence |
|---|---|---|---|---|
| ground | ground | a, d, e | fixed ADE frame | high |
| input_crank | binary_link | a, b | actuated crank | high |
| loop_1_coupler | binary_link | b, c | first-stage coupler | high |
| central_body | rigid_body | c, d, f | rigid CDF body | high |
| link_eh | binary_link | e, h | lower distal coupler | high |
| distal_body | rigid_body | f, h, i | rigid FHI distal output body after merging source F/G | high |

## Joint incidence

| Node | Joint type | Incident bodies | Equivalent lower pairs |
|---|---|---|---|
| a | revolute | ground, input_crank | 1 |
| b | revolute | input_crank, loop_1_coupler | 1 |
| c | revolute | loop_1_coupler, central_body | 1 |
| d | revolute | ground, central_body | 1 |
| e | revolute | ground, link_eh | 1 |
| f | revolute | central_body, distal_body | 1 |
| h | revolute | link_eh, distal_body | 1 |

## Dimensions

| Dimension | Body | Nodes | Value | Units | Source | Uncertainty |
|---|---|---|---|---|---|---|
| L_ab | input_crank | a, b | 24.3 | mm | photo_nominal | 1.9 |
| L_bc | loop_1_coupler | b, c | 42.0 | mm | photo_nominal | 3.4 |
| L_cd | central_body | c, d | 20.3 | mm | photo_nominal | 1.6 |
| L_ad | ground | a, d | 39.5 | mm | photo_nominal | 3.2 |
| L_ae | ground | a, e | 49.1 | mm | photo_nominal | 3.9 |
| L_de | ground | d, e | 10.8 | mm | photo_nominal | 1.0 |
| L_cf | central_body | c, f | 35.8 | mm | photo_nominal | 2.9 |
| L_df | central_body | d, f | 41.7 | mm | photo_nominal | 3.3 |
| L_eh | link_eh | e, h | 22.4 | mm | photo_nominal | 1.8 |
| L_fh | distal_body | f, h | 20.2 | mm | photo_nominal | 1.6 |
| L_fi | distal_body | f, i | 34.3 | mm | photo_nominal | 2.7 |
| L_hi | distal_body | h, i | 37.9 | mm | photo_nominal | 3.0 |

## Loops

| Loop | Node cycle | Body cycle |
|---|---|---|
| loop_1 | a, b, c, d | input_crank, loop_1_coupler, central_body, ground |
| loop_2 | d, f, h, e | central_body, distal_body, link_eh, ground |

## Model readiness

| Analysis | Status |
|---|---|
| topology_plot | ready |
| numeric_link_table | ready_nominal_photo_estimates |
| kinematic_sweep | ready_nominal_photo_geometry |
| optimization | placeholder_waiting_for_validated_kinematics_and_objective |
| input_torque | ready_nominal_placeholder_mass_model |

## Workspace analysis settings

| Setting | Value |
|---|---|
| q_min_deg | 0.0 |
| q_max_deg | 90.0 |
| steps | 181 |
| solver_tolerance_mm | 1e-05 |
| max_iterations | 100 |

## Nominal body mass model

| Body | Mass [g] | Centre node weights |
|---|---|---|
| input_crank | 5.0 | a: 0.5, b: 0.5 |
| loop_1_coupler | 4.0 | b: 0.5, c: 0.5 |
| central_body | 8.0 | c: 0.333333, d: 0.333334, f: 0.333333 |
| link_eh | 1.0 | e: 0.5, h: 0.5 |
| distal_body | 6.0 | f: 0.333333, h: 0.333334, i: 0.333333 |

## Nominal point masses

| Mass | Node | Mass [g] |
|---|---|---|
| joint_b | b | 3.0 |
| joint_c | c | 3.0 |
| joint_f | f | 3.0 |
| joint_h | h | 3.0 |
| output_payload | i | 0.0 |

Mass-model status: `nominal_placeholder`.
