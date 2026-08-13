# Mechanism 2 source abstraction

Source of truth: `mechanism.yaml` (schema version 1).

## Validation summary

- mechanism: mechanism_2
- nodes: 8 (7 revolute + 1 reference)
- bodies including ground: 6
- equivalent lower pairs: 7
- independent loops: 2
- planar Gruebler mobility: 1
- warning: mechanism status is co_optimization_candidate
- warning: mass model status is nominal_placeholder; torque results are nominal

## Photo calibration

| Field | Value |
|---|---|
| image | sources/real_thing.jpg |
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
| d | D | revolute | ADE-to-CDG revolute connection | yes | high |
| e | E | revolute | lower coupler ground pivot | yes | high |
| f | F | revolute | lower coupler / distal-body joint | no | high |
| g | G | revolute | CDG-to-FGH revolute connection | no | high |
| h | H | reference | mechanism output TCP | no | high |

## Bodies and members

| Body | Kind | Nodes | Role | Confidence |
|---|---|---|---|---|
| ground | ground | a, d, e | fixed ADE frame | high |
| input_crank | binary_link | a, b | actuated crank | high |
| loop_1_coupler | binary_link | b, c | first-stage coupler | high |
| central_body | rigid_body | c, d, g | rigid CDG body | high |
| link_ef | binary_link | e, f | lower distal coupler | high |
| distal_body | rigid_body | f, g, h | rigid FGH distal output body | high |

## Joint incidence

| Node | Joint type | Incident bodies | Equivalent lower pairs |
|---|---|---|---|
| a | revolute | ground, input_crank | 1 |
| b | revolute | input_crank, loop_1_coupler | 1 |
| c | revolute | loop_1_coupler, central_body | 1 |
| d | revolute | ground, central_body | 1 |
| e | revolute | ground, link_ef | 1 |
| f | revolute | link_ef, distal_body | 1 |
| g | revolute | central_body, distal_body | 1 |

## Dimensions

| Dimension | Body | Nodes | Value | Units | Source | Uncertainty |
|---|---|---|---|---|---|---|
| L_ab | input_crank | a, b | 41.21262052403357 | mm | co_optimization_candidate | — |
| L_bc | loop_1_coupler | b, c | 55.49547660333985 | mm | co_optimization_candidate | — |
| L_cd | central_body | c, d | 18.01317331092241 | mm | co_optimization_candidate | — |
| L_ad | ground | a, d | 53.086947856021716 | mm | co_optimization_candidate | — |
| L_ae | ground | a, e | 68.0193254685869 | mm | co_optimization_candidate | — |
| L_de | ground | d, e | 16.600139225279275 | mm | co_optimization_candidate | — |
| L_cg | central_body | c, g | 51.377253887887335 | mm | co_optimization_candidate | — |
| L_dg | central_body | d, g | 54.98622848521524 | mm | co_optimization_candidate | — |
| L_ef | link_ef | e, f | 29.719502246514605 | mm | co_optimization_candidate | — |
| L_fg | distal_body | f, g | 30.243037220089587 | mm | co_optimization_candidate | — |
| L_gh | distal_body | g, h | 52.24451640290604 | mm | co_optimization_candidate | — |
| L_fh | distal_body | f, h | 55.71330726166383 | mm | co_optimization_candidate | — |

## Loops

| Loop | Node cycle | Body cycle |
|---|---|---|
| loop_1 | a, b, c, d | input_crank, loop_1_coupler, central_body, ground |
| loop_2 | d, g, f, e | central_body, distal_body, link_ef, ground |

## Nominal human hand model

Reference: `index`; pose: `mildly_curled`.

| Node | Index | Kind | Position [mm] |
|---|---|---|---|
| hand_wrist_dorsal | — | palm_reference | -90.0, 0.0 |
| hand_mcp | 1 | anatomical_revolute | 0.0, 0.0 |
| hand_pip | 2 | anatomical_revolute | 39.993908, -0.698096 |
| hand_dip | 3 | anatomical_revolute | 64.959646, -2.006495 |
| hand_distal_slot_midpoint | RP4 | attachment_slot_reference | 76.540523, -13.057889 |
| hand_tip | — | terminal_reference | 89.864514, -4.185389 |

| Segment | Joints | Length [mm] | Width [mm] | Shape |
|---|---|---|---|---|
| palm | hand_wrist_dorsal, hand_mcp | 90.0 | 25.0 | rounded_rectangle |
| proximal_phalanx | hand_mcp, hand_pip | 40.0 | 20.0 | rounded_rectangle |
| middle_phalanx | hand_pip, hand_dip | 25.0 | 15.0 | rounded_rectangle |
| distal_phalanx | hand_dip, hand_tip | 25.0 | 10.0 | rounded_rectangle |

## Exoskeleton attachments

| Attachment | Mechanism | Hand reference | Hand interface | Connector | Dorsal clearance [mm] |
|---|---|---|---|---|---|
| dorsal_input_mount | d | hand_mcp | fixed_dorsal_mount | upper_surface_of_first_joint | 1.0 |
| distal_output_rod | h | hand_distal_slot_midpoint | revolute_prismatic_pin_in_slot | binary_rod | — |

## Model readiness

| Analysis | Status |
|---|---|
| topology_plot | ready |
| combined_hand_abstraction | ready_assumed_nominal_index_attachment |
| numeric_link_table | ready_remeasured_dimensions |
| kinematic_sweep | ready_remeasured_geometry |
| optimization | adam_skeleton_waiting_for_hand_coupled_kinematics |
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
| central_body | 8.0 | c: 0.333333, d: 0.333334, g: 0.333333 |
| link_ef | 1.0 | e: 0.5, f: 0.5 |
| distal_body | 6.0 | f: 0.333333, g: 0.333334, h: 0.333333 |

## Nominal point masses

| Mass | Node | Mass [g] |
|---|---|---|
| joint_b | b | 3.0 |
| joint_c | c | 3.0 |
| joint_f | f | 3.0 |
| joint_g | g | 3.0 |
| output_payload | h | 0.0 |

Mass-model status: `nominal_placeholder`.
