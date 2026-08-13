from __future__ import annotations

import math
import subprocess
import tempfile
import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
ENTRY_POINT = ROOT / "run_optimization.sh"
VARIABLES = ROOT / "src" / "co-optimization" / "config" / "optimizable_variables.yaml"
OBJECTIVES = ROOT / "src" / "co-optimization" / "config" / "objectives.yaml"
NOMINAL = ROOT / "designs" / "mechanism_2" / "nominal" / "mechanism.yaml"


class CoOptimizationSkeletonTests(unittest.TestCase):
    def test_perpendicularity_is_the_only_design_objective(self) -> None:
        config = yaml.safe_load(OBJECTIVES.read_text(encoding="utf-8"))
        self.assertEqual(config["optimization_scope"], {
            "problem_unit": "one_independent_mechanism_per_finger",
            "initialization": "shared_nominal_mechanism_2",
            "design_variable_sharing": "none",
            "aggregate_reports_are_postprocessing_only": True,
        })
        self.assertEqual(
            [row["finger"] for row in config["finger_problems"]],
            ["index", "middle", "ring", "little"],
        )
        components = config["loss_components"]
        self.assertEqual(
            [name for name, row in components.items() if row["enabled"]],
            [
                "task_space_reachability",
                "hand_mechanism_non_collision",
                "output_link_perpendicularity",
            ],
        )
        collision = components["hand_mechanism_non_collision"]
        self.assertEqual(collision["passive_hand_progress_bounds"], [0.0, 1.0])
        self.assertTrue(collision["require_all_evaluation_poses"])
        self.assertGreaterEqual(collision["evaluation_poses"], 31)
        self.assertEqual(
            components["output_link_perpendicularity"]["evaluation_poses"],
            collision["evaluation_poses"],
        )
        self.assertEqual(components["task_space_reachability"]["guidance_weight"], 1.0)
        self.assertEqual(collision["guidance_weight"], 5.0)
        self.assertEqual(components["output_link_perpendicularity"]["weight"], 1.0)
        self.assertEqual(
            components["output_link_perpendicularity"]["optimization_role"],
            "design_objective",
        )
        self.assertTrue(all(
            not row["enabled"] for row in config["placeholder_objectives"].values()
        ))
        constraints = config["constraint_penalties"]
        self.assertTrue(constraints["closure_error"]["enabled"])
        self.assertEqual(
            constraints["closure_error"]["mode"],
            "hard_rejection_with_reachability_guidance",
        )
        self.assertEqual(constraints["closure_error"]["tolerance_mm"], 0.1)
        self.assertEqual(
            constraints["closure_error"]["evaluation_poses"],
            collision["evaluation_poses"],
        )
        self.assertTrue(constraints["collision_penalty"]["enabled"])
        hand_motion = constraints["hand_motion_admissibility"]
        self.assertTrue(hand_motion["enabled"])
        self.assertEqual(hand_motion["mode"], "hard_kinematic_constraint")
        self.assertEqual(hand_motion["prescribed_input"], "mechanism_crank_angle_q")
        self.assertEqual(hand_motion["passive_output"], "hand_curl_progress_s")
        self.assertTrue(hand_motion["monotonic_non_decreasing"])
        self.assertTrue(hand_motion["prohibit_dorsal_flexion"])
        self.assertTrue(all(
            not row["enabled"] for name, row in constraints.items()
            if name not in {
                "hand_motion_admissibility", "closure_error", "collision_penalty",
            }
        ))
        self.assertEqual(components["output_link_perpendicularity"]["objective_order"], 1)
        self.assertEqual(
            components["hand_mechanism_non_collision"]["hard_rejection"],
            "any_unintended_signed_distance_below_zero",
        )
        self.assertNotIn("distal_contact_point_travel", components)
        self.assertEqual(
            components["output_link_perpendicularity"]["target"],
            "perpendicular_to_distal_surface",
        )

    def test_variables_match_nominal_mechanism_and_attachment_lengths(self) -> None:
        variable_data = yaml.safe_load(VARIABLES.read_text(encoding="utf-8"))
        nominal_data = yaml.safe_load(NOMINAL.read_text(encoding="utf-8"))
        configured = {row["id"]: row["initial"] for row in variable_data["variables"]}
        dimensions = {row["id"]: row["value"] for row in nominal_data["dimensions"]}
        fixed_ids = variable_data["model"]["fixed_mechanism_dimensions"]
        self.assertEqual(
            set(variable_data["model"]["mechanism_dimension_variables"]),
            set(dimensions) - set(fixed_ids),
        )
        self.assertNotIn("L_ad", configured)
        self.assertEqual(fixed_ids, ["L_ad"])
        self.assertEqual(dimensions["L_ad"], 54)
        self.assertFalse(next(
            row for row in nominal_data["dimensions"] if row["id"] == "L_ad"
        )["optimizable"])
        for variable_id in set(dimensions) - set(fixed_ids):
            self.assertEqual(configured[variable_id], dimensions[variable_id])
        attachments = {
            row["id"]: row for row in nominal_data["exoskeleton_attachments"]
        }
        self.assertEqual(
            configured["L_tip_rod"],
            attachments["distal_output_rod"]["assumed_length_mm"],
        )

    def test_adam_entry_point_builds_four_independent_finger_designs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="kinematic-chain-adam-") as directory:
            output_dir = Path(directory)
            completed = subprocess.run(
                [
                    str(ENTRY_POINT),
                    "--iterations", "3",
                    "--output-dir", str(output_dir),
                ],
                cwd=ROOT,
                text=True,
                capture_output=True,
                check=True,
            )
            self.assertEqual(completed.stdout.count("independent Adam problem"), 4)
            self.assertEqual(completed.stdout.count("Analyzing materialized"), 4)
            self.assertEqual(completed.stdout.count("starting Adam: 3 iterations"), 4)
            self.assertEqual(completed.stdout.count("iteration    3/3"), 4)
            initial_variables = {
                row["id"]: row["initial"]
                for row in yaml.safe_load(
                    VARIABLES.read_text(encoding="utf-8")
                )["variables"]
            }
            manifest = yaml.safe_load(
                (output_dir / "run_manifest.yaml").read_text(encoding="utf-8")
            )
            self.assertEqual(manifest["optimization_scope"], "independent_per_finger")
            self.assertEqual(manifest["design_variable_sharing"], "none")
            for finger in ("index", "middle", "ring", "little"):
                finger_dir = output_dir / "fingers" / finger
                candidate_dir = finger_dir / "candidate_0001"
                candidate = yaml.safe_load(
                    (candidate_dir / "candidate.yaml").read_text(encoding="utf-8")
                )
                self.assertEqual(candidate["target_finger"], finger)
                self.assertEqual(candidate["optimization_scope"], "independent_per_finger")
                self.assertEqual(candidate["scalarization"], {
                    "method": "normalized_weighted_sum",
                    "weights": {
                        "output_link_perpendicularity": 1.0,
                    },
                    "constraint_guidance_weights": {
                        "fixed_contact_rod_closure": 1.0,
                        "hand_mechanism_clearance": 5.0,
                    },
                })
                self.assertFalse(
                    candidate["initialization"]["design_variables_shared_with_other_fingers"]
                )
                optimizer = candidate["optimizer"]
                self.assertEqual(optimizer["name"], "torch.optim.Adam")
                self.assertIn(optimizer["device"], {"cpu", "cuda"})
                self.assertEqual(optimizer["geometry_evaluator_device"], "cpu")
                self.assertTrue(math.isfinite(optimizer["final_total_loss"]))
                self.assertEqual(
                    {
                        name: row["value"]
                        for name, row in candidate["initial_candidate_variables"].items()
                    },
                    initial_variables,
                )
                self.assertEqual(
                    list(candidate["objective_losses"]),
                    [f"{finger}_finger_design"],
                )
                variables = candidate["candidate_variables"]
                self.assertEqual(len(variables), 12)
                self.assertEqual(set(variables), set(initial_variables))
                self.assertNotIn("L_ad", variables)
                self.assertEqual(candidate["fixed_upstream_parameters"]["L_ad_mm"], 54.0)
                self.assertTrue(all(row["units"] == "mm" for row in variables.values()))
                materialized = yaml.safe_load(
                    (candidate_dir / "mechanism.yaml").read_text(encoding="utf-8")
                )
                materialized_ad = next(
                    row for row in materialized["dimensions"] if row["id"] == "L_ad"
                )
                self.assertEqual(materialized_ad["value"], 54.0)
                self.assertEqual(materialized_ad["value_source"], "fixed_nominal_design")
                components = candidate["component_losses"][
                    f"{finger}_finger_design"
                ]
                for name in (
                    "horizontal_fixed_contact_rod_error_mm",
                    "curled_fixed_contact_rod_error_mm",
                    "horizontal_r4_x_mm",
                    "horizontal_r4_y_mm",
                    "curled_r4_x_mm",
                    "curled_r4_y_mm",
                    "hand_mechanism_non_collision",
                    "minimum_signed_clearance_mm",
                    "collision_free_pose_fraction",
                    "whole_intended_workspace_collision_free",
                    "output_link_perpendicularity",
                    "maximum_perpendicular_deviation_deg",
                    "maximum_workspace_rod_closure_error_mm",
                    "rod_closure_within_tolerance_fraction",
                    "whole_intended_workspace_rod_closure_feasible",
                    "hand_motion_admissible",
                    "hand_motion_monotonic_non_decreasing",
                    "terminal_hand_curl_progress",
                ):
                    self.assertIn(name, components)
                weighted_components = sum(
                    components[name] for name in (
                        "weighted_closure_constraint_guidance",
                        "weighted_collision_constraint_guidance",
                        "weighted_output_link_perpendicularity",
                    )
                )
                self.assertAlmostEqual(
                    candidate["objective_losses"][f"{finger}_finger_design"],
                    weighted_components,
                )
                self.assertEqual(
                    components["collision_evaluated_pose_count"],
                    components["collision_requested_pose_count"],
                )
                self.assertEqual(
                    candidate["constraint_acceptance"]["collision_free"],
                    bool(components["whole_intended_workspace_collision_free"]),
                )
                if candidate["constraint_acceptance"]["collision_free"]:
                    self.assertGreaterEqual(
                        candidate["constraint_acceptance"]["minimum_signed_clearance_mm"],
                        0.0,
                    )
                else:
                    self.assertTrue(candidate["status"].startswith("rejected_"))
                self.assertEqual(
                    candidate["constraint_acceptance"]["rod_closure_feasible"],
                    bool(components[
                        "whole_intended_workspace_rod_closure_feasible"
                    ]),
                )
                self.assertEqual(
                    candidate["constraint_acceptance"]["rod_closure_tolerance_mm"],
                    0.1,
                )
                if candidate["constraint_acceptance"]["rod_closure_feasible"]:
                    self.assertLessEqual(
                        candidate["constraint_acceptance"][
                            "maximum_rod_closure_error_mm"
                        ],
                        candidate["constraint_acceptance"][
                            "rod_closure_tolerance_mm"
                        ],
                    )
                else:
                    self.assertTrue(candidate["status"].startswith("rejected_"))
                self.assertEqual(
                    candidate["constraint_acceptance"]["hand_motion_feasible"],
                    bool(components["hand_motion_admissible"]),
                )
                motion = candidate["motion_solution"][f"{finger}_finger_design"]
                q_schedule = motion["prescribed_input_crank_deg"]
                hand_schedule = motion["passive_hand_curl_progress"]
                self.assertEqual(len(q_schedule), components["collision_requested_pose_count"])
                self.assertEqual(len(hand_schedule), len(q_schedule))
                self.assertEqual(q_schedule[0], 0.0)
                self.assertEqual(hand_schedule[0], 0.0)
                self.assertTrue(all(
                    right >= left
                    for left, right in zip(hand_schedule, hand_schedule[1:])
                ))
                self.assertTrue(all(0.0 <= value <= 1.0 for value in hand_schedule))
                self.assertTrue((finger_dir / "history.csv").is_file())
                history_header = (
                    (finger_dir / "history.csv").read_text(encoding="utf-8")
                    .splitlines()[0].split(",")
                )
                for name in (
                    "task_space_reachability",
                    "hand_mechanism_non_collision",
                    "output_link_perpendicularity",
                    "rod_closure_feasible",
                    "maximum_rod_closure_error_mm",
                    "hand_motion_feasible",
                ):
                    self.assertIn(name, history_header)
                event_files = list(
                    (finger_dir / "tensorboard").glob("events.out.tfevents.*")
                )
                self.assertEqual(len(event_files), 1)
                event_data = event_files[0].read_bytes()
                self.assertIn(b"loss/total", event_data)
                self.assertIn(b"constraints/minimum_signed_clearance_mm", event_data)
                self.assertIn(b"loss/component/output_link_perpendicularity", event_data)
                self.assertIn(b"loss/component/task_space_reachability", event_data)
                self.assertIn(b"constraints/rod_closure_feasible", event_data)
                self.assertIn(b"constraints/hand_motion_feasible", event_data)
                self.assertIn(
                    b"constraints/maximum_rod_closure_error_mm", event_data
                )
                self.assertIn(
                    f"loss/objective/{finger}_finger_design".encode(),
                    event_data,
                )
                mechanism = yaml.safe_load(
                    (candidate_dir / "mechanism.yaml").read_text(encoding="utf-8")
                )
                self.assertEqual(
                    mechanism["optimization_provenance"]["target_finger"], finger
                )
                self.assertEqual(mechanism["mechanism"]["id"], f"mechanism_2_{finger}")
                self.assertEqual(
                    mechanism["human_hand_model"]["reference_finger"], finger
                )
                self.assertEqual(
                    [row["objective"]["finger"]
                     for row in mechanism["finger_analysis_targets"]],
                    [finger],
                )
                self.assertEqual(
                    mechanism["optimization_provenance"]["terminal_input_deg"],
                    q_schedule[-1],
                )
                self.assertEqual(
                    mechanism["optimization_provenance"]["prescribed_coordinate"],
                    "mechanism_input_crank_q",
                )
                self.assertIsNone(
                    mechanism["optimization_provenance"]["hand_crank_ratio"]
                )
                analysis_dir = candidate_dir / "artifacts"
                self.assertTrue(analysis_dir.is_dir())
                self.assertTrue(
                    (analysis_dir / "combined" / "fingers" / finger
                     / "combined_workspace_report.png").is_file()
                )
                self.assertEqual(
                    sorted(path.name for path in
                           (analysis_dir / "combined" / "fingers").iterdir()),
                    [finger],
                )
                summary = yaml.safe_load(
                    (analysis_dir / "combined" / "combined_workspace_summary.yaml")
                    .read_text(encoding="utf-8")
                )
                self.assertEqual(
                    [row["finger"] for row in summary["fingers"]], [finger]
                )
                self.assertEqual(
                    summary["motion_mapping"],
                    "crank_driven_passive_monotone_hand_curl",
                )
                reported_motion = summary["fingers"][0]["hand_motion"]
                self.assertTrue(reported_motion["progress_bounds_satisfied"])
                self.assertTrue(reported_motion["monotonic_non_decreasing"])
                self.assertTrue(reported_motion["dorsal_flexion_prohibited"])
                self.assertAlmostEqual(
                    reported_motion["terminal_progress"], hand_schedule[-1], places=6,
                )
                self.assertAlmostEqual(
                    summary["mechanism_solved_q_range_deg"][-1],
                    mechanism["optimization_provenance"]["terminal_input_deg"],
                )


if __name__ == "__main__":
    unittest.main()
