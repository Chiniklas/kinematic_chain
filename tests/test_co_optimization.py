from __future__ import annotations

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
    def test_task_space_reachability_is_the_only_enabled_objective(self) -> None:
        config = yaml.safe_load(OBJECTIVES.read_text(encoding="utf-8"))
        components = config["loss_components"]
        self.assertTrue(components["task_space_reachability"]["enabled"])
        self.assertTrue(all(
            not row["enabled"]
            for name, row in components.items()
            if name != "task_space_reachability"
        ))
        self.assertTrue(all(
            not row["enabled"] for row in config["placeholder_objectives"].values()
        ))
        self.assertTrue(all(
            not row["enabled"] for row in config["constraint_penalties"].values()
        ))

    def test_variables_match_nominal_mechanism_and_attachment_lengths(self) -> None:
        variable_data = yaml.safe_load(VARIABLES.read_text(encoding="utf-8"))
        nominal_data = yaml.safe_load(NOMINAL.read_text(encoding="utf-8"))
        configured = {row["id"]: row["initial"] for row in variable_data["variables"]}
        dimensions = {row["id"]: row["value"] for row in nominal_data["dimensions"]}
        self.assertEqual(
            {variable_id: configured[variable_id] for variable_id in dimensions},
            dimensions,
        )
        attachments = {
            row["id"]: row for row in nominal_data["exoskeleton_attachments"]
        }
        self.assertEqual(
            configured["L_tip_rod"],
            attachments["distal_output_rod"]["assumed_length_mm"],
        )

    def test_adam_entry_point_reduces_multi_objective_loss(self) -> None:
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
            self.assertIn("4 objectives, 13 variables", completed.stdout)
            candidate_dir = output_dir / "candidate_0001"
            candidate = yaml.safe_load(
                (candidate_dir / "candidate.yaml").read_text(encoding="utf-8")
            )
            optimizer = candidate["optimizer"]
            self.assertLess(optimizer["final_total_loss"], optimizer["initial_total_loss"])
            self.assertEqual(len(candidate["objective_losses"]), 4)
            variables = candidate["candidate_variables"]
            initial_variables = {
                row["id"]: row["initial"]
                for row in yaml.safe_load(
                    VARIABLES.read_text(encoding="utf-8")
                )["variables"]
            }
            self.assertEqual(len(variables), 13)
            self.assertEqual(
                set(variables),
                {
                    "L_ab", "L_bc", "L_cd", "L_ad", "L_ae", "L_de",
                    "L_cg", "L_dg", "L_ef", "L_fg", "L_gh", "L_fh",
                    "L_tip_rod",
                },
            )
            self.assertNotEqual(
                variables["L_tip_rod"]["value"],
                initial_variables["L_tip_rod"],
            )
            for components in candidate["component_losses"].values():
                self.assertIn("horizontal_slot_rod_error_mm", components)
                self.assertIn("curled_slot_rod_error_mm", components)
                self.assertIn("horizontal_slot_translation_mm", components)
                self.assertIn("curled_slot_translation_mm", components)
            self.assertTrue(all(row["units"] == "mm" for row in variables.values()))
            self.assertTrue((output_dir / "history.csv").is_file())
            mechanism = yaml.safe_load(
                (candidate_dir / "mechanism.yaml").read_text(encoding="utf-8")
            )
            self.assertEqual(
                mechanism["mechanism"]["status"], "co_optimization_candidate",
            )
            analysis_dirs = list(candidate_dir.glob("analysis_*"))
            self.assertEqual(len(analysis_dirs), 1)
            expected_analysis_files = {
                "mechanism/abstraction.png",
                "mechanism/mechanism_tables.md",
                "mechanism/link_lengths.csv",
                "mechanism/workspace/workspace_report.png",
                "mechanism/workspace/workspace_samples.csv",
                "mechanism/torque/torque_report.png",
                "mechanism/torque/torque_samples.csv",
                "combined/combined_abstraction.png",
                "combined/combined_workspace_report.png",
                "combined/combined_workspace_samples.csv",
                "combined/combined_workspace_summary.yaml",
            }
            self.assertTrue(all(
                (analysis_dirs[0] / filename).is_file()
                for filename in expected_analysis_files
            ))
            for finger in ("index", "middle", "ring", "little"):
                finger_dir = analysis_dirs[0] / "combined" / "fingers" / finger
                self.assertTrue(all(
                    (finger_dir / filename).is_file()
                    for filename in (
                        "combined_abstraction.png",
                        "combined_workspace_report.png",
                        "combined_workspace_samples.csv",
                        "combined_workspace_summary.yaml",
                    )
                ))


if __name__ == "__main__":
    unittest.main()
