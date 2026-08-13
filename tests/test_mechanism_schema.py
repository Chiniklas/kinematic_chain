from __future__ import annotations

import csv
import copy
import sys
import unittest
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "analysis"))

from mechanism_schema import (  # noqa: E402
    AbstractionError,
    body_memberships,
    dimension_pairs,
    load_abstraction,
    validate_abstraction,
)
from combined_analysis import analyze_combined, draw_combined_report  # noqa: E402
from plot_linkage import draw_abstraction  # noqa: E402
from plot_combined_abstraction import (  # noqa: E402
    apply_finger_objective,
    draw_combined_abstraction,
    load_finger_objective,
)
from torque_analysis import analyze_torque, total_mass_g  # noqa: E402
from workspace_sweep import sweep_workspace  # noqa: E402


NOMINAL_SOURCE = ROOT / "designs" / "mechanism_2" / "nominal"
ABSTRACTION = NOMINAL_SOURCE / "mechanism.yaml"
DIMENSION_CSV = NOMINAL_SOURCE / "artifacts" / "mechanism" / "link_lengths.csv"
OBJECTIVES_DIR = ROOT / "src" / "co-optimization" / "config" / "objectives"


class MechanismSchemaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.data = load_abstraction(ABSTRACTION)
        cls.summary = validate_abstraction(cls.data)

    def test_mechanism_2_node_sequence(self) -> None:
        self.assertEqual([node["id"] for node in self.data["nodes"]], list("abcdefgh"))

    def test_nominal_design_directory_is_grouped(self) -> None:
        self.assertEqual(
            {path.name for path in NOMINAL_SOURCE.iterdir() if path.is_file()},
            {"README.md", "mechanism.yaml"},
        )
        expected_artifacts = {
            "mechanism/abstraction.png",
            "mechanism/link_lengths.csv",
            "mechanism/mechanism_tables.md",
            "mechanism/workspace/workspace_report.png",
            "mechanism/workspace/workspace_samples.csv",
            "mechanism/torque/torque_report.png",
            "mechanism/torque/torque_samples.csv",
            "combined/combined_abstraction.png",
            "combined/combined_workspace_report.png",
            "combined/combined_workspace_samples.csv",
            "combined/combined_workspace_summary.yaml",
        }
        for finger in ("index", "middle", "ring", "little"):
            expected_artifacts.update({
                f"combined/fingers/{finger}/combined_abstraction.png",
                f"combined/fingers/{finger}/combined_workspace_report.png",
                f"combined/fingers/{finger}/combined_workspace_samples.csv",
                f"combined/fingers/{finger}/combined_workspace_summary.yaml",
            })
        artifacts = NOMINAL_SOURCE / "artifacts"
        actual_artifacts = {
            path.relative_to(artifacts).as_posix()
            for path in artifacts.rglob("*") if path.is_file()
            and path.name != "design_animation.gif"
        }
        self.assertEqual(actual_artifacts, expected_artifacts)

    def test_tip_is_a_reference_on_distal_body(self) -> None:
        nodes = {node["id"]: node for node in self.data["nodes"]}
        self.assertEqual(nodes["h"]["kind"], "reference")
        self.assertEqual(body_memberships(self.data)["h"], {"distal_body"})

    def test_nominal_hand_and_attachment_interfaces(self) -> None:
        hand = self.data["human_hand_model"]
        phalanges = {row["id"]: row for row in hand["phalanges"]}
        self.assertEqual(hand["reference_finger"], "index")
        self.assertEqual(
            [row["objective"]["finger"] for row in self.data["finger_analysis_targets"]],
            ["index", "middle", "ring", "little"],
        )
        self.assertEqual(hand["pose"], "mildly_curled")
        self.assertEqual(
            (hand["palm"]["length_mm"], hand["palm"]["width_mm"]),
            (90.0, 25.0),
        )
        self.assertEqual(hand["palm"]["orientation"], "horizontal")
        self.assertEqual(
            {
                row_id: (row["length_mm"], row["width_mm"])
                for row_id, row in phalanges.items()
            },
            {
                "proximal_phalanx": (40.0, 20.0),
                "middle_phalanx": (25.0, 15.0),
                "distal_phalanx": (25.0, 10.0),
            },
        )
        attachments = {row["id"]: row for row in self.data["exoskeleton_attachments"]}
        input_mount = attachments["dorsal_input_mount"]
        self.assertEqual(input_mount["mechanism_node"], "d")
        self.assertEqual(input_mount["hand_reference"], "hand_mcp")
        self.assertEqual(input_mount["dorsal_clearance_mm"], 1.0)
        self.assertEqual(
            input_mount["clearance_control"], "upstream_manual_design_parameter"
        )
        self.assertFalse(input_mount["optimizable"])
        self.assertEqual(input_mount["alignment_member"], ["a", "d"])
        self.assertEqual(input_mount["alignment"], "horizontal")
        output = attachments["distal_output_rod"]
        self.assertEqual(output["mechanism_node"], "h")
        self.assertEqual(output["hand_reference"], "hand_distal_contact")
        self.assertEqual(output["hand_interface"], "revolute")
        self.assertEqual(output["assumed_length_mm"], 28.0)
        self.assertEqual(output["previous_assumption_mm"], 15.0)
        self.assertEqual(
            output["value_source"],
            "retained_pre_fixed_R4_baseline",
        )
        self.assertEqual(output["surface"], "distal_phalanx_upper_dorsal")
        self.assertEqual(output["longitudinal_fraction"], 0.5)
        self.assertEqual(output["pair_dofs"], ["rotation"])
        self.assertNotIn("translation_range_mm", output)
        hand_joints = {row["id"]: row for row in hand["joints"]}
        self.assertEqual(
            [hand_joints[node_id]["node_index"] for node_id in (
                "hand_mcp", "hand_pip", "hand_dip",
            )],
            [1, 2, 3],
        )
        self.assertEqual(hand_joints["hand_distal_contact"]["attachment_index"], 4)
        contact = hand_joints["hand_distal_contact"]["position_mm"]
        dip = hand_joints["hand_dip"]["position_mm"]
        tip = hand_joints["hand_tip"]["position_mm"]
        upper_midpoint = ((dip[0] + tip[0]) / 2, (dip[1] + tip[1]) / 2)
        self.assertAlmostEqual(contact[0], upper_midpoint[0], places=5)
        self.assertAlmostEqual(contact[1], upper_midpoint[1], places=5)

    def test_every_declared_loop_side_has_a_dimension(self) -> None:
        pairs = dimension_pairs(self.data)
        for loop in self.data["loops"]:
            for index, node_id in enumerate(loop["nodes"]):
                next_id = loop["nodes"][(index + 1) % len(loop["nodes"])]
                self.assertIn(frozenset((node_id, next_id)), pairs, loop["id"])

    def test_current_mobility_matches_actuation(self) -> None:
        self.assertEqual(self.summary.planar_mobility, 1)
        self.assertFalse(any("actuator" in warning for warning in self.summary.warnings))

    def test_current_lengths_match_remeasured_design(self) -> None:
        expected = {
            "L_ab": 31,
            "L_bc": 54,
            "L_cd": 28,
            "L_ad": 54,
            "L_ae": 66,
            "L_de": 14,
            "L_cg": 50,
            "L_dg": 57,
            "L_ef": 30,
            "L_fg": 28,
            "L_gh": 50,
            "L_fh": 57,
        }
        actual = {row["id"]: row["value"] for row in self.data["dimensions"]}
        self.assertEqual(actual, expected)
        self.assertFalse(any(
            row.get("value_source") == "photo_nominal"
            for row in self.data["dimensions"]
        ))

    def test_exported_csv_matches_yaml_dimension_ids(self) -> None:
        with DIMENSION_CSV.open(newline="", encoding="utf-8") as stream:
            csv_ids = [row["dimension_id"] for row in csv.DictReader(stream)]
        yaml_ids = [row["id"] for row in self.data["dimensions"]]
        self.assertEqual(csv_ids, yaml_ids)

    def test_plot_labels_every_dimension(self) -> None:
        figure, axes = draw_abstraction(self.data)
        try:
            plotted_text = {
                label.get_text().splitlines()[0]
                for label in axes.texts
                if label.get_text()
            }
            expected = {dimension["id"] for dimension in self.data["dimensions"]}
            self.assertTrue(expected <= plotted_text)
        finally:
            import matplotlib.pyplot as plt
            plt.close(figure)

    def test_combined_abstraction_renders_hand_and_attachments(self) -> None:
        figure, axes = draw_combined_abstraction(self.data)
        try:
            labels = {text.get_text() for text in axes.texts}
            self.assertTrue(any("proximal_phalanx" in label for label in labels))
            self.assertTrue(any("rod" in label for label in labels))
            self.assertTrue({"J1", "J2", "J3", "R4"} <= labels)
        finally:
            import matplotlib.pyplot as plt
            plt.close(figure)

    def test_all_long_fingers_build_combined_abstractions(self) -> None:
        expected_lengths = {
            "index": [40.0, 25.0, 25.0],
            "middle": [49.0, 27.0, 27.0],
            "ring": [44.0, 26.0, 26.0],
            "little": [34.0, 23.0, 23.0],
        }
        for finger, lengths in expected_lengths.items():
            objective_path = (
                ROOT / "src" / "co-optimization" / "config" / "objectives"
                / f"{finger}.yaml"
            )
            data = apply_finger_objective(
                self.data, load_finger_objective(objective_path), objective_path,
            )
            self.assertEqual(data["human_hand_model"]["reference_finger"], finger)
            self.assertEqual(
                [row["length_mm"] for row in data["human_hand_model"]["phalanges"]],
                lengths,
            )
            figure, axes = draw_combined_abstraction(data)
            try:
                self.assertIn(f"nominal {finger}-finger", axes.get_title())
            finally:
                import matplotlib.pyplot as plt
                plt.close(figure)

    def test_workspace_sweep_closes_declared_dimensions(self) -> None:
        result = sweep_workspace(self.data, q_min=0.0, q_max=90.0, steps=19)
        self.assertEqual(result.poses[0].q_deg, 0.0)
        self.assertGreater(result.poses[-1].q_deg, 60.0)
        self.assertLess(result.poses[-1].q_deg, 90.0)
        self.assertLess(max(pose.max_residual_mm for pose in result.poses), 1e-4)
        self.assertEqual(result.output_node, "h")
        self.assertLess(
            result.poses[-1].positions["h"][1],
            result.poses[0].positions["h"][1],
        )

    def test_combined_sweep_covers_all_fingers_and_fixed_r4(self) -> None:
        result = analyze_combined(self.data, steps=19)
        self.assertEqual([row.finger for row in result.fingers], [
            "index", "middle", "ring", "little",
        ])
        for finger in result.fingers:
            self.assertEqual(finger.samples[0].progress, 0.0)
            self.assertEqual(finger.samples[-1].progress, 1.0)
            for sample in finger.samples:
                self.assertAlmostEqual(
                    sample.contact[0],
                    (sample.distal_start[0] + sample.distal_end[0]) / 2.0,
                )
                self.assertAlmostEqual(
                    sample.contact[1],
                    (sample.distal_start[1] + sample.distal_end[1]) / 2.0,
                )
                self.assertTrue(np.isfinite(sample.rod_error_mm))
                self.assertGreaterEqual(sample.rod_error_mm, 0.0)
                self.assertLess(sample.mechanism_residual_mm, 1e-4)
        figure = draw_combined_report(result)
        try:
            self.assertEqual(len(figure.axes), 4)
            self.assertTrue(all(axes.patches for axes in figure.axes))
            self.assertTrue(all("full assembly sweep" in axes.get_title()
                                for axes in figure.axes))
        finally:
            import matplotlib.pyplot as plt
            plt.close(figure)

    def test_nominal_torque_analysis_uses_yaml_mass_model(self) -> None:
        sweep = sweep_workspace(self.data, q_min=0.0, q_max=5.0, steps=6)
        torque = analyze_torque(self.data, sweep)
        self.assertEqual(len(torque.q_deg), 6)
        self.assertTrue(np.all(np.isfinite(torque.total_nm)))
        self.assertAlmostEqual(total_mass_g(self.data), 36.0)

    def test_validator_accepts_an_unrelated_four_bar(self) -> None:
        data = {
            "schema_version": 1,
            "mechanism": {"id": "four_bar_fixture", "status": "validated"},
            "nodes": [
                {"id": "p", "kind": "revolute", "layout": [0, 0]},
                {"id": "q", "kind": "revolute", "layout": [1, 1]},
                {"id": "r", "kind": "revolute", "layout": [3, 1]},
                {"id": "s", "kind": "revolute", "layout": [4, 0]},
            ],
            "bodies": [
                {"id": "frame", "kind": "ground", "nodes": ["p", "s"]},
                {"id": "crank", "kind": "binary_link", "nodes": ["p", "q"]},
                {"id": "coupler", "kind": "binary_link", "nodes": ["q", "r"]},
                {"id": "rocker", "kind": "binary_link", "nodes": ["r", "s"]},
            ],
            "joints": [
                {"node": "p", "type": "revolute", "bodies": ["frame", "crank"]},
                {"node": "q", "type": "revolute", "bodies": ["crank", "coupler"]},
                {"node": "r", "type": "revolute", "bodies": ["coupler", "rocker"]},
                {"node": "s", "type": "revolute", "bodies": ["rocker", "frame"]},
            ],
            "dimensions": [
                {"id": "L_ps", "body": "frame", "nodes": ["p", "s"], "value": 4},
                {"id": "L_pq", "body": "crank", "nodes": ["p", "q"], "value": 1},
                {"id": "L_qr", "body": "coupler", "nodes": ["q", "r"], "value": 2},
                {"id": "L_rs", "body": "rocker", "nodes": ["r", "s"], "value": 1},
            ],
            "loops": [{
                "id": "main",
                "nodes": ["p", "q", "r", "s"],
                "bodies": ["crank", "coupler", "rocker", "frame"],
            }],
            "actuators": [{
                "id": "q_in", "joint": "p", "body": "crank", "reference_body": "frame"
            }],
            "outputs": [{"id": "out", "node": "r", "body": "rocker"}],
        }
        summary = validate_abstraction(data)
        self.assertEqual(summary.mechanism_id, "four_bar_fixture")
        self.assertEqual(summary.planar_mobility, 1)
        self.assertFalse(summary.warnings)

    def test_incidence_mismatch_is_rejected(self) -> None:
        data = copy.deepcopy(self.data)
        data["joints"][0]["bodies"] = ["ground", "loop_1_coupler"]
        with self.assertRaises(AbstractionError):
            validate_abstraction(data)


if __name__ == "__main__":
    unittest.main()
