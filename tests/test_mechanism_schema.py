from __future__ import annotations

import csv
import copy
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from mechanism_schema import (  # noqa: E402
    AbstractionError,
    body_memberships,
    dimension_pairs,
    load_abstraction,
    photo_nominal_length,
    validate_abstraction,
)
from plot_linkage import draw_abstraction  # noqa: E402


ABSTRACTION = ROOT / "sources" / "mechanism_2" / "mechanism.yaml"
DIMENSION_CSV = ROOT / "sources" / "mechanism_2" / "link_lengths.csv"


class MechanismSchemaTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.data = load_abstraction(ABSTRACTION)
        cls.summary = validate_abstraction(cls.data)

    def test_mechanism_2_node_sequence(self) -> None:
        self.assertEqual([node["id"] for node in self.data["nodes"]], list("abcdefhi"))
        self.assertNotIn("g", {node["id"] for node in self.data["nodes"]})

    def test_tip_is_a_reference_on_distal_body(self) -> None:
        nodes = {node["id"]: node for node in self.data["nodes"]}
        self.assertEqual(nodes["i"]["kind"], "reference")
        self.assertEqual(body_memberships(self.data)["i"], {"distal_body"})

    def test_every_declared_loop_side_has_a_dimension(self) -> None:
        pairs = dimension_pairs(self.data)
        for loop in self.data["loops"]:
            for index, node_id in enumerate(loop["nodes"]):
                next_id = loop["nodes"][(index + 1) % len(loop["nodes"])]
                self.assertIn(frozenset((node_id, next_id)), pairs, loop["id"])

    def test_current_mobility_matches_actuation(self) -> None:
        self.assertEqual(self.summary.planar_mobility, 1)
        self.assertFalse(any("actuator" in warning for warning in self.summary.warnings))

    def test_nominal_lengths_match_photo_calibration(self) -> None:
        for dimension in self.data["dimensions"]:
            expected = photo_nominal_length(self.data, *dimension["nodes"])
            self.assertAlmostEqual(dimension["value"], expected, delta=0.11)

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
