#!/usr/bin/env python3
"""Load and validate a generic planar body-joint mechanism abstraction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


DEFAULT_ABSTRACTION = (
    Path(__file__).resolve().parents[1] / "sources" / "mechanism_2" / "mechanism.yaml"
)


class AbstractionError(ValueError):
    """Raised when an abstraction violates the schema or incidence rules."""


@dataclass(frozen=True)
class ValidationSummary:
    mechanism_id: str
    node_count: int
    body_count: int
    revolute_node_count: int
    equivalent_lower_pairs: int
    independent_loops: int
    planar_mobility: int
    missing_dimension_ids: tuple[str, ...]
    warnings: tuple[str, ...]


def _table_by_id(rows: Iterable[dict[str, Any]], table_name: str) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise AbstractionError(f"{table_name}[{index}] must be a mapping")
        row_id = row.get("id")
        if not isinstance(row_id, str) or not row_id:
            raise AbstractionError(f"{table_name}[{index}] needs a non-empty string id")
        if row_id in result:
            raise AbstractionError(f"duplicate {table_name} id: {row_id}")
        result[row_id] = row
    return result


def load_abstraction(path: Path = DEFAULT_ABSTRACTION) -> dict[str, Any]:
    """Load YAML and validate it before returning the plain data structure."""
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise AbstractionError("the YAML document root must be a mapping")
    validate_abstraction(data)
    return data


def node_layout(data: dict[str, Any]) -> dict[str, tuple[float, float]]:
    """Return validated diagram coordinates keyed by node id."""
    result: dict[str, tuple[float, float]] = {}
    for node in data["nodes"]:
        layout = node.get("layout")
        if not isinstance(layout, list) or len(layout) != 2:
            raise AbstractionError(f"node {node['id']} needs layout: [x, y]")
        x, y = layout
        if not all(isinstance(value, (int, float)) and math.isfinite(value)
                   for value in (x, y)):
            raise AbstractionError(f"node {node['id']} has an invalid layout")
        result[node["id"]] = (float(x), float(y))
    return result


def photo_node_positions(data: dict[str, Any]) -> dict[str, tuple[float, float]]:
    """Return source-photo pixel centres keyed by node id."""
    result: dict[str, tuple[float, float]] = {}
    for node in data["nodes"]:
        pixel = node.get("photo_pixel")
        if not isinstance(pixel, list) or len(pixel) != 2:
            raise AbstractionError(f"node {node['id']} needs photo_pixel: [x, y]")
        x, y = pixel
        if not all(isinstance(value, (int, float)) and math.isfinite(value)
                   for value in (x, y)):
            raise AbstractionError(f"node {node['id']} has an invalid photo_pixel")
        result[node["id"]] = (float(x), float(y))
    return result


def photo_nominal_length(data: dict[str, Any], node_1: str, node_2: str) -> float:
    """Calculate a nominal millimetre length from photo pixels and calibration."""
    calibration = data.get("photo_calibration")
    if not isinstance(calibration, dict):
        raise AbstractionError("photo_calibration is required for photo-derived lengths")
    pixels_per_mm = calibration.get("pixels_per_mm")
    if not isinstance(pixels_per_mm, (int, float)) or pixels_per_mm <= 0:
        raise AbstractionError("photo_calibration.pixels_per_mm must be positive")
    positions = photo_node_positions(data)
    if node_1 not in positions or node_2 not in positions:
        raise AbstractionError(f"unknown photo nodes: {node_1}, {node_2}")
    return math.dist(positions[node_1], positions[node_2]) / pixels_per_mm


def body_memberships(data: dict[str, Any]) -> dict[str, set[str]]:
    """Return the bodies containing each node."""
    memberships = {node["id"]: set() for node in data["nodes"]}
    for body in data["bodies"]:
        for node_id in body["nodes"]:
            memberships[node_id].add(body["id"])
    return memberships


def dimension_pairs(data: dict[str, Any]) -> dict[frozenset[str], dict[str, Any]]:
    """Return dimension records keyed by their unordered node pair."""
    return {frozenset(row["nodes"]): row for row in data["dimensions"]}


def validate_abstraction(data: dict[str, Any]) -> ValidationSummary:
    """Validate schema references and calculate the implied planar mobility."""
    if data.get("schema_version") != 1:
        raise AbstractionError("only schema_version: 1 is supported")
    mechanism = data.get("mechanism")
    if not isinstance(mechanism, dict) or not isinstance(mechanism.get("id"), str):
        raise AbstractionError("mechanism.id is required")

    required_tables = ("nodes", "bodies", "joints", "dimensions")
    for name in required_tables:
        if not isinstance(data.get(name), list) or not data[name]:
            raise AbstractionError(f"{name} must be a non-empty list")

    nodes = _table_by_id(data["nodes"], "nodes")
    bodies = _table_by_id(data["bodies"], "bodies")
    dimensions = _table_by_id(data["dimensions"], "dimensions")
    node_layout(data)

    calibration = data.get("photo_calibration")
    if calibration is not None:
        if not isinstance(calibration, dict):
            raise AbstractionError("photo_calibration must be a mapping")
        pixels_per_mm = calibration.get("pixels_per_mm")
        if not isinstance(pixels_per_mm, (int, float)) or pixels_per_mm <= 0:
            raise AbstractionError("photo_calibration.pixels_per_mm must be positive")
        relative_uncertainty = calibration.get("relative_uncertainty")
        if not isinstance(relative_uncertainty, (int, float)) or not 0 < relative_uncertainty < 1:
            raise AbstractionError(
                "photo_calibration.relative_uncertainty must be between zero and one"
            )
        image_size = calibration.get("image_size_pixels")
        if (not isinstance(image_size, list) or len(image_size) != 2
                or not all(isinstance(value, int) and value > 0 for value in image_size)):
            raise AbstractionError("photo_calibration.image_size_pixels must be [width, height]")
        for node_id, (x, y) in photo_node_positions(data).items():
            if not 0 <= x < image_size[0] or not 0 <= y < image_size[1]:
                raise AbstractionError(f"node {node_id} photo_pixel lies outside the image")

    ground_bodies = [body for body in bodies.values() if body.get("kind") == "ground"]
    if len(ground_bodies) != 1:
        raise AbstractionError("exactly one body with kind: ground is required")

    for body in bodies.values():
        body_nodes = body.get("nodes")
        if not isinstance(body_nodes, list) or len(body_nodes) < 2:
            raise AbstractionError(f"body {body['id']} needs at least two nodes")
        if len(body_nodes) != len(set(body_nodes)):
            raise AbstractionError(f"body {body['id']} repeats a node")
        unknown = set(body_nodes) - set(nodes)
        if unknown:
            raise AbstractionError(f"body {body['id']} has unknown nodes: {sorted(unknown)}")
        if body.get("kind") == "binary_link" and len(body_nodes) != 2:
            raise AbstractionError(f"binary body {body['id']} must contain exactly two nodes")

    memberships = body_memberships(data)
    seen_joint_nodes: set[str] = set()
    equivalent_pairs = 0
    for index, joint in enumerate(data["joints"]):
        if not isinstance(joint, dict):
            raise AbstractionError(f"joints[{index}] must be a mapping")
        node_id = joint.get("node")
        if node_id not in nodes:
            raise AbstractionError(f"joint references unknown node: {node_id}")
        if node_id in seen_joint_nodes:
            raise AbstractionError(f"duplicate joint incidence for node: {node_id}")
        seen_joint_nodes.add(node_id)
        if nodes[node_id].get("kind") != "revolute" or joint.get("type") != "revolute":
            raise AbstractionError(f"joint {node_id} must describe a revolute node")
        incident = joint.get("bodies")
        if not isinstance(incident, list) or len(incident) < 2:
            raise AbstractionError(f"joint {node_id} needs at least two incident bodies")
        if len(incident) != len(set(incident)):
            raise AbstractionError(f"joint {node_id} repeats an incident body")
        unknown = set(incident) - set(bodies)
        if unknown:
            raise AbstractionError(f"joint {node_id} has unknown bodies: {sorted(unknown)}")
        if set(incident) != memberships[node_id]:
            raise AbstractionError(
                f"joint {node_id} incidence {sorted(incident)} does not match body table "
                f"{sorted(memberships[node_id])}"
            )
        equivalent_pairs += len(incident) - 1

    revolute_nodes = {node_id for node_id, node in nodes.items()
                      if node.get("kind") == "revolute"}
    if seen_joint_nodes != revolute_nodes:
        missing = sorted(revolute_nodes - seen_joint_nodes)
        extra = sorted(seen_joint_nodes - revolute_nodes)
        raise AbstractionError(f"joint table mismatch; missing={missing}, extra={extra}")

    pair_records: dict[frozenset[str], str] = {}
    photo_nominal_count = 0
    for row in dimensions.values():
        pair = row.get("nodes")
        if not isinstance(pair, list) or len(pair) != 2 or pair[0] == pair[1]:
            raise AbstractionError(f"dimension {row['id']} needs two distinct nodes")
        if not set(pair) <= set(nodes):
            raise AbstractionError(f"dimension {row['id']} references an unknown node")
        body_id = row.get("body")
        if body_id not in bodies:
            raise AbstractionError(f"dimension {row['id']} references unknown body {body_id}")
        if not set(pair) <= set(bodies[body_id]["nodes"]):
            raise AbstractionError(
                f"dimension {row['id']} nodes are not both in body {body_id}"
            )
        key = frozenset(pair)
        if key in pair_records:
            raise AbstractionError(
                f"dimensions {pair_records[key]} and {row['id']} duplicate the same node pair"
            )
        pair_records[key] = row["id"]
        value = row.get("value")
        if value is not None and (not isinstance(value, (int, float)) or value <= 0):
            raise AbstractionError(f"dimension {row['id']} value must be positive or null")
        if row.get("value_source") == "photo_nominal":
            if calibration is None:
                raise AbstractionError(
                    f"dimension {row['id']} is photo-derived but photo_calibration is missing"
                )
            uncertainty = row.get("uncertainty_mm")
            if not isinstance(uncertainty, (int, float)) or uncertainty <= 0:
                raise AbstractionError(
                    f"photo-derived dimension {row['id']} needs positive uncertainty_mm"
                )
            expected = photo_nominal_length(data, pair[0], pair[1])
            if value is None or not math.isclose(value, expected, abs_tol=0.11):
                raise AbstractionError(
                    f"dimension {row['id']} is {value}, but its calibrated photo length "
                    f"is {expected:.3f} mm"
                )
            photo_nominal_count += 1

    for body in bodies.values():
        if body.get("kind") == "binary_link" or (
            body.get("kind") == "ground" and len(body["nodes"]) == 2
        ):
            pair = frozenset(body["nodes"])
            if pair not in pair_records:
                raise AbstractionError(f"body {body['id']} has no dimension record")
        elif body.get("kind") in {"ground", "rigid_body"}:
            body_dimension_count = sum(
                1 for row in dimensions.values() if row.get("body") == body["id"]
            )
            minimum_dimension_count = 2 * len(body["nodes"]) - 3
            if body_dimension_count < minimum_dimension_count:
                raise AbstractionError(
                    f"rigid body {body['id']} needs at least {minimum_dimension_count} "
                    f"independent dimensions, found {body_dimension_count}"
                )

    loops = data.get("loops", [])
    if not isinstance(loops, list):
        raise AbstractionError("loops must be a list")
    for index, loop in enumerate(loops):
        loop_nodes = loop.get("nodes")
        loop_bodies = loop.get("bodies")
        if not isinstance(loop_nodes, list) or len(loop_nodes) < 3:
            raise AbstractionError(f"loops[{index}] needs at least three nodes")
        if not isinstance(loop_bodies, list) or len(loop_bodies) != len(loop_nodes):
            raise AbstractionError(f"loop {loop.get('id', index)} needs one body per side")
        for side, body_id in enumerate(loop_bodies):
            if body_id not in bodies:
                raise AbstractionError(f"loop references unknown body: {body_id}")
            endpoints = {loop_nodes[side], loop_nodes[(side + 1) % len(loop_nodes)]}
            if not endpoints <= set(bodies[body_id]["nodes"]):
                raise AbstractionError(
                    f"loop {loop.get('id', index)} side {sorted(endpoints)} is not in {body_id}"
                )

    for actuator in data.get("actuators", []):
        if actuator.get("joint") not in revolute_nodes:
            raise AbstractionError(f"actuator {actuator.get('id')} has an invalid joint")
        if actuator.get("body") not in bodies or actuator.get("reference_body") not in bodies:
            raise AbstractionError(f"actuator {actuator.get('id')} has an invalid body")

    for output in data.get("outputs", []):
        if output.get("node") not in nodes or output.get("body") not in bodies:
            raise AbstractionError(f"output {output.get('id')} has an invalid reference")
        if output["node"] not in bodies[output["body"]]["nodes"]:
            raise AbstractionError(f"output {output.get('id')} node is not in its body")

    body_count = len(bodies)
    mobility = 3 * (body_count - 1) - 2 * equivalent_pairs
    graph_vertices = body_count
    graph_edges = equivalent_pairs
    independent_loops = graph_edges - graph_vertices + 1
    missing_dimensions = tuple(
        row_id for row_id, row in dimensions.items() if row.get("value") is None
    )
    warnings: list[str] = []
    if missing_dimensions:
        warnings.append(f"{len(missing_dimensions)} dimensions have no numeric value")
    if photo_nominal_count:
        warnings.append(
            f"{photo_nominal_count} dimensions are nominal photo estimates, not measured values"
        )
    actuator_count = len(data.get("actuators", []))
    if mobility > actuator_count:
        warnings.append(
            f"mobility is {mobility} with {actuator_count} declared actuator(s), leaving "
            f"{mobility - actuator_count} unactuated DOF(s); passive behavior must be "
            "defined before a driven sweep"
        )
    elif mobility < actuator_count:
        warnings.append(
            f"mobility is {mobility}, below the {actuator_count} declared actuator(s); "
            "the actuation model is overconstrained"
        )
    if mechanism.get("status") != "validated":
        warnings.append(f"mechanism status is {mechanism.get('status', 'unspecified')}")

    return ValidationSummary(
        mechanism_id=mechanism["id"],
        node_count=len(nodes),
        body_count=body_count,
        revolute_node_count=len(revolute_nodes),
        equivalent_lower_pairs=equivalent_pairs,
        independent_loops=independent_loops,
        planar_mobility=mobility,
        missing_dimension_ids=missing_dimensions,
        warnings=tuple(warnings),
    )


def summary_lines(summary: ValidationSummary) -> list[str]:
    """Format a compact validation summary for command-line tools."""
    lines = [
        f"mechanism: {summary.mechanism_id}",
        f"nodes: {summary.node_count} "
        f"({summary.revolute_node_count} revolute + "
        f"{summary.node_count - summary.revolute_node_count} reference)",
        f"bodies including ground: {summary.body_count}",
        f"equivalent lower pairs: {summary.equivalent_lower_pairs}",
        f"independent loops: {summary.independent_loops}",
        f"planar Gruebler mobility: {summary.planar_mobility}",
    ]
    lines.extend(f"warning: {warning}" for warning in summary.warnings)
    return lines
