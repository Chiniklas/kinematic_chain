#!/usr/bin/env python3
"""Load and validate a generic planar body-joint mechanism abstraction."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


DEFAULT_ABSTRACTION = (
    Path(__file__).resolve().parents[2]
    / "designs" / "mechanism_2" / "nominal" / "mechanism.yaml"
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


def l_bracket_segments(
    body: dict[str, Any], positions: dict[str, tuple[float, float]],
) -> tuple[tuple[tuple[float, float], tuple[float, float]], ...] | None:
    """Return the two visual arms of an L-bracket body, if declared."""
    if body.get("render_shape") != "l_bracket":
        return None
    corner = positions[body["render_corner_node"]]
    return tuple(
        (corner, positions[node_id]) for node_id in body["render_arm_nodes"]
    )


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
        render_shape = body.get("render_shape")
        if render_shape is not None and render_shape != "l_bracket":
            raise AbstractionError(
                f"body {body['id']} has unsupported render_shape: {render_shape}"
            )
        if render_shape == "l_bracket":
            corner = body.get("render_corner_node")
            arms = body.get("render_arm_nodes")
            if (body.get("kind") != "rigid_body" or len(body_nodes) != 3
                    or corner not in body_nodes
                    or not isinstance(arms, list) or len(arms) != 2
                    or len(set(arms)) != 2
                    or set(arms) != set(body_nodes) - {corner}):
                raise AbstractionError(
                    f"body {body['id']} L-bracket rendering needs one corner and "
                    "the other two body nodes as arms"
                )
            flesh_scale = body.get("render_flesh_scale", 1.0)
            if not isinstance(flesh_scale, (int, float)) or flesh_scale <= 0:
                raise AbstractionError(
                    f"body {body['id']} render_flesh_scale must be positive"
                )

    hand_model = data.get("human_hand_model")
    hand_joints: dict[str, dict[str, Any]] = {}
    if hand_model is not None:
        if not isinstance(hand_model, dict):
            raise AbstractionError("human_hand_model must be a mapping")
        hand_joint_rows = hand_model.get("joints")
        phalanges = hand_model.get("phalanges")
        if not isinstance(hand_joint_rows, list) or len(hand_joint_rows) < 2:
            raise AbstractionError("human_hand_model.joints needs at least two rows")
        if not isinstance(phalanges, list) or not phalanges:
            raise AbstractionError("human_hand_model.phalanges must be a non-empty list")
        hand_joints = _table_by_id(hand_joint_rows, "human_hand_model.joints")
        for joint in hand_joints.values():
            position = joint.get("position_mm")
            if (not isinstance(position, list) or len(position) != 2
                    or not all(isinstance(value, (int, float)) and math.isfinite(value)
                               for value in position)):
                raise AbstractionError(
                    f"human hand joint {joint['id']} needs finite position_mm: [x, y]"
                )
        phalanx_rows = _table_by_id(phalanges, "human_hand_model.phalanges")
        palm = hand_model.get("palm")
        if not isinstance(palm, dict):
            raise AbstractionError("human_hand_model.palm must be a mapping")
        hand_segments = [palm, *phalanx_rows.values()]
        for segment in hand_segments:
            endpoints = segment.get("joints")
            if (not isinstance(endpoints, list) or len(endpoints) != 2
                    or endpoints[0] == endpoints[1]
                    or not set(endpoints) <= set(hand_joints)):
                raise AbstractionError(
                    f"human segment {segment['id']} needs two known distinct joints"
                )
            length = segment.get("length_mm")
            width = segment.get("width_mm")
            if (not isinstance(length, (int, float)) or length <= 0
                    or not isinstance(width, (int, float)) or width <= 0):
                raise AbstractionError(
                    f"human segment {segment['id']} needs positive length_mm and width_mm"
                )
            start = hand_joints[endpoints[0]]["position_mm"]
            end = hand_joints[endpoints[1]]["position_mm"]
            if not math.isclose(float(length), math.dist(start, end), abs_tol=1e-6):
                raise AbstractionError(
                    f"human segment {segment['id']} length does not match its joint positions"
                )
            if segment.get("shape") != "rounded_rectangle":
                raise AbstractionError(
                    f"human segment {segment['id']} must use shape: rounded_rectangle"
                )
        palm_start, palm_end = (hand_joints[node_id]["position_mm"] for node_id in palm["joints"])
        if not math.isclose(palm_start[1], palm_end[1], abs_tol=1e-9):
            raise AbstractionError("human palm must be horizontal")

        attachment_rows = data.get("exoskeleton_attachments")
        if not isinstance(attachment_rows, list) or not attachment_rows:
            raise AbstractionError(
                "human_hand_model requires a non-empty exoskeleton_attachments list"
            )
        attachments = _table_by_id(attachment_rows, "exoskeleton_attachments")
        for attachment in attachments.values():
            if attachment.get("mechanism_node") not in nodes:
                raise AbstractionError(
                    f"attachment {attachment['id']} references an unknown mechanism node"
                )
            if attachment.get("hand_reference") not in hand_joints:
                raise AbstractionError(
                    f"attachment {attachment['id']} references an unknown hand joint"
                )
            offset = attachment.get("offset_from_hand_reference_mm")
            if offset is not None and (
                not isinstance(offset, list) or len(offset) != 2
                or not all(isinstance(value, (int, float)) and math.isfinite(value)
                           for value in offset)
            ):
                raise AbstractionError(
                    f"attachment {attachment['id']} has an invalid hand offset"
                )

        target_rows = data.get("finger_analysis_targets")
        if not isinstance(target_rows, list) or not target_rows:
            raise AbstractionError(
                "human_hand_model requires non-empty finger_analysis_targets"
            )
        seen_fingers: set[str] = set()
        for index, target in enumerate(target_rows):
            if not isinstance(target, dict):
                raise AbstractionError(f"finger_analysis_targets[{index}] must be a mapping")
            objective = target.get("objective")
            finger = objective.get("finger") if isinstance(objective, dict) else None
            if not isinstance(finger, str) or not finger or finger in seen_fingers:
                raise AbstractionError(
                    "finger_analysis_targets need unique non-empty objective.finger values"
                )
            seen_fingers.add(finger)
            lengths = target.get("phalanx_lengths_mm")
            ranges = target.get("joint_flexion_ranges_deg")
            if not isinstance(lengths, dict) or not isinstance(ranges, dict):
                raise AbstractionError(
                    f"finger target {finger} needs lengths and joint flexion ranges"
                )
            for segment_id in ("proximal", "middle", "distal"):
                value = lengths.get(segment_id)
                if not isinstance(value, (int, float)) or value <= 0:
                    raise AbstractionError(
                        f"finger target {finger} has invalid {segment_id} length"
                    )
            for joint_id in ("mcp", "pip", "dip"):
                limits = ranges.get(joint_id)
                if (not isinstance(limits, dict)
                        or not isinstance(limits.get("min"), (int, float))
                        or not isinstance(limits.get("max"), (int, float))
                        or limits["min"] < 0 or limits["min"] >= limits["max"]):
                    raise AbstractionError(
                        f"finger target {finger} has invalid {joint_id} flexion range"
                    )
        reference_finger = hand_model.get("reference_finger")
        reference_target = next(
            (row for row in target_rows
             if row["objective"]["finger"] == reference_finger),
            None,
        )
        if reference_target is None:
            raise AbstractionError(
                "human_hand_model.reference_finger needs an embedded analysis target"
            )
        hand_lengths = {
            "proximal": phalanx_rows["proximal_phalanx"]["length_mm"],
            "middle": phalanx_rows["middle_phalanx"]["length_mm"],
            "distal": phalanx_rows["distal_phalanx"]["length_mm"],
        }
        if any(
            not math.isclose(
                float(hand_lengths[name]),
                float(reference_target["phalanx_lengths_mm"][name]),
                abs_tol=1e-9,
            )
            for name in hand_lengths
        ):
            raise AbstractionError(
                "human_hand_model phalanx lengths must match its embedded target"
            )
            clearance = attachment.get("dorsal_clearance_mm")
            if clearance is not None and (
                not isinstance(clearance, (int, float))
                or not math.isfinite(clearance)
                or clearance < 0
            ):
                raise AbstractionError(
                    f"attachment {attachment['id']} has invalid dorsal_clearance_mm"
                )
            if clearance is not None and attachment.get("clearance_control") != (
                "upstream_manual_design_parameter"
            ):
                raise AbstractionError(
                    f"attachment {attachment['id']} must declare upstream clearance control"
                )
            if attachment.get("connector") == "binary_rod":
                rod_length = attachment.get("assumed_length_mm")
                if not isinstance(rod_length, (int, float)) or rod_length <= 0:
                    raise AbstractionError(
                        f"rod attachment {attachment['id']} needs positive assumed_length_mm"
                    )
                hand_interface = attachment.get("hand_interface")
                if hand_interface != "revolute":
                    raise AbstractionError(
                        f"rod attachment {attachment['id']} must use a fixed revolute"
                    )
                if attachment.get("surface") != "distal_phalanx_upper_dorsal":
                    raise AbstractionError(
                        f"rod attachment {attachment['id']} must use upper distal surface"
                    )
                if not math.isclose(
                    float(attachment.get("longitudinal_fraction", math.nan)), 0.5,
                ):
                    raise AbstractionError(
                        f"rod attachment {attachment['id']} must use distal midpoint"
                    )
                if attachment.get("pair_dofs") != ["rotation"]:
                    raise AbstractionError(
                        f"rod attachment {attachment['id']} needs rotation only"
                    )
                if any(key in attachment for key in (
                    "translation_axis", "translation_range_mm", "nominal_translation_mm",
                )):
                    raise AbstractionError(
                        f"fixed rod attachment {attachment['id']} cannot translate"
                    )
                contact = hand_joints[attachment["hand_reference"]]
                if contact.get("kind") != "attachment_revolute":
                    raise AbstractionError(
                        f"rod attachment {attachment['id']} needs attachment_revolute reference"
                    )
                distal = phalanx_rows["distal_phalanx"]
                dip, tip = (
                    hand_joints[node_id]["position_mm"] for node_id in distal["joints"]
                )
                expected = ((dip[0] + tip[0]) / 2.0, (dip[1] + tip[1]) / 2.0)
                if not math.isclose(
                    math.dist(contact["position_mm"], expected), 0.0, abs_tol=1e-6,
                ):
                    raise AbstractionError(
                        f"rod attachment {attachment['id']} must be at upper distal midpoint"
                    )

    analysis = data.get("analysis", {})
    if analysis is not None and not isinstance(analysis, dict):
        raise AbstractionError("analysis must be a mapping")
    sweep = analysis.get("workspace_sweep") if isinstance(analysis, dict) else None
    if sweep is not None:
        if not isinstance(sweep, dict):
            raise AbstractionError("analysis.workspace_sweep must be a mapping")
        q_min, q_max = sweep.get("q_min_deg"), sweep.get("q_max_deg")
        steps = sweep.get("steps")
        tolerance = sweep.get("solver_tolerance_mm")
        iterations = sweep.get("max_iterations")
        if (not all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in (q_min, q_max))
                or q_min >= q_max or q_min > 0 or q_max < 0):
            raise AbstractionError("workspace sweep range must include zero")
        if not isinstance(steps, int) or steps < 3:
            raise AbstractionError("workspace sweep steps must be an integer of at least 3")
        if not isinstance(tolerance, (int, float)) or tolerance <= 0:
            raise AbstractionError("workspace solver tolerance must be positive")
        if not isinstance(iterations, int) or iterations < 1:
            raise AbstractionError("workspace max_iterations must be a positive integer")

    mass_model = data.get("mass_model")
    if mass_model is not None:
        if not isinstance(mass_model, dict):
            raise AbstractionError("mass_model must be a mapping")
        gravity = mass_model.get("gravity_m_s2")
        if not isinstance(gravity, (int, float)) or gravity <= 0:
            raise AbstractionError("mass_model.gravity_m_s2 must be positive")
        seen_mass_bodies: set[str] = set()
        for row in mass_model.get("bodies", []):
            if not isinstance(row, dict) or row.get("body") not in bodies:
                raise AbstractionError("mass_model contains an unknown body")
            body_id = row["body"]
            if body_id in seen_mass_bodies:
                raise AbstractionError(f"mass_model repeats body {body_id}")
            seen_mass_bodies.add(body_id)
            mass = row.get("mass_g")
            if not isinstance(mass, (int, float)) or mass < 0:
                raise AbstractionError(f"mass for body {body_id} must be non-negative")
            weights = row.get("center_node_weights")
            if weights is not None:
                if not isinstance(weights, dict) or not weights:
                    raise AbstractionError(f"body {body_id} centre weights must be a mapping")
                if not set(weights) <= set(bodies[body_id]["nodes"]):
                    raise AbstractionError(f"body {body_id} centre weights reference other nodes")
                if (not all(isinstance(value, (int, float)) and value >= 0
                            for value in weights.values()) or sum(weights.values()) <= 0):
                    raise AbstractionError(f"body {body_id} centre weights must be non-negative")
        seen_point_masses: set[str] = set()
        for row in mass_model.get("point_masses", []):
            if not isinstance(row, dict) or not isinstance(row.get("id"), str):
                raise AbstractionError("each point mass needs an id")
            if row["id"] in seen_point_masses:
                raise AbstractionError(f"duplicate point mass {row['id']}")
            seen_point_masses.add(row["id"])
            if row.get("node") not in nodes:
                raise AbstractionError(f"point mass {row['id']} references an unknown node")
            mass = row.get("mass_g")
            if not isinstance(mass, (int, float)) or mass < 0:
                raise AbstractionError(f"point mass {row['id']} must be non-negative")

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
        if actuator["body"] == actuator["reference_body"]:
            raise AbstractionError(f"actuator {actuator.get('id')} repeats its body")
        joint = actuator["joint"]
        if joint not in bodies[actuator["body"]]["nodes"] or joint not in bodies[actuator["reference_body"]]["nodes"]:
            raise AbstractionError(f"actuator {actuator.get('id')} bodies do not meet at its joint")

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
    if mass_model is not None and mass_model.get("status") != "validated":
        warnings.append(
            f"mass model status is {mass_model.get('status', 'unspecified')}; "
            "torque results are nominal"
        )

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
