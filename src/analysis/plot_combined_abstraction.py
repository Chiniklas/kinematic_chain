#!/usr/bin/env python3
"""Render the mechanism together with its nominal human-hand attachments."""

from __future__ import annotations

import argparse
import copy
import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import yaml
from matplotlib.colors import to_rgba
from matplotlib.patches import Circle, FancyBboxPatch, Polygon
from matplotlib.transforms import Affine2D

from mechanism_schema import DEFAULT_ABSTRACTION, load_abstraction, node_layout


Point = tuple[float, float]


def apply_finger_objective(
    data: dict[str, Any], objective: dict[str, Any], source_path: Path | None = None,
) -> dict[str, Any]:
    """Return a combined model sized for one finger objective."""
    result = copy.deepcopy(data)
    hand = result["human_hand_model"]
    objective_meta = objective.get("objective", {})
    finger = objective_meta.get("finger")
    lengths = objective.get("phalanx_lengths_mm")
    if not isinstance(finger, str) or not isinstance(lengths, dict):
        raise ValueError("finger objective needs objective.finger and phalanx_lengths_mm")
    hand["id"] = f"nominal_{finger}_finger"
    hand["reference_finger"] = finger
    if source_path is not None:
        hand["source_objective"] = str(source_path)

    segment_ids = ("proximal_phalanx", "middle_phalanx", "distal_phalanx")
    length_ids = ("proximal", "middle", "distal")
    phalanges = {row["id"]: row for row in hand["phalanges"]}
    for segment_id, length_id in zip(segment_ids, length_ids):
        phalanges[segment_id]["length_mm"] = float(lengths[length_id])

    flexion = hand["nominal_flexion_deg"]
    headings = [
        -math.radians(float(flexion["mcp"])),
        -math.radians(float(flexion["mcp"]) + float(flexion["pip"])),
        -math.radians(
            float(flexion["mcp"]) + float(flexion["pip"]) + float(flexion["dip"])
        ),
    ]
    segment_lengths = [float(lengths[length_id]) for length_id in length_ids]
    points: list[Point] = [(0.0, 0.0)]
    for length, heading in zip(segment_lengths, headings):
        points.append((
            points[-1][0] + length * math.cos(heading),
            points[-1][1] + length * math.sin(heading),
        ))
    upper_midpoint = (
        (points[2][0] + points[3][0]) / 2,
        (points[2][1] + points[3][1]) / 2,
    )
    distal_width = float(phalanges["distal_phalanx"]["width_mm"])
    distal_heading = headings[2]
    dorsal_normal = (-math.sin(distal_heading), math.cos(distal_heading))
    lower_midpoint = (
        upper_midpoint[0] - distal_width * dorsal_normal[0],
        upper_midpoint[1] - distal_width * dorsal_normal[1],
    )
    positions = {
        "hand_wrist_dorsal": (-float(hand["palm"]["length_mm"]), 0.0),
        "hand_mcp": points[0],
        "hand_pip": points[1],
        "hand_dip": points[2],
        "hand_distal_slot_midpoint": lower_midpoint,
        "hand_tip": points[3],
    }
    for joint in hand["joints"]:
        joint["position_mm"] = [float(value) for value in positions[joint["id"]]]
    output_rod = _attachment_by_id(result, "distal_output_rod")
    distal_length = float(lengths["distal"])
    output_rod["translation_range_mm"] = [0.0, distal_length]
    output_rod["nominal_translation_mm"] = distal_length / 2.0
    return result


def load_finger_objective(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as stream:
        objective = yaml.safe_load(stream)
    if not isinstance(objective, dict) or objective.get("schema_version") != 1:
        raise ValueError(f"invalid finger objective: {path}")
    return objective


def _centroid(points: list[Point]) -> Point:
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def _attachment_by_id(data: dict[str, Any], attachment_id: str) -> dict[str, Any]:
    for attachment in data.get("exoskeleton_attachments", []):
        if attachment.get("id") == attachment_id:
            return attachment
    raise ValueError(f"missing exoskeleton attachment {attachment_id}")


def _horizontal_slot_mount_transform(
    source_a: Point,
    source_d: Point,
    source_h: Point,
    target_d: Point,
    slot_start: Point,
    slot_end: Point,
    rod_length: float,
):
    """Place horizontal AD and select a lower-surface slot coordinate for the rod."""
    ad_dx = source_d[0] - source_a[0]
    ad_dy = source_d[1] - source_a[1]
    if math.hypot(ad_dx, ad_dy) == 0:
        raise ValueError("combined abstraction needs distinct A and D nodes")
    angle = -math.atan2(ad_dy, ad_dx)
    cosine, sine = math.cos(angle), math.sin(angle)

    def rotate_about_d(point: Point) -> Point:
        x = point[0] - source_d[0]
        y = point[1] - source_d[1]
        return (cosine * x - sine * y, sine * x + cosine * y)

    h_vector = rotate_about_d(source_h)
    quadratic_a = h_vector[0] ** 2 + h_vector[1] ** 2
    slot_length = math.dist(slot_start, slot_end)
    candidates: list[tuple[float, float, float, Point]] = []
    closest: list[tuple[float, float, float, Point]] = []
    for fraction in [index / 400 for index in range(401)]:
        slot_point = (
            slot_start[0] + fraction * (slot_end[0] - slot_start[0]),
            slot_start[1] + fraction * (slot_end[1] - slot_start[1]),
        )
        delta = (target_d[0] - slot_point[0], target_d[1] - slot_point[1])
        quadratic_b = 2 * (h_vector[0] * delta[0] + h_vector[1] * delta[1])
        quadratic_c = delta[0] ** 2 + delta[1] ** 2 - rod_length ** 2
        discriminant = quadratic_b ** 2 - 4 * quadratic_a * quadratic_c
        if discriminant >= 0:
            root = math.sqrt(discriminant)
            for scale in (
                (-quadratic_b - root) / (2 * quadratic_a),
                (-quadratic_b + root) / (2 * quadratic_a),
            ):
                if scale > 0:
                    candidates.append((abs(fraction - 0.5), abs(scale - 14.0), scale, slot_point))
        projection = (
            (slot_point[0] - target_d[0]) * h_vector[0]
            + (slot_point[1] - target_d[1]) * h_vector[1]
        ) / quadratic_a
        if projection > 0:
            projected_h = (
                target_d[0] + projection * h_vector[0],
                target_d[1] + projection * h_vector[1],
            )
            closest.append((
                math.dist(projected_h, slot_point), abs(fraction - 0.5),
                projection, slot_point,
            ))
    if candidates:
        _, _, scale, slot_point = min(candidates)
        connector_length = rod_length
        closure_feasible = True
    elif closest:
        connector_length, _, scale, slot_point = min(closest)
        closure_feasible = False
    else:
        raise ValueError("horizontal AD placement has no positive-scale assembly")

    def transform(point: Point) -> Point:
        x, y = rotate_about_d(point)
        return (
            target_d[0] + scale * x,
            target_d[1] + scale * y,
        )

    return (
        transform,
        transform(source_h),
        slot_point,
        connector_length,
        closure_feasible,
        math.dist(slot_start, slot_point) if slot_length else 0.0,
    )


def _lower_distal_surface(
    hand: dict[str, Any], hand_joints: dict[str, Point],
) -> tuple[Point, Point]:
    distal = next(row for row in hand["phalanges"] if row["id"] == "distal_phalanx")
    start, end = (hand_joints[node_id] for node_id in distal["joints"])
    length = math.dist(start, end)
    dorsal_normal = (-(end[1] - start[1]) / length, (end[0] - start[0]) / length)
    width = float(distal["width_mm"])
    return (
        (start[0] - width * dorsal_normal[0], start[1] - width * dorsal_normal[1]),
        (end[0] - width * dorsal_normal[0], end[1] - width * dorsal_normal[1]),
    )


def draw_combined_abstraction(data: dict[str, Any]):
    hand = data.get("human_hand_model")
    if not isinstance(hand, dict):
        raise ValueError("combined abstraction requires human_hand_model")
    hand_joint_rows = {row["id"]: row for row in hand.get("joints", [])}
    hand_joints = {
        row["id"]: (float(row["position_mm"][0]), float(row["position_mm"][1]))
        for row in hand.get("joints", [])
    }
    input_mount = _attachment_by_id(data, "dorsal_input_mount")
    output_rod = _attachment_by_id(data, "distal_output_rod")
    input_reference = hand_joints[input_mount["hand_reference"]]
    dorsal_clearance = float(input_mount["dorsal_clearance_mm"])
    target_input = (
        input_reference[0],
        input_reference[1] + dorsal_clearance,
    )
    rod_length = float(output_rod["assumed_length_mm"])
    slot_start, slot_end = _lower_distal_surface(hand, hand_joints)

    diagram_positions = node_layout(data)
    (
        transform, target_h, hand_attachment, connector_length,
        rod_closure_feasible, slider_translation,
    ) = _horizontal_slot_mount_transform(
        diagram_positions["a"],
        diagram_positions[input_mount["mechanism_node"]],
        diagram_positions[output_rod["mechanism_node"]],
        target_input,
        slot_start,
        slot_end,
        rod_length,
    )
    mechanism_positions = {
        node_id: transform(point) for node_id, point in diagram_positions.items()
    }
    node_rows = {node["id"]: node for node in data["nodes"]}

    figure, axes = plt.subplots(figsize=(15, 7.5), constrained_layout=True)
    figure.patch.set_facecolor("white")
    axes.set_facecolor("#f8fafc")

    hand_colors = ["#f8d5bb", "#f6c7a6", "#f3b98f", "#efa876"]
    hand_segments = [hand["palm"], *hand["phalanges"]]
    for index, segment in enumerate(hand_segments):
        start, end = (hand_joints[node_id] for node_id in segment["joints"])
        length = float(segment["length_mm"])
        width = float(segment["width_mm"])
        if not math.isclose(math.dist(start, end), length, abs_tol=1e-6):
            raise ValueError(f"hand segment {segment['id']} length does not match joints")
        angle_deg = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
        patch = FancyBboxPatch(
            (0, -width),
            length,
            width,
            boxstyle=f"round,pad=0,rounding_size={width / 2}",
            facecolor=hand_colors[index % len(hand_colors)],
            edgecolor="#9a3412",
            linewidth=2.2,
            alpha=0.88,
            zorder=1,
        )
        patch.set_transform(
            Affine2D().rotate_deg(angle_deg).translate(*start) + axes.transData
        )
        axes.add_patch(patch)
        normal = (
            -(end[1] - start[1]) / length,
            (end[0] - start[0]) / length,
        )
        axes.text(
            (start[0] + end[0]) / 2 - normal[0] * (width + 3.0),
            (start[1] + end[1]) / 2 - normal[1] * (width + 3.0),
            f"{segment['id']}\n{length:g} × {width:g} mm",
            ha="center",
            va="top",
            fontsize=8.5,
            color="#7c2d12",
        )

    dorsal_surface_ids = [hand["palm"]["joints"][0]] + [
        hand["palm"]["joints"][1]
    ] + [
        phalanx["joints"][1] for phalanx in hand["phalanges"]
    ]
    dorsal_surface = [hand_joints[node_id] for node_id in dorsal_surface_ids]
    axes.plot(
        *zip(*dorsal_surface),
        color="#9a3412",
        linewidth=2.2,
        solid_capstyle="round",
        solid_joinstyle="round",
        zorder=5,
    )
    axes.plot(
        [slot_start[0], slot_end[0]],
        [slot_start[1], slot_end[1]],
        color="#7c3aed",
        linewidth=4.0,
        solid_capstyle="round",
        zorder=6,
    )
    axes.text(
        (slot_start[0] + slot_end[0]) / 2,
        (slot_start[1] + slot_end[1]) / 2 + 3.0,
        f"RP4 slot 0–{math.dist(slot_start, slot_end):g} mm",
        ha="center",
        va="bottom",
        fontsize=8,
        color="#6d28d9",
        fontweight="bold",
    )

    for joint_id, point in hand_joints.items():
        if hand_joint_rows[joint_id].get("kind") == "attachment_slot_reference":
            continue
        axes.add_patch(Circle(
            point,
            radius=1.8,
            facecolor="white",
            edgecolor="#7c2d12",
            linewidth=2,
            zorder=6,
        ))
        node_index = hand_joint_rows[joint_id].get("node_index")
        if joint_id != output_rod["hand_reference"]:
            axes.text(
                point[0], point[1] - 12.5, joint_id,
                ha="center", va="top", fontsize=8, color="#7c2d12",
            )

    for body in sorted(data["bodies"], key=lambda row: row.get("draw_order", 2)):
        points = [mechanism_positions[node_id] for node_id in body["nodes"]]
        color = body.get("color", "#334155")
        if len(points) == 2:
            axes.plot(
                *zip(*points), color=color, linewidth=5.5,
                solid_capstyle="round", zorder=10,
            )
        else:
            axes.add_patch(Polygon(
                points,
                closed=True,
                facecolor=to_rgba(color, 0.14),
                edgecolor=color,
                linewidth=3.5,
                hatch="////" if body.get("kind") == "ground" else None,
                joinstyle="round",
                zorder=8,
            ))
        label = _centroid(points)
        axes.text(
            *label,
            body["id"],
            ha="center",
            va="center",
            fontsize=8,
            color=color,
            fontweight="bold",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.75, "pad": 1},
            zorder=14,
        )

    for node_id, point in mechanism_positions.items():
        is_reference = node_rows[node_id].get("kind") == "reference"
        axes.scatter(
            *point,
            s=100,
            marker="s" if is_reference else "o",
            facecolor="#fee2e2" if is_reference else "white",
            edgecolor="black",
            linewidth=1.8,
            zorder=18,
        )
        axes.annotate(
            node_id, point, xytext=(0, 11), textcoords="offset points",
            ha="center", va="bottom", fontsize=10, fontweight="bold", zorder=19,
        )

    rod_color = "#7c3aed" if rod_closure_feasible else "#dc2626"
    axes.plot(
        [target_h[0], hand_attachment[0]],
        [target_h[1], hand_attachment[1]],
        color=rod_color,
        linewidth=5,
        linestyle="-" if rod_closure_feasible else "--",
        solid_capstyle="round",
        zorder=16,
    )
    axes.add_patch(Circle(
        hand_attachment,
        radius=2.0,
        facecolor="white",
        edgecolor=rod_color,
        linewidth=2.2,
        zorder=24,
    ))
    rod_label = (
        f"output rod\n{rod_length:g} mm · s={slider_translation:.1f} mm"
        if rod_closure_feasible
        else f"{rod_length:g} mm rod infeasible\nminimum here: {connector_length:.1f} mm"
    )
    axes.text(
        (target_h[0] + hand_attachment[0]) / 2 + 3.0,
        (target_h[1] + hand_attachment[1]) / 2 + 3.0,
        rod_label,
        ha="left",
        va="center",
        fontsize=8.5,
        color=rod_color,
        fontweight="bold",
    )
    axes.annotate(
        "RP4: rotating pin translating on lower-surface slot",
        xy=hand_attachment,
        xytext=(slot_end[0] + 10, slot_end[1] - 17),
        arrowprops={"arrowstyle": "->", "color": rod_color, "linewidth": 1.5},
        fontsize=9,
        color=rod_color,
    )
    axes.annotate(
        f"{input_mount['mechanism_node'].upper()}: {dorsal_clearance:g} mm manual clearance above J1",
        xy=target_input,
        xytext=(target_input[0] - 7, target_input[1] + 31),
        arrowprops={"arrowstyle": "->", "color": "#7c3aed", "linewidth": 1.5},
        fontsize=9,
        color="#6d28d9",
    )
    axes.plot(
        [input_reference[0], target_input[0]],
        [input_reference[1], target_input[1]],
        color="#7c3aed",
        linestyle="--",
        linewidth=1.6,
        zorder=7,
    )
    axes.text(
        input_reference[0] + 2.0,
        (input_reference[1] + target_input[1]) / 2,
        f"c_DJ1 = {dorsal_clearance:g} mm",
        ha="left",
        va="center",
        fontsize=8,
        color="#6d28d9",
        fontweight="bold",
        zorder=20,
    )

    badge_offsets = {1: (-17, -20), 2: (0, -21), 3: (0, -21), 4: (0, -21)}
    for joint_id, row in hand_joint_rows.items():
        node_index = row.get("node_index")
        if not isinstance(node_index, int):
            continue
        axes.annotate(
            f"J{node_index}",
            xy=hand_joints[joint_id],
            xytext=badge_offsets.get(node_index, (0, -20)),
            textcoords="offset points",
            ha="center",
            va="center",
            fontsize=8,
            fontweight="bold",
            color="white",
            bbox={
                "boxstyle": "circle,pad=0.28",
                "facecolor": "#7c2d12",
                "edgecolor": "white",
                "linewidth": 1.2,
            },
            arrowprops={"arrowstyle": "-", "color": "#7c2d12", "linewidth": 1.0},
            zorder=30,
        )
    axes.annotate(
        "RP4",
        xy=hand_attachment,
        xytext=(-12, -20),
        textcoords="offset points",
        ha="center",
        va="center",
        fontsize=7.5,
        fontweight="bold",
        color="white",
        bbox={
            "boxstyle": "circle,pad=0.24",
            "facecolor": "#6d28d9",
            "edgecolor": "white",
            "linewidth": 1.2,
        },
        arrowprops={"arrowstyle": "-", "color": "#6d28d9", "linewidth": 1.0},
        zorder=31,
    )

    all_points = [
        *hand_joints.values(), *mechanism_positions.values(), target_input, target_h,
        slot_start, slot_end, hand_attachment,
    ]
    xs, ys = zip(*all_points)
    axes.set_xlim(min(xs) - 18, max(xs) + 36)
    axes.set_ylim(min(ys) - 27, max(ys) + 42)
    axes.set_aspect("equal", adjustable="box")
    axes.axis("off")
    axes.set_title(
        f"Mechanism 2 + mildly curled nominal {hand['reference_finger']}-finger abstraction",
        fontsize=16,
        fontweight="bold",
        pad=16,
    )
    figure.text(
        0.5,
        0.02,
        "Hand dimensions and attachment offsets are in mm · mechanism placement is a diagrammatic similarity transform, not a physical-scale pose",
        ha="center",
        fontsize=9.5,
        color="#475569",
    )
    return figure, axes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abstraction", type=Path, nargs="?", default=DEFAULT_ABSTRACTION)
    parser.add_argument("--finger-objective", type=Path, default=None)
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = load_abstraction(args.abstraction)
    if args.finger_objective is not None:
        objective_path = args.finger_objective.resolve()
        data = apply_finger_objective(
            data, load_finger_objective(objective_path), objective_path,
        )
    output = args.output or Path("runs") / data["mechanism"]["id"] / "combined_abstraction.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, _ = draw_combined_abstraction(data)
    figure.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    print(f"Wrote {output}")
    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
