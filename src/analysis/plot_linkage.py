#!/usr/bin/env python3
"""Draw any planar mechanism that follows the generic abstraction YAML schema."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
from matplotlib.colors import to_rgba
from matplotlib.patches import Arc, Circle, Polygon

from mechanism_schema import (
    DEFAULT_ABSTRACTION,
    load_abstraction,
    l_bracket_segments,
    node_layout,
    summary_lines,
    validate_abstraction,
)
from plot_primitives import draw_l_bracket


Point = tuple[float, float]


def centroid(points: list[Point]) -> Point:
    return (
        sum(point[0] for point in points) / len(points),
        sum(point[1] for point in points) / len(points),
    )


def _normal_offset(a: Point, b: Point, amount: float) -> Point:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length == 0.0:
        return 0.0, 0.0
    return -amount * dy / length, amount * dx / length


def _dimension_label(dimension: dict[str, Any], default_units: str) -> str:
    """Format a compact two-line dimension annotation."""
    value = dimension.get("value")
    units = dimension.get("units", default_units)
    if value is None:
        value_text = "not set"
    else:
        value_text = f"{value:g}"
        if units:
            value_text += f" {units}"
    return f"{dimension['id']}\n{value_text}"


def _dimension_label_position(
    a: Point,
    b: Point,
    body_points: list[Point],
    offset: float = 0.30,
) -> Point:
    """Place a dimension outside a rigid body or opposite a binary body label."""
    midpoint = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
    if len(body_points) == 2:
        dx, dy = _normal_offset(a, b, -offset)
        return midpoint[0] + dx, midpoint[1] + dy

    body_center = centroid(body_points)
    dx, dy = midpoint[0] - body_center[0], midpoint[1] - body_center[1]
    distance = math.hypot(dx, dy)
    if distance == 0:
        dx, dy = _normal_offset(a, b, offset)
    else:
        dx, dy = offset * dx / distance, offset * dy / distance
    return midpoint[0] + dx, midpoint[1] + dy


def draw_abstraction(data: dict[str, Any]):
    """Return ``(figure, axes)`` for a validated abstraction document."""
    summary = validate_abstraction(data)
    positions = node_layout(data)
    nodes = {node["id"]: node for node in data["nodes"]}
    bodies = sorted(data["bodies"], key=lambda body: body.get("draw_order", 2))
    bodies_by_id = {body["id"]: body for body in data["bodies"]}

    figure, axes = plt.subplots(figsize=(14, 7.5), constrained_layout=True)
    axes.set_facecolor("white")
    figure.patch.set_facecolor("white")

    for body in bodies:
        points = [positions[node_id] for node_id in body["nodes"]]
        color = body.get("color", "#334155")
        kind = body.get("kind")
        if kind == "ground":
            if len(points) == 2:
                a, b = points
                axes.plot(*zip(a, b), color=color, linewidth=12,
                          solid_capstyle="round", zorder=1)
                nx, ny = _normal_offset(a, b, -0.18)
                ground_polygon = [
                    (a[0] + nx, a[1] + ny),
                    (b[0] + nx, b[1] + ny),
                    (b[0] + 2.2 * nx, b[1] + 2.2 * ny),
                    (a[0] + 2.2 * nx, a[1] + 2.2 * ny),
                ]
            else:
                ground_polygon = points
            axes.add_patch(Polygon(
                ground_polygon, closed=True, facecolor=to_rgba(color, 0.12),
                edgecolor=color, linewidth=5 if len(points) > 2 else 1.2,
                hatch="////", joinstyle="round", zorder=0,
            ))
        elif l_bracket_segments(body, positions) is not None:
            draw_l_bracket(
                axes,
                l_bracket_segments(body, positions) or (),
                color,
                linewidth=7 * float(body.get("render_flesh_scale", 1.0)),
                zorder=body.get("draw_order", 2) + 1,
            )
        elif len(points) == 2:
            axes.plot(*zip(*points), color=color, linewidth=7,
                      solid_capstyle="round", zorder=body.get("draw_order", 2) + 1)
        else:
            axes.add_patch(Polygon(
                points, closed=True, facecolor=to_rgba(color, 0.16),
                edgecolor=color, linewidth=5, joinstyle="round",
                zorder=body.get("draw_order", 2) + 1,
            ))

        label_at = centroid(points)
        if len(points) == 2:
            dx, dy = _normal_offset(points[0], points[1], 0.18)
            label_at = label_at[0] + dx, label_at[1] + dy
        axes.text(
            *label_at, body["id"], ha="center", va="center", fontsize=8,
            color=color, fontweight="bold",
            bbox={"facecolor": "white", "edgecolor": "none", "alpha": 0.78, "pad": 1},
            zorder=15,
        )

    default_units = data.get("mechanism", {}).get("dimension_units", "")
    for dimension in data.get("dimensions", []):
        node_a, node_b = dimension["nodes"]
        a, b = positions[node_a], positions[node_b]
        body = bodies_by_id[dimension["body"]]
        body_points = [positions[node_id] for node_id in body["nodes"]]
        label_at = _dimension_label_position(a, b, body_points)
        axes.text(
            *label_at,
            _dimension_label(dimension, default_units),
            ha="center",
            va="center",
            fontsize=7.2,
            color="#0f172a",
            linespacing=0.92,
            bbox={
                "boxstyle": "round,pad=0.18",
                "facecolor": "white",
                "edgecolor": body.get("color", "#64748b"),
                "linewidth": 0.8,
                "alpha": 0.92,
            },
            zorder=18,
        )

    for node_id, point in positions.items():
        node = nodes[node_id]
        if node.get("kind") == "reference":
            axes.scatter(*point, s=130, marker="s", facecolor="#fee2e2",
                         edgecolor="black", linewidth=2.0, zorder=20)
        else:
            axes.add_patch(Circle(
                point, radius=0.115, facecolor="white", edgecolor="black",
                linewidth=2.2, zorder=20,
            ))
        axes.annotate(
            node_id, point, xytext=(0, 15), textcoords="offset points",
            ha="center", va="bottom", fontsize=12, fontweight="bold", zorder=21,
        )

    for actuator in data.get("actuators", []):
        center = positions[actuator["joint"]]
        radius = 0.48
        direction = actuator.get("positive_direction")
        theta1, theta2 = (30, 300) if direction == "clockwise" else (30, 300)
        axes.add_patch(Arc(center, 2 * radius, 2 * radius, theta1=theta1, theta2=theta2,
                           color="#7c3aed", linewidth=2.5, zorder=25))
        angle = math.radians(theta2)
        arrow_tip = center[0] + radius * math.cos(angle), center[1] + radius * math.sin(angle)
        tangent = (-math.sin(angle), math.cos(angle))
        if direction == "clockwise":
            tangent = (-tangent[0], -tangent[1])
        axes.annotate(
            "", xy=arrow_tip,
            xytext=(arrow_tip[0] - 0.28 * tangent[0], arrow_tip[1] - 0.28 * tangent[1]),
            arrowprops={"arrowstyle": "-|>", "color": "#7c3aed", "linewidth": 2.5},
            zorder=26,
        )
        axes.text(center[0] - 0.52, center[1] + 0.55, actuator["id"],
                  color="#7c3aed", fontsize=10, fontweight="bold")

    all_points = list(positions.values())
    xs, ys = zip(*all_points)
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    margin = max(0.6, 0.07 * span)
    axes.set_xlim(min(xs) - margin, max(xs) + margin)
    axes.set_ylim(min(ys) - margin, max(ys) + margin)
    axes.set_aspect("equal", adjustable="box")
    axes.axis("off")

    mechanism = data["mechanism"]
    axes.set_title(mechanism["name"], fontsize=17, fontweight="bold", pad=18)
    figure.text(
        0.5, 0.025,
        f"{summary.node_count} nodes · {summary.body_count} bodies · "
        f"mobility from current incidence table: {summary.planar_mobility} · "
        f"status: {mechanism.get('status', 'unspecified')} · layout not to scale",
        ha="center", fontsize=10, color="#475569",
    )
    return figure, axes


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abstraction", type=Path, nargs="?", default=DEFAULT_ABSTRACTION,
                        help="mechanism abstraction YAML")
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="output image (default: runs/<mechanism-id>/abstraction.png)")
    parser.add_argument("--show", action="store_true", help="open the interactive window")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = load_abstraction(args.abstraction)
    summary = validate_abstraction(data)
    for line in summary_lines(summary):
        print(line)
    output = args.output or Path("runs") / summary.mechanism_id / "abstraction.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, _ = draw_abstraction(data)
    figure.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    print(f"Wrote {output}")
    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
