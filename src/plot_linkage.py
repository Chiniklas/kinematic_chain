#!/usr/bin/env python3
"""Interactively draw the parameterized finger-exoskeleton linkage graph."""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from typing import TypeAlias

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon


Point: TypeAlias = tuple[float, float]


@dataclass(frozen=True)
class MechanismParameters:
    """Nominal mechanism geometry in millimetres and degrees."""

    # Serial-chain origin and invariant link lengths.
    r01: Point = (0.0, 0.0)
    link_1_length: float = 40.0       # R01 -> R12
    link_2_length: float = 24.0       # R12 -> R23
    link_3_length: float = 20.0       # R23 -> T3

    # Absolute link angles, measured counter-clockwise from +x.
    link_1_angle_deg: float = -179.657
    link_2_angle_deg: float = 180.0
    link_3_angle_deg: float = 178.254

    # Auxiliary points in their owning body's local frame.
    # Link-local +x follows that link from proximal to distal joint.
    a1_on_link_1: Point = (34.783, 8.774)
    b2_on_link_2: Point = (4.192, -10.179)
    c3_on_link_3: Point = (3.774, -9.829)

    # Base points are offsets from R01 in the fixed world frame.
    p0_on_base: Point = (13.892, 14.251)
    base_vertex_on_base: Point = (37.604, -19.521)


@dataclass(frozen=True)
class PlotParameters:
    """All display parameters, independent of mechanism geometry."""

    figure_size: tuple[float, float] = (13.0, 4.8)
    joint_radius_mm: float = 1.25
    body_gray: str = "0.82"
    base_gray: str = "0.72"
    edge_color: str = "black"
    edge_width: float = 2.0
    rod_width: float = 2.5
    membership_width: float = 1.4
    joint_label_size: float = 10.0
    body_label_size: float = 11.0
    rod_label_size: float = 10.0
    margin_fraction: float = 0.06


POINT_PARAMETER_NAMES = {
    "r01",
    "a1_on_link_1",
    "b2_on_link_2",
    "c3_on_link_3",
    "p0_on_base",
    "base_vertex_on_base",
}


def load_parameters(path: Path) -> MechanismParameters:
    """Load mechanism parameters from an optimizer-generated JSON file."""
    data = json.loads(path.read_text(encoding="utf-8"))
    if "mechanism_parameters" in data:
        data = data["mechanism_parameters"]
    valid_names = set(asdict(MechanismParameters()))
    unknown = set(data) - valid_names
    if unknown:
        raise ValueError(f"unknown mechanism parameters: {sorted(unknown)}")
    values = {
        name: tuple(value) if name in POINT_PARAMETER_NAMES else value
        for name, value in data.items()
    }
    return MechanismParameters(**values)


def save_parameters(path: Path, params: MechanismParameters, **metadata) -> None:
    """Save parameters with optional optimization metadata."""
    payload = {"mechanism_parameters": asdict(params), **metadata}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


RIGID_BODIES = [
    (("T3", "C3", "R23"), "link 3"),
    (("R23", "B2", "R12"), "link 2"),
    (("R12", "A1", "R01"), "link 1"),
]

PASSIVE_RODS = [
    ("B2", "P0", "rod P0-B2", (0.0, 3.0)),
    ("A1", "C3", "rod A1-C3", (0.0, -3.0)),
]

MEMBERSHIP_LINES = [("A1", "R12"), ("B2", "R12"), ("C3", "R23")]

# Label displacement is in screen points so it remains readable at any scale.
JOINT_LABEL_OFFSETS = {
    "T3": (-2, -22),
    "C3": (-4, 14),
    "R23": (-5, -23),
    "B2": (0, 14),
    "R12": (2, -23),
    "A1": (0, -24),
    "R01": (-2, -23),
    "P0": (0, 14),
}


def transform_point(origin: Point, theta: float, local_point: Point) -> Point:
    """Transform a body-local point to world coordinates; ``theta`` is radians."""
    c, s = math.cos(theta), math.sin(theta)
    x, y = local_point
    return origin[0] + c * x - s * y, origin[1] + s * x + c * y


def point_on_link(origin: Point, length: float, theta: float) -> Point:
    """Return the distal point of a link with an exact prescribed length."""
    return transform_point(origin, theta, (length, 0.0))


def midpoint(a: Point, b: Point, offset: Point = (0.0, 0.0)) -> Point:
    return ((a[0] + b[0]) / 2 + offset[0], (a[1] + b[1]) / 2 + offset[1])


def centroid(points: list[Point]) -> Point:
    count = len(points)
    return sum(p[0] for p in points) / count, sum(p[1] for p in points) / count


def build_geometry(params: MechanismParameters) -> tuple[dict[str, Point], Point]:
    """Derive every world-space point from the parameter set."""
    theta_1 = math.radians(params.link_1_angle_deg)
    theta_2 = math.radians(params.link_2_angle_deg)
    theta_3 = math.radians(params.link_3_angle_deg)

    r01 = params.r01
    r12 = point_on_link(r01, params.link_1_length, theta_1)
    r23 = point_on_link(r12, params.link_2_length, theta_2)
    t3 = point_on_link(r23, params.link_3_length, theta_3)

    joints = {
        "T3": t3,
        "C3": transform_point(r23, theta_3, params.c3_on_link_3),
        "R23": r23,
        "B2": transform_point(r12, theta_2, params.b2_on_link_2),
        "R12": r12,
        "A1": transform_point(r01, theta_1, params.a1_on_link_1),
        "R01": r01,
        "P0": transform_point(r01, 0.0, params.p0_on_base),
    }
    base_vertex = transform_point(r01, 0.0, params.base_vertex_on_base)
    return joints, base_vertex


def validate_lengths(joints: dict[str, Point], params: MechanismParameters) -> None:
    """Fail early if a future edit violates a prescribed serial-link length."""
    prescribed = [
        ("R01", "R12", params.link_1_length),
        ("R12", "R23", params.link_2_length),
        ("R23", "T3", params.link_3_length),
    ]
    for start, end, expected in prescribed:
        actual = math.dist(joints[start], joints[end])
        if not math.isclose(actual, expected, rel_tol=1e-12, abs_tol=1e-12):
            raise ValueError(f"{start}-{end} is {actual:.9g} mm, expected {expected:g} mm")


def draw_link_polygon(ax, points: list[Point], style: PlotParameters) -> None:
    ax.add_patch(
        Polygon(
            points,
            closed=True,
            facecolor=style.body_gray,
            edgecolor=style.edge_color,
            linewidth=style.edge_width,
            joinstyle="round",
            zorder=2,
        )
    )


def draw_mechanism(
    params: MechanismParameters = MechanismParameters(),
    style: PlotParameters = PlotParameters(),
):
    """Create and return ``(figure, axes)`` for the parameterized mechanism."""
    joints, base_vertex = build_geometry(params)
    validate_lengths(joints, params)
    fig, ax = plt.subplots(figsize=style.figure_size, constrained_layout=True)

    for names, body_name in RIGID_BODIES:
        points = [joints[name] for name in names]
        draw_link_polygon(ax, points, style)
        ax.text(
            *centroid(points), body_name, ha="center", va="center",
            fontsize=style.body_label_size, fontweight="bold", zorder=6,
        )

    base_points = [joints["R01"], joints["P0"], base_vertex]
    ax.add_patch(
        Polygon(
            base_points, closed=True, facecolor=style.base_gray,
            edgecolor=style.edge_color, linewidth=style.edge_width, hatch="///",
            joinstyle="round", zorder=1,
        )
    )
    ax.text(
        *centroid(base_points), "Base", ha="center", va="center",
        fontsize=style.body_label_size, fontweight="bold", zorder=6,
    )

    for start, end, rod_name, label_offset in PASSIVE_RODS:
        a, b = joints[start], joints[end]
        ax.plot(
            (a[0], b[0]), (a[1], b[1]), color=style.edge_color,
            linewidth=style.rod_width, solid_capstyle="round", zorder=4,
        )
        ax.text(
            *midpoint(a, b, label_offset), rod_name, ha="center", va="center",
            fontsize=style.rod_label_size,
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 1.5},
            zorder=7,
        )

    for start, end in MEMBERSHIP_LINES:
        a, b = joints[start], joints[end]
        ax.plot(
            (a[0], b[0]), (a[1], b[1]), color="0.25",
            linewidth=style.membership_width, linestyle=(0, (4, 3)), zorder=5,
        )

    for name, point in joints.items():
        ax.add_patch(
            Circle(
                point, radius=style.joint_radius_mm, facecolor="white",
                edgecolor=style.edge_color, linewidth=style.edge_width, zorder=10,
            )
        )
        ax.annotate(
            name, point, xytext=JOINT_LABEL_OFFSETS[name], textcoords="offset points",
            ha="center", va="center", fontsize=style.joint_label_size,
            fontweight="bold", zorder=11,
        )

    all_points = list(joints.values()) + [base_vertex]
    xs, ys = zip(*all_points)
    width, height = max(xs) - min(xs), max(ys) - min(ys)
    margin = style.margin_fraction * max(width, height)
    arrow_y = min(ys) - 0.35 * margin
    ax.annotate(
        "transmission: Base → 1 → 2 → 3",
        xy=(min(xs) + margin, arrow_y),
        xytext=(max(xs) - 2 * margin, arrow_y),
        arrowprops={"arrowstyle": "-|>", "linewidth": 1.8, "color": style.edge_color},
        ha="center", va="center", fontsize=style.rod_label_size, zorder=12,
    )

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(ys) - 1.1 * margin, max(ys) + margin)
    ax.axis("off")
    fig.patch.set_facecolor("white")
    ax.set_facecolor("white")
    return fig, ax


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path, default=None,
                        help="mechanism parameter JSON from optimize_linkage.py")
    parser.add_argument("--l1", type=float, default=None,
                        help="R01-R12 length in mm (default: 40)")
    parser.add_argument("--l2", type=float, default=None,
                        help="R12-R23 length in mm (default: 24)")
    parser.add_argument("--l3", type=float, default=None,
                        help="R23-T3 length in mm (default: 20)")
    parser.add_argument("--theta1", type=float, default=None)
    parser.add_argument("--theta2", type=float, default=None)
    parser.add_argument("--theta3", type=float, default=None)
    parser.add_argument("-o", "--output", type=Path, default=None,
                        help="also save the figure to this path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base = load_parameters(args.params) if args.params else MechanismParameters()
    params = replace(
        base,
        link_1_length=args.l1 if args.l1 is not None else base.link_1_length,
        link_2_length=args.l2 if args.l2 is not None else base.link_2_length,
        link_3_length=args.l3 if args.l3 is not None else base.link_3_length,
        link_1_angle_deg=(args.theta1 if args.theta1 is not None
                          else base.link_1_angle_deg),
        link_2_angle_deg=(args.theta2 if args.theta2 is not None
                          else base.link_2_angle_deg),
        link_3_angle_deg=(args.theta3 if args.theta3 is not None
                          else base.link_3_angle_deg),
    )
    fig, _ = draw_mechanism(params)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=220, bbox_inches="tight", facecolor="white")
        print(f"Wrote {args.output}")
    plt.show()


if __name__ == "__main__":
    main()
