#!/usr/bin/env python3
"""Sweep the R01 input and generate an interactive kinematic workspace report."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon

from plot_linkage import (
    JOINT_LABEL_OFFSETS,
    MechanismParameters,
    Point,
    build_geometry,
    load_parameters,
    point_on_link,
    transform_point,
)


@dataclass(frozen=True)
class Pose:
    q_deg: float
    theta_1: float
    theta_2: float
    theta_3: float
    joints: dict[str, Point]


@dataclass(frozen=True)
class ClosureConstants:
    rod_p0_b2: float
    rod_a1_c3: float
    b2_radius: float
    c3_radius: float
    b2_local_angle: float
    c3_local_angle: float
    initial_joints: dict[str, Point]


def circle_intersection(c0: Point, r0: float, c1: Point, r1: float) -> list[Point]:
    """Return the zero, one, or two intersections of two circles."""
    dx, dy = c1[0] - c0[0], c1[1] - c0[1]
    distance = math.hypot(dx, dy)
    tolerance = 1e-10
    if distance < tolerance:
        return []
    if distance > r0 + r1 + tolerance or distance < abs(r0 - r1) - tolerance:
        return []

    along = (r0 * r0 - r1 * r1 + distance * distance) / (2.0 * distance)
    height_sq = max(0.0, r0 * r0 - along * along)
    height = math.sqrt(height_sq)
    ux, uy = dx / distance, dy / distance
    foot = (c0[0] + along * ux, c0[1] + along * uy)
    offset = (-uy * height, ux * height)
    first = (foot[0] + offset[0], foot[1] + offset[1])
    if height <= tolerance:
        return [first]
    return [first, (foot[0] - offset[0], foot[1] - offset[1])]


def nearest(candidates: list[Point], reference: Point) -> Point | None:
    if not candidates:
        return None
    return min(candidates, key=lambda point: math.dist(point, reference))


def closure_constants(params: MechanismParameters) -> ClosureConstants:
    joints, _ = build_geometry(params)
    return ClosureConstants(
        rod_p0_b2=math.dist(joints["P0"], joints["B2"]),
        rod_a1_c3=math.dist(joints["A1"], joints["C3"]),
        b2_radius=math.hypot(*params.b2_on_link_2),
        c3_radius=math.hypot(*params.c3_on_link_3),
        b2_local_angle=math.atan2(params.b2_on_link_2[1], params.b2_on_link_2[0]),
        c3_local_angle=math.atan2(params.c3_on_link_3[1], params.c3_on_link_3[0]),
        initial_joints=joints,
    )


def solve_pose(
    q_deg: float,
    params: MechanismParameters,
    closure: ClosureConstants,
    previous: Pose | None = None,
) -> Pose | None:
    """Solve both passive loops for one R01 input displacement."""
    theta_1 = math.radians(params.link_1_angle_deg + q_deg)
    r01 = params.r01
    p0 = transform_point(r01, 0.0, params.p0_on_base)
    r12 = point_on_link(r01, params.link_1_length, theta_1)
    a1 = transform_point(r01, theta_1, params.a1_on_link_1)

    b2_reference = previous.joints["B2"] if previous else closure.initial_joints["B2"]
    b2 = nearest(
        circle_intersection(r12, closure.b2_radius, p0, closure.rod_p0_b2),
        b2_reference,
    )
    if b2 is None:
        return None
    theta_2 = math.atan2(b2[1] - r12[1], b2[0] - r12[0]) - closure.b2_local_angle
    r23 = point_on_link(r12, params.link_2_length, theta_2)

    c3_reference = previous.joints["C3"] if previous else closure.initial_joints["C3"]
    c3 = nearest(
        circle_intersection(r23, closure.c3_radius, a1, closure.rod_a1_c3),
        c3_reference,
    )
    if c3 is None:
        return None
    theta_3 = math.atan2(c3[1] - r23[1], c3[0] - r23[0]) - closure.c3_local_angle
    t3 = point_on_link(r23, params.link_3_length, theta_3)

    joints = {
        "T3": t3,
        "C3": c3,
        "R23": r23,
        "B2": b2,
        "R12": r12,
        "A1": a1,
        "R01": r01,
        "P0": p0,
    }
    return Pose(q_deg, theta_1, theta_2, theta_3, joints)


def sweep_workspace(
    params: MechanismParameters,
    q_min: float,
    q_max: float,
    steps: int,
) -> tuple[list[Pose], ClosureConstants]:
    """Track the initial assembly branch outward from q=0 in both directions."""
    if q_min >= 0.0 or q_max <= 0.0:
        raise ValueError("the sweep interval must contain q=0 (the initial pose)")
    if steps < 3:
        raise ValueError("steps must be at least 3")

    closure = closure_constants(params)
    initial = solve_pose(0.0, params, closure)
    if initial is None:
        raise RuntimeError("the nominal initial pose does not satisfy loop closure")

    negative_count = min(
        steps - 2,
        max(1, round((steps - 1) * abs(q_min) / (q_max - q_min))),
    )
    positive_count = (steps - 1) - negative_count

    negative: list[Pose] = []
    previous = initial
    for index in range(1, negative_count + 1):
        q = q_min * index / negative_count
        pose = solve_pose(q, params, closure, previous)
        if pose is None:
            break
        negative.append(pose)
        previous = pose

    positive: list[Pose] = []
    previous = initial
    for index in range(1, positive_count + 1):
        q = q_max * index / positive_count
        pose = solve_pose(q, params, closure, previous)
        if pose is None:
            break
        positive.append(pose)
        previous = pose

    return list(reversed(negative)) + [initial] + positive, closure


def draw_pose(
    ax,
    pose: Pose,
    *,
    alpha: float,
    linewidth: float,
    show_moving_joints: bool = False,
    facecolor: str = "0.82",
) -> None:
    joints = pose.joints
    for names in [
        ("T3", "C3", "R23"),
        ("R23", "B2", "R12"),
        ("R12", "A1", "R01"),
    ]:
        ax.add_patch(
            Polygon(
                [joints[name] for name in names], closed=True,
                facecolor=facecolor, edgecolor="black", linewidth=linewidth,
                alpha=alpha, zorder=3,
            )
        )
    for start, end in [("P0", "B2"), ("A1", "C3")]:
        a, b = joints[start], joints[end]
        ax.plot(
            (a[0], b[0]), (a[1], b[1]), color="black",
            linewidth=linewidth, alpha=alpha, zorder=4,
        )
    if show_moving_joints:
        for name in ("T3", "C3", "R23", "B2", "R12", "A1"):
            ax.add_patch(
                Circle(
                    joints[name], radius=0.62, facecolor="white",
                    edgecolor="0.25", linewidth=0.9, alpha=min(1.0, alpha + 0.35),
                    zorder=5,
                )
            )


def annotate_initial_pose(ax, pose: Pose) -> None:
    for name, point in pose.joints.items():
        ax.add_patch(
            Circle(point, radius=1.1, facecolor="white", edgecolor="black",
                   linewidth=1.5, zorder=10)
        )
        ax.annotate(
            name, point, xytext=JOINT_LABEL_OFFSETS[name], textcoords="offset points",
            ha="center", va="center", fontsize=8, fontweight="bold", zorder=11,
        )


def unwrap_degrees(values: list[float]) -> list[float]:
    result = [math.degrees(values[0])]
    for value in values[1:]:
        candidate = math.degrees(value)
        while candidate - result[-1] > 180.0:
            candidate -= 360.0
        while candidate - result[-1] < -180.0:
            candidate += 360.0
        result.append(candidate)
    return result


def wrap_radians(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def relative_joint_flexion(poses: list[Pose]) -> tuple[list[float], list[float]]:
    """Return anatomical PIP/DIP flexion relative to the open pose in degrees."""
    initial_pip = poses[0].theta_2 - poses[0].theta_1
    initial_dip = poses[0].theta_3 - poses[0].theta_2
    pip = [
        math.degrees(wrap_radians((pose.theta_2 - pose.theta_1) - initial_pip))
        for pose in poses
    ]
    dip = [
        math.degrees(wrap_radians((pose.theta_3 - pose.theta_2) - initial_dip))
        for pose in poses
    ]
    return pip, dip


def path_length(points: list[Point]) -> float:
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def representative_pose_indices(poses: list[Pose], count: int = 6) -> list[int]:
    """Choose approximately equal input-angle intervals, always including q=0."""
    if len(poses) == 1:
        return [0]
    count = max(2, min(count, len(poses)))
    q_min, q_max = poses[0].q_deg, poses[-1].q_deg
    targets = [q_min + i * (q_max - q_min) / (count - 1) for i in range(count)]
    targets.append(0.0)
    return sorted({min(range(len(poses)), key=lambda i: abs(poses[i].q_deg - target))
                   for target in targets})


def draw_report(
    poses: list[Pose],
    closure: ClosureConstants,
    params: MechanismParameters,
):
    """Build the workspace, motion-response, and numerical-summary report."""
    figure = plt.figure(figsize=(15, 8.5), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, width_ratios=(1.65, 1.0))
    workspace_ax = figure.add_subplot(grid[:, 0])
    angle_ax = figure.add_subplot(grid[0, 1])
    position_ax = figure.add_subplot(grid[1, 1])

    initial_index = min(range(len(poses)), key=lambda i: abs(poses[i].q_deg))
    initial = poses[initial_index]
    end_index = len(poses) - 1
    end_pose = poses[end_index]
    sample_indices = representative_pose_indices(poses, min(6, len(poses)))
    for index in sample_indices:
        if index not in {initial_index, end_index}:
            pose = poses[index]
            draw_pose(
                workspace_ax, pose, alpha=0.25, linewidth=1.15,
                show_moving_joints=True,
            )
            workspace_ax.annotate(
                f"q={pose.q_deg:.0f}°",
                pose.joints["T3"],
                xytext=(7, 8),
                textcoords="offset points",
                fontsize=8,
                color="0.25",
                bbox={"boxstyle": "round,pad=0.15", "facecolor": "white",
                      "edgecolor": "0.75", "alpha": 0.88},
                zorder=9,
            )
    if end_index != initial_index:
        draw_pose(
            workspace_ax, end_pose, alpha=1.0, linewidth=1.8,
            show_moving_joints=True, facecolor="0.62",
        )
        workspace_ax.annotate(
            f"end q={end_pose.q_deg:.0f}°",
            end_pose.joints["T3"],
            xytext=(7, 8),
            textcoords="offset points",
            fontsize=9,
            fontweight="bold",
            bbox={"boxstyle": "round,pad=0.2", "facecolor": "white",
                  "edgecolor": "0.3"},
            zorder=12,
        )
    draw_pose(workspace_ax, initial, alpha=1.0, linewidth=2.0, facecolor="0.82")

    base_vertex = transform_point(params.r01, 0.0, params.base_vertex_on_base)
    workspace_ax.add_patch(
        Polygon(
            [initial.joints["R01"], initial.joints["P0"], base_vertex],
            closed=True, facecolor="0.72", edgecolor="black", hatch="///",
            linewidth=1.8, zorder=1,
        )
    )

    t3_points = [pose.joints["T3"] for pose in poses]
    r23_points = [pose.joints["R23"] for pose in poses]
    workspace_ax.plot(*zip(*r23_points), color="0.55", linestyle="--",
                      linewidth=1.5, label="R23 locus", zorder=6)
    workspace_ax.plot(*zip(*t3_points), color="black", linewidth=3.0,
                      label="T3 workspace path", zorder=7)
    workspace_ax.plot([], [], color="0.35", linewidth=1.2, alpha=0.5,
                      label="intermediate poses")
    workspace_ax.plot([], [], color="black", linewidth=2.0,
                      label="initial / end poses")
    workspace_ax.scatter(*initial.joints["T3"], s=75, facecolor="white",
                         edgecolor="black", linewidth=1.8, zorder=12)
    annotate_initial_pose(workspace_ax, initial)
    workspace_ax.set_title("Workspace sweep — initial pose at q = 0°")
    workspace_ax.set_xlabel("x [mm]")
    workspace_ax.set_ylabel("y [mm]")
    workspace_ax.set_aspect("equal", adjustable="datalim")
    workspace_ax.grid(True, color="0.9", linewidth=0.7)
    workspace_ax.legend(loc="best")

    q_values = [pose.q_deg for pose in poses]
    pip_flexion, dip_flexion = relative_joint_flexion(poses)
    angle_ax.plot(q_values, pip_flexion,
                  color="0.25", linewidth=2.0, label="PIP flexion")
    angle_ax.plot(q_values, dip_flexion,
                  color="0.60", linewidth=2.0, linestyle="--", label="DIP flexion")
    angle_ax.axvline(0.0, color="black", linewidth=1.0, linestyle=":")
    angle_ax.set_title("Anatomical relative-joint response")
    angle_ax.set_xlabel("R01 input q [deg]")
    angle_ax.set_ylabel("relative flexion [deg]")
    angle_ax.grid(True, color="0.9")
    angle_ax.legend()

    t3_x = [point[0] for point in t3_points]
    t3_y = [point[1] for point in t3_points]
    position_ax.plot(q_values, t3_x, color="0.25", linewidth=2.0, label="T3 x")
    position_ax.plot(q_values, t3_y, color="0.60", linewidth=2.0,
                     linestyle="--", label="T3 y")
    position_ax.axvline(0.0, color="black", linewidth=1.0, linestyle=":")
    position_ax.set_title("Distal-tip position")
    position_ax.set_xlabel("R01 input q [deg]")
    position_ax.set_ylabel("position [mm]")
    position_ax.grid(True, color="0.9")
    position_ax.legend(loc="best")

    summary = (
        f"Feasible branch: {min(q_values):.1f}° to {max(q_values):.1f}°\n"
        f"Samples: {len(poses)} | T3 path length: {path_length(t3_points):.1f} mm\n"
        f"T3 bounds: x [{min(t3_x):.1f}, {max(t3_x):.1f}] mm, "
        f"y [{min(t3_y):.1f}, {max(t3_y):.1f}] mm\n"
        f"Serial lengths: 40 / 24 / 20 mm | "
        f"rods: {closure.rod_p0_b2:.2f} / {closure.rod_a1_c3:.2f} mm"
    )
    figure.suptitle("Finger Exoskeleton Kinematic Workspace Report", fontsize=16,
                    fontweight="bold")
    figure.text(0.5, 0.01, summary, ha="center", va="bottom", fontsize=10,
                bbox={"boxstyle": "round,pad=0.45", "facecolor": "0.96",
                      "edgecolor": "0.45"})
    return figure


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path, default=None,
                        help="mechanism parameter JSON from optimize_linkage.py")
    parser.add_argument("--q-min", type=float, default=-180.0,
                        help="requested negative R01 rotation in degrees")
    parser.add_argument("--q-max", type=float, default=180.0,
                        help="requested positive R01 rotation in degrees")
    parser.add_argument("--steps", type=int, default=1441,
                        help="number of requested sweep samples")
    parser.add_argument("-o", "--output", type=Path,
                        default=Path("runs/nominal/workspace_report.png"),
                        help="report path (default: runs/nominal/workspace_report.png)")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    params = load_parameters(args.params) if args.params else MechanismParameters()
    poses, closure = sweep_workspace(params, args.q_min, args.q_max, args.steps)
    figure = draw_report(poses, closure, params)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, bbox_inches="tight", facecolor="white")
    print(
        f"Wrote {args.output}; solved {len(poses)} poses over "
        f"q=[{poses[0].q_deg:.2f}, {poses[-1].q_deg:.2f}] deg"
    )
    plt.show()


if __name__ == "__main__":
    main()
