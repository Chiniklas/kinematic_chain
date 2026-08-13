#!/usr/bin/env python3
"""Solve and report a YAML-defined planar mechanism workspace sweep."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from collections.abc import Sequence
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import to_rgba
from matplotlib.patches import Circle, Polygon

from mechanism_schema import DEFAULT_ABSTRACTION, l_bracket_segments, load_abstraction
from plot_primitives import draw_l_bracket


Point = tuple[float, float]


@dataclass(frozen=True)
class Pose:
    """One solved assembly configuration."""

    q_deg: float
    positions: dict[str, Point]
    max_residual_mm: float


@dataclass(frozen=True)
class SweepResult:
    """A branch-tracked set of poses and its requested range."""

    poses: tuple[Pose, ...]
    requested_min_deg: float
    requested_max_deg: float
    output_node: str


class KinematicSolveError(RuntimeError):
    """Raised when the declared abstraction cannot be solved."""


def initial_positions(data: dict[str, Any]) -> dict[str, Point]:
    """Return physical initial coordinates, preferring explicit or photo geometry."""
    explicit = {
        node["id"]: node.get("initial_position_mm")
        for node in data["nodes"]
    }
    if all(isinstance(value, list) and len(value) == 2 for value in explicit.values()):
        return {
            node_id: (float(value[0]), float(value[1]))
            for node_id, value in explicit.items()
        }

    calibration = data.get("photo_calibration", {})
    scale = calibration.get("pixels_per_mm")
    pixels = {node["id"]: node.get("photo_pixel") for node in data["nodes"]}
    if (isinstance(scale, (int, float)) and scale > 0
            and all(isinstance(value, list) and len(value) == 2 for value in pixels.values())):
        origin_id = data["actuators"][0]["joint"] if data.get("actuators") else data["nodes"][0]["id"]
        origin = pixels[origin_id]
        return {
            node_id: (
                (float(value[0]) - float(origin[0])) / float(scale),
                -(float(value[1]) - float(origin[1])) / float(scale),
            )
            for node_id, value in pixels.items()
        }

    raise KinematicSolveError(
        "workspace analysis needs initial_position_mm for every node or calibrated photo pixels"
    )


class DistanceConstraintSolver:
    """Generic planar distance-constraint solver with one rotary input."""

    def __init__(self, data: dict[str, Any]):
        self.data = data
        self.nodes = {node["id"]: node for node in data["nodes"]}
        self.initial = initial_positions(data)
        self.fixed = {
            node_id for node_id, node in self.nodes.items() if node.get("fixed", False)
        }
        self.unknown = [node_id for node_id in self.nodes if node_id not in self.fixed]
        self.index = {node_id: 2 * index for index, node_id in enumerate(self.unknown)}
        self.dimensions = [row for row in data["dimensions"] if row.get("value") is not None]

        actuators = data.get("actuators", [])
        if len(actuators) != 1 or actuators[0].get("type") != "rotary":
            raise KinematicSolveError("workspace analysis currently requires one rotary actuator")
        self.actuator = actuators[0]
        self.pivot = self.actuator["joint"]
        if self.pivot not in self.fixed:
            raise KinematicSolveError("the rotary actuator joint must be fixed")
        bodies = {body["id"]: body for body in data["bodies"]}
        driven_body = bodies[self.actuator["body"]]
        driven_nodes = [node_id for node_id in driven_body["nodes"] if node_id != self.pivot]
        if len(driven_nodes) != 1:
            raise KinematicSolveError("the actuated body must currently be a binary link")
        self.driven = driven_nodes[0]
        if self.driven not in self.index:
            raise KinematicSolveError("the driven endpoint must be a moving node")

        pivot = self.initial[self.pivot]
        endpoint = self.initial[self.driven]
        self.initial_angle = math.atan2(endpoint[1] - pivot[1], endpoint[0] - pivot[0])
        dimension = next(
            (row for row in self.dimensions
             if set(row["nodes"]) == {self.pivot, self.driven}),
            None,
        )
        if dimension is None:
            raise KinematicSolveError("the actuated binary link needs a numeric dimension")
        self.crank_length = float(dimension["value"])

        config = data.get("analysis", {}).get("workspace_sweep", {})
        self.tolerance = float(config.get("solver_tolerance_mm", 1e-6))
        self.max_iterations = int(config.get("max_iterations", 80))

    def vector(self, positions: dict[str, Point]) -> np.ndarray:
        return np.asarray(
            [coordinate for node_id in self.unknown for coordinate in positions[node_id]],
            dtype=float,
        )

    def positions(self, vector: np.ndarray) -> dict[str, Point]:
        result = dict(self.initial)
        for node_id, offset in self.index.items():
            result[node_id] = (float(vector[offset]), float(vector[offset + 1]))
        return result

    def _target(self, q_deg: float) -> Point:
        pivot = self.initial[self.pivot]
        sign = -1.0 if self.actuator.get("positive_direction") == "clockwise" else 1.0
        angle = self.initial_angle + sign * math.radians(q_deg)
        return (
            pivot[0] + self.crank_length * math.cos(angle),
            pivot[1] + self.crank_length * math.sin(angle),
        )

    def residual_jacobian(self, vector: np.ndarray, q_deg: float) -> tuple[np.ndarray, np.ndarray]:
        positions = self.positions(vector)
        residuals: list[float] = []
        rows: list[np.ndarray] = []
        width = len(vector)
        for dimension in self.dimensions:
            node_a, node_b = dimension["nodes"]
            if node_a in self.fixed and node_b in self.fixed:
                continue
            a, b = positions[node_a], positions[node_b]
            dx, dy = a[0] - b[0], a[1] - b[1]
            distance = math.hypot(dx, dy)
            if distance < 1e-12:
                distance = 1e-12
            residuals.append(distance - float(dimension["value"]))
            row = np.zeros(width, dtype=float)
            if node_a in self.index:
                offset = self.index[node_a]
                row[offset:offset + 2] = (dx / distance, dy / distance)
            if node_b in self.index:
                offset = self.index[node_b]
                row[offset:offset + 2] = (-dx / distance, -dy / distance)
            rows.append(row)

        target = self._target(q_deg)
        driven_offset = self.index[self.driven]
        for axis in range(2):
            residuals.append(positions[self.driven][axis] - target[axis])
            row = np.zeros(width, dtype=float)
            row[driven_offset + axis] = 1.0
            rows.append(row)
        return np.asarray(residuals), np.vstack(rows)

    def solve(self, q_deg: float, guess: dict[str, Point]) -> Pose | None:
        vector = self.vector(guess)
        damping = 1e-6
        for _ in range(self.max_iterations):
            residual, jacobian = self.residual_jacobian(vector, q_deg)
            maximum = float(np.max(np.abs(residual)))
            if maximum <= self.tolerance:
                return Pose(q_deg, self.positions(vector), maximum)
            normal = jacobian.T @ jacobian + damping * np.eye(len(vector))
            gradient = jacobian.T @ residual
            try:
                step = np.linalg.solve(normal, -gradient)
            except np.linalg.LinAlgError:
                step = np.linalg.lstsq(jacobian, -residual, rcond=None)[0]

            current_norm = float(residual @ residual)
            accepted = False
            scale = 1.0
            for _ in range(12):
                candidate = vector + scale * step
                candidate_residual, _ = self.residual_jacobian(candidate, q_deg)
                if float(candidate_residual @ candidate_residual) < current_norm:
                    vector = candidate
                    damping = max(1e-10, damping * 0.3)
                    accepted = True
                    break
                scale *= 0.5
            if not accepted:
                damping *= 10.0
                if damping > 1e10:
                    return None

        residual, _ = self.residual_jacobian(vector, q_deg)
        maximum = float(np.max(np.abs(residual)))
        if maximum <= max(self.tolerance * 10.0, 1e-5):
            return Pose(q_deg, self.positions(vector), maximum)
        return None


def sweep_workspace(
    data: dict[str, Any],
    q_min: float | None = None,
    q_max: float | None = None,
    steps: int | None = None,
    q_values_deg: Sequence[float] | None = None,
) -> SweepResult:
    """Track the initial assembly branch from q=0 in both directions."""
    if q_values_deg is not None:
        values = np.asarray(q_values_deg, dtype=float)
        if (values.ndim != 1 or len(values) < 3
                or not np.all(np.isfinite(values))
                or not math.isclose(float(values[0]), 0.0, abs_tol=1e-9)
                or bool(np.any(np.diff(values) < -1e-9))):
            raise ValueError(
                "explicit q schedule must contain at least three finite, "
                "nondecreasing values starting at zero"
            )
        solver = DistanceConstraintSolver(data)
        initial = solver.solve(0.0, solver.initial)
        if initial is None:
            raise KinematicSolveError(
                "nominal geometry does not satisfy the distance constraints"
            )
        poses = [initial]
        previous = initial
        for q in values[1:]:
            pose = solver.solve(float(q), previous.positions)
            if pose is None:
                break
            poses.append(pose)
            previous = pose
        outputs = data.get("outputs", [])
        if not outputs:
            raise KinematicSolveError("workspace analysis needs at least one output node")
        return SweepResult(
            tuple(poses), float(values[0]), float(values[-1]), outputs[0]["node"],
        )

    config = data.get("analysis", {}).get("workspace_sweep", {})
    q_min = float(config.get("q_min_deg", 0.0) if q_min is None else q_min)
    q_max = float(config.get("q_max_deg", 90.0) if q_max is None else q_max)
    steps = int(config.get("steps", 181) if steps is None else steps)
    if q_min >= q_max or q_min > 0 or q_max < 0 or steps < 3:
        raise ValueError("workspace range must include zero and use at least three steps")

    solver = DistanceConstraintSolver(data)
    initial = solver.solve(0.0, solver.initial)
    if initial is None:
        raise KinematicSolveError("nominal geometry does not satisfy the distance constraints")

    negative_count = (
        round((steps - 1) * abs(q_min) / (q_max - q_min)) if q_min < 0 else 0
    )
    positive_count = steps - 1 - negative_count
    negative: list[Pose] = []
    previous = initial
    for q in np.linspace(0.0, q_min, negative_count + 1)[1:] if negative_count else []:
        pose = solver.solve(float(q), previous.positions)
        if pose is None:
            break
        negative.append(pose)
        previous = pose
    positive: list[Pose] = []
    previous = initial
    for q in np.linspace(0.0, q_max, positive_count + 1)[1:] if positive_count else []:
        pose = solver.solve(float(q), previous.positions)
        if pose is None:
            break
        positive.append(pose)
        previous = pose

    outputs = data.get("outputs", [])
    if not outputs:
        raise KinematicSolveError("workspace analysis needs at least one output node")
    poses = tuple(list(reversed(negative)) + [initial] + positive)
    return SweepResult(poses, q_min, q_max, outputs[0]["node"])


def _draw_pose(axes, data: dict[str, Any], pose: Pose, alpha: float, linewidth: float) -> None:
    for body in data["bodies"]:
        points = [pose.positions[node_id] for node_id in body["nodes"]]
        color = body.get("color", "#334155")
        bracket = l_bracket_segments(body, pose.positions)
        if bracket is not None:
            draw_l_bracket(
                axes,
                bracket,
                color,
                linewidth=max(2.8, linewidth * 2.5)
                * float(body.get("render_flesh_scale", 1.0)),
                alpha=alpha,
                zorder=3,
            )
        elif len(points) == 2:
            axes.plot(*zip(*points), color=color, alpha=alpha, linewidth=linewidth,
                      solid_capstyle="round", zorder=3)
        else:
            axes.add_patch(Polygon(
                points, closed=True, facecolor=to_rgba(color, 0.10 * alpha),
                edgecolor=to_rgba(color, alpha), linewidth=linewidth, zorder=2,
            ))


def draw_workspace_report(data: dict[str, Any], result: SweepResult):
    """Build a workspace, output-coordinate, and closure-quality report."""
    poses = result.poses
    figure = plt.figure(figsize=(15, 8.5), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, width_ratios=(1.6, 1.0))
    workspace_ax = figure.add_subplot(grid[:, 0])
    coordinate_ax = figure.add_subplot(grid[0, 1])
    residual_ax = figure.add_subplot(grid[1, 1])

    sample_indices = sorted(set(np.linspace(0, len(poses) - 1, min(7, len(poses)), dtype=int)))
    initial_index = min(range(len(poses)), key=lambda index: abs(poses[index].q_deg))
    for index in sample_indices:
        pose = poses[index]
        is_initial = index == initial_index
        _draw_pose(workspace_ax, data, pose, 1.0 if is_initial else 0.24,
                   2.2 if is_initial else 1.0)

    output_points = [pose.positions[result.output_node] for pose in poses]
    workspace_ax.plot(*zip(*output_points), color="#111827", linewidth=2.8,
                      label=f"{result.output_node} workspace path", zorder=8)
    for index in (0, initial_index, len(poses) - 1):
        pose = poses[index]
        point = pose.positions[result.output_node]
        workspace_ax.add_patch(Circle(point, 0.75, facecolor="white", edgecolor="black",
                                      linewidth=1.4, zorder=10))
        workspace_ax.annotate(f"q={pose.q_deg:.1f}°", point, xytext=(7, 8),
                              textcoords="offset points", fontsize=8, zorder=11)
    workspace_ax.set_title("Nominal workspace and representative poses")
    workspace_ax.set_xlabel("x [mm]")
    workspace_ax.set_ylabel("y [mm]")
    workspace_ax.set_aspect("equal", adjustable="datalim")
    workspace_ax.grid(True, color="0.9")
    workspace_ax.legend(loc="best")

    q_values = np.asarray([pose.q_deg for pose in poses])
    x_values = np.asarray([point[0] for point in output_points])
    y_values = np.asarray([point[1] for point in output_points])
    coordinate_ax.plot(q_values, x_values, label=f"{result.output_node} x", linewidth=2)
    coordinate_ax.plot(q_values, y_values, label=f"{result.output_node} y", linewidth=2)
    coordinate_ax.axvline(0, color="0.25", linestyle=":")
    coordinate_ax.set_title("Output position")
    coordinate_ax.set_xlabel("input q [deg]")
    coordinate_ax.set_ylabel("position [mm]")
    coordinate_ax.grid(True, color="0.9")
    coordinate_ax.legend()

    residuals = np.asarray([pose.max_residual_mm for pose in poses])
    residual_ax.semilogy(q_values, np.maximum(residuals, 1e-12), color="#7c3aed", linewidth=2)
    residual_ax.axvline(0, color="0.25", linestyle=":")
    residual_ax.set_title("Maximum distance-constraint residual")
    residual_ax.set_xlabel("input q [deg]")
    residual_ax.set_ylabel("residual [mm]")
    residual_ax.grid(True, color="0.9")

    for axes in (coordinate_ax, residual_ax):
        axes.set_xlim(result.requested_min_deg, result.requested_max_deg)
        if poses[0].q_deg > result.requested_min_deg:
            axes.axvspan(result.requested_min_deg, poses[0].q_deg,
                         color="#dc2626", alpha=0.08)
        if poses[-1].q_deg < result.requested_max_deg:
            axes.axvspan(poses[-1].q_deg, result.requested_max_deg,
                         color="#dc2626", alpha=0.08)

    path_length = sum(math.dist(a, b) for a, b in zip(output_points, output_points[1:]))
    mechanism = data["mechanism"]
    figure.suptitle(f"{mechanism['name']} — nominal workspace sweep", fontsize=16,
                    fontweight="bold")
    workspace_ax.text(
        0.02, 0.02,
        f"Solved q=[{poses[0].q_deg:.1f}°, {poses[-1].q_deg:.1f}°] "
        f"of requested [{result.requested_min_deg:.1f}°, {result.requested_max_deg:.1f}°] "
        f"\n{len(poses)} feasible poses · output path {path_length:.2f} mm"
        "\nPhoto-derived nominal geometry",
        transform=workspace_ax.transAxes, ha="left", va="bottom", fontsize=9,
        color="#475569",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "0.6"},
    )
    return figure


def write_workspace_csv(path: Path, data: dict[str, Any], result: SweepResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    node_ids = [node["id"] for node in data["nodes"]]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["q_deg", "max_residual_mm", *[
            column for node_id in node_ids for column in (f"{node_id}_x_mm", f"{node_id}_y_mm")
        ]])
        for pose in result.poses:
            writer.writerow([
                f"{pose.q_deg:.10g}", f"{pose.max_residual_mm:.10g}",
                *[f"{coordinate:.10g}" for node_id in node_ids for coordinate in pose.positions[node_id]],
            ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abstraction", type=Path, nargs="?", default=DEFAULT_ABSTRACTION)
    parser.add_argument("--q-min", type=float, default=None)
    parser.add_argument("--q-max", type=float, default=None)
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = load_abstraction(args.abstraction)
    result = sweep_workspace(data, args.q_min, args.q_max, args.steps)
    mechanism_id = data["mechanism"]["id"]
    output = args.output or Path("runs") / mechanism_id / "workspace_report.png"
    csv_path = args.csv or output.with_name("workspace_samples.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure = draw_workspace_report(data, result)
    figure.savefig(output, dpi=180, bbox_inches="tight", facecolor="white")
    write_workspace_csv(csv_path, data, result)
    print(f"Wrote {output}")
    print(f"Wrote {csv_path}")
    print(
        f"Solved {len(result.poses)} poses over "
        f"q=[{result.poses[0].q_deg:.3g}, {result.poses[-1].q_deg:.3g}] deg"
    )
    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
