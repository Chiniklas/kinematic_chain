#!/usr/bin/env python3
"""Compute nominal quasi-static input torque for a YAML-defined mechanism."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np

from mechanism_schema import DEFAULT_ABSTRACTION, load_abstraction
from workspace_sweep import Pose, SweepResult, sweep_workspace


@dataclass(frozen=True)
class TorqueResult:
    q_deg: np.ndarray
    total_nm: np.ndarray
    components_nm: dict[str, np.ndarray]
    model_status: str


class MassModelError(ValueError):
    """Raised when torque analysis has no coherent mass model."""


def _weighted_center(pose: Pose, weights: dict[str, float]) -> tuple[float, float]:
    total = sum(weights.values())
    if total <= 0:
        raise MassModelError("centre weights must have a positive sum")
    return (
        sum(weight * pose.positions[node_id][0] for node_id, weight in weights.items()) / total,
        sum(weight * pose.positions[node_id][1] for node_id, weight in weights.items()) / total,
    )


def mass_components(
    data: dict[str, Any], pose: Pose
) -> dict[str, tuple[float, tuple[float, float]]]:
    """Return named mass components as ``(kg, centre_mm)``."""
    model = data.get("mass_model")
    if not isinstance(model, dict):
        raise MassModelError("torque analysis requires a mass_model table")
    bodies = {body["id"]: body for body in data["bodies"]}
    result: dict[str, tuple[float, tuple[float, float]]] = {}
    for row in model.get("bodies", []):
        body_id = row["body"]
        body = bodies[body_id]
        mass_g = float(row["mass_g"])
        if mass_g <= 0:
            continue
        weights = row.get("center_node_weights")
        if weights is None:
            weights = {node_id: 1.0 for node_id in body["nodes"]}
        result[f"body:{body_id}"] = (
            mass_g / 1000.0,
            _weighted_center(pose, {key: float(value) for key, value in weights.items()}),
        )
    for row in model.get("point_masses", []):
        mass_g = float(row["mass_g"])
        if mass_g <= 0:
            continue
        result[f"point:{row['id']}"] = (
            mass_g / 1000.0,
            pose.positions[row["node"]],
        )
    if not result or sum(mass for mass, _ in result.values()) <= 0:
        raise MassModelError("mass_model must contain at least one positive mass")
    return result


def total_mass_g(data: dict[str, Any]) -> float:
    model = data.get("mass_model", {})
    return sum(float(row["mass_g"]) for row in model.get("bodies", [])) + sum(
        float(row["mass_g"]) for row in model.get("point_masses", [])
    )


def analyze_torque(data: dict[str, Any], sweep: SweepResult) -> TorqueResult:
    """Calculate signed holding torque from gravitational virtual work."""
    if len(sweep.poses) < 3:
        raise ValueError("torque differentiation requires at least three poses")
    model = data.get("mass_model", {})
    gravity = float(model.get("gravity_m_s2", 9.80665))
    q_deg = np.asarray([pose.q_deg for pose in sweep.poses], dtype=float)
    q_rad = np.radians(q_deg)
    if np.any(np.diff(q_rad) <= 0):
        raise ValueError("workspace poses must use strictly increasing input angles")

    first = mass_components(data, sweep.poses[0])
    potential = {name: np.empty(len(sweep.poses), dtype=float) for name in first}
    for index, pose in enumerate(sweep.poses):
        for name, (mass_kg, center_mm) in mass_components(data, pose).items():
            potential[name][index] = mass_kg * gravity * center_mm[1] / 1000.0
    edge_order = 2 if len(sweep.poses) >= 3 else 1
    components = {
        name: np.gradient(values, q_rad, edge_order=edge_order)
        for name, values in potential.items()
    }
    total = np.sum(np.vstack(list(components.values())), axis=0)
    return TorqueResult(
        q_deg=q_deg,
        total_nm=total,
        components_nm=components,
        model_status=str(model.get("status", "unspecified")),
    )


def draw_torque_report(data: dict[str, Any], result: TorqueResult):
    figure, (curve_ax, contribution_ax) = plt.subplots(
        1, 2, figsize=(14, 6.5), gridspec_kw={"width_ratios": (1.7, 1.0)},
    )
    total_mnm = 1000.0 * result.total_nm
    curve_ax.plot(result.q_deg, total_mnm, color="black", linewidth=2.8,
                  label="total holding torque")
    colors = plt.cm.tab10(np.linspace(0, 1, max(1, len(result.components_nm))))
    for color, (name, values) in zip(colors, result.components_nm.items()):
        curve_ax.plot(result.q_deg, 1000.0 * values, linewidth=1.2,
                      alpha=0.75, color=color, label=name)
    curve_ax.axhline(0, color="0.35", linewidth=0.8)
    curve_ax.axvline(0, color="0.35", linewidth=0.8, linestyle=":")
    curve_ax.set_title("Quasi-static gravitational holding torque")
    curve_ax.set_xlabel("input q [deg]")
    curve_ax.set_ylabel("torque [mN·m]")
    curve_ax.grid(True, color="0.9")
    curve_ax.legend(fontsize=7, ncol=2)

    peak_index = int(np.argmax(np.abs(total_mnm)))
    curve_ax.scatter(result.q_deg[peak_index], total_mnm[peak_index], color="black", zorder=5)
    curve_ax.annotate(
        f"peak |τ|={abs(total_mnm[peak_index]):.3f} mN·m\n"
        f"q={result.q_deg[peak_index]:.1f}°",
        (result.q_deg[peak_index], total_mnm[peak_index]),
        xytext=(10, -42), textcoords="offset points", fontsize=9,
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "0.5"},
    )

    names = list(result.components_nm)
    peaks = [1000.0 * float(np.max(np.abs(result.components_nm[name]))) for name in names]
    contribution_ax.barh(names, peaks, color="0.72", edgecolor="black")
    contribution_ax.set_title("Peak component magnitudes")
    contribution_ax.set_xlabel("peak |torque| [mN·m]")
    contribution_ax.grid(True, axis="x", color="0.9")
    moving_mass_g = total_mass_g(data)
    contribution_ax.text(
        0.02, 0.02,
        f"Mass model: {result.model_status}\nTotal moving mass: {moving_mass_g:.1f} g\n"
        "Positive torque: actuator holds against +q gravity load\n"
        "Photo geometry and nominal masses: not a rated-load result",
        transform=contribution_ax.transAxes, ha="left", va="bottom", fontsize=8.5,
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "0.5"},
    )
    figure.suptitle(
        f"{data['mechanism']['name']} — nominal input-torque analysis",
        fontsize=16, fontweight="bold", y=0.98,
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.93))
    return figure


def write_torque_csv(path: Path, result: TorqueResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    names = list(result.components_nm)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["q_deg", "input_torque_Nm", "input_torque_mNm",
                         *[f"{name}_Nm" for name in names]])
        for index, q_deg in enumerate(result.q_deg):
            writer.writerow([
                f"{q_deg:.10g}", f"{result.total_nm[index]:.12g}",
                f"{1000.0 * result.total_nm[index]:.12g}",
                *[f"{result.components_nm[name][index]:.12g}" for name in names],
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
    sweep = sweep_workspace(data, args.q_min, args.q_max, args.steps)
    result = analyze_torque(data, sweep)
    mechanism_id = data["mechanism"]["id"]
    output = args.output or Path("runs") / mechanism_id / "torque_report.png"
    csv_path = args.csv or output.with_name("torque_samples.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    figure = draw_torque_report(data, result)
    figure.savefig(output, dpi=180, bbox_inches="tight", facecolor="white")
    write_torque_csv(csv_path, result)
    print(f"Wrote {output}")
    print(f"Wrote {csv_path}")
    print(f"Peak nominal |input torque|: {1000.0 * np.max(np.abs(result.total_nm)):.6g} mN·m")
    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
