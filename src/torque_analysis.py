#!/usr/bin/env python3
"""Compute quasi-static R01 input torque from gravity and moving-part masses."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from plot_linkage import Point, load_parameters
from workspace_sweep import Pose


@dataclass(frozen=True)
class MassParameters:
    """Lumped mass assumptions in grams."""

    link_1_g: float = 5.0
    link_2_g: float = 4.0
    link_3_g: float = 3.0
    moving_joint_g: float = 3.0
    rod_p0_b2_g: float = 1.0
    rod_a1_c3_g: float = 1.0
    tcp_payload_g: float = 0.0
    gravity_m_s2: float = 9.80665


@dataclass(frozen=True)
class TorqueResult:
    q_deg: np.ndarray
    total_nm: np.ndarray
    components_nm: dict[str, np.ndarray]
    potential_j: dict[str, np.ndarray]


def midpoint(a: Point, b: Point) -> Point:
    return (0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]))


def mass_components(pose: Pose, masses: MassParameters) -> dict[str, tuple[float, Point]]:
    """Return component mass in kg and its nominal center of mass in mm."""
    joints = pose.joints
    return {
        "link 1": (masses.link_1_g / 1000.0, midpoint(joints["R01"], joints["R12"])),
        "link 2": (masses.link_2_g / 1000.0, midpoint(joints["R12"], joints["R23"])),
        "link 3": (masses.link_3_g / 1000.0, midpoint(joints["R23"], joints["T3"])),
        "joint R12": (masses.moving_joint_g / 1000.0, joints["R12"]),
        "joint R23": (masses.moving_joint_g / 1000.0, joints["R23"]),
        "joint T3": (masses.moving_joint_g / 1000.0, joints["T3"]),
        "rod P0-B2": (
            masses.rod_p0_b2_g / 1000.0,
            midpoint(joints["P0"], joints["B2"]),
        ),
        "rod A1-C3": (
            masses.rod_a1_c3_g / 1000.0,
            midpoint(joints["A1"], joints["C3"]),
        ),
        "TCP payload": (masses.tcp_payload_g / 1000.0, joints["T3"]),
    }


def analyze_torque(poses: list[Pose], masses: MassParameters) -> TorqueResult:
    """Calculate signed holding torque at R01 using gravitational virtual work."""
    if len(poses) < 3:
        raise ValueError("at least three poses are required for torque differentiation")
    q_deg = np.asarray([pose.q_deg for pose in poses], dtype=float)
    q_rad = np.radians(q_deg)
    if np.any(np.diff(q_rad) <= 0.0):
        raise ValueError("poses must have strictly increasing input angles")

    names = list(mass_components(poses[0], masses))
    potential = {name: np.empty(len(poses), dtype=float) for name in names}
    for index, pose in enumerate(poses):
        for name, (mass_kg, center_mm) in mass_components(pose, masses).items():
            potential[name][index] = mass_kg * masses.gravity_m_s2 * center_mm[1] / 1000.0

    components = {
        name: np.gradient(values, q_rad, edge_order=2)
        for name, values in potential.items()
    }
    total = np.sum(np.vstack(list(components.values())), axis=0)
    return TorqueResult(q_deg, total, components, potential)


def write_torque_csv(path: Path, result: TorqueResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    names = list(result.components_nm)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["q_deg", "input_torque_Nm", "input_torque_mNm",
                         *[f"{name}_Nm" for name in names]])
        for index, q in enumerate(result.q_deg):
            writer.writerow([
                f"{q:.9g}",
                f"{result.total_nm[index]:.12g}",
                f"{1000.0 * result.total_nm[index]:.12g}",
                *[f"{result.components_nm[name][index]:.12g}" for name in names],
            ])


def draw_torque_report(result: TorqueResult, masses: MassParameters):
    figure, (curve_ax, contribution_ax) = plt.subplots(
        1, 2, figsize=(14, 6), gridspec_kw={"width_ratios": (1.7, 1.0)}
    )
    torque_mnm = 1000.0 * result.total_nm
    curve_ax.plot(result.q_deg, torque_mnm, color="black", linewidth=2.8,
                  label="total holding torque")
    styles = ["--", ":", "-."]
    grouped = {
        "links": sum(result.components_nm[name] for name in ("link 1", "link 2", "link 3")),
        "joint hardware": sum(
            result.components_nm[name] for name in ("joint R12", "joint R23", "joint T3")
        ),
        "passive rods": result.components_nm["rod P0-B2"] + result.components_nm["rod A1-C3"],
        "TCP payload": result.components_nm["TCP payload"],
    }
    for (name, values), linestyle in zip(grouped.items(), styles * 2):
        if name == "TCP payload" and masses.tcp_payload_g == 0.0:
            continue
        curve_ax.plot(result.q_deg, 1000.0 * values, color="0.5",
                      linestyle=linestyle, linewidth=1.5, label=name)
    curve_ax.axhline(0.0, color="0.3", linewidth=0.9)
    curve_ax.set_title("Quasi-static R01 holding torque")
    curve_ax.set_xlabel("R01 input q [deg]")
    curve_ax.set_ylabel("torque [mN·m]  (numerically equal to N·mm)")
    curve_ax.grid(True, color="0.9")
    curve_ax.legend(fontsize=9)

    peak_index = int(np.argmax(np.abs(torque_mnm)))
    peak_value = torque_mnm[peak_index]
    curve_ax.scatter(result.q_deg[peak_index], peak_value, color="black", zorder=5)
    curve_ax.annotate(
        f"peak |τ| = {abs(peak_value):.3f} mN·m\nat q={result.q_deg[peak_index]:.1f}°",
        (result.q_deg[peak_index], peak_value), xytext=(12, 12),
        textcoords="offset points", fontsize=9,
        bbox={"boxstyle": "round,pad=0.25", "facecolor": "white", "edgecolor": "0.5"},
    )

    peak_components = {
        name: 1000.0 * float(np.max(np.abs(values)))
        for name, values in grouped.items()
        if name != "TCP payload" or masses.tcp_payload_g > 0.0
    }
    labels = list(peak_components)
    values = [peak_components[name] for name in labels]
    contribution_ax.barh(labels, values, color="0.72", edgecolor="black")
    contribution_ax.set_title("Peak component magnitudes")
    contribution_ax.set_xlabel("peak |torque contribution| [mN·m]")
    contribution_ax.grid(True, axis="x", color="0.9")

    mass_summary = (
        f"Assumed masses\n"
        f"links: {masses.link_1_g:g}/{masses.link_2_g:g}/{masses.link_3_g:g} g\n"
        f"R12/R23/T3 hardware: {masses.moving_joint_g:g} g each\n"
        f"passive rods: {masses.rod_p0_b2_g:g}/{masses.rod_a1_c3_g:g} g\n"
        f"TCP payload: {masses.tcp_payload_g:g} g\n\n"
        f"Signed τ < 0: gravity assists positive-q closure\n"
        f"Signed τ > 0: actuator works against gravity"
    )
    contribution_ax.text(
        0.02, 0.02, mass_summary, transform=contribution_ax.transAxes,
        ha="left", va="bottom", fontsize=9,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "white", "edgecolor": "0.5"},
    )
    figure.suptitle("Finger Linkage Input-Torque Analysis", fontsize=16, fontweight="bold")
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    return figure


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params", type=Path, required=True,
                        help="parameter JSON from an optimization run")
    parser.add_argument("--q-max", type=float, default=90.0)
    parser.add_argument("--samples", type=int, default=901)
    parser.add_argument("--link-masses-g", type=float, nargs=3, metavar=("L1", "L2", "L3"),
                        default=(5.0, 4.0, 3.0))
    parser.add_argument("--joint-mass-g", type=float, default=3.0)
    parser.add_argument("--rod-masses-g", type=float, nargs=2, metavar=("P0_B2", "A1_C3"),
                        default=(1.0, 1.0))
    parser.add_argument("--tcp-payload-g", type=float, default=0.0)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--csv", type=Path, default=None)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def main() -> None:
    # Imported here to keep the reusable torque model independent of the optimizer.
    from optimize_linkage import solve_trajectory

    args = parse_args()
    params = load_parameters(args.params)
    q_values = np.linspace(0.0, args.q_max, args.samples)
    solved = solve_trajectory(params, q_values)
    if solved is None:
        raise RuntimeError("mechanism cannot close over the requested torque sweep")
    poses, _ = solved
    masses = MassParameters(
        link_1_g=args.link_masses_g[0],
        link_2_g=args.link_masses_g[1],
        link_3_g=args.link_masses_g[2],
        moving_joint_g=args.joint_mass_g,
        rod_p0_b2_g=args.rod_masses_g[0],
        rod_a1_c3_g=args.rod_masses_g[1],
        tcp_payload_g=args.tcp_payload_g,
    )
    result = analyze_torque(poses, masses)
    output = args.output or args.params.parent / "torque_report.png"
    csv_path = args.csv or args.params.parent / "torque_samples.csv"
    output.parent.mkdir(parents=True, exist_ok=True)
    report = draw_torque_report(result, masses)
    report.savefig(output, dpi=180, bbox_inches="tight", facecolor="white")
    write_torque_csv(csv_path, result)
    peak = float(np.max(np.abs(result.total_nm)))
    print(f"Wrote {output}")
    print(f"Wrote {csv_path}")
    print(f"Peak quasi-static input torque: {1000.0 * peak:.6g} mN·m")
    if args.no_show:
        plt.close("all")
    else:
        plt.show()


if __name__ == "__main__":
    main()
