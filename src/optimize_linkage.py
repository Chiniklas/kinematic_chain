#!/usr/bin/env python3
"""Optimize passive-link pivot locations for monotonic biomimetic finger closure."""

from __future__ import annotations

import argparse
import math
from dataclasses import replace
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import differential_evolution, minimize

from plot_linkage import MechanismParameters, Point, save_parameters
from torque_analysis import MassParameters, analyze_torque, draw_torque_report, write_torque_csv
from workspace_sweep import (
    ClosureConstants,
    Pose,
    closure_constants,
    draw_pose,
    draw_report,
    relative_joint_flexion,
    solve_pose,
)


# Variables: P0(x,y), A1(x,y), B2(x,y), C3(x,y), all in their owning-body
# coordinate frames. Bounds keep pivots on realistically sized triangular plates.
BOUNDS = [
    (-5.0, 35.0), (-30.0, 30.0),    # P0 on Base
    (2.0, 38.0), (-20.0, 20.0),     # A1 on link 1
    (2.0, 22.0), (-25.0, 25.0),     # B2 on link 2
    (2.0, 18.0), (-20.0, 20.0),     # C3 on link 3
]


def vector_to_parameters(values: np.ndarray, base: MechanismParameters) -> MechanismParameters:
    """Apply an optimizer vector without changing the 40/24/20 mm phalanges."""
    return replace(
        base,
        p0_on_base=(float(values[0]), float(values[1])),
        a1_on_link_1=(float(values[2]), float(values[3])),
        b2_on_link_2=(float(values[4]), float(values[5])),
        c3_on_link_3=(float(values[6]), float(values[7])),
    )


def parameters_to_vector(params: MechanismParameters) -> np.ndarray:
    return np.array([
        *params.p0_on_base,
        *params.a1_on_link_1,
        *params.b2_on_link_2,
        *params.c3_on_link_3,
    ], dtype=float)


def solve_trajectory(
    params: MechanismParameters,
    q_values: np.ndarray,
) -> tuple[list[Pose], ClosureConstants] | None:
    """Solve a continuous assembly branch for increasing R01 input."""
    closure = closure_constants(params)
    poses: list[Pose] = []
    previous = None
    for q in q_values:
        pose = solve_pose(float(q), params, closure, previous)
        if pose is None:
            return None
        poses.append(pose)
        previous = pose
    return poses, closure


def relative_flexion(poses: list[Pose]) -> tuple[np.ndarray, np.ndarray]:
    """Return PIP and DIP flexion relative to the open pose, in degrees."""
    pip, dip = relative_joint_flexion(poses)
    return np.asarray(pip), np.asarray(dip)


def cross(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-16:
        return math.dist(point, start)
    projection = ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq
    projection = min(1.0, max(0.0, projection))
    closest = (start[0] + projection * dx, start[1] + projection * dy)
    return math.dist(point, closest)


def segment_distance(a: Point, b: Point, c: Point, d: Point) -> float:
    """Minimum distance between two 2D line segments; zero when they cross."""
    ab_c, ab_d = cross(a, b, c), cross(a, b, d)
    cd_a, cd_b = cross(c, d, a), cross(c, d, b)
    if ab_c * ab_d <= 0.0 and cd_a * cd_b <= 0.0:
        return 0.0
    return min(
        point_segment_distance(a, c, d),
        point_segment_distance(b, c, d),
        point_segment_distance(c, a, b),
        point_segment_distance(d, a, b),
    )


def transmission_sine(pivot: Point, coupler_joint: Point, rod_end: Point) -> float:
    """Absolute sine of the angle between a driven link and its passive rod."""
    ax, ay = coupler_joint[0] - pivot[0], coupler_joint[1] - pivot[1]
    bx, by = rod_end[0] - coupler_joint[0], rod_end[1] - coupler_joint[1]
    denominator = math.hypot(ax, ay) * math.hypot(bx, by)
    return abs(ax * by - ay * bx) / denominator if denominator > 1e-12 else 0.0


def minimum_transmission_sine(poses: list[Pose]) -> float:
    values = []
    for pose in poses:
        joints = pose.joints
        values.extend([
            transmission_sine(joints["R12"], joints["B2"], joints["P0"]),
            transmission_sine(joints["R23"], joints["C3"], joints["A1"]),
        ])
    return min(values)


class BiomimeticObjective:
    """Callable nonlinear objective with target and constraint diagnostics."""

    def __init__(
        self,
        base: MechanismParameters,
        q_values: np.ndarray,
        pip_ratio: float,
        dip_ratio: float,
        monotonic_weight: float = 100.0,
    ) -> None:
        self.base = base
        self.q_values = q_values
        self.target_pip = pip_ratio * q_values
        self.target_dip = dip_ratio * self.target_pip
        self.monotonic_weight = monotonic_weight
        self.reference = parameters_to_vector(base)
        self.evaluations = 0

    def __call__(self, values: np.ndarray) -> float:
        self.evaluations += 1
        params = vector_to_parameters(values, self.base)
        result = solve_trajectory(params, self.q_values)
        if result is None:
            return 1.0e6
        poses, closure = result
        pip, dip = relative_flexion(poses)

        # Tracking errors are normalized by 10 degrees.
        tracking = np.mean(
            ((pip - self.target_pip) / 10.0) ** 2
            + ((dip - self.target_dip) / 10.0) ** 2
        )

        # Reject local reopening even if the sampled end pose is close to target.
        pip_reverse = np.minimum(np.diff(pip), 0.0)
        dip_reverse = np.minimum(np.diff(dip), 0.0)
        monotonic = self.monotonic_weight * np.sum(pip_reverse**2 + dip_reverse**2)

        # Anatomical hinge constraints: neither joint may bend dorsally, and the
        # rigid model is restricted to typical flexion ranges.
        anatomical = 200.0 * np.sum(
            np.minimum(pip, 0.0) ** 2
            + np.minimum(dip, 0.0) ** 2
            + np.maximum(pip - 110.0, 0.0) ** 2
            + np.maximum(dip - 90.0, 0.0) ** 2
        )

        # Link 3 may approach link 1 during closure but must never pass through
        # it. A 2 mm centerline clearance is used as a soft collision boundary.
        collision = 0.0
        for pose in poses[1:]:
            joints = pose.joints
            clearance = segment_distance(
                joints["R01"], joints["R12"], joints["R23"], joints["T3"]
            )
            collision += 500.0 * max(0.0, 2.0 - clearance) ** 2

        # Keep away from toggle positions: sin(15 degrees) is the soft threshold.
        minimum_sine = minimum_transmission_sine(poses)
        transmission = 250.0 * max(0.0, math.sin(math.radians(15.0)) - minimum_sine) ** 2

        # Weak regularization prevents solutions from needlessly sitting on bounds.
        spans = np.array([high - low for low, high in BOUNDS])
        regularization = 0.015 * np.mean(((values - self.reference) / spans) ** 2)

        # Avoid zero-area ternary links and ill-conditioned pivots on the
        # phalange centerlines. Indices are the transverse local coordinates.
        transverse_offsets = values[[3, 5, 7]]
        pivot_offset_penalty = 100.0 * np.sum(
            np.maximum(2.0 - np.abs(transverse_offsets), 0.0) ** 2
        )

        # Very long rods are mechanically unattractive even when kinematically valid.
        rod_penalty = (
            max(0.0, closure.rod_p0_b2 - 80.0) ** 2
            + max(0.0, closure.rod_a1_c3 - 65.0) ** 2
        ) * 0.01
        return float(
            tracking + monotonic + anatomical + collision
            + transmission + regularization + pivot_offset_penalty + rod_penalty
        )


def trajectory_metrics(
    poses: list[Pose],
    closure: ClosureConstants,
    objective: BiomimeticObjective,
) -> dict[str, float | int]:
    pip, dip = relative_flexion(poses)
    return {
        "pip_rmse_deg": float(np.sqrt(np.mean((pip - objective.target_pip) ** 2))),
        "dip_rmse_deg": float(np.sqrt(np.mean((dip - objective.target_dip) ** 2))),
        "pip_final_deg": float(pip[-1]),
        "dip_final_deg": float(dip[-1]),
        # Changes below 0.05 degree are below the reporting resolution.
        "pip_reverse_steps": int(np.count_nonzero(np.diff(pip) < -0.05)),
        "dip_reverse_steps": int(np.count_nonzero(np.diff(dip) < -0.05)),
        "minimum_transmission_angle_deg": float(
            math.degrees(math.asin(minimum_transmission_sine(poses)))
        ),
        "minimum_link1_link3_clearance_mm": float(min(
            segment_distance(
                pose.joints["R01"], pose.joints["R12"],
                pose.joints["R23"], pose.joints["T3"],
            )
            for pose in poses[1:]
        )),
        "rod_p0_b2_mm": float(closure.rod_p0_b2),
        "rod_a1_c3_mm": float(closure.rod_a1_c3),
    }


def dense_validation(
    poses: list[Pose],
    closure: ClosureConstants,
) -> dict[str, float | int | bool]:
    """Validate constraints on a finer grid than the optimization samples."""
    pip, dip = relative_flexion(poses)
    clearances = [
        segment_distance(
            pose.joints["R01"], pose.joints["R12"],
            pose.joints["R23"], pose.joints["T3"],
        )
        for pose in poses[1:]
    ]
    rod_1_errors = [
        abs(math.dist(pose.joints["P0"], pose.joints["B2"]) - closure.rod_p0_b2)
        for pose in poses
    ]
    rod_2_errors = [
        abs(math.dist(pose.joints["A1"], pose.joints["C3"]) - closure.rod_a1_c3)
        for pose in poses
    ]
    return {
        "samples": len(poses),
        "pip_min_increment_deg": float(np.min(np.diff(pip))),
        "dip_min_increment_deg": float(np.min(np.diff(dip))),
        "minimum_link1_link3_clearance_mm": float(min(clearances)),
        "maximum_rod_length_error_mm": float(max(rod_1_errors + rod_2_errors)),
        "monotonic": bool(np.all(np.diff(pip) >= 0.0) and np.all(np.diff(dip) >= 0.0)),
        "collision_free_centerlines": bool(min(clearances) >= 2.0),
    }


def draw_optimization_report(
    q_values: np.ndarray,
    objective: BiomimeticObjective,
    baseline_poses: list[Pose],
    optimized_poses: list[Pose],
    baseline_params: MechanismParameters,
    optimized_params: MechanismParameters,
    baseline_score: float,
    optimized_score: float,
):
    figure, (angle_ax, mechanism_ax) = plt.subplots(
        1, 2, figsize=(15, 6.5),
        gridspec_kw={"width_ratios": (1.0, 1.35)},
    )
    baseline_pip, baseline_dip = relative_flexion(baseline_poses)
    optimized_pip, optimized_dip = relative_flexion(optimized_poses)

    angle_ax.plot(q_values, objective.target_pip, color="black", linewidth=2.5,
                  label="PIP target")
    angle_ax.plot(q_values, objective.target_dip, color="0.45", linewidth=2.5,
                  label="DIP target")
    angle_ax.plot(q_values, baseline_pip, color="black", linestyle=":",
                  linewidth=1.6, label="PIP before")
    angle_ax.plot(q_values, baseline_dip, color="0.55", linestyle=":",
                  linewidth=1.6, label="DIP before")
    angle_ax.plot(q_values, optimized_pip, color="black", linestyle="--",
                  linewidth=2.1, label="PIP optimized")
    angle_ax.plot(q_values, optimized_dip, color="0.55", linestyle="--",
                  linewidth=2.1, label="DIP optimized")
    angle_ax.set_title("Relative joint-flexion synthesis")
    angle_ax.set_xlabel("R01 input q [deg]")
    angle_ax.set_ylabel("relative flexion [deg]")
    angle_ax.grid(True, color="0.9")
    angle_ax.legend(ncol=2, fontsize=9)

    sample_indices = np.linspace(0, len(optimized_poses) - 1, 6).round().astype(int)
    for index in sample_indices:
        pose = optimized_poses[index]
        draw_pose(mechanism_ax, pose, alpha=0.22, linewidth=1.0,
                  show_moving_joints=True)
        mechanism_ax.annotate(
            f"{pose.q_deg:.0f}°", pose.joints["T3"], xytext=(5, 5),
            textcoords="offset points", fontsize=8,
        )
    draw_pose(mechanism_ax, optimized_poses[0], alpha=0.9, linewidth=1.8)
    mechanism_ax.set_title("Optimized closure sequence")
    mechanism_ax.set_xlabel("x [mm]")
    mechanism_ax.set_ylabel("y [mm]")
    mechanism_ax.set_aspect("equal", adjustable="datalim")
    mechanism_ax.grid(True, color="0.9")

    figure.suptitle("Biomimetic Linkage Optimization", fontsize=16, fontweight="bold")
    figure.text(
        0.5, 0.025,
        f"Objective: {baseline_score:.3f} → {optimized_score:.3f}  |  "
        f"Phalanges fixed at {optimized_params.link_1_length:g}/"
        f"{optimized_params.link_2_length:g}/{optimized_params.link_3_length:g} mm  |  "
        f"Optimized variables: P0, A1, B2, C3",
        ha="center", fontsize=10,
    )
    figure.tight_layout(rect=(0.0, 0.07, 1.0, 0.93))
    return figure


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--q-max", type=float, default=90.0,
                        help="desired MCP closing range in degrees")
    parser.add_argument("--samples", type=int, default=19,
                        help="closure samples used by the optimizer")
    parser.add_argument("--pip-ratio", type=float, default=1.0,
                        help="target PIP flexion / R01 input ratio")
    parser.add_argument("--dip-ratio", type=float, default=0.70,
                        help="target DIP flexion / PIP flexion ratio")
    parser.add_argument("--monotonic-weight", type=float, default=100.0,
                        help="penalty for any PIP/DIP reopening")
    parser.add_argument("--maxiter", type=int, default=140,
                        help="differential-evolution generations")
    parser.add_argument("--popsize", type=int, default=12,
                        help="differential-evolution population multiplier")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=None,
        help="result directory (default: runs/opt_YYYYMMDD_HHMMSS)",
    )
    parser.add_argument("--params-output", type=Path, default=None,
                        help="override the parameter JSON path")
    parser.add_argument("--report", type=Path, default=None,
                        help="override the optimization report path")
    parser.add_argument("--workspace-report", type=Path, default=None,
                        help="override the workspace report path")
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.samples < 3 or args.q_max <= 0.0:
        raise ValueError("samples must be >= 3 and q-max must be positive")

    base = MechanismParameters()
    started_at = datetime.now().astimezone()
    run_dir = args.run_dir or Path("runs") / f"opt_{started_at:%Y%m%d_%H%M%S}"
    run_dir.mkdir(parents=True, exist_ok=True)
    params_output = args.params_output or run_dir / "parameters.json"
    report_output = args.report or run_dir / "optimization_report.png"
    workspace_output = args.workspace_report or run_dir / "workspace_report.png"
    torque_output = run_dir / "torque_report.png"
    torque_csv_output = run_dir / "torque_samples.csv"
    for output in (params_output, report_output, workspace_output, torque_output,
                   torque_csv_output):
        output.parent.mkdir(parents=True, exist_ok=True)
    q_values = np.linspace(0.0, args.q_max, args.samples)
    objective = BiomimeticObjective(
        base, q_values, pip_ratio=args.pip_ratio, dip_ratio=args.dip_ratio,
        monotonic_weight=args.monotonic_weight,
    )
    baseline_result = solve_trajectory(base, q_values)
    if baseline_result is None:
        raise RuntimeError("baseline mechanism cannot traverse the requested range")
    baseline_poses, _ = baseline_result
    baseline_score = objective(parameters_to_vector(base))

    print("Running global differential-evolution search...")
    global_result = differential_evolution(
        objective,
        BOUNDS,
        maxiter=args.maxiter,
        popsize=args.popsize,
        seed=args.seed,
        tol=1e-6,
        polish=False,
        workers=1,
        updating="immediate",
        disp=True,
    )
    print("Refining the best design with bounded Nelder-Mead search...")
    local_result = minimize(
        objective,
        global_result.x,
        method="Nelder-Mead",
        bounds=BOUNDS,
        options={"maxiter": 800, "xatol": 1e-5, "fatol": 1e-7, "disp": True},
    )
    best_values = local_result.x if local_result.fun < global_result.fun else global_result.x
    optimized_score = float(min(local_result.fun, global_result.fun))
    optimized = vector_to_parameters(best_values, base)
    optimized_result = solve_trajectory(optimized, q_values)
    if optimized_result is None:
        raise RuntimeError("optimizer returned a design with failed loop closure")
    optimized_poses, optimized_closure = optimized_result
    validation_q = np.linspace(0.0, args.q_max, 901)
    validation_result = solve_trajectory(optimized, validation_q)
    if validation_result is None:
        raise RuntimeError("optimized design failed dense closure validation")
    validation_poses, validation_closure = validation_result
    validation = dense_validation(validation_poses, validation_closure)
    if not validation["monotonic"] or not validation["collision_free_centerlines"]:
        raise RuntimeError(f"optimized design failed anatomical validation: {validation}")

    masses = MassParameters()
    torque = analyze_torque(validation_poses, masses)
    peak_torque_nm = float(np.max(np.abs(torque.total_nm)))
    metrics = trajectory_metrics(optimized_poses, optimized_closure, objective)
    metrics["objective"] = optimized_score
    metrics["baseline_objective"] = baseline_score
    metrics["evaluations"] = objective.evaluations
    save_parameters(
        params_output,
        optimized,
        target={
            "q_max_deg": args.q_max,
            "pip_per_q": args.pip_ratio,
            "dip_per_pip": args.dip_ratio,
        },
        run={
            "created_at": started_at.isoformat(timespec="seconds"),
            "directory": str(run_dir),
            "seed": args.seed,
        },
        metrics=metrics,
        validation=validation,
        torque_analysis={
            "model": "quasi-static gravity via dU/dq",
            "peak_input_torque_Nm": peak_torque_nm,
            "peak_input_torque_mNm": 1000.0 * peak_torque_nm,
            "mass_assumptions_g": {
                "link_1": masses.link_1_g,
                "link_2": masses.link_2_g,
                "link_3": masses.link_3_g,
                "each_moving_joint": masses.moving_joint_g,
                "rod_p0_b2": masses.rod_p0_b2_g,
                "rod_a1_c3": masses.rod_a1_c3_g,
                "tcp_payload": masses.tcp_payload_g,
            },
        },
    )

    comparison = draw_optimization_report(
        q_values, objective, baseline_poses, optimized_poses, base, optimized,
        baseline_score, optimized_score,
    )
    comparison.savefig(report_output, dpi=180, bbox_inches="tight", facecolor="white")
    workspace = draw_report(optimized_poses, optimized_closure, optimized)
    workspace.savefig(workspace_output, dpi=180, bbox_inches="tight",
                       facecolor="white")
    torque_figure = draw_torque_report(torque, masses)
    torque_figure.savefig(torque_output, dpi=180, bbox_inches="tight",
                          facecolor="white")
    write_torque_csv(torque_csv_output, torque)

    print(f"Wrote {params_output}")
    print(f"Wrote {report_output}")
    print(f"Wrote {workspace_output}")
    print(f"Wrote {torque_output}")
    print(f"Wrote {torque_csv_output}")
    print(
        f"score {baseline_score:.4f} -> {optimized_score:.4f}; "
        f"PIP RMSE={metrics['pip_rmse_deg']:.2f} deg; "
        f"DIP RMSE={metrics['dip_rmse_deg']:.2f} deg"
    )
    if args.no_show:
        plt.close("all")
    else:
        plt.show()


if __name__ == "__main__":
    main()
