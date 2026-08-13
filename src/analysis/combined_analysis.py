#!/usr/bin/env python3
"""Analyze the mechanism and four human-finger models as one coupled system."""

from __future__ import annotations

import argparse
import csv
import math
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.colors import to_rgba
from matplotlib.patches import Circle, FancyBboxPatch, Polygon
from matplotlib.transforms import Affine2D

from mechanism_schema import DEFAULT_ABSTRACTION, load_abstraction
from plot_combined_abstraction import load_finger_objective
from workspace_sweep import SweepResult, sweep_workspace


Point = tuple[float, float]
FINGERS = ("index", "middle", "ring", "little")
COLORS = {
    "index": "#2563eb",
    "middle": "#059669",
    "ring": "#d97706",
    "little": "#dc2626",
}


@dataclass(frozen=True)
class CombinedSample:
    finger: str
    q_deg: float
    progress: float
    h: Point
    slot_start: Point
    slot_end: Point
    slider: Point
    slider_translation_mm: float
    rod_error_mm: float
    mechanism_residual_mm: float


@dataclass(frozen=True)
class FingerCombinedResult:
    finger: str
    slot_length_mm: float
    phalanx_lengths_mm: tuple[float, float, float]
    joint_flexion_max_deg: tuple[float, float, float]
    samples: tuple[CombinedSample, ...]
    best_curled_q_deg: float
    best_curled_error_mm: float
    best_curled_translation_mm: float


@dataclass(frozen=True)
class CombinedAnalysisResult:
    mechanism_data: dict[str, Any]
    mechanism_sweep: SweepResult
    rod_length_mm: float
    dorsal_clearance_mm: float
    fingers: tuple[FingerCombinedResult, ...]


def _attachment(data: dict[str, Any], attachment_id: str) -> dict[str, Any]:
    return next(
        row for row in data["exoskeleton_attachments"] if row["id"] == attachment_id
    )


def _mechanism_frame(
    positions: dict[str, Point],
) -> Callable[[Point], Point]:
    a, d = positions["a"], positions["d"]
    dx, dy = d[0] - a[0], d[1] - a[1]
    length = math.hypot(dx, dy)
    x_axis = (dx / length, dy / length)
    y_axis = (-x_axis[1], x_axis[0])

    def transform(point: Point) -> Point:
        relative = (point[0] - d[0], point[1] - d[1])
        return (
            relative[0] * x_axis[0] + relative[1] * x_axis[1],
            relative[0] * y_axis[0] + relative[1] * y_axis[1],
        )

    return transform


def _distal_slot(
    objective: dict[str, Any], progress: float, distal_width_mm: float,
    dorsal_clearance_mm: float,
) -> tuple[Point, Point]:
    lengths = objective["phalanx_lengths_mm"]
    ranges = objective["joint_flexion_ranges_deg"]
    proximal = float(lengths["proximal"])
    middle = float(lengths["middle"])
    distal = float(lengths["distal"])
    mcp = math.radians(progress * float(ranges["mcp"]["max"]))
    pip = math.radians(progress * float(ranges["pip"]["max"]))
    dip = math.radians(progress * float(ranges["dip"]["max"]))
    headings = (-mcp, -(mcp + pip), -(mcp + pip + dip))
    dip_point = (
        proximal * math.cos(headings[0]) + middle * math.cos(headings[1]),
        proximal * math.sin(headings[0]) + middle * math.sin(headings[1]),
    )
    tip_point = (
        dip_point[0] + distal * math.cos(headings[2]),
        dip_point[1] + distal * math.sin(headings[2]),
    )
    dorsal_normal = (-math.sin(headings[2]), math.cos(headings[2]))
    offset = (
        -distal_width_mm * dorsal_normal[0],
        -distal_width_mm * dorsal_normal[1] - dorsal_clearance_mm,
    )
    return (
        (dip_point[0] + offset[0], dip_point[1] + offset[1]),
        (tip_point[0] + offset[0], tip_point[1] + offset[1]),
    )


def _slot_closure(
    h: Point, slot: tuple[Point, Point], rod_length_mm: float,
) -> tuple[float, float, Point]:
    start, end = slot
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    length = math.sqrt(length_squared)
    from_start = (h[0] - start[0], h[1] - start[1])
    projection = max(
        0.0,
        min(1.0, (from_start[0] * dx + from_start[1] * dy) / length_squared),
    )
    closest = (start[0] + projection * dx, start[1] + projection * dy)
    minimum_distance = math.dist(h, closest)
    endpoint_distances = (math.dist(h, start), math.dist(h, end))
    maximum_distance = max(endpoint_distances)
    if rod_length_mm < minimum_distance:
        fraction = projection
        error = minimum_distance - rod_length_mm
    elif rod_length_mm > maximum_distance:
        fraction = 0.0 if endpoint_distances[0] >= endpoint_distances[1] else 1.0
        error = rod_length_mm - maximum_distance
    else:
        quadratic_b = 2 * ((start[0] - h[0]) * dx + (start[1] - h[1]) * dy)
        quadratic_c = (
            (start[0] - h[0]) ** 2 + (start[1] - h[1]) ** 2
            - rod_length_mm ** 2
        )
        discriminant = max(
            0.0, quadratic_b ** 2 - 4 * length_squared * quadratic_c,
        )
        root = math.sqrt(discriminant)
        roots = (
            (-quadratic_b - root) / (2 * length_squared),
            (-quadratic_b + root) / (2 * length_squared),
        )
        valid = [value for value in roots if -1e-12 <= value <= 1.0 + 1e-12]
        if not valid:
            raise RuntimeError("rod/slot intersection was lost to numerical precision")
        fraction = min(valid, key=lambda value: abs(value - 0.5))
        fraction = max(0.0, min(1.0, fraction))
        error = 0.0
    slider = (start[0] + fraction * dx, start[1] + fraction * dy)
    return error, fraction * length, slider


def analyze_combined(
    data: dict[str, Any], objectives_dir: Path, q_min: float = 0.0,
    q_max: float = 90.0, steps: int = 181,
) -> CombinedAnalysisResult:
    sweep = sweep_workspace(data, q_min=q_min, q_max=q_max, steps=steps)
    input_mount = _attachment(data, "dorsal_input_mount")
    output_rod = _attachment(data, "distal_output_rod")
    clearance = float(input_mount["dorsal_clearance_mm"])
    rod_length = float(output_rod["assumed_length_mm"])
    distal_width = float(next(
        row["width_mm"] for row in data["human_hand_model"]["phalanges"]
        if row["id"] == "distal_phalanx"
    ))
    transform = _mechanism_frame(sweep.poses[0].positions)
    q_start, q_end = sweep.poses[0].q_deg, sweep.poses[-1].q_deg
    q_span = q_end - q_start
    results: list[FingerCombinedResult] = []
    for finger in FINGERS:
        objective = load_finger_objective(objectives_dir / f"{finger}.yaml")
        samples: list[CombinedSample] = []
        for pose in sweep.poses:
            progress = 0.0 if q_span == 0 else (pose.q_deg - q_start) / q_span
            slot = _distal_slot(objective, progress, distal_width, clearance)
            h = transform(pose.positions["h"])
            error, translation, slider = _slot_closure(h, slot, rod_length)
            samples.append(CombinedSample(
                finger, pose.q_deg, progress, h, slot[0], slot[1], slider,
                translation, error, pose.max_residual_mm,
            ))
        curled_slot = _distal_slot(objective, 1.0, distal_width, clearance)
        curled_results = [
            (*_slot_closure(transform(pose.positions["h"]), curled_slot, rod_length),
             pose.q_deg)
            for pose in sweep.poses
        ]
        best_error, best_translation, _, best_q = min(
            curled_results, key=lambda row: row[0]
        )
        results.append(FingerCombinedResult(
            finger,
            math.dist(*curled_slot),
            (
                float(objective["phalanx_lengths_mm"]["proximal"]),
                float(objective["phalanx_lengths_mm"]["middle"]),
                float(objective["phalanx_lengths_mm"]["distal"]),
            ),
            (
                float(objective["joint_flexion_ranges_deg"]["mcp"]["max"]),
                float(objective["joint_flexion_ranges_deg"]["pip"]["max"]),
                float(objective["joint_flexion_ranges_deg"]["dip"]["max"]),
            ),
            tuple(samples),
            best_q,
            best_error,
            best_translation,
        ))
    return CombinedAnalysisResult(data, sweep, rod_length, clearance, tuple(results))


def _draw_finger_axes(finger_result: FingerCombinedResult, axes: np.ndarray) -> None:
    samples = finger_result.samples
    color = COLORS[finger_result.finger]
    workspace_ax, error_ax, slider_ax = axes
    h_path = [sample.h for sample in samples]
    horizontal_slot = (samples[0].slot_start, samples[0].slot_end)
    curled_slot = (samples[-1].slot_start, samples[-1].slot_end)
    workspace_ax.plot(*zip(*h_path), color="#111827", linewidth=2, label="TCP H")
    workspace_ax.plot(
        *zip(*horizontal_slot), color="#2563eb", linewidth=5,
        label="horizontal RP4 slot",
    )
    workspace_ax.plot(
        *zip(*curled_slot), color="#dc2626", linewidth=5,
        label="curled RP4 slot",
    )
    endpoint_samples = (
        (samples[0], "horizontal"),
        (samples[-1], "synchronized curl"),
    )
    for sample, label in endpoint_samples:
        workspace_ax.plot(
            [sample.h[0], sample.slider[0]], [sample.h[1], sample.slider[1]],
            linestyle="--", linewidth=1.5, label=f"{label} rod",
        )
    workspace_ax.set_title(f"{finger_result.finger.capitalize()} task space")
    workspace_ax.set_aspect("equal", adjustable="datalim")
    workspace_ax.grid(True, color="0.9")
    workspace_ax.legend(fontsize=7, loc="best")
    workspace_ax.set_ylabel("y from D [mm]")

    q = np.asarray([sample.q_deg for sample in samples])
    errors = np.asarray([sample.rod_error_mm for sample in samples])
    translations = np.asarray([sample.slider_translation_mm for sample in samples])
    error_ax.plot(q, errors, color=color, linewidth=2)
    error_ax.axhline(1.0, color="#64748b", linestyle=":", label="1 mm tolerance")
    error_ax.set_title(
        f"Synchronized closure · max {errors.max():.1f} mm\n"
        f"best static curl {finger_result.best_curled_error_mm:.1f} mm "
        f"at q={finger_result.best_curled_q_deg:.1f}°"
    )
    error_ax.set_ylabel("rod/slot error [mm]")
    error_ax.grid(True, color="0.9")
    error_ax.legend(fontsize=7)

    slider_ax.plot(q, translations, color=color, linewidth=2)
    slider_ax.axhline(0, color="#64748b", linestyle=":")
    slider_ax.axhline(finger_result.slot_length_mm, color="#64748b", linestyle=":")
    slider_ax.set_title(
        f"RP4 translation · range 0–{finger_result.slot_length_mm:g} mm"
    )
    slider_ax.set_ylabel("slot coordinate [mm]")
    slider_ax.grid(True, color="0.9")
    for current in (workspace_ax, error_ax, slider_ax):
        current.set_xlabel(
            "x from D [mm]" if current is workspace_ax else "input q [deg]"
        )


def draw_combined_report(result: CombinedAnalysisResult):
    figure, axes = plt.subplots(
        len(result.fingers), 3, figsize=(18, 18), constrained_layout=True,
    )
    for row, finger_result in enumerate(result.fingers):
        _draw_finger_axes(finger_result, axes[row])
    figure.suptitle(
        f"Combined mechanism–hand sweep · nominal rod {result.rod_length_mm:g} mm · "
        f"D–J1 clearance {result.dorsal_clearance_mm:g} mm",
        fontsize=17,
        fontweight="bold",
    )
    return figure


def _finger_joint_points(
    result: CombinedAnalysisResult,
    finger: FingerCombinedResult,
    progress: float,
) -> tuple[Point, Point, Point, Point]:
    mcp, pip, dip = (
        math.radians(progress * angle) for angle in finger.joint_flexion_max_deg
    )
    headings = (-mcp, -(mcp + pip), -(mcp + pip + dip))
    points: list[Point] = [(0.0, -result.dorsal_clearance_mm)]
    for length, heading in zip(finger.phalanx_lengths_mm, headings):
        points.append((
            points[-1][0] + length * math.cos(heading),
            points[-1][1] + length * math.sin(heading),
        ))
    return tuple(points)  # type: ignore[return-value]


def _draw_rounded_segment(
    axes, start: Point, end: Point, width: float, color: str, alpha: float,
    linewidth: float, zorder: float,
) -> None:
    length = math.dist(start, end)
    angle = math.degrees(math.atan2(end[1] - start[1], end[0] - start[0]))
    patch = FancyBboxPatch(
        (0.0, -width),
        length,
        width,
        boxstyle=f"round,pad=0,rounding_size={width / 2}",
        facecolor=to_rgba(color, 0.22 * alpha),
        edgecolor=to_rgba(color, alpha),
        linewidth=linewidth,
        zorder=zorder,
    )
    patch.set_transform(Affine2D().rotate_deg(angle).translate(*start) + axes.transData)
    axes.add_patch(patch)


def _draw_combined_pose(
    axes,
    result: CombinedAnalysisResult,
    finger: FingerCombinedResult,
    index: int,
    alpha: float,
    linewidth: float,
) -> None:
    pose = result.mechanism_sweep.poses[index]
    sample = finger.samples[index]
    transform = _mechanism_frame(result.mechanism_sweep.poses[0].positions)
    mechanism_positions = {
        node_id: transform(point) for node_id, point in pose.positions.items()
    }
    for body in result.mechanism_data["bodies"]:
        points = [mechanism_positions[node_id] for node_id in body["nodes"]]
        color = body.get("color", "#334155")
        if len(points) == 2:
            axes.plot(
                *zip(*points), color=color, alpha=alpha, linewidth=linewidth,
                solid_capstyle="round", zorder=5,
            )
        else:
            axes.add_patch(Polygon(
                points,
                closed=True,
                facecolor=to_rgba(color, 0.10 * alpha),
                edgecolor=to_rgba(color, alpha),
                linewidth=linewidth,
                zorder=4,
            ))

    hand = result.mechanism_data["human_hand_model"]
    phalanx_widths = tuple(float(row["width_mm"]) for row in hand["phalanges"])
    joints = _finger_joint_points(result, finger, sample.progress)
    colors = ("#c2410c", "#c2410c", "#c2410c")
    for start, end, width, color in zip(
        joints, joints[1:], phalanx_widths, colors,
    ):
        _draw_rounded_segment(
            axes, start, end, width, color, alpha, linewidth, 2,
        )
    axes.plot(
        [sample.h[0], sample.slider[0]],
        [sample.h[1], sample.slider[1]],
        color=COLORS[finger.finger],
        alpha=alpha,
        linewidth=linewidth,
        linestyle="--" if sample.rod_error_mm > 1e-9 else "-",
        zorder=7,
    )
    axes.add_patch(Circle(
        sample.slider, 0.7, facecolor=to_rgba("#7c3aed", alpha),
        edgecolor=to_rgba("#4c1d95", alpha), linewidth=linewidth, zorder=8,
    ))


def _draw_assembly_sweep(
    axes, result: CombinedAnalysisResult, finger: FingerCombinedResult,
) -> None:
    samples = finger.samples
    sample_indices = sorted(set(np.linspace(
        0, len(samples) - 1, min(7, len(samples)), dtype=int,
    )))
    for index in sample_indices:
        is_endpoint = index in (0, len(samples) - 1)
        _draw_combined_pose(
            axes, result, finger, index,
            0.95 if is_endpoint else 0.20,
            2.0 if is_endpoint else 0.9,
        )

    palm = result.mechanism_data["human_hand_model"]["palm"]
    palm_start = (-float(palm["length_mm"]), -result.dorsal_clearance_mm)
    palm_end = (0.0, -result.dorsal_clearance_mm)
    _draw_rounded_segment(
        axes, palm_start, palm_end, float(palm["width_mm"]),
        "#9a3412", 0.95, 2.0, 1,
    )

    h_path = [sample.h for sample in samples]
    slider_path = [sample.slider for sample in samples]
    fingertip_path = [
        _finger_joint_points(result, finger, sample.progress)[-1]
        for sample in samples
    ]
    axes.plot(*zip(*h_path), color="#111827", linewidth=2.5, label="TCP H path", zorder=9)
    axes.plot(
        *zip(*slider_path), color="#7c3aed", linewidth=2.0,
        label="RP4 slider path", zorder=9,
    )
    axes.plot(
        *zip(*fingertip_path), color="#ea580c", linewidth=2.0,
        label="human fingertip path", zorder=9,
    )
    for index in (0, len(samples) - 1):
        sample = samples[index]
        axes.add_patch(Circle(
            sample.h, 0.9, facecolor="white", edgecolor="black",
            linewidth=1.4, zorder=10,
        ))
        axes.annotate(
            f"q={sample.q_deg:.1f}°",
            sample.h,
            xytext=(7, 7),
            textcoords="offset points",
            fontsize=8,
            zorder=11,
        )
    axes.set_title("Full hand–mechanism assembly and representative poses")
    axes.set_xlabel("x from D [mm]")
    axes.set_ylabel("y from D [mm]")
    axes.set_aspect("equal", adjustable="datalim")
    axes.grid(True, color="0.9")
    axes.legend(loc="best")
    axes.text(
        0.02,
        0.02,
        f"Solved q=[{samples[0].q_deg:.1f}°, {samples[-1].q_deg:.1f}°] "
        f"of requested [{result.mechanism_sweep.requested_min_deg:.1f}°, "
        f"{result.mechanism_sweep.requested_max_deg:.1f}°]\n"
        f"{len(samples)} synchronized poses · rod {result.rod_length_mm:g} mm · "
        f"D–J1 {result.dorsal_clearance_mm:g} mm",
        transform=axes.transAxes,
        ha="left",
        va="bottom",
        fontsize=9,
        color="#475569",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": "0.6"},
    )


def draw_finger_report(
    result: CombinedAnalysisResult, finger_result: FingerCombinedResult,
):
    """Render a legacy workspace-style report for the complete coupled assembly."""
    figure = plt.figure(figsize=(15, 8.5), constrained_layout=True)
    grid = figure.add_gridspec(2, 2, width_ratios=(1.6, 1.0))
    assembly_ax = figure.add_subplot(grid[:, 0])
    error_ax = figure.add_subplot(grid[0, 1])
    slider_ax = figure.add_subplot(grid[1, 1])
    _draw_assembly_sweep(assembly_ax, result, finger_result)

    samples = finger_result.samples
    q = np.asarray([sample.q_deg for sample in samples])
    errors = np.asarray([sample.rod_error_mm for sample in samples])
    translations = np.asarray([sample.slider_translation_mm for sample in samples])
    color = COLORS[finger_result.finger]
    error_ax.plot(q, errors, color=color, linewidth=2)
    error_ax.axhline(1.0, color="#64748b", linestyle=":", label="1 mm tolerance")
    error_ax.set_title(
        f"Rod/slot closure error · max {errors.max():.1f} mm\n"
        f"best static curl {finger_result.best_curled_error_mm:.1f} mm "
        f"at q={finger_result.best_curled_q_deg:.1f}°"
    )
    error_ax.set_xlabel("input q [deg]")
    error_ax.set_ylabel("error [mm]")
    error_ax.grid(True, color="0.9")
    error_ax.legend(fontsize=8)

    slider_ax.plot(q, translations, color=color, linewidth=2)
    slider_ax.axhline(0.0, color="#64748b", linestyle=":")
    slider_ax.axhline(
        finger_result.slot_length_mm, color="#64748b", linestyle=":",
    )
    slider_ax.set_title(
        f"RP4 translation · allowed 0–{finger_result.slot_length_mm:g} mm"
    )
    slider_ax.set_xlabel("input q [deg]")
    slider_ax.set_ylabel("slot coordinate [mm]")
    slider_ax.grid(True, color="0.9")
    for axes in (error_ax, slider_ax):
        axes.set_xlim(
            result.mechanism_sweep.requested_min_deg,
            result.mechanism_sweep.requested_max_deg,
        )
        if samples[-1].q_deg < result.mechanism_sweep.requested_max_deg:
            axes.axvspan(
                samples[-1].q_deg,
                result.mechanism_sweep.requested_max_deg,
                color="#dc2626",
                alpha=0.08,
            )
    figure.suptitle(
        f"Mechanism 2 + {finger_result.finger} finger — combined assembly sweep",
        fontsize=16,
        fontweight="bold",
    )
    return figure


def write_combined_csv(
    path: Path, result: CombinedAnalysisResult,
    fingers: tuple[FingerCombinedResult, ...] | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "finger", "q_deg", "curl_progress", "h_x_mm", "h_y_mm",
        "hand_mcp_x_mm", "hand_mcp_y_mm", "hand_pip_x_mm", "hand_pip_y_mm",
        "hand_dip_x_mm", "hand_dip_y_mm", "hand_tip_x_mm", "hand_tip_y_mm",
        "slot_start_x_mm", "slot_start_y_mm", "slot_end_x_mm", "slot_end_y_mm",
        "slider_x_mm", "slider_y_mm", "slider_translation_mm", "slot_length_mm",
        "rod_length_mm", "rod_error_mm", "mechanism_residual_mm",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for finger_result in fingers or result.fingers:
            for sample in finger_result.samples:
                hand_mcp, hand_pip, hand_dip, hand_tip = _finger_joint_points(
                    result, finger_result, sample.progress,
                )
                writer.writerow({
                    "finger": sample.finger,
                    "q_deg": sample.q_deg,
                    "curl_progress": sample.progress,
                    "h_x_mm": sample.h[0], "h_y_mm": sample.h[1],
                    "hand_mcp_x_mm": hand_mcp[0], "hand_mcp_y_mm": hand_mcp[1],
                    "hand_pip_x_mm": hand_pip[0], "hand_pip_y_mm": hand_pip[1],
                    "hand_dip_x_mm": hand_dip[0], "hand_dip_y_mm": hand_dip[1],
                    "hand_tip_x_mm": hand_tip[0], "hand_tip_y_mm": hand_tip[1],
                    "slot_start_x_mm": sample.slot_start[0],
                    "slot_start_y_mm": sample.slot_start[1],
                    "slot_end_x_mm": sample.slot_end[0],
                    "slot_end_y_mm": sample.slot_end[1],
                    "slider_x_mm": sample.slider[0], "slider_y_mm": sample.slider[1],
                    "slider_translation_mm": sample.slider_translation_mm,
                    "slot_length_mm": finger_result.slot_length_mm,
                    "rod_length_mm": result.rod_length_mm,
                    "rod_error_mm": sample.rod_error_mm,
                    "mechanism_residual_mm": sample.mechanism_residual_mm,
                })


def write_summary(
    path: Path, result: CombinedAnalysisResult,
    fingers: tuple[FingerCombinedResult, ...] | None = None,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    document = {
        "schema_version": 1,
        "analysis": "combined_mechanism_hand_sweep",
        "motion_mapping": "normalized_actuator_progress_to_linear_joint_flexion",
        "closure_tolerance_mm": 1.0,
        "rod_length_mm": result.rod_length_mm,
        "dorsal_clearance_mm": result.dorsal_clearance_mm,
        "mechanism_requested_q_range_deg": [
            result.mechanism_sweep.requested_min_deg,
            result.mechanism_sweep.requested_max_deg,
        ],
        "mechanism_solved_q_range_deg": [
            result.mechanism_sweep.poses[0].q_deg,
            result.mechanism_sweep.poses[-1].q_deg,
        ],
        "mechanism_completed_requested_range": math.isclose(
            result.mechanism_sweep.poses[-1].q_deg,
            result.mechanism_sweep.requested_max_deg,
        ),
        "fingers": [],
    }
    for finger in fingers or result.fingers:
        errors = [sample.rod_error_mm for sample in finger.samples]
        document["fingers"].append({
            "finger": finger.finger,
            "slot_length_mm": round(finger.slot_length_mm, 6),
            "horizontal_error_mm": round(errors[0], 6),
            "synchronized_curled_error_mm": round(errors[-1], 6),
            "maximum_synchronized_error_mm": round(max(errors), 6),
            "samples_within_1mm_fraction": round(
                sum(error <= 1.0 for error in errors) / len(errors), 6,
            ),
            "best_static_curled_q_deg": finger.best_curled_q_deg,
            "best_static_curled_error_mm": round(finger.best_curled_error_mm, 6),
            "best_static_curled_translation_mm": round(
                finger.best_curled_translation_mm, 6,
            ),
        })
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")


def write_per_finger_outputs(path: Path, result: CombinedAnalysisResult) -> None:
    for finger in result.fingers:
        finger_dir = path / finger.finger
        finger_dir.mkdir(parents=True, exist_ok=True)
        report_path = finger_dir / "combined_workspace_report.png"
        csv_path = finger_dir / "combined_workspace_samples.csv"
        summary_path = finger_dir / "combined_workspace_summary.yaml"
        figure = draw_finger_report(result, finger)
        figure.savefig(report_path, dpi=180, bbox_inches="tight", facecolor="white")
        plt.close(figure)
        write_combined_csv(csv_path, result, (finger,))
        write_summary(summary_path, result, (finger,))
        print(f"Wrote {report_path}")
        print(f"Wrote {csv_path}")
        print(f"Wrote {summary_path}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abstraction", type=Path, nargs="?", default=DEFAULT_ABSTRACTION)
    parser.add_argument("--objectives-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--summary", type=Path, required=True)
    parser.add_argument("--per-finger-dir", type=Path)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = load_abstraction(args.abstraction)
    result = analyze_combined(data, args.objectives_dir.resolve())
    figure = draw_combined_report(result)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    write_combined_csv(args.csv, result)
    write_summary(args.summary, result)
    if args.per_finger_dir is not None:
        write_per_finger_outputs(args.per_finger_dir.resolve(), result)
    print(f"Wrote {args.output}")
    print(f"Wrote {args.csv}")
    print(f"Wrote {args.summary}")


if __name__ == "__main__":
    main()
