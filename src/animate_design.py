#!/usr/bin/env python3
"""Animate one self-contained mechanism.yaml and print a motion report."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation, HTMLWriter, PillowWriter
from matplotlib.patches import Circle


HERE = Path(__file__).resolve().parent
ANALYSIS_DIR = HERE / "analysis"
if str(ANALYSIS_DIR) not in sys.path:
    sys.path.insert(0, str(ANALYSIS_DIR))

from combined_analysis import (  # noqa: E402
    COLORS,
    _draw_combined_pose,
    _draw_rounded_segment,
    _finger_joint_points,
    _mechanism_frame,
    _perpendicular_deviation_deg,
    analyze_combined,
)
from mechanism_schema import DEFAULT_ABSTRACTION, load_abstraction  # noqa: E402
from plot_combined_abstraction import finger_analysis_targets  # noqa: E402


def resolve_mechanism(path: Path) -> Path:
    """Accept either a mechanism.yaml or the design directory containing it."""
    candidate = path / "mechanism.yaml" if path.is_dir() else path
    if not candidate.is_file():
        raise FileNotFoundError(f"mechanism design not found: {candidate}")
    return candidate.resolve()


def select_finger(data: dict, requested: str | None) -> str:
    targets = finger_analysis_targets(data)
    if requested is not None:
        if requested not in targets:
            raise ValueError(
                f"finger {requested!r} is not embedded in this mechanism.yaml; "
                f"available: {', '.join(targets)}"
            )
        return requested
    reference = data["human_hand_model"].get("reference_finger")
    if reference in targets:
        return str(reference)
    if len(targets) == 1:
        return next(iter(targets))
    raise ValueError("select an embedded finger with --finger")


def default_output(mechanism: Path, finger: str) -> Path:
    return (
        mechanism.parent / "artifacts" / "combined" / "fingers" / finger
        / "design_animation.gif"
    )


def frame_indices(sample_count: int, requested_frames: int, ping_pong: bool) -> list[int]:
    forward = list(dict.fromkeys(
        int(value) for value in np.linspace(
            0, sample_count - 1, min(sample_count, requested_frames)
        ).round()
    ))
    if ping_pong and len(forward) > 2:
        return forward + forward[-2:0:-1]
    return forward


def plot_limits(result, finger) -> tuple[tuple[float, float], tuple[float, float]]:
    transform = _mechanism_frame(result.mechanism_sweep.poses[0].positions)
    points: list[tuple[float, float]] = []
    for pose in result.mechanism_sweep.poses:
        points.extend(transform(point) for point in pose.positions.values())
    for sample in finger.samples:
        points.extend(_finger_joint_points(result, finger, sample.progress))
        points.extend((sample.h, sample.contact))
    palm = result.mechanism_data["human_hand_model"]["palm"]
    palm_y = -result.dorsal_clearance_mm
    points.extend((
        (-float(palm["length_mm"]), palm_y),
        (0.0, palm_y - float(palm["width_mm"])),
    ))
    xs, ys = zip(*points)
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    margin = max(8.0, 0.08 * span)
    return (
        (min(xs) - margin, max(xs) + margin),
        (min(ys) - margin, max(ys) + margin),
    )


def build_animation(result, finger, indices: list[int], interval_ms: float):
    figure, axes = plt.subplots(figsize=(13, 8), constrained_layout=True)
    figure.patch.set_facecolor("white")
    x_limits, y_limits = plot_limits(result, finger)
    hand = result.mechanism_data["human_hand_model"]
    palm = hand["palm"]
    palm_start = (-float(palm["length_mm"]), -result.dorsal_clearance_mm)
    palm_end = (0.0, -result.dorsal_clearance_mm)
    h_path = [sample.h for sample in finger.samples]
    r4_path = [sample.contact for sample in finger.samples]
    tip_path = [
        _finger_joint_points(result, finger, sample.progress)[-1]
        for sample in finger.samples
    ]

    def draw(frame_number: int):
        index = indices[frame_number]
        sample = finger.samples[index]
        axes.clear()
        axes.set_facecolor("#f8fafc")
        _draw_rounded_segment(
            axes,
            palm_start,
            palm_end,
            float(palm["width_mm"]),
            "#9a3412",
            0.95,
            2.0,
            1,
        )
        axes.plot(*zip(*h_path), color="#111827", alpha=0.25, linewidth=1.5)
        axes.plot(*zip(*r4_path), color="#7c3aed", alpha=0.30, linewidth=1.5)
        axes.plot(*zip(*tip_path), color="#ea580c", alpha=0.25, linewidth=1.5)
        _draw_combined_pose(axes, result, finger, index, 1.0, 2.2)
        axes.add_patch(Circle(
            sample.h, 0.9, facecolor="white", edgecolor="black",
            linewidth=1.4, zorder=12,
        ))
        perpendicular = _perpendicular_deviation_deg(sample)
        axes.set_title(
            f"{result.mechanism_data['mechanism']['name']} · {finger.finger} finger\n"
            f"q={sample.q_deg:.2f}° · curl s={sample.progress:.3f} · "
            f"closure={sample.rod_error_mm:.3f} mm · "
            f"rod-normal deviation={perpendicular:.2f}°",
            fontsize=14,
            fontweight="bold",
        )
        axes.set_xlim(*x_limits)
        axes.set_ylim(*y_limits)
        axes.set_aspect("equal", adjustable="box")
        axes.set_xlabel("x from D [mm]")
        axes.set_ylabel("y from D [mm]")
        axes.grid(True, color="0.90")
        axes.text(
            0.015,
            0.015,
            f"frame {frame_number + 1}/{len(indices)} · fixed R4 upper-distal contact",
            transform=axes.transAxes,
            fontsize=9,
            color="#475569",
            bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "alpha": 0.85},
        )
        return tuple(axes.lines) + tuple(axes.patches) + tuple(axes.texts)

    animation = FuncAnimation(
        figure,
        draw,
        frames=len(indices),
        interval=interval_ms,
        blit=False,
        repeat=True,
    )
    return figure, animation


def animation_writer(output: Path, fps: float):
    suffix = output.suffix.lower()
    if suffix == ".gif":
        return PillowWriter(fps=fps)
    if suffix == ".mp4":
        return FFMpegWriter(fps=fps, bitrate=2200)
    if suffix == ".html":
        return HTMLWriter(fps=fps, embed_frames=True)
    raise ValueError("animation output must end in .gif, .mp4, or .html")


def print_report(mechanism: Path, output: Path, result, finger) -> None:
    samples = finger.samples
    closure_errors = [sample.rod_error_mm for sample in samples]
    deviations = [_perpendicular_deviation_deg(sample) for sample in samples]
    residuals = [sample.mechanism_residual_mm for sample in samples]
    progresses = [sample.progress for sample in samples]
    provenance = result.mechanism_data.get("optimization_provenance", {})
    monotone = all(right >= left for left, right in zip(progresses, progresses[1:]))
    print("\nDesign animation report")
    print(f"  mechanism: {result.mechanism_data['mechanism']['id']}")
    print(f"  source: {mechanism}")
    print(f"  finger: {finger.finger}")
    print(f"  motion model: {provenance.get('intended_workspace_mapping', 'nominal normalized-progress visualization')}")
    print(f"  sampled poses: {len(samples)}")
    print(f"  crank range: {samples[0].q_deg:.3f}° -> {samples[-1].q_deg:.3f}°")
    print(f"  hand curl: {progresses[0]:.4f} -> {progresses[-1]:.4f}; monotone={monotone}")
    print(f"  output rod: {result.rod_length_mm:.3f} mm")
    print(f"  maximum rod-closure error: {max(closure_errors):.6f} mm")
    print(f"  maximum rod-normal deviation: {max(deviations):.3f}°")
    print(f"  maximum mechanism residual: {max(residuals):.3e} mm")
    if provenance:
        print(f"  collision constraint: {'PASS' if provenance.get('collision_free') else 'REJECTED'}")
        clearance = provenance.get("minimum_signed_clearance_mm")
        if isinstance(clearance, (int, float)) and math.isfinite(clearance):
            print(f"  minimum signed clearance: {clearance:.6f} mm")
        print(f"  rod-closure constraint: {'PASS' if provenance.get('rod_closure_feasible') else 'REJECTED'}")
        print(f"  downward-curl constraint: {'PASS' if provenance.get('hand_motion_feasible') else 'REJECTED'}")
    else:
        print("  hard-constraint status: nominal visualization; see analysis reports")
    print(f"  animation: {output}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "design",
        type=Path,
        nargs="?",
        default=DEFAULT_ABSTRACTION,
        help="mechanism.yaml or its containing design directory",
    )
    parser.add_argument("--finger", choices=("index", "middle", "ring", "little"))
    parser.add_argument(
        "-o", "--output", type=Path,
        help="default: <design>/artifacts/combined/fingers/<finger>/design_animation.gif",
    )
    parser.add_argument("--fps", type=float, default=15.0)
    parser.add_argument("--frames", type=int, default=60, help="maximum forward frames")
    parser.add_argument(
        "--one-way", action="store_true",
        help="do not append reverse frames for smooth ping-pong playback",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.fps <= 0 or args.frames < 2:
        raise ValueError("--fps must be positive and --frames must be at least 2")
    mechanism = resolve_mechanism(args.design)
    data = load_abstraction(mechanism)
    finger_name = select_finger(data, args.finger)
    result = analyze_combined(data, fingers=(finger_name,))
    finger = result.fingers[0]
    indices = frame_indices(len(finger.samples), args.frames, not args.one_way)
    output = (args.output or default_output(mechanism, finger_name)).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure, animation = build_animation(result, finger, indices, 1000.0 / args.fps)
    try:
        animation.save(output, writer=animation_writer(output, args.fps), dpi=120)
    finally:
        plt.close(figure)
    print_report(mechanism, output, result, finger)


if __name__ == "__main__":
    main()
