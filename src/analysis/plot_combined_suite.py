#!/usr/bin/env python3
"""Render four finger-specific combined abstractions and a four-panel overview."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from mechanism_schema import DEFAULT_ABSTRACTION, load_abstraction
from plot_combined_abstraction import (
    apply_finger_objective,
    draw_combined_abstraction,
    load_finger_objective,
)


FINGERS = ("index", "middle", "ring", "little")


def render_suite(
    abstraction: Path, objectives_dir: Path, output_dir: Path, overview: Path,
    group_by_finger: bool = False,
) -> None:
    base_data = load_abstraction(abstraction)
    output_dir.mkdir(parents=True, exist_ok=True)
    rendered: list[tuple[str, np.ndarray]] = []
    for finger in FINGERS:
        objective_path = objectives_dir / f"{finger}.yaml"
        data = apply_finger_objective(
            base_data, load_finger_objective(objective_path), objective_path,
        )
        figure, _ = draw_combined_abstraction(data)
        individual = (
            output_dir / finger / "combined_abstraction.png"
            if group_by_finger
            else output_dir / f"combined_abstraction_{finger}.png"
        )
        individual.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(individual, dpi=220, bbox_inches="tight", facecolor="white")
        figure.canvas.draw()
        rendered.append((finger, np.asarray(figure.canvas.buffer_rgba()).copy()))
        plt.close(figure)
        print(f"Wrote {individual}")

    overview.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(2, 2, figsize=(22, 12), constrained_layout=True)
    figure.patch.set_facecolor("white")
    for axes_row, (finger, image) in zip(axes.flat, rendered):
        axes_row.imshow(image)
        axes_row.set_title(finger.capitalize(), fontsize=15, fontweight="bold")
        axes_row.axis("off")
    figure.suptitle(
        "Mechanism 2 — four long-finger combined abstractions",
        fontsize=20,
        fontweight="bold",
    )
    figure.savefig(overview, dpi=180, bbox_inches="tight", facecolor="white")
    plt.close(figure)
    print(f"Wrote {overview}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abstraction", type=Path, nargs="?", default=DEFAULT_ABSTRACTION)
    parser.add_argument("--objectives-dir", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--overview", type=Path, required=True)
    parser.add_argument("--group-by-finger", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    render_suite(
        args.abstraction.resolve(),
        args.objectives_dir.resolve(),
        args.output_dir.resolve(),
        args.overview.resolve(),
        args.group_by_finger,
    )


if __name__ == "__main__":
    main()
