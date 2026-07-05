#!/usr/bin/env python3
"""Draw topology abstractions of the SyLink finger and thumb mechanisms."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Polygon


Point = tuple[float, float]


def draw_bar(ax, a: Point, b: Point, *, width: float = 2.4, zorder: int = 4) -> None:
    ax.plot((a[0], b[0]), (a[1], b[1]), color="black", linewidth=width,
            solid_capstyle="round", zorder=zorder)


def draw_body(ax, points: list[Point], *, gray: str = "0.82", hatch: str | None = None) -> None:
    ax.add_patch(
        Polygon(points, closed=True, facecolor=gray, edgecolor="black",
                linewidth=2.0, hatch=hatch, joinstyle="round", zorder=2)
    )


def draw_joint(ax, point: Point, label: str, offset: Point = (0.0, 0.18)) -> None:
    ax.add_patch(Circle(point, radius=0.09, facecolor="white", edgecolor="black",
                        linewidth=1.8, zorder=10))
    ax.text(point[0] + offset[0], point[1] + offset[1], label,
            ha="center", va="center", fontsize=9, fontweight="bold", zorder=11)


def label_bar(ax, a: Point, b: Point, label: str, normal_offset: float = 0.11) -> None:
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    nx, ny = (-dy / length, dx / length) if length else (0.0, 0.0)
    x = 0.5 * (a[0] + b[0]) + normal_offset * nx
    y = 0.5 * (a[1] + b[1]) + normal_offset * ny
    ax.text(x, y, label, ha="center", va="center", fontsize=10,
            bbox={"facecolor": "white", "edgecolor": "none", "pad": 0.5}, zorder=8)


def draw_finger(ax) -> None:
    """Equivalent crossed four-bar coupling the PIP and DIP joints (Fig. 4c)."""
    points = {
        "MCP": (-0.62, -1.25),
        "Op": (0.0, 0.0),          # PIP anatomical axis
        "Jp": (0.78, -0.13),       # fixed-link auxiliary pivot
        "Jd": (0.50, 1.48),        # distal auxiliary pivot
        "Od": (1.48, 1.48),        # DIP anatomical axis
        "T": (2.32, 1.92),
    }

    # The fixed proximal frame and distal phalanx are ternary rigid bodies.
    draw_body(ax, [points["MCP"], points["Op"], points["Jp"]],
              gray="0.76", hatch="///")
    draw_body(ax, [points["Jd"], points["Od"], points["T"]], gray="0.72")

    # Four-bar loop: Op --lp-- Jp --lj-- Jd --ld-- Od --ls-- Op.
    draw_bar(ax, points["Op"], points["Od"], width=3.0)  # middle phalanx ls
    draw_bar(ax, points["Jp"], points["Jd"], width=2.5)  # crossed coupler lj

    label_bar(ax, points["Op"], points["Jp"], r"$l_p$")
    label_bar(ax, points["Jp"], points["Jd"], r"$l_j$", -0.22)
    label_bar(ax, points["Jd"], points["Od"], r"$l_d$", 0.15)
    label_bar(ax, points["Od"], points["Op"], r"$l_s$", -0.22)

    draw_joint(ax, points["MCP"], "MCP", (-0.18, -0.13))
    draw_joint(ax, points["Op"], r"$O_p$ / PIP", (-0.22, 0.22))
    draw_joint(ax, points["Jp"], r"$J_p$", (0.08, -0.18))
    draw_joint(ax, points["Jd"], r"$J_d$", (-0.06, 0.20))
    draw_joint(ax, points["Od"], r"$O_d$ / DIP", (0.24, -0.18))
    draw_joint(ax, points["T"], "tip", (0.10, 0.18))

    ax.text(0.18, -0.62, "fixed proximal frame", fontsize=9, rotation=-16,
            ha="center", va="center")
    ax.annotate("PIP input", xy=(0.07, 0.14), xytext=(-0.55, 0.68),
                arrowprops={"arrowstyle": "->", "linewidth": 1.4}, fontsize=9)
    ax.text(0.88, 2.30, "Four fingers: one crossed four-bar",
            ha="center", fontsize=13, fontweight="bold")
    ax.text(0.88, 2.08, "PIP input mechanically couples DIP motion",
            ha="center", fontsize=9)


def draw_thumb(ax) -> None:
    """Two stacked crossed four-bars coupling CMC, MCP, and IP (Fig. 4f)."""
    points = {
        "Base": (-0.58, -0.48),
        "Oc": (0.0, 0.0),          # CMC anatomical axis
        "Jc": (0.78, 0.18),        # base auxiliary pivot
        "Am": (-0.24, 1.24),       # lower-loop auxiliary pivot
        "Om": (0.48, 1.80),        # MCP anatomical axis
        "Nm": (1.56, 2.30),        # upper-loop auxiliary pivot
        "Bi": (-0.04, 3.08),       # distal auxiliary pivot
        "Oi": (0.92, 3.76),        # IP anatomical axis
        "T": (1.58, 4.62),
    }

    # The middle triangular body connects the two four-bar stages rigidly.
    draw_body(ax, [points["Base"], points["Oc"], points["Jc"]],
              gray="0.76", hatch="///")
    draw_body(ax, [points["Am"], points["Om"], points["Nm"]], gray="0.82")
    draw_body(ax, [points["Bi"], points["Oi"], points["T"]], gray="0.70")

    # Lower loop: Oc --lc-- Jc --la-- Am --lm-- Om --lf-- Oc.
    draw_bar(ax, points["Oc"], points["Om"], width=3.0)
    draw_bar(ax, points["Jc"], points["Am"], width=2.5)
    label_bar(ax, points["Oc"], points["Jc"], r"$l_c$", -0.11)
    label_bar(ax, points["Jc"], points["Am"], r"$l_a$", 0.18)
    label_bar(ax, points["Am"], points["Om"], r"$l_m$", 0.15)
    label_bar(ax, points["Om"], points["Oc"], r"$l_f$", -0.18)

    # Upper loop: Om --ln-- Nm --lb-- Bi --li-- Oi --le-- Om.
    draw_bar(ax, points["Om"], points["Oi"], width=3.0)
    draw_bar(ax, points["Nm"], points["Bi"], width=2.5)
    label_bar(ax, points["Om"], points["Nm"], r"$l_n$", 0.15)
    label_bar(ax, points["Nm"], points["Bi"], r"$l_b$", 0.20)
    label_bar(ax, points["Bi"], points["Oi"], r"$l_i$", 0.15)
    label_bar(ax, points["Oi"], points["Om"], r"$l_e$", -0.20)

    draw_joint(ax, points["Oc"], r"$O_c$ / CMC", (-0.30, 0.10))
    draw_joint(ax, points["Jc"], r"$J_c$", (0.12, -0.16))
    draw_joint(ax, points["Am"], r"$A_m$", (-0.18, 0.02))
    draw_joint(ax, points["Om"], r"$O_m$ / MCP", (-0.28, 0.18))
    draw_joint(ax, points["Nm"], r"$N_m$", (0.20, 0.02))
    draw_joint(ax, points["Bi"], r"$B_i$", (-0.18, 0.10))
    draw_joint(ax, points["Oi"], r"$O_i$ / IP", (0.20, 0.12))
    draw_joint(ax, points["T"], "tip", (0.18, 0.02))

    ax.annotate("CMC input", xy=(0.08, 0.12), xytext=(-0.60, 0.65),
                arrowprops={"arrowstyle": "->", "linewidth": 1.4}, fontsize=9)
    ax.text(0.70, 5.28, "Thumb: dual stacked crossed four-bars",
            ha="center", fontsize=13, fontweight="bold")
    ax.text(0.70, 5.06, "one CMC input couples MCP and IP motion",
            ha="center", fontsize=9)


def draw_abstraction():
    figure, axes = plt.subplots(1, 2, figsize=(14, 6.5), constrained_layout=True)
    draw_finger(axes[0])
    draw_thumb(axes[1])
    axes[0].set_xlim(-1.0, 2.8)
    axes[0].set_ylim(-1.7, 2.55)
    axes[1].set_xlim(-1.0, 2.1)
    axes[1].set_ylim(-0.8, 5.48)
    for ax in axes:
        ax.set_aspect("equal", adjustable="box")
        ax.axis("off")

    legend = [
        Patch(facecolor="0.78", edgecolor="black", label="rigid multi-pivot body"),
        Line2D([0], [0], color="black", linewidth=2.5, label="binary link / coupler"),
        Line2D([0], [0], marker="o", markerfacecolor="white", markeredgecolor="black",
               linestyle="none", markersize=8, label="revolute pivot"),
    ]
    figure.legend(handles=legend, loc="lower center", ncol=3, frameon=False)
    figure.suptitle("SyLink Finger Mechanism — Topology Abstraction",
                    fontsize=16, fontweight="bold")
    figure.text(
        0.5, 0.035,
        "Abstracted from SyLink Hand, Fig. 4(c,f). Symbolic topology only; not a dimensional CAD reconstruction.",
        ha="center", fontsize=9,
    )
    return figure


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-o", "--output", type=Path,
                        default=Path("runs/nominal/sylink/mechanism_abstraction.png"))
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure = draw_abstraction()
    figure.savefig(args.output, dpi=220, bbox_inches="tight", facecolor="white")
    print(f"Wrote {args.output}")
    if args.no_show:
        plt.close(figure)
    else:
        plt.show()


if __name__ == "__main__":
    main()
