"""Shared drawing primitives for mechanism analysis plots."""

from __future__ import annotations

from collections.abc import Sequence

from matplotlib.colors import to_rgb, to_rgba


Point = tuple[float, float]


def _darken(color: str, factor: float = 0.58) -> tuple[float, float, float]:
    red, green, blue = to_rgb(color)
    return red * factor, green * factor, blue * factor


def draw_l_bracket(
    axes,
    segments: Sequence[tuple[Point, Point]],
    color: str,
    linewidth: float,
    *,
    alpha: float = 1.0,
    zorder: float = 3.0,
) -> None:
    """Draw two arms as one rounded, outlined L-shaped plate."""
    if len(segments) != 2 or segments[0][0] != segments[1][0]:
        raise ValueError("an L bracket needs two segments sharing one corner")
    corner = segments[0][0]
    first_end = segments[0][1]
    second_end = segments[1][1]
    points = (first_end, corner, second_end)
    outline_width = linewidth + max(2.0, 0.28 * linewidth)
    axes.plot(
        *zip(*points),
        color=to_rgba(_darken(color), alpha),
        linewidth=outline_width,
        solid_capstyle="round",
        solid_joinstyle="round",
        zorder=zorder,
    )
    axes.plot(
        *zip(*points),
        color=to_rgba(color, 0.88 * alpha),
        linewidth=linewidth,
        solid_capstyle="round",
        solid_joinstyle="round",
        zorder=zorder + 0.1,
    )
