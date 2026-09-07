"""Stack animations into one GIF so two designs can be compared frame by frame.

    python3 tools/stack_gifs.py --out runs/compare.gif \
        --gif runs/a/design_animation.gif --label "short_ad" \
        --gif runs/b/design_animation.gif --label "long_ad"

Frames are paired by index. Inputs of different lengths are cut to the shortest,
and differing widths are centred on a common canvas rather than scaled, so the two
mechanisms keep the same millimetre scale on screen.
"""
from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image, ImageDraw


def read_frames(path: Path) -> tuple[list[Image.Image], int]:
    image = Image.open(path)
    frames, duration = [], image.info.get("duration", 60)
    index = 0
    while True:
        try:
            image.seek(index)
        except EOFError:
            break
        frames.append(image.convert("RGB").copy())
        index += 1
    if not frames:
        raise SystemExit(f"no frames in {path}")
    return frames, duration


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gif", action="append", required=True, dest="gifs",
                        help="input GIF; repeat for each row, top first")
    parser.add_argument("--label", action="append", default=[], dest="labels",
                        help="caption drawn on the matching row")
    parser.add_argument("--out", required=True)
    parser.add_argument("--gap", type=int, default=8, help="pixels between rows")
    args = parser.parse_args()

    if len(args.gifs) < 2:
        raise SystemExit("give at least two --gif inputs")

    tracks = [read_frames(Path(p)) for p in args.gifs]
    count = min(len(frames) for frames, _ in tracks)
    duration = max(d for _, d in tracks)
    width = max(frame.width for frames, _ in tracks for frame in frames[:1])
    heights = [frames[0].height for frames, _ in tracks]
    total = sum(heights) + args.gap * (len(tracks) - 1)

    labels = list(args.labels) + [""] * (len(tracks) - len(args.labels))
    composed = []
    for index in range(count):
        canvas = Image.new("RGB", (width, total), "white")
        y = 0
        for (frames, _), height, label in zip(tracks, heights, labels):
            frame = frames[index]
            canvas.paste(frame, ((width - frame.width) // 2, y))
            if label:
                draw = ImageDraw.Draw(canvas)
                box = draw.textbbox((0, 0), label)
                pad = 6
                draw.rectangle(
                    (12, y + 12, 12 + box[2] - box[0] + 2 * pad,
                     y + 12 + box[3] - box[1] + 2 * pad),
                    fill="white", outline="#94a3b8")
                draw.text((12 + pad, y + 12 + pad), label, fill="#0b0b0b")
            y += height + args.gap
        composed.append(canvas)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    composed[0].save(out, save_all=True, append_images=composed[1:],
                     duration=duration, loop=0, optimize=True)
    print(f"{len(composed)} frames, {width}x{total}, {duration}ms -> {out}")


if __name__ == "__main__":
    main()
