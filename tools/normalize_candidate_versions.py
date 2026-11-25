#!/usr/bin/env python3
"""Build the uniform docs/candidates/V*/ presentation layer from archived results."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import yaml
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
CANDIDATES = ROOT / "docs" / "candidates"
ARCHIVE = CANDIDATES / "_archive"


VERSIONS = [
    {
        "version": "V1",
        "date": "2026-08-25",
        "title": "Initial index-finger candidate",
        "status": "historical",
        "fingers": {
            "index": ("index_finger_v1/mechanism.yaml", "yaml"),
        },
        "preview": "index_finger_v1/abstraction.png",
        "motion": "index_finger_v1/design_animation.gif",
    },
    {
        "version": "V2",
        "date": "2026-09-06",
        "title": "Tilted-mount index-finger candidate",
        "status": "historical",
        "fingers": {
            "index": ("index_finger_tilted_v2/mechanism.yaml", "yaml"),
        },
        "preview_from_motion": "index_finger_tilted_v2/design_animation.gif",
        "motion": "index_finger_tilted_v2/design_animation.gif",
    },
    {
        "version": "V3",
        "date": "2026-09-08",
        "title": "Extended-mount middle-finger candidate",
        "status": "historical",
        "fingers": {
            "middle": ("middle_finger_long_ad/mechanism.yaml", "yaml"),
        },
        "preview_from_motion": "middle_finger_long_ad/design_animation.gif",
        "motion": "middle_finger_long_ad/design_animation.gif",
    },
    {
        "version": "V4",
        "date": "2026-11-09",
        "title": "Direct-drive thumb candidate",
        "status": "historical",
        "fingers": {
            "thumb": ("thumb_direct/parameters.json", "json"),
        },
        "preview": "thumb_direct/preview_straight.png",
        "motion": "thumb_direct/thumb_direct.gif",
    },
    {
        "version": "V5",
        "date": "2026-11-16",
        "title": "Ring- and little-finger reoptimization",
        "status": "historical",
        "fingers": {
            "ring": ("ring_little_reoptimization/ring/parameters.json", "json"),
            "little": ("ring_little_reoptimization/little/parameters.json", "json"),
        },
        "preview": "ring_little_reoptimization/optimized_motion.png",
        "motion_from_preview": "ring_little_reoptimization/optimized_motion.png",
        "motion_kind": "single-frame GIF because the archived result has no animation",
    },
    {
        "version": "V6",
        "date": "2026-11-16",
        "title": "Non-returning straight-output thumb candidate",
        "status": "historical",
        "fingers": {
            "thumb": ("thumb_nonreturn_v1/parameters_relayout.json", "json"),
        },
        "preview": "thumb_nonreturn_v1/end.png",
        "motion": "thumb_nonreturn_v1/relayout.gif",
    },
    {
        "version": "V7",
        "date": "2026-11-17",
        "title": "Four-finger independent X-offset optimization",
        "status": "historical",
        "fingers": {
            finger: (f"four_finger_offsets_2026_11_17/{finger}/mechanism.yaml", "yaml")
            for finger in ("index", "middle", "ring", "little")
        },
        "preview": "four_finger_offsets_2026_11_17/comparison.png",
        "motion": "four_finger_offsets_2026_11_17/motion.gif",
    },
    {
        "version": "V8",
        "date": "2026-11-18",
        "title": "Direct H-to-R4 four-finger feasibility study",
        "status": "historical_feasibility_study",
        "fingers": {
            finger: (f"h_direct_2026_11_18/selected/{finger}/result.json", "json")
            for finger in ("index", "middle", "ring", "little")
        },
        "preview": "h_direct_2026_11_18/refined.png",
        "motion": "h_direct_2026_11_18/motion.gif",
    },
    {
        "version": "V9",
        "date": "2026-11-18",
        "title": "Four-finger GH-clearance contour candidate",
        "status": "current",
        "current_dimensions": "h_direct_clearance_2026_11_18/current_dimensions.yaml",
        "fingers": {
            finger: ("h_direct_clearance_2026_11_18/current_dimensions.yaml", "current")
            for finger in ("index", "middle", "ring", "little")
        },
        "preview": "h_direct_clearance_2026_11_18/clearance_before_after.png",
        "motion": "h_direct_clearance_2026_11_18/motion.gif",
    },
]


class NoAliasDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def read_source(relative_path: str, source_format: str, finger: str):
    path = ARCHIVE / relative_path
    if source_format == "json":
        return json.loads(path.read_text(encoding="utf-8"))
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if source_format == "current":
        return {
            "units": document["units"],
            "status": document["status"],
            "criteria": document["criteria"],
            "note": document["note"],
            "dimensions": document["fingers"][finger],
        }
    return document


def write_yaml(path: Path, document) -> None:
    path.write_text(
        yaml.dump(
            document,
            Dumper=NoAliasDumper,
            allow_unicode=True,
            sort_keys=False,
            width=100,
        ),
        encoding="utf-8",
    )


def first_frame(source: Path, destination: Path) -> None:
    with Image.open(source) as image:
        image.seek(0)
        image.convert("RGBA").save(destination)


def static_gif(source: Path, destination: Path) -> None:
    with Image.open(source) as image:
        image.convert("P", palette=Image.ADAPTIVE).save(destination, format="GIF")


def build_version(spec) -> None:
    destination = CANDIDATES / spec["version"]
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir()

    sources = {}
    for finger, (relative_path, source_format) in spec["fingers"].items():
        source = ARCHIVE / relative_path
        sources[finger] = str(source.relative_to(ROOT))
        write_yaml(
            destination / f"{finger}.yaml",
            {
                "schema_version": 1,
                "version": spec["version"],
                "finger": finger,
                "source": {
                    "path": str(source.relative_to(ROOT)),
                    "format": source_format,
                },
                "candidate": read_source(relative_path, source_format, finger),
            },
        )

    if "preview" in spec:
        shutil.copy2(ARCHIVE / spec["preview"], destination / "preview.png")
        preview_source = str((ARCHIVE / spec["preview"]).relative_to(ROOT))
    else:
        preview_source_path = ARCHIVE / spec["preview_from_motion"]
        first_frame(preview_source_path, destination / "preview.png")
        preview_source = str(preview_source_path.relative_to(ROOT)) + " (first frame)"

    if "motion" in spec:
        shutil.copy2(ARCHIVE / spec["motion"], destination / "motion.gif")
        motion_source = str((ARCHIVE / spec["motion"]).relative_to(ROOT))
    else:
        motion_source_path = ARCHIVE / spec["motion_from_preview"]
        static_gif(motion_source_path, destination / "motion.gif")
        motion_source = str(motion_source_path.relative_to(ROOT)) + " (single frame)"

    manifest = {
        "schema_version": 1,
        "version": spec["version"],
        "date": spec["date"],
        "title": spec["title"],
        "status": spec["status"],
        "fingers": list(spec["fingers"]),
        "standard_files": {
            "finger_candidates": [f"{finger}.yaml" for finger in spec["fingers"]],
            "preview": "preview.png",
            "motion": "motion.gif",
        },
        "sources": {
            "finger_candidates": sources,
            "preview": preview_source,
            "motion": motion_source,
        },
    }
    if "motion_kind" in spec:
        manifest["notes"] = [spec["motion_kind"]]
    write_yaml(destination / "manifest.yaml", manifest)


def main() -> None:
    for spec in VERSIONS:
        build_version(spec)
    print(f"Built {len(VERSIONS)} standardized candidate versions in {CANDIDATES}")


if __name__ == "__main__":
    main()
