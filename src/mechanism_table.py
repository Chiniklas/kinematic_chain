#!/usr/bin/env python3
"""Validate a mechanism YAML and export its abstraction as Markdown and CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any

from mechanism_schema import DEFAULT_ABSTRACTION, load_abstraction, summary_lines, validate_abstraction


def _cell(value: Any) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, list):
        return ", ".join(str(item) for item in value)
    return str(value)


def _markdown_table(headers: list[str], rows: list[list[Any]]) -> list[str]:
    result = [
        "| " + " | ".join(headers) + " |",
        "|" + "|".join("---" for _ in headers) + "|",
    ]
    result.extend("| " + " | ".join(_cell(value) for value in row) + " |" for row in rows)
    return result


def render_markdown(data: dict[str, Any], source_path: Path) -> str:
    summary = validate_abstraction(data)
    mechanism = data["mechanism"]
    lines = [
        f"# {mechanism['name']}",
        "",
        f"Source of truth: `{source_path.name}` (schema version {data['schema_version']}).",
        "",
        "## Validation summary",
        "",
        *[f"- {line}" for line in summary_lines(summary)],
        "",
        "## Photo calibration",
        "",
        *_markdown_table(
            ["Field", "Value"],
            [[name, value] for name, value in data.get("photo_calibration", {}).items()],
        ),
        "",
        "## Nodes",
        "",
        *_markdown_table(
            ["Node", "Source", "Kind", "Role", "Fixed", "Confidence"],
            [[node["id"], node.get("source_label"), node.get("kind"), node.get("role"),
              node.get("fixed", False), node.get("confidence")] for node in data["nodes"]],
        ),
        "",
        "## Bodies and members",
        "",
        *_markdown_table(
            ["Body", "Kind", "Nodes", "Role", "Confidence"],
            [[body["id"], body.get("kind"), body.get("nodes"), body.get("role"),
              body.get("confidence")] for body in data["bodies"]],
        ),
        "",
        "## Joint incidence",
        "",
        *_markdown_table(
            ["Node", "Joint type", "Incident bodies", "Equivalent lower pairs"],
            [[joint["node"], joint.get("type"), joint.get("bodies"),
              len(joint["bodies"]) - 1] for joint in data["joints"]],
        ),
        "",
        "## Dimensions",
        "",
        *_markdown_table(
            ["Dimension", "Body", "Nodes", "Value", "Units", "Source", "Uncertainty"],
            [[dimension["id"], dimension.get("body"), dimension.get("nodes"),
              dimension.get("value"), dimension.get("units"),
              dimension.get("value_source"), dimension.get("uncertainty_mm")]
             for dimension in data["dimensions"]],
        ),
        "",
        "## Loops",
        "",
        *_markdown_table(
            ["Loop", "Node cycle", "Body cycle"],
            [[loop["id"], loop.get("nodes"), loop.get("bodies")]
             for loop in data.get("loops", [])],
        ),
        "",
        "## Model readiness",
        "",
        *_markdown_table(
            ["Analysis", "Status"],
            [[name, status] for name, status in data.get("model_readiness", {}).items()],
        ),
        "",
        "## Workspace analysis settings",
        "",
        *_markdown_table(
            ["Setting", "Value"],
            [[name, value] for name, value in
             data.get("analysis", {}).get("workspace_sweep", {}).items()],
        ),
        "",
        "## Nominal body mass model",
        "",
        *_markdown_table(
            ["Body", "Mass [g]", "Centre node weights"],
            [[row.get("body"), row.get("mass_g"),
              ", ".join(f"{node}: {weight}" for node, weight in
                        row.get("center_node_weights", {}).items())]
             for row in data.get("mass_model", {}).get("bodies", [])],
        ),
        "",
        "## Nominal point masses",
        "",
        *_markdown_table(
            ["Mass", "Node", "Mass [g]"],
            [[row.get("id"), row.get("node"), row.get("mass_g")]
             for row in data.get("mass_model", {}).get("point_masses", [])],
        ),
        "",
        f"Mass-model status: `{data.get('mass_model', {}).get('status', 'not provided')}`.",
        "",
    ]
    return "\n".join(lines)


def write_dimension_csv(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bodies = {body["id"]: body for body in data["bodies"]}
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "dimension_id", "body_id", "node_1", "node_2", "body_role",
            "value", "units", "value_source", "uncertainty_mm", "confidence",
        ])
        for dimension in data["dimensions"]:
            body = bodies[dimension["body"]]
            value = "" if dimension.get("value") is None else dimension["value"]
            writer.writerow([
                dimension["id"], dimension["body"], *dimension["nodes"],
                body.get("role", ""), value, dimension.get("units", ""),
                dimension.get("value_source", ""), dimension.get("uncertainty_mm", ""),
                body.get("confidence", ""),
            ])


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("abstraction", type=Path, nargs="?", default=DEFAULT_ABSTRACTION)
    parser.add_argument("--markdown", type=Path, default=None,
                        help="write a Markdown report")
    parser.add_argument("--csv", type=Path, default=None,
                        help="write the dimension table as CSV")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = load_abstraction(args.abstraction)
    summary = validate_abstraction(data)
    for line in summary_lines(summary):
        print(line)
    if args.markdown is not None:
        args.markdown.parent.mkdir(parents=True, exist_ok=True)
        args.markdown.write_text(render_markdown(data, args.abstraction), encoding="utf-8")
        print(f"Wrote {args.markdown}")
    if args.csv is not None:
        write_dimension_csv(args.csv, data)
        print(f"Wrote {args.csv}")


if __name__ == "__main__":
    main()
