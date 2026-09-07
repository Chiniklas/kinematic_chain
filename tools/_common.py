"""Shared plumbing for the analysis tools.

`run_adam.py` lives in a directory with a hyphen in its name, so it cannot be
imported normally. Everything here loads it by path and exposes the handful of
helpers the tools need.
"""
from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
CONFIG_ROOT = ROOT / "src" / "co-optimization" / "config"
DESIGNS = sorted(p.name for p in CONFIG_ROOT.iterdir()
                 if p.is_dir() and (p / "objectives.yaml").exists())


def load_run_adam():
    """Import run_adam.py by path (its package directory name is not an identifier)."""
    path = ROOT / "src" / "co-optimization" / "run_adam.py"
    spec = importlib.util.spec_from_file_location("run_adam", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["run_adam"] = module
    sys.path.insert(0, str(path.parent))
    spec.loader.exec_module(module)
    return module


def load_design(design: str, finger: str, weights: dict[str, float] | None = None):
    """Return (module, problem) for one design profile narrowed to one finger.

    `weights` may override guidance/objective weights without touching the YAML:
    keys `closure`, `collision`, `rod`, `perpendicularity`.
    """
    import copy

    module = load_run_adam()
    config = CONFIG_ROOT / design
    if not config.is_dir():
        raise SystemExit(f"unknown design {design!r}; expected one of {DESIGNS}")
    problem = module.load_problem(config / "objectives.yaml",
                                  config / "optimizable_variables.yaml")
    available = [objective.finger for objective in problem.objectives]
    if finger not in available:
        raise SystemExit(f"unknown finger {finger!r}; expected one of {available}")
    problem = replace(problem, objectives=tuple(
        o for o in problem.objectives if o.finger == finger))

    if weights:
        cfg = copy.deepcopy(problem.component_config)
        if "closure" in weights:
            cfg["task_space_reachability"]["guidance_weight"] = weights["closure"]
        if "collision" in weights:
            cfg["hand_mechanism_non_collision"]["guidance_weight"] = weights["collision"]
        if "perpendicularity" in weights:
            cfg["output_link_perpendicularity"]["weight"] = weights["perpendicularity"]
        if "rod" in weights:
            cfg.setdefault("output_rod_length", {
                "enabled": True, "optimization_role": "design_objective",
                "objective_order": 2, "normalization_mm": 30.0,
                "metric": "squared_normalized_output_rod_length",
            })
            cfg["output_rod_length"]["enabled"] = True
            cfg["output_rod_length"]["weight"] = weights["rod"]
        problem = replace(problem, component_config=cfg)
    return module, problem


def variable_ids(problem) -> list[str]:
    return [v.id for v in problem.variables]


def initial_vector(problem) -> np.ndarray:
    return np.array([v.initial for v in problem.variables])


def bounds(problem) -> tuple[np.ndarray, np.ndarray]:
    return (np.array([v.minimum for v in problem.variables]),
            np.array([v.maximum for v in problem.variables]))


def seeded_problem(problem, values, caps: dict[str, float] | None = None):
    """A copy of `problem` starting at `values`, optionally with raised upper bounds."""
    caps = caps or {}
    return replace(problem, variables=tuple(
        replace(var, initial=float(val), maximum=caps.get(var.id, var.maximum))
        for var, val in zip(problem.variables, values)))


def passes(evaluation) -> int:
    return (int(evaluation.collision_free)
            + int(evaluation.rod_closure_feasible)
            + int(evaluation.hand_motion_feasible))


def violation(evaluation, tolerance_mm: float = 0.1) -> float:
    """Total hard-constraint violation in mm; 0 means feasible."""
    return (max(0.0, -evaluation.minimum_clearance_mm)
            + max(0.0, evaluation.maximum_rod_closure_error_mm - tolerance_mm))


def triangle_angle(a: float, b: float, c: float) -> float:
    """Interior angle in degrees between sides `a` and `b`, opposite side `c`."""
    cosine = (a * a + b * b - c * c) / (2 * a * b)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def fgh_angle(lengths: dict[str, float]) -> float:
    return triangle_angle(lengths["L_fg"], lengths["L_gh"], lengths["L_fh"])


def mark(ok: bool) -> str:
    return "PASS" if ok else "FAIL"


def report_row(label: str, evaluation, lengths: dict[str, Any]) -> str:
    return (f"{label:>18} | {passes(evaluation)}/3 "
            f"rod={lengths['L_tip_rod']:6.2f} "
            f"clear={evaluation.minimum_clearance_mm:+7.3f} "
            f"{mark(evaluation.minimum_clearance_mm >= 0):>4} "
            f"closure={evaluation.maximum_rod_closure_error_mm:8.4f} "
            f"{mark(evaluation.maximum_rod_closure_error_mm <= 0.1):>4} "
            f"curl={mark(evaluation.hand_motion_feasible):>4}")
