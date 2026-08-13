#!/usr/bin/env python3
"""Run the configuration-driven multi-objective Adam optimization skeleton."""

from __future__ import annotations

import argparse
import copy
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml


HERE = Path(__file__).resolve().parent
PROJECT_ROOT = HERE.parents[1]
DEFAULT_OBJECTIVES = HERE / "config" / "objectives.yaml"
DEFAULT_VARIABLES = HERE / "config" / "optimizable_variables.yaml"
MECHANISM_DIMENSION_IDS = (
    "L_ab", "L_bc", "L_cd", "L_ad", "L_ae", "L_de",
    "L_cg", "L_dg", "L_ef", "L_fg", "L_gh", "L_fh",
)


class OptimizationConfigError(ValueError):
    """Raised when the co-optimization configuration is incomplete or invalid."""


@dataclass(frozen=True)
class Variable:
    id: str
    initial: float
    minimum: float
    maximum: float


@dataclass(frozen=True)
class Objective:
    id: str
    weight: float
    source_path: Path
    data: dict[str, Any]


@dataclass(frozen=True)
class Problem:
    model: dict[str, Any]
    nominal_design_path: Path
    nominal_design: dict[str, Any]
    dorsal_clearance_mm: float
    distal_phalanx_width_mm: float
    variables: tuple[Variable, ...]
    objectives: tuple[Objective, ...]
    component_config: dict[str, Any]
    adam_config: dict[str, Any]
    normalize_objective_weights: bool


@dataclass(frozen=True)
class Evaluation:
    total_loss: float
    objective_losses: dict[str, float]
    component_losses: dict[str, dict[str, float]]


@dataclass(frozen=True)
class OptimizationResult:
    values: np.ndarray
    evaluation: Evaluation
    initial_evaluation: Evaluation
    iterations_completed: int
    converged: bool
    history: tuple[dict[str, float], ...]


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise OptimizationConfigError(f"configuration file not found: {path}")
    with path.open(encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise OptimizationConfigError(f"configuration must be a mapping: {path}")
    if data.get("schema_version") != 1:
        raise OptimizationConfigError(f"unsupported schema_version in {path}")
    return data


def _positive_number(value: Any, label: str) -> float:
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise OptimizationConfigError(f"{label} must be a positive finite number")
    return float(value)


def _dotted_value(data: dict[str, Any], dotted_path: str) -> float:
    value: Any = data
    for part in dotted_path.split("."):
        if not isinstance(value, dict) or part not in value:
            raise OptimizationConfigError(f"objective is missing target path {dotted_path}")
        value = value[part]
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise OptimizationConfigError(f"objective target {dotted_path} must be numeric")
    return float(value)


def load_problem(objectives_path: Path, variables_path: Path) -> Problem:
    manifest = _load_yaml(objectives_path)
    variable_data = _load_yaml(variables_path)

    rows = variable_data.get("variables")
    if not isinstance(rows, list) or not rows:
        raise OptimizationConfigError("optimizable_variables.yaml needs a non-empty variables list")
    variables: list[Variable] = []
    seen_variable_ids: set[str] = set()
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            raise OptimizationConfigError(f"variables[{index}] must be a mapping")
        variable_id = row.get("id")
        if not isinstance(variable_id, str) or not variable_id:
            raise OptimizationConfigError(f"variables[{index}] needs an id")
        if variable_id in seen_variable_ids:
            raise OptimizationConfigError(f"duplicate variable id: {variable_id}")
        initial, minimum, maximum = row.get("initial"), row.get("min"), row.get("max")
        if not all(isinstance(value, (int, float)) and math.isfinite(value)
                   for value in (initial, minimum, maximum)):
            raise OptimizationConfigError(f"variable {variable_id} bounds must be finite numbers")
        if not float(minimum) <= float(initial) <= float(maximum) or minimum == maximum:
            raise OptimizationConfigError(f"variable {variable_id} must satisfy min <= initial <= max")
        variables.append(Variable(variable_id, float(initial), float(minimum), float(maximum)))
        seen_variable_ids.add(variable_id)

    model = variable_data.get("model")
    if not isinstance(model, dict):
        raise OptimizationConfigError("optimizable_variables.yaml needs a model mapping")
    dimension_variables = model.get("mechanism_dimension_variables")
    if (not isinstance(dimension_variables, list)
            or tuple(dimension_variables) != MECHANISM_DIMENSION_IDS):
        raise OptimizationConfigError(
            "model.mechanism_dimension_variables must list the 12 mechanism dimensions"
        )
    tip_rod_variable = model.get("tip_rod_variable")
    for variable_id in [*dimension_variables, tip_rod_variable]:
        if variable_id not in seen_variable_ids:
            raise OptimizationConfigError(
                f"kinematic model references unknown variable {variable_id}"
            )
    nominal_design_relative = model.get("nominal_design")
    if not isinstance(nominal_design_relative, str):
        raise OptimizationConfigError("model.nominal_design must be a path")
    nominal_design = _load_yaml(
        (variables_path.parent / nominal_design_relative).resolve()
    )
    input_mount = next(
        (
            row for row in nominal_design.get("exoskeleton_attachments", [])
            if row.get("id") == "dorsal_input_mount"
        ),
        None,
    )
    if not isinstance(input_mount, dict):
        raise OptimizationConfigError("nominal design needs dorsal_input_mount")
    dorsal_clearance = input_mount.get("dorsal_clearance_mm")
    if (not isinstance(dorsal_clearance, (int, float))
            or not math.isfinite(dorsal_clearance) or dorsal_clearance < 0):
        raise OptimizationConfigError(
            "nominal dorsal_input_mount needs non-negative dorsal_clearance_mm"
        )
    if input_mount.get("clearance_control") != "upstream_manual_design_parameter":
        raise OptimizationConfigError("dorsal clearance must be controlled upstream")
    phalanges = {
        row.get("id"): row
        for row in nominal_design.get("human_hand_model", {}).get("phalanges", [])
    }
    distal_width = phalanges.get("distal_phalanx", {}).get("width_mm")
    if (not isinstance(distal_width, (int, float))
            or not math.isfinite(distal_width) or distal_width <= 0):
        raise OptimizationConfigError("nominal design needs a positive distal width")

    objective_rows = manifest.get("objectives")
    if not isinstance(objective_rows, list) or len(objective_rows) < 2:
        raise OptimizationConfigError("multi-objective Adam needs at least two objectives")
    objectives: list[Objective] = []
    seen_objective_ids: set[str] = set()
    for index, row in enumerate(objective_rows):
        if not isinstance(row, dict):
            raise OptimizationConfigError(f"objectives[{index}] must be a mapping")
        objective_id = row.get("id")
        relative_file = row.get("file")
        if not isinstance(objective_id, str) or not objective_id:
            raise OptimizationConfigError(f"objectives[{index}] needs an id")
        if objective_id in seen_objective_ids:
            raise OptimizationConfigError(f"duplicate objective id: {objective_id}")
        if not isinstance(relative_file, str) or not relative_file:
            raise OptimizationConfigError(f"objective {objective_id} needs a file")
        source_path = (objectives_path.parent / relative_file).resolve()
        data = _load_yaml(source_path)
        declared_id = data.get("objective", {}).get("id")
        if declared_id != objective_id:
            raise OptimizationConfigError(
                f"objective id mismatch: manifest={objective_id}, file={declared_id}"
            )
        if data.get("objective", {}).get("task_space_reference") != (
            "distal_phalanx_lower_slot_rp4"
        ):
            raise OptimizationConfigError(
                f"objective {objective_id} must target distal_phalanx_lower_slot_rp4"
            )
        for target_path in (
            "phalanx_lengths_mm.proximal",
            "phalanx_lengths_mm.middle",
            "phalanx_lengths_mm.distal",
            "joint_flexion_ranges_deg.mcp.max",
            "joint_flexion_ranges_deg.pip.max",
            "joint_flexion_ranges_deg.dip.max",
        ):
            _dotted_value(data, target_path)
        objectives.append(Objective(
            objective_id,
            _positive_number(row.get("weight", 1.0), f"objective {objective_id} weight"),
            source_path,
            data,
        ))
        seen_objective_ids.add(objective_id)

    scalarization = manifest.get("scalarization", {})
    if scalarization.get("method") != "weighted_sum":
        raise OptimizationConfigError("only scalarization.method: weighted_sum is supported")
    components = manifest.get("loss_components")
    if not isinstance(components, dict):
        raise OptimizationConfigError("loss_components must be a mapping")
    reachability_config = components.get("task_space_reachability")
    if not isinstance(reachability_config, dict) or not reachability_config.get("enabled"):
        raise OptimizationConfigError("task_space_reachability must be the active component")
    _positive_number(reachability_config.get("weight"), "task_space_reachability.weight")
    _positive_number(
        reachability_config.get("normalization_mm"),
        "task_space_reachability.normalization_mm",
    )
    _positive_number(
        reachability_config.get("soft_min_temperature"),
        "task_space_reachability.soft_min_temperature",
    )
    search_range = reachability_config.get("curled_input_search_deg")
    samples = reachability_config.get("curled_input_samples")
    if (not isinstance(search_range, list) or len(search_range) != 2
            or not all(isinstance(value, (int, float)) for value in search_range)
            or search_range[0] >= search_range[1]):
        raise OptimizationConfigError("curled_input_search_deg needs [minimum, maximum]")
    if not isinstance(samples, int) or samples < 3:
        raise OptimizationConfigError("curled_input_samples must be at least 3")
    horizontal_input = reachability_config.get("horizontal_input_deg")
    if (not isinstance(horizontal_input, (int, float))
            or not math.isclose(float(horizontal_input), float(search_range[0]))):
        raise OptimizationConfigError(
            "horizontal_input_deg must equal the start of curled_input_search_deg"
        )
    for component_id, component in components.items():
        if component_id != "task_space_reachability" and component.get("enabled"):
            raise OptimizationConfigError(
                f"only task_space_reachability may be enabled, not {component_id}"
            )

    adam = manifest.get("adam")
    if not isinstance(adam, dict):
        raise OptimizationConfigError("adam configuration is required")
    return Problem(
        model=model,
        nominal_design_path=(
            variables_path.parent / nominal_design_relative
        ).resolve(),
        nominal_design=nominal_design,
        dorsal_clearance_mm=float(dorsal_clearance),
        distal_phalanx_width_mm=float(distal_width),
        variables=tuple(variables),
        objectives=tuple(objectives),
        component_config=components,
        adam_config=adam,
        normalize_objective_weights=bool(
            scalarization.get("normalize_objective_weights", True)
        ),
    )


Point = tuple[float, float]


def _circle_intersections(a: Point, radius_a: float, b: Point, radius_b: float) -> tuple[Point, ...]:
    dx, dy = b[0] - a[0], b[1] - a[1]
    center_distance = math.hypot(dx, dy)
    tolerance = 1e-9
    if (center_distance <= tolerance
            or center_distance > radius_a + radius_b + tolerance
            or center_distance < abs(radius_a - radius_b) - tolerance):
        return ()
    along = (
        radius_a ** 2 - radius_b ** 2 + center_distance ** 2
    ) / (2 * center_distance)
    height_squared = radius_a ** 2 - along ** 2
    if height_squared < -tolerance:
        return ()
    height = math.sqrt(max(0.0, height_squared))
    unit_x, unit_y = dx / center_distance, dy / center_distance
    base = (a[0] + along * unit_x, a[1] + along * unit_y)
    normal = (-unit_y, unit_x)
    first = (base[0] + height * normal[0], base[1] + height * normal[1])
    if height <= tolerance:
        return (first,)
    return (first, (base[0] - height * normal[0], base[1] - height * normal[1]))


def _nearest(points: tuple[Point, ...], previous: Point) -> Point:
    return min(points, key=lambda point: math.dist(point, previous))


def _mechanism_workspace(
    lengths: dict[str, float], q_values_deg: np.ndarray,
) -> tuple[tuple[float, Point], ...]:
    """Analytically track the nominal A-B-C-D-G-F-H assembly branch."""
    a = (-lengths["L_ad"], 0.0)
    d = (0.0, 0.0)
    e_candidates = _circle_intersections(
        a, lengths["L_ae"], d, lengths["L_de"],
    )
    if not e_candidates:
        return ()
    e = max(e_candidates, key=lambda point: point[1])
    previous: dict[str, Point] | None = None
    workspace: list[tuple[float, Point]] = []
    for q_deg in q_values_deg:
        crank_angle = math.radians(90.0 - float(q_deg))
        b = (
            a[0] + lengths["L_ab"] * math.cos(crank_angle),
            a[1] + lengths["L_ab"] * math.sin(crank_angle),
        )
        c_candidates = _circle_intersections(
            b, lengths["L_bc"], d, lengths["L_cd"],
        )
        if not c_candidates:
            break
        c = (_nearest(c_candidates, previous["c"]) if previous
             else max(c_candidates, key=lambda point: point[1]))
        g_candidates = _circle_intersections(
            c, lengths["L_cg"], d, lengths["L_dg"],
        )
        if not g_candidates:
            break
        g = (_nearest(g_candidates, previous["g"]) if previous
             else max(g_candidates, key=lambda point: point[0]))
        f_candidates = _circle_intersections(
            e, lengths["L_ef"], g, lengths["L_fg"],
        )
        if not f_candidates:
            break
        f = (_nearest(f_candidates, previous["f"]) if previous
             else min(f_candidates, key=lambda point: point[1]))
        h_candidates = _circle_intersections(
            g, lengths["L_gh"], f, lengths["L_fh"],
        )
        if not h_candidates:
            break
        h = (_nearest(h_candidates, previous["h"]) if previous
             else max(h_candidates, key=lambda point: point[0]))
        workspace.append((float(q_deg), h))
        previous = {"c": c, "g": g, "f": f, "h": h}
    return tuple(workspace)


def _hand_distal_slot(
    problem: Problem, objective: Objective, curled: bool,
) -> tuple[Point, Point]:
    lengths = objective.data["phalanx_lengths_mm"]
    proximal = float(lengths["proximal"])
    middle = float(lengths["middle"])
    distal = float(lengths["distal"])
    if curled:
        ranges = objective.data["joint_flexion_ranges_deg"]
        mcp = math.radians(float(ranges["mcp"]["max"]))
        pip = math.radians(float(ranges["pip"]["max"]))
        dip = math.radians(float(ranges["dip"]["max"]))
    else:
        mcp = pip = dip = 0.0
    headings = (-mcp, -(mcp + pip), -(mcp + pip + dip))
    dip = (
        proximal * math.cos(headings[0]) + middle * math.cos(headings[1]),
        proximal * math.sin(headings[0]) + middle * math.sin(headings[1]),
    )
    tip = (
        dip[0] + distal * math.cos(headings[2]),
        dip[1] + distal * math.sin(headings[2]),
    )
    dorsal_normal = (-math.sin(headings[2]), math.cos(headings[2]))
    lower_offset = (
        -problem.distal_phalanx_width_mm * dorsal_normal[0],
        -problem.distal_phalanx_width_mm * dorsal_normal[1]
        - problem.dorsal_clearance_mm,
    )
    return (
        (dip[0] + lower_offset[0], dip[1] + lower_offset[1]),
        (tip[0] + lower_offset[0], tip[1] + lower_offset[1]),
    )


def _slot_rod_error(h: Point, slot: tuple[Point, Point], rod_length: float) -> tuple[float, float]:
    start, end = slot
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    length = math.sqrt(length_squared)
    hx, hy = h[0] - start[0], h[1] - start[1]
    projection = max(0.0, min(1.0, (hx * dx + hy * dy) / length_squared))
    closest = (start[0] + projection * dx, start[1] + projection * dy)
    minimum_distance = math.dist(h, closest)
    endpoint_distances = (math.dist(h, start), math.dist(h, end))
    maximum_distance = max(endpoint_distances)
    if rod_length < minimum_distance:
        return minimum_distance - rod_length, projection * length
    if rod_length > maximum_distance:
        far_fraction = 0.0 if endpoint_distances[0] >= endpoint_distances[1] else 1.0
        return rod_length - maximum_distance, far_fraction * length
    quadratic_b = 2 * ((start[0] - h[0]) * dx + (start[1] - h[1]) * dy)
    quadratic_c = (start[0] - h[0]) ** 2 + (start[1] - h[1]) ** 2 - rod_length ** 2
    discriminant = max(0.0, quadratic_b ** 2 - 4 * length_squared * quadratic_c)
    root = math.sqrt(discriminant)
    fractions = [
        (-quadratic_b - root) / (2 * length_squared),
        (-quadratic_b + root) / (2 * length_squared),
    ]
    valid = [fraction for fraction in fractions if 0.0 <= fraction <= 1.0]
    fraction = min(valid, key=lambda value: abs(value - 0.5)) if valid else projection
    return 0.0, fraction * length


def _soft_min(values: np.ndarray, temperature: float) -> float:
    minimum = float(np.min(values))
    return minimum - temperature * math.log(
        float(np.mean(np.exp(-(values - minimum) / temperature)))
    )


def evaluate(problem: Problem, values: np.ndarray) -> Evaluation:
    """Evaluate horizontal and curled lower-slot reachability through the output rod."""
    if values.shape != (len(problem.variables),):
        raise ValueError("candidate vector has the wrong shape")
    design_lengths = {
        variable.id: float(value) for variable, value in zip(problem.variables, values)
    }
    config = problem.component_config["task_space_reachability"]
    q_min, q_max = (float(value) for value in config["curled_input_search_deg"])
    q_values = np.linspace(q_min, q_max, int(config["curled_input_samples"]))
    workspace = _mechanism_workspace(design_lengths, q_values)
    normalization = float(config["normalization_mm"])
    temperature = float(config["soft_min_temperature"])
    rod_length = design_lengths[problem.model["tip_rod_variable"]]

    objective_losses: dict[str, float] = {}
    component_losses: dict[str, dict[str, float]] = {}
    weighted_total = 0.0
    objective_weight_sum = 0.0
    for objective in problem.objectives:
        horizontal_slot = _hand_distal_slot(problem, objective, curled=False)
        curled_slot = _hand_distal_slot(problem, objective, curled=True)
        if not workspace:
            horizontal_loss = curled_loss = 1.0e6
            best_q = math.nan
            horizontal_error = curled_error = normalization * 1.0e3
            horizontal_translation = curled_translation = math.nan
        else:
            horizontal_h = workspace[0][1]
            horizontal_error, horizontal_translation = _slot_rod_error(
                horizontal_h, horizontal_slot, rod_length,
            )
            horizontal_loss = (horizontal_error / normalization) ** 2
            curled_results = [
                _slot_rod_error(h, curled_slot, rod_length) for _, h in workspace
            ]
            curled_errors = np.asarray([result[0] for result in curled_results])
            curled_squared = (curled_errors / normalization) ** 2
            curled_loss = _soft_min(curled_squared, temperature)
            best_index = int(np.argmin(curled_squared))
            best_q = workspace[best_index][0]
            curled_error = float(curled_errors[best_index])
            curled_translation = curled_results[best_index][1]
        objective_loss = 0.5 * (horizontal_loss + curled_loss)
        components = {
            "task_space_reachability": objective_loss,
            "horizontal_slot_start_x_mm": horizontal_slot[0][0],
            "horizontal_slot_start_y_mm": horizontal_slot[0][1],
            "curled_slot_start_x_mm": curled_slot[0][0],
            "curled_slot_start_y_mm": curled_slot[0][1],
            "horizontal_slot_rod_error_mm": horizontal_error,
            "curled_slot_rod_error_mm": curled_error,
            "horizontal_slot_translation_mm": horizontal_translation,
            "curled_slot_translation_mm": curled_translation,
            "curled_best_input_deg": best_q,
        }
        objective_losses[objective.id] = objective_loss
        component_losses[objective.id] = components
        weighted_total += objective.weight * objective_loss
        objective_weight_sum += objective.weight
    if problem.normalize_objective_weights:
        weighted_total /= objective_weight_sum
    return Evaluation(weighted_total, objective_losses, component_losses)


def finite_difference_gradient(
    problem: Problem,
    values: np.ndarray,
    relative_step: float,
) -> np.ndarray:
    """Central finite differences keep the Adam engine independent of the future model."""
    lower = np.asarray([variable.minimum for variable in problem.variables])
    upper = np.asarray([variable.maximum for variable in problem.variables])
    gradient = np.zeros_like(values)
    for index, value in enumerate(values):
        step = relative_step * max(1.0, abs(float(value)))
        plus = values.copy()
        minus = values.copy()
        plus[index] = min(upper[index], value + step)
        minus[index] = max(lower[index], value - step)
        denominator = plus[index] - minus[index]
        if denominator == 0:
            continue
        gradient[index] = (
            evaluate(problem, plus).total_loss - evaluate(problem, minus).total_loss
        ) / denominator
    return gradient


def optimize(
    problem: Problem,
    iterations: int | None = None,
    learning_rate: float | None = None,
) -> OptimizationResult:
    """Minimize the weighted scalarization while preserving per-objective metrics."""
    config = problem.adam_config
    configured_iterations = config.get("iterations") if iterations is None else iterations
    if not isinstance(configured_iterations, int) or configured_iterations < 1:
        raise OptimizationConfigError("Adam iterations must be a positive integer")
    rate = config.get("learning_rate") if learning_rate is None else learning_rate
    rate = _positive_number(rate, "Adam learning_rate")
    beta1 = float(config.get("beta1", 0.9))
    beta2 = float(config.get("beta2", 0.999))
    epsilon = _positive_number(config.get("epsilon", 1e-8), "Adam epsilon")
    relative_step = _positive_number(
        config.get("finite_difference_relative_step", 1e-4),
        "finite_difference_relative_step",
    )
    tolerance = _positive_number(
        config.get("gradient_tolerance", 1e-8), "gradient_tolerance"
    )
    if not 0 <= beta1 < 1 or not 0 <= beta2 < 1:
        raise OptimizationConfigError("Adam beta1 and beta2 must be in [0, 1)")

    values = np.asarray([variable.initial for variable in problem.variables], dtype=float)
    lower = np.asarray([variable.minimum for variable in problem.variables])
    upper = np.asarray([variable.maximum for variable in problem.variables])
    first_moment = np.zeros_like(values)
    second_moment = np.zeros_like(values)
    initial_evaluation = evaluate(problem, values)
    best_values = values.copy()
    best_evaluation = initial_evaluation
    current_evaluation = initial_evaluation
    history: list[dict[str, float]] = [{
        "iteration": 0.0,
        "total_loss": initial_evaluation.total_loss,
        "gradient_norm": math.nan,
        "step_norm": 0.0,
        **initial_evaluation.objective_losses,
    }]
    converged = False
    completed = 0
    for iteration in range(1, configured_iterations + 1):
        gradient = finite_difference_gradient(problem, values, relative_step)
        gradient_norm = float(np.linalg.norm(gradient))
        if gradient_norm <= tolerance:
            converged = current_evaluation.total_loss < 1.0e5
            completed = iteration - 1
            break
        first_moment = beta1 * first_moment + (1.0 - beta1) * gradient
        second_moment = beta2 * second_moment + (1.0 - beta2) * gradient * gradient
        corrected_first = first_moment / (1.0 - beta1 ** iteration)
        corrected_second = second_moment / (1.0 - beta2 ** iteration)
        candidate = np.clip(
            values - rate * corrected_first / (np.sqrt(corrected_second) + epsilon),
            lower,
            upper,
        )
        step_norm = float(np.linalg.norm(candidate - values))
        values = candidate
        current_evaluation = evaluate(problem, values)
        if current_evaluation.total_loss < best_evaluation.total_loss:
            best_values = values.copy()
            best_evaluation = current_evaluation
        history.append({
            "iteration": float(iteration),
            "total_loss": current_evaluation.total_loss,
            "gradient_norm": gradient_norm,
            "step_norm": step_norm,
            **current_evaluation.objective_losses,
        })
        completed = iteration
    return OptimizationResult(
        best_values,
        best_evaluation,
        initial_evaluation,
        completed,
        converged,
        tuple(history),
    )


def _result_document(problem: Problem, result: OptimizationResult) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "status": "task_space_reachability_optimization_result",
        "model": problem.model.get("id", "unspecified"),
        "fixed_upstream_parameters": {
            "dorsal_clearance_mm": problem.dorsal_clearance_mm,
            "distal_phalanx_width_mm": problem.distal_phalanx_width_mm,
        },
        "warning": (
            "Only two-pose lower distal slot reachability is active. Energy, torque, "
            "collision, singularity, joint-limit, smoothness, and structural objectives "
            "are disabled. Do not apply this candidate without feasibility validation."
        ),
        "optimizer": {
            "name": "adam",
            "iterations_completed": result.iterations_completed,
            "converged": result.converged,
            "initial_total_loss": result.initial_evaluation.total_loss,
            "final_total_loss": result.evaluation.total_loss,
        },
        "candidate_variables": {
            variable.id: {
                "value": float(value),
                "units": "mm",
                "bounds": [variable.minimum, variable.maximum],
            }
            for variable, value in zip(problem.variables, result.values)
        },
        "objective_losses": result.evaluation.objective_losses,
        "component_losses": result.evaluation.component_losses,
    }


def _candidate_initial_positions(lengths: dict[str, float]) -> dict[str, Point]:
    """Reconstruct a closed q=0 assembly for a materialized candidate YAML."""
    a = (-lengths["L_ad"], 0.0)
    d = (0.0, 0.0)
    e_candidates = _circle_intersections(a, lengths["L_ae"], d, lengths["L_de"])
    if not e_candidates:
        raise OptimizationConfigError("candidate ground triangle A-D-E cannot assemble")
    e = max(e_candidates, key=lambda point: point[1])
    b = (a[0], lengths["L_ab"])
    c_candidates = _circle_intersections(b, lengths["L_bc"], d, lengths["L_cd"])
    if not c_candidates:
        raise OptimizationConfigError("candidate first loop A-B-C-D cannot assemble")
    c = max(c_candidates, key=lambda point: point[1])
    g_candidates = _circle_intersections(c, lengths["L_cg"], d, lengths["L_dg"])
    if not g_candidates:
        raise OptimizationConfigError("candidate rigid triangle C-D-G cannot assemble")
    g = max(g_candidates, key=lambda point: point[0])
    f_candidates = _circle_intersections(e, lengths["L_ef"], g, lengths["L_fg"])
    if not f_candidates:
        raise OptimizationConfigError("candidate second loop D-G-F-E cannot assemble")
    f = min(f_candidates, key=lambda point: point[1])
    h_candidates = _circle_intersections(g, lengths["L_gh"], f, lengths["L_fh"])
    if not h_candidates:
        raise OptimizationConfigError("candidate rigid triangle F-G-H cannot assemble")
    h = max(h_candidates, key=lambda point: point[0])
    return {"a": a, "b": b, "c": c, "d": d, "e": e, "f": f, "g": g, "h": h}


def _candidate_mechanism_document(
    problem: Problem, result: OptimizationResult, candidate_id: str,
) -> dict[str, Any]:
    document = copy.deepcopy(problem.nominal_design)
    values = {
        variable.id: float(value)
        for variable, value in zip(problem.variables, result.values)
    }
    for dimension in document["dimensions"]:
        dimension["value"] = values[dimension["id"]]
        dimension["value_source"] = "co_optimization_candidate"
    output_rod = next(
        row for row in document["exoskeleton_attachments"]
        if row["id"] == "distal_output_rod"
    )
    output_rod["assumed_length_mm"] = values[problem.model["tip_rod_variable"]]
    output_rod["value_source"] = "co_optimization_candidate"
    positions = _candidate_initial_positions(values)
    for node in document["nodes"]:
        node["initial_position_mm"] = list(positions[node["id"]])
    document["mechanism"]["status"] = "co_optimization_candidate"
    document["optimization_provenance"] = {
        "candidate_id": candidate_id,
        "parent_nominal_design": str(problem.nominal_design_path),
        "optimizer": "adam",
        "objective_status": "task_space_reachability_only",
        "promoted_to_nominal": False,
    }
    return document


def write_outputs(output_dir: Path, problem: Problem, result: OptimizationResult) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    candidate_id = "candidate_0001"
    candidate_dir = output_dir / candidate_id
    candidate_dir.mkdir(parents=True, exist_ok=True)
    result_path = candidate_dir / "candidate.yaml"
    mechanism_path = candidate_dir / "mechanism.yaml"
    history_path = output_dir / "history.csv"
    result_path.write_text(
        yaml.safe_dump(_result_document(problem, result), sort_keys=False),
        encoding="utf-8",
    )
    mechanism_path.write_text(
        yaml.safe_dump(
            _candidate_mechanism_document(problem, result, candidate_id),
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    objective_ids = [objective.id for objective in problem.objectives]
    with history_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "iteration", "total_loss", "gradient_norm", "step_norm", *objective_ids
            ],
            lineterminator="\n",
        )
        writer.writeheader()
        writer.writerows(result.history)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--objectives", type=Path, default=DEFAULT_OBJECTIVES)
    parser.add_argument("--variables", type=Path, default=DEFAULT_VARIABLES)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--iterations", type=int, default=None)
    parser.add_argument("--learning-rate", type=float, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    problem = load_problem(args.objectives.resolve(), args.variables.resolve())
    result = optimize(problem, args.iterations, args.learning_rate)
    write_outputs(args.output_dir.resolve(), problem, result)
    print(
        f"Adam multi-objective skeleton: {len(problem.objectives)} objectives, "
        f"{len(problem.variables)} variables"
    )
    print(
        f"Loss: {result.initial_evaluation.total_loss:.8g} -> "
        f"{result.evaluation.total_loss:.8g} in {result.iterations_completed} iterations"
    )
    print(f"Wrote {args.output_dir.resolve() / 'candidate_0001' / 'candidate.yaml'}")
    print(f"Wrote {args.output_dir.resolve() / 'candidate_0001' / 'mechanism.yaml'}")
    print(f"Wrote {args.output_dir.resolve() / 'history.csv'}")


if __name__ == "__main__":
    main()
