#!/usr/bin/env python3
"""Run one independent multi-objective Adam design job for each long finger."""

from __future__ import annotations

import argparse
import copy
import csv
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from tensorboard_logger import TensorBoardLogger


HERE = Path(__file__).resolve().parent
PROJECT_ROOT = HERE.parents[1]
DEFAULT_OBJECTIVES = HERE / "config" / "objectives.yaml"
DEFAULT_VARIABLES = HERE / "config" / "optimizable_variables.yaml"
MECHANISM_DIMENSION_IDS = (
    "L_ab", "L_bc", "L_cd", "L_ad", "L_ae", "L_de",
    "L_cg", "L_dg", "L_ef", "L_fg", "L_gh", "L_fh",
)
OPTIMIZABLE_MECHANISM_DIMENSION_IDS = tuple(
    dimension_id for dimension_id in MECHANISM_DIMENSION_IDS
    if dimension_id != "L_ad"
)
FIXED_MECHANISM_DIMENSION_IDS = ("L_ad",)
FINGERS = ("index", "middle", "ring", "little")
ACTIVE_COMPONENT_IDS = (
    "output_link_perpendicularity",
)
CONSTRAINT_GUIDANCE_IDS = (
    "task_space_reachability",
    "hand_mechanism_non_collision",
)
RECORDED_COMPONENT_IDS = (*CONSTRAINT_GUIDANCE_IDS, *ACTIVE_COMPONENT_IDS)


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
    finger: str
    id: str
    source_path: Path
    data: dict[str, Any]


@dataclass(frozen=True)
class Problem:
    model: dict[str, Any]
    nominal_design_path: Path
    nominal_design: dict[str, Any]
    dorsal_clearance_mm: float
    distal_phalanx_width_mm: float
    fixed_mechanism_dimensions: dict[str, float]
    variables: tuple[Variable, ...]
    objectives: tuple[Objective, ...]
    component_config: dict[str, Any]
    constraint_config: dict[str, Any]
    adam_config: dict[str, Any]
    normalize_objective_weights: bool


@dataclass(frozen=True)
class Evaluation:
    total_loss: float
    objective_losses: dict[str, float]
    component_losses: dict[str, dict[str, float]]
    collision_free: bool
    minimum_clearance_mm: float
    rod_closure_feasible: bool
    maximum_rod_closure_error_mm: float
    hand_motion_feasible: bool
    input_schedules_deg: dict[str, tuple[float, ...]]
    hand_progress_schedules: dict[str, tuple[float, ...]]


@dataclass(frozen=True)
class OptimizationResult:
    values: np.ndarray
    evaluation: Evaluation
    initial_evaluation: Evaluation
    iterations_completed: int
    converged: bool
    history: tuple[dict[str, float], ...]


@dataclass(frozen=True)
class WorkspaceMetrics:
    collision_loss: float
    minimum_clearance_mm: float
    collision_free_pose_fraction: float
    evaluated_pose_count: int
    perpendicularity_loss: float
    maximum_perpendicular_deviation_deg: float
    maximum_rod_closure_error_mm: float
    rod_closure_loss: float
    rod_closure_within_tolerance_fraction: float


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
            or tuple(dimension_variables) != OPTIMIZABLE_MECHANISM_DIMENSION_IDS):
        raise OptimizationConfigError(
            "model.mechanism_dimension_variables must list the 11 optimizable "
            "mechanism dimensions; L_ad is fixed"
        )
    fixed_dimension_ids = model.get("fixed_mechanism_dimensions")
    if (not isinstance(fixed_dimension_ids, list)
            or tuple(fixed_dimension_ids) != FIXED_MECHANISM_DIMENSION_IDS):
        raise OptimizationConfigError(
            "model.fixed_mechanism_dimensions must contain only fixed L_ad"
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
    nominal_dimensions = {
        row.get("id"): row.get("value")
        for row in nominal_design.get("dimensions", [])
        if isinstance(row, dict)
    }
    fixed_mechanism_dimensions: dict[str, float] = {}
    for dimension_id in fixed_dimension_ids:
        value = nominal_dimensions.get(dimension_id)
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise OptimizationConfigError(
                f"nominal design needs finite fixed dimension {dimension_id}"
            )
        if dimension_id in seen_variable_ids:
            raise OptimizationConfigError(
                f"fixed dimension {dimension_id} cannot also be optimizable"
            )
        fixed_mechanism_dimensions[dimension_id] = float(value)
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

    scope = manifest.get("optimization_scope")
    if not isinstance(scope, dict):
        raise OptimizationConfigError("objectives.yaml needs optimization_scope")
    expected_scope = {
        "problem_unit": "one_independent_mechanism_per_finger",
        "initialization": "shared_nominal_mechanism_2",
        "design_variable_sharing": "none",
        "aggregate_reports_are_postprocessing_only": True,
    }
    for key, expected in expected_scope.items():
        if scope.get(key) != expected:
            raise OptimizationConfigError(
                f"optimization_scope.{key} must be {expected!r}"
            )

    objective_rows = manifest.get("finger_problems")
    if not isinstance(objective_rows, list) or not objective_rows:
        raise OptimizationConfigError(
            "objectives.yaml needs a non-empty finger_problems list"
        )
    objectives: list[Objective] = []
    seen_objective_ids: set[str] = set()
    for index, row in enumerate(objective_rows):
        if not isinstance(row, dict):
            raise OptimizationConfigError(f"objectives[{index}] must be a mapping")
        objective_id = row.get("id")
        finger = row.get("finger")
        relative_file = row.get("file")
        if finger not in FINGERS:
            raise OptimizationConfigError(
                f"finger_problems[{index}].finger must be one of {FINGERS}"
            )
        if not isinstance(objective_id, str) or not objective_id:
            raise OptimizationConfigError(f"objectives[{index}] needs an id")
        if objective_id in seen_objective_ids:
            raise OptimizationConfigError(f"duplicate objective id: {objective_id}")
        if not isinstance(relative_file, str) or not relative_file:
            raise OptimizationConfigError(f"objective {objective_id} needs a file")
        source_path = (objectives_path.parent / relative_file).resolve()
        data = _load_yaml(source_path)
        declared_id = data.get("objective", {}).get("id")
        declared_finger = data.get("objective", {}).get("finger")
        if declared_id != objective_id:
            raise OptimizationConfigError(
                f"objective id mismatch: manifest={objective_id}, file={declared_id}"
            )
        if declared_finger != finger:
            raise OptimizationConfigError(
                f"finger mismatch: manifest={finger}, file={declared_finger}"
            )
        if data.get("objective", {}).get("task_space_reference") != (
            "distal_phalanx_upper_midpoint_r4"
        ):
            raise OptimizationConfigError(
                f"objective {objective_id} must target distal_phalanx_upper_midpoint_r4"
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
            finger,
            objective_id,
            source_path,
            data,
        ))
        seen_objective_ids.add(objective_id)

    configured_fingers = tuple(objective.finger for objective in objectives)
    if configured_fingers != FINGERS:
        raise OptimizationConfigError(
            f"finger_problems must list {FINGERS} in order, got {configured_fingers}"
        )

    scalarization = manifest.get("scalarization", {})
    if scalarization.get("method") != "weighted_sum":
        raise OptimizationConfigError("only scalarization.method: weighted_sum is supported")
    components = manifest.get("loss_components")
    if not isinstance(components, dict):
        raise OptimizationConfigError("loss_components must be a mapping")
    reachability_config = components.get("task_space_reachability")
    if not isinstance(reachability_config, dict) or not reachability_config.get("enabled"):
        raise OptimizationConfigError("task_space_reachability must be the active component")
    if reachability_config.get("implementation_status") != (
        "active_full_intended_workspace_rod_closure"
    ):
        raise OptimizationConfigError(
            "task_space_reachability must evaluate full-workspace rod closure"
        )
    if reachability_config.get("optimization_role") != "hard_constraint_guidance":
        raise OptimizationConfigError("task_space_reachability must guide a hard constraint")
    _positive_number(
        reachability_config.get("guidance_weight"),
        "task_space_reachability.guidance_weight",
    )
    _positive_number(
        reachability_config.get("normalization_mm"),
        "task_space_reachability.normalization_mm",
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
    collision_config = components.get("hand_mechanism_non_collision")
    if not isinstance(collision_config, dict) or not collision_config.get("enabled"):
        raise OptimizationConfigError(
            "hand_mechanism_non_collision must be active over the intended workspace"
        )
    if collision_config.get("implementation_status") != (
        "active_full_intended_workspace_sampled_constraint"
    ):
        raise OptimizationConfigError(
            "hand_mechanism_non_collision implementation status is inconsistent"
        )
    if collision_config.get("optimization_role") != "hard_constraint_guidance":
        raise OptimizationConfigError(
            "hand_mechanism_non_collision must guide a hard constraint"
        )
    collision_poses = collision_config.get("evaluation_poses")
    if not isinstance(collision_poses, int) or collision_poses < 3:
        raise OptimizationConfigError("collision evaluation_poses must be at least 3")
    if collision_config.get("passive_hand_progress_bounds") != [0.0, 1.0]:
        raise OptimizationConfigError(
            "collision passive_hand_progress_bounds must cover [0.0, 1.0]"
        )
    if collision_config.get("require_all_evaluation_poses") is not True:
        raise OptimizationConfigError(
            "collision must require every intended-workspace evaluation pose"
        )
    link_axis_samples = collision_config.get("link_axis_samples")
    if not isinstance(link_axis_samples, int) or link_axis_samples < 3:
        raise OptimizationConfigError("collision link_axis_samples must be at least 3")
    for name in (
        "guidance_weight",
        "safety_clearance_mm",
        "mechanism_link_radius_mm",
        "output_rod_radius_mm",
        "dorsal_mount_exclusion_radius_mm",
        "r4_contact_exclusion_radius_mm",
        "smooth_hinge_width_mm",
        "smooth_max_temperature",
    ):
        _positive_number(collision_config.get(name), f"hand_mechanism_non_collision.{name}")

    perpendicular_config = components.get("output_link_perpendicularity")
    if not isinstance(perpendicular_config, dict) or not perpendicular_config.get("enabled"):
        raise OptimizationConfigError(
            "output_link_perpendicularity must be an active workspace objective"
        )
    if perpendicular_config.get("implementation_status") != (
        "active_full_intended_workspace"
    ):
        raise OptimizationConfigError(
            "output_link_perpendicularity implementation status is inconsistent"
        )
    if perpendicular_config.get("optimization_role") != "design_objective":
        raise OptimizationConfigError(
            "output_link_perpendicularity must be the sole design objective"
        )
    if perpendicular_config.get("evaluation_poses") != collision_poses:
        raise OptimizationConfigError(
            "perpendicularity and collision must use the same intended-workspace poses"
        )
    for name in ("weight", "smooth_max_temperature"):
        _positive_number(
            perpendicular_config.get(name), f"output_link_perpendicularity.{name}"
        )

    implemented_components = set(RECORDED_COMPONENT_IDS)
    for component_id, component in components.items():
        if component_id not in implemented_components and component.get("enabled"):
            raise OptimizationConfigError(
                f"optimization component is not implemented: {component_id}"
            )

    constraints = manifest.get("constraint_penalties")
    if not isinstance(constraints, dict):
        raise OptimizationConfigError("constraint_penalties must be a mapping")
    motion_config = constraints.get("hand_motion_admissibility")
    if (not isinstance(motion_config, dict)
            or not motion_config.get("enabled")
            or motion_config.get("mode") != "hard_kinematic_constraint"):
        raise OptimizationConfigError(
            "hand_motion_admissibility must be an active hard kinematic constraint"
        )
    if motion_config.get("progress_bounds") != [0.0, 1.0]:
        raise OptimizationConfigError("hand curl progress must stay within [0, 1]")
    if (motion_config.get("monotonic_non_decreasing") is not True
            or motion_config.get("prohibit_dorsal_flexion") is not True
            or motion_config.get("require_terminal_maximum_curl") is not True):
        raise OptimizationConfigError(
            "hand motion must curl monotonically downward to the measured maximum"
        )
    if motion_config.get("evaluation_poses") != collision_poses:
        raise OptimizationConfigError(
            "hand motion and collision must use the same workspace poses"
        )
    _positive_number(
        motion_config.get("terminal_progress_tolerance"),
        "hand_motion_admissibility.terminal_progress_tolerance",
    )
    closure_config = constraints.get("closure_error")
    if not isinstance(closure_config, dict) or not closure_config.get("enabled"):
        raise OptimizationConfigError(
            "closure_error must be an active hard workspace constraint"
        )
    if closure_config.get("mode") != "hard_rejection_with_reachability_guidance":
        raise OptimizationConfigError("closure_error mode is inconsistent")
    if closure_config.get("evaluation_poses") != collision_poses:
        raise OptimizationConfigError(
            "rod closure and collision must use the same intended-workspace poses"
        )
    if closure_config.get("require_all_evaluation_poses") is not True:
        raise OptimizationConfigError(
            "rod closure must hold at every intended-workspace pose"
        )
    _positive_number(closure_config.get("tolerance_mm"), "closure_error.tolerance_mm")
    collision_constraint = constraints.get("collision_penalty")
    if (not isinstance(collision_constraint, dict)
            or not collision_constraint.get("enabled")
            or collision_constraint.get("mode")
            != "hard_rejection_with_clearance_guidance"):
        raise OptimizationConfigError(
            "collision_penalty must be an active hard constraint with guidance"
        )
    for constraint_id, constraint in constraints.items():
        if constraint_id not in {
            "hand_motion_admissibility", "closure_error", "collision_penalty",
        } and constraint.get("enabled"):
            raise OptimizationConfigError(
                f"optimization constraint is not implemented: {constraint_id}"
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
        fixed_mechanism_dimensions=fixed_mechanism_dimensions,
        variables=tuple(variables),
        objectives=tuple(objectives),
        component_config=components,
        constraint_config=constraints,
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


def _mechanism_workspace_poses(
    lengths: dict[str, float], q_values_deg: np.ndarray,
) -> tuple[tuple[float, dict[str, Point]], ...]:
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
    workspace: list[tuple[float, dict[str, Point]]] = []
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
        workspace.append((
            float(q_deg),
            {"a": a, "b": b, "c": c, "d": d, "e": e, "f": f, "g": g, "h": h},
        ))
        previous = {"c": c, "g": g, "f": f, "h": h}
    return tuple(workspace)


def _mechanism_workspace(
    lengths: dict[str, float], q_values_deg: np.ndarray,
) -> tuple[tuple[float, Point], ...]:
    return tuple(
        (q_deg, positions["h"])
        for q_deg, positions in _mechanism_workspace_poses(lengths, q_values_deg)
    )


def _hand_joint_geometry(
    problem: Problem, objective: Objective, progress: float,
) -> tuple[tuple[Point, Point, Point, Point], tuple[float, float, float]]:
    if not 0.0 <= progress <= 1.0:
        raise ValueError("finger trajectory progress must be within [0, 1]")
    lengths = objective.data["phalanx_lengths_mm"]
    ranges = objective.data["joint_flexion_ranges_deg"]
    mcp = math.radians(progress * float(ranges["mcp"]["max"]))
    pip = math.radians(progress * float(ranges["pip"]["max"]))
    dip = math.radians(progress * float(ranges["dip"]["max"]))
    headings = (-mcp, -(mcp + pip), -(mcp + pip + dip))
    segment_lengths = (
        float(lengths["proximal"]),
        float(lengths["middle"]),
        float(lengths["distal"]),
    )
    joints: list[Point] = [(0.0, -problem.dorsal_clearance_mm)]
    for length, heading in zip(segment_lengths, headings):
        joints.append((
            joints[-1][0] + length * math.cos(heading),
            joints[-1][1] + length * math.sin(heading),
        ))
    return tuple(joints), headings  # type: ignore[return-value]


def _hand_distal_contact_progress(
    problem: Problem, objective: Objective, progress: float,
) -> Point:
    joints, _ = _hand_joint_geometry(problem, objective, progress)
    dip, tip = joints[2], joints[3]
    return ((dip[0] + tip[0]) / 2.0, (dip[1] + tip[1]) / 2.0)


def _hand_distal_contact(
    problem: Problem, objective: Objective, curled: bool,
) -> Point:
    return _hand_distal_contact_progress(problem, objective, 1.0 if curled else 0.0)


def _fixed_contact_rod_error(
    h: Point, contact: Point, rod_length: float,
) -> float:
    return abs(math.dist(h, contact) - rod_length)


def _piecewise_roots(
    coordinates: np.ndarray, residuals: np.ndarray,
) -> list[float]:
    roots: list[float] = []
    for index in range(len(coordinates) - 1):
        left, right = float(residuals[index]), float(residuals[index + 1])
        if math.isclose(left, 0.0, abs_tol=1e-12):
            roots.append(float(coordinates[index]))
        if left * right < 0.0:
            fraction = abs(left) / (abs(left) + abs(right))
            roots.append(float(
                coordinates[index]
                + fraction * (coordinates[index + 1] - coordinates[index])
            ))
    if math.isclose(float(residuals[-1]), 0.0, abs_tol=1e-12):
        roots.append(float(coordinates[-1]))
    return roots


def _compliant_motion_schedule(
    problem: Problem,
    objective: Objective,
    workspace: tuple[tuple[float, Point], ...],
    rod_length: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Drive q and solve the passive hand progress from fixed H–R4 closure.

    The current ideal hand has one zero-stiffness curl coordinate constrained to the
    measured horizontal-to-maximum-curl path.  It is an output of the crank-driven
    mechanism, not a prescribed time law or a hand/crank transmission ratio.
    """
    pose_count = int(
        problem.component_config["hand_mechanism_non_collision"]["evaluation_poses"]
    )
    q_grid = np.asarray([q for q, _ in workspace], dtype=float)
    h_grid = tuple(h for _, h in workspace)
    if len(q_grid) < 2 or not math.isclose(q_grid[0], 0.0, abs_tol=1e-9):
        empty = np.asarray([], dtype=float)
        return empty, empty

    curled_contact = _hand_distal_contact_progress(problem, objective, 1.0)
    curled_residuals = np.asarray([
        math.dist(h, curled_contact) - rod_length for h in h_grid
    ])
    terminal_roots = [q for q in _piecewise_roots(q_grid, curled_residuals) if q > 0.0]
    if terminal_roots:
        terminal_q = min(terminal_roots)
    else:
        terminal_q = float(q_grid[int(np.argmin(np.abs(curled_residuals)))])
    if terminal_q <= 0.0:
        empty = np.asarray([], dtype=float)
        return empty, empty

    q_schedule = np.linspace(0.0, terminal_q, pose_count)
    # Interpolation on the dense q-search workspace selects the passive branch;
    # exact mechanism poses and exact closure residuals are evaluated afterward.
    h_x = np.interp(q_schedule, q_grid, [point[0] for point in h_grid])
    h_y = np.interp(q_schedule, q_grid, [point[1] for point in h_grid])
    progresses_grid = np.linspace(0.0, 1.0, 201)
    contacts = tuple(
        _hand_distal_contact_progress(problem, objective, float(progress))
        for progress in progresses_grid
    )
    progress_schedule = [0.0]
    for pose_index, h in enumerate(zip(h_x[1:], h_y[1:]), start=1):
        residuals = np.asarray([
            math.dist(h, contact) - rod_length for contact in contacts
        ])
        roots = _piecewise_roots(progresses_grid, residuals)
        previous_progress = progress_schedule[-1]
        forward_roots = [
            progress for progress in roots
            if progress >= previous_progress - 1e-9
        ]
        if forward_roots:
            selected = (
                max(forward_roots)
                if pose_index == pose_count - 1 and terminal_roots
                else min(
                    forward_roots,
                    key=lambda progress: abs(progress - previous_progress),
                )
            )
        else:
            forward_indices = np.flatnonzero(
                progresses_grid >= previous_progress - 1e-9
            )
            if len(forward_indices) == 0:
                empty = np.asarray([], dtype=float)
                return empty, empty
            selected_index = min(
                forward_indices,
                key=lambda index: (
                    abs(float(residuals[index])),
                    abs(float(progresses_grid[index]) - previous_progress),
                ),
            )
            selected = float(progresses_grid[selected_index])
        progress_schedule.append(max(previous_progress, selected))
    return q_schedule, np.asarray(progress_schedule, dtype=float)


def _point_segment_distance(point: Point, start: Point, end: Point) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-18:
        return math.dist(point, start)
    fraction = max(0.0, min(
        1.0,
        ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
        / length_squared,
    ))
    projection = (start[0] + fraction * dx, start[1] + fraction * dy)
    return math.dist(point, projection)


def _cross(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _segments_intersect(a: Point, b: Point, c: Point, d: Point) -> bool:
    tolerance = 1e-10
    ab_c, ab_d = _cross(a, b, c), _cross(a, b, d)
    cd_a, cd_b = _cross(c, d, a), _cross(c, d, b)
    if ((ab_c > tolerance and ab_d < -tolerance)
            or (ab_c < -tolerance and ab_d > tolerance)) and (
        (cd_a > tolerance and cd_b < -tolerance)
        or (cd_a < -tolerance and cd_b > tolerance)
    ):
        return True
    return any((
        abs(ab_c) <= tolerance and _point_segment_distance(c, a, b) <= tolerance,
        abs(ab_d) <= tolerance and _point_segment_distance(d, a, b) <= tolerance,
        abs(cd_a) <= tolerance and _point_segment_distance(a, c, d) <= tolerance,
        abs(cd_b) <= tolerance and _point_segment_distance(b, c, d) <= tolerance,
    ))


def _segment_distance(a: Point, b: Point, c: Point, d: Point) -> float:
    if _segments_intersect(a, b, c, d):
        return 0.0
    return min(
        _point_segment_distance(a, c, d),
        _point_segment_distance(b, c, d),
        _point_segment_distance(c, a, b),
        _point_segment_distance(d, a, b),
    )


def _trim_segment_endpoint(
    start: Point, end: Point, trim_mm: float, trim_start: bool,
) -> tuple[Point, Point] | None:
    length = math.dist(start, end)
    if length <= trim_mm:
        return None
    fraction = trim_mm / length
    if trim_start:
        start = (
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
        )
    else:
        end = (
            end[0] + fraction * (start[0] - end[0]),
            end[1] + fraction * (start[1] - end[1]),
        )
    return start, end


def _hand_capsules(
    problem: Problem, objective: Objective, progress: float,
) -> tuple[tuple[str, Point, Point, float], ...]:
    joints, headings = _hand_joint_geometry(problem, objective, progress)
    hand = problem.nominal_design["human_hand_model"]
    palm = hand["palm"]
    palm_width = float(palm["width_mm"])
    palm_y = -problem.dorsal_clearance_mm - palm_width / 2.0
    capsules: list[tuple[str, Point, Point, float]] = [(
        "palm",
        (-float(palm["length_mm"]), palm_y),
        (0.0, palm_y),
        palm_width / 2.0,
    )]
    phalanx_rows = {
        row["id"]: row for row in hand["phalanges"]
    }
    segment_ids = ("proximal_phalanx", "middle_phalanx", "distal_phalanx")
    for index, (segment_id, heading) in enumerate(zip(segment_ids, headings)):
        width = float(phalanx_rows[segment_id]["width_mm"])
        dorsal_normal = (-math.sin(heading), math.cos(heading))
        offset = (-width * dorsal_normal[0] / 2.0, -width * dorsal_normal[1] / 2.0)
        capsules.append((
            segment_id,
            (joints[index][0] + offset[0], joints[index][1] + offset[1]),
            (joints[index + 1][0] + offset[0], joints[index + 1][1] + offset[1]),
            width / 2.0,
        ))
    return tuple(capsules)


def _mechanism_segments(
    problem: Problem, positions: dict[str, Point], mount_exclusion_mm: float,
) -> tuple[tuple[str, Point, Point], ...]:
    segments: list[tuple[str, Point, Point]] = []
    for dimension in problem.nominal_design["dimensions"]:
        if dimension["body"] == "ground":
            continue
        first_id, second_id = dimension["nodes"]
        start, end = positions[first_id], positions[second_id]
        if first_id == "d":
            trimmed = _trim_segment_endpoint(start, end, mount_exclusion_mm, True)
        elif second_id == "d":
            trimmed = _trim_segment_endpoint(start, end, mount_exclusion_mm, False)
        else:
            trimmed = (start, end)
        if trimmed is not None:
            segments.append((dimension["id"], *trimmed))
    return tuple(segments)


def _mechanism_polygons(
    problem: Problem, positions: dict[str, Point], mount_exclusion_mm: float,
) -> tuple[tuple[str, tuple[Point, ...]], ...]:
    polygons: list[tuple[str, tuple[Point, ...]]] = []
    for body in problem.nominal_design["bodies"]:
        node_ids = body["nodes"]
        if body["kind"] == "ground" or len(node_ids) < 3:
            continue
        points = [positions[node_id] for node_id in node_ids]
        if "d" in node_ids:
            index = node_ids.index("d")
            previous_point = points[index - 1]
            d_point = points[index]
            next_point = points[(index + 1) % len(points)]
            previous_trim = _trim_segment_endpoint(
                d_point, previous_point, mount_exclusion_mm, True,
            )
            next_trim = _trim_segment_endpoint(
                d_point, next_point, mount_exclusion_mm, True,
            )
            if previous_trim is None or next_trim is None:
                continue
            points = (
                points[:index]
                + [previous_trim[0], next_trim[0]]
                + points[index + 1:]
            )
        polygons.append((body["id"], tuple(points)))
    return tuple(polygons)


def _point_in_polygon(point: Point, polygon: tuple[Point, ...]) -> bool:
    inside = False
    x, y = point
    previous = polygon[-1]
    for current in polygon:
        x1, y1 = previous
        x2, y2 = current
        if ((y1 > y) != (y2 > y)):
            crossing_x = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < crossing_x:
                inside = not inside
        previous = current
    return inside


def _point_polygon_signed_distance(point: Point, polygon: tuple[Point, ...]) -> float:
    distance = min(
        _point_segment_distance(point, polygon[index - 1], polygon[index])
        for index in range(len(polygon))
    )
    return -distance if _point_in_polygon(point, polygon) else distance


def _segment_polygon_signed_distance(
    start: Point, end: Point, polygon: tuple[Point, ...], samples: int,
) -> float:
    if any(
        _segments_intersect(start, end, polygon[index - 1], polygon[index])
        for index in range(len(polygon))
    ):
        return 0.0
    return min(
        _point_polygon_signed_distance((
            start[0] + fraction * (end[0] - start[0]),
            start[1] + fraction * (end[1] - start[1]),
        ), polygon)
        for fraction in np.linspace(0.0, 1.0, samples)
    )


def _softplus(value: float, width: float) -> float:
    scaled = value / width
    if scaled > 40.0:
        return value
    if scaled < -40.0:
        return width * math.exp(scaled)
    return width * math.log1p(math.exp(scaled))


def _smooth_max(values: list[float], temperature: float) -> float:
    maximum = max(values)
    return maximum + temperature * math.log(
        sum(math.exp((value - maximum) / temperature) for value in values)
        / len(values)
    )


def _sampled_capsule_clearances(
    start: Point,
    end: Point,
    radius: float,
    hand_start: Point,
    hand_end: Point,
    hand_radius: float,
    samples: int,
) -> tuple[float, ...]:
    """Provide collision gradients even while two finite segment axes intersect."""
    return tuple(
        _point_segment_distance(
            (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            ),
            hand_start,
            hand_end,
        ) - radius - hand_radius
        for fraction in np.linspace(0.0, 1.0, samples)
    )


def _workspace_collision(
    problem: Problem,
    objective: Objective,
    design_lengths: dict[str, float],
    rod_length: float,
    q_values: np.ndarray,
    hand_progress_values: np.ndarray,
) -> WorkspaceMetrics:
    """Evaluate the crank-driven mechanism and its passive hand response."""
    config = problem.component_config["hand_mechanism_non_collision"]
    pose_count = int(config["evaluation_poses"])
    if (q_values.shape != (pose_count,)
            or hand_progress_values.shape != (pose_count,)):
        mechanism_poses = ()
    else:
        mechanism_poses = _mechanism_workspace_poses(design_lengths, q_values)
    if len(mechanism_poses) != pose_count:
        return WorkspaceMetrics(
            collision_loss=1.0e6,
            minimum_clearance_mm=-1.0e6,
            collision_free_pose_fraction=0.0,
            evaluated_pose_count=len(mechanism_poses),
            perpendicularity_loss=1.0e6,
            maximum_perpendicular_deviation_deg=90.0,
            maximum_rod_closure_error_mm=1.0e6,
            rod_closure_loss=1.0e6,
            rod_closure_within_tolerance_fraction=0.0,
        )

    safety_clearance = float(config["safety_clearance_mm"])
    mechanism_radius = float(config["mechanism_link_radius_mm"])
    rod_radius = float(config["output_rod_radius_mm"])
    mount_exclusion = float(config["dorsal_mount_exclusion_radius_mm"])
    r4_exclusion = float(config["r4_contact_exclusion_radius_mm"])
    link_axis_samples = int(config["link_axis_samples"])
    hinge_width = float(config["smooth_hinge_width_mm"])
    smooth_temperature = float(config["smooth_max_temperature"])
    perpendicular_config = problem.component_config["output_link_perpendicularity"]
    perpendicular_temperature = float(perpendicular_config["smooth_max_temperature"])
    closure_config = problem.constraint_config["closure_error"]
    closure_tolerance = float(closure_config["tolerance_mm"])
    closure_normalization = float(
        problem.component_config["task_space_reachability"]["normalization_mm"]
    )
    normalized_violations: list[float] = []
    pose_minima: list[float] = []
    perpendicular_errors: list[float] = []
    perpendicular_deviations_deg: list[float] = []
    rod_closure_errors: list[float] = []

    for progress, (_, positions) in zip(hand_progress_values, mechanism_poses):
        hand_capsules = _hand_capsules(problem, objective, float(progress))
        pose_clearances: list[float] = []
        guidance_clearances: list[float] = []
        for _, start, end in _mechanism_segments(
            problem, positions, mount_exclusion,
        ):
            for _, hand_start, hand_end, hand_radius in hand_capsules:
                pose_clearances.append(
                    _segment_distance(start, end, hand_start, hand_end)
                    - mechanism_radius - hand_radius
                )
                guidance_clearances.extend(_sampled_capsule_clearances(
                    start,
                    end,
                    mechanism_radius,
                    hand_start,
                    hand_end,
                    hand_radius,
                    link_axis_samples,
                ))
        for _, polygon in _mechanism_polygons(
            problem, positions, mount_exclusion,
        ):
            for _, hand_start, hand_end, hand_radius in hand_capsules:
                centerline_clearance = _segment_polygon_signed_distance(
                    hand_start, hand_end, polygon, link_axis_samples,
                )
                pose_clearances.append(
                    centerline_clearance - mechanism_radius - hand_radius
                )
                guidance_clearances.extend(
                    _point_polygon_signed_distance((
                        hand_start[0] + fraction * (hand_end[0] - hand_start[0]),
                        hand_start[1] + fraction * (hand_end[1] - hand_start[1]),
                    ), polygon) - mechanism_radius - hand_radius
                    for fraction in np.linspace(0.0, 1.0, link_axis_samples)
                )

        hand_joints, _ = _hand_joint_geometry(problem, objective, float(progress))
        distal_start, distal_end = hand_joints[2], hand_joints[3]
        contact = _hand_distal_contact_progress(problem, objective, float(progress))
        closure_error = _fixed_contact_rod_error(
            positions["h"], contact, rod_length,
        )
        rod_closure_errors.append(closure_error)
        rod_dx = positions["h"][0] - contact[0]
        rod_dy = positions["h"][1] - contact[1]
        rod_norm = math.hypot(rod_dx, rod_dy)
        distal_dx = distal_end[0] - distal_start[0]
        distal_dy = distal_end[1] - distal_start[1]
        distal_norm = math.hypot(distal_dx, distal_dy)
        if rod_norm <= 1e-12 or distal_norm <= 1e-12:
            tangential_component = 1.0
        else:
            tangential_component = abs(
                (rod_dx * distal_dx + rod_dy * distal_dy)
                / (rod_norm * distal_norm)
            )
            tangential_component = max(0.0, min(1.0, tangential_component))
        perpendicular_errors.append(tangential_component ** 2)
        perpendicular_deviations_deg.append(
            math.degrees(math.asin(tangential_component))
        )
        trimmed_rod = _trim_segment_endpoint(
            positions["h"], contact, r4_exclusion, False,
        )
        if trimmed_rod is not None:
            for _, hand_start, hand_end, hand_radius in hand_capsules:
                pose_clearances.append(
                    _segment_distance(*trimmed_rod, hand_start, hand_end)
                    - rod_radius - hand_radius
                )
                guidance_clearances.extend(_sampled_capsule_clearances(
                    *trimmed_rod,
                    rod_radius,
                    hand_start,
                    hand_end,
                    hand_radius,
                    link_axis_samples,
                ))

        pose_minimum = min(pose_clearances)
        pose_minima.append(pose_minimum)
        for clearance in guidance_clearances:
            violation = _softplus(safety_clearance - clearance, hinge_width)
            normalized_violations.append((violation / safety_clearance) ** 2)

    return WorkspaceMetrics(
        collision_loss=_smooth_max(normalized_violations, smooth_temperature),
        minimum_clearance_mm=min(pose_minima),
        collision_free_pose_fraction=(
            sum(clearance >= 0.0 for clearance in pose_minima) / pose_count
        ),
        evaluated_pose_count=pose_count,
        perpendicularity_loss=_smooth_max(
            perpendicular_errors, perpendicular_temperature,
        ),
        maximum_perpendicular_deviation_deg=max(perpendicular_deviations_deg),
        maximum_rod_closure_error_mm=max(rod_closure_errors),
        rod_closure_loss=float(np.mean(
            (np.asarray(rod_closure_errors) / closure_normalization) ** 2
        )),
        rod_closure_within_tolerance_fraction=(
            sum(error <= closure_tolerance for error in rod_closure_errors)
            / pose_count
        ),
    )


def _candidate_lengths(problem: Problem, values: np.ndarray) -> dict[str, float]:
    """Combine fixed nominal dimensions with one finger's optimizable vector."""
    return {
        **problem.fixed_mechanism_dimensions,
        **{
            variable.id: float(value)
            for variable, value in zip(problem.variables, values)
        },
    }


def evaluate(problem: Problem, values: np.ndarray) -> Evaluation:
    """Evaluate endpoint reachability and full-workspace hand collision."""
    if values.shape != (len(problem.variables),):
        raise ValueError("candidate vector has the wrong shape")
    design_lengths = _candidate_lengths(problem, values)
    config = problem.component_config["task_space_reachability"]
    q_min, q_max = (float(value) for value in config["curled_input_search_deg"])
    q_values = np.linspace(q_min, q_max, int(config["curled_input_samples"]))
    workspace = _mechanism_workspace(design_lengths, q_values)
    normalization = float(config["normalization_mm"])
    rod_length = design_lengths[problem.model["tip_rod_variable"]]

    objective_losses: dict[str, float] = {}
    component_losses: dict[str, dict[str, float]] = {}
    total = 0.0
    minimum_clearance = math.inf
    all_collision_free = True
    maximum_rod_closure_error = 0.0
    all_rod_closure_feasible = True
    all_hand_motion_feasible = True
    input_schedules: dict[str, tuple[float, ...]] = {}
    hand_progress_schedules: dict[str, tuple[float, ...]] = {}
    for objective in problem.objectives:
        horizontal_contact = _hand_distal_contact(problem, objective, curled=False)
        curled_contact = _hand_distal_contact(problem, objective, curled=True)
        if workspace:
            q_schedule, hand_progress_schedule = _compliant_motion_schedule(
                problem, objective, workspace, rod_length,
            )
        else:
            q_schedule = hand_progress_schedule = np.asarray([], dtype=float)
        coupled_workspace = (
            _mechanism_workspace(design_lengths, q_schedule)
            if len(q_schedule) else ()
        )
        requested_pose_count = int(
            problem.component_config["hand_mechanism_non_collision"]["evaluation_poses"]
        )
        if len(coupled_workspace) != requested_pose_count:
            horizontal_loss = curled_loss = 1.0e6
            best_q = math.nan
            horizontal_error = curled_error = normalization * 1.0e3
            workspace_metrics = WorkspaceMetrics(
                collision_loss=1.0e6,
                minimum_clearance_mm=-1.0e6,
                collision_free_pose_fraction=0.0,
                evaluated_pose_count=0,
                perpendicularity_loss=1.0e6,
                maximum_perpendicular_deviation_deg=90.0,
                maximum_rod_closure_error_mm=1.0e6,
                rod_closure_loss=1.0e6,
                rod_closure_within_tolerance_fraction=0.0,
            )
            input_schedules[objective.id] = ()
            hand_progress_schedules[objective.id] = ()
        else:
            horizontal_h = coupled_workspace[0][1]
            horizontal_error = _fixed_contact_rod_error(
                horizontal_h, horizontal_contact, rod_length,
            )
            horizontal_loss = (horizontal_error / normalization) ** 2
            best_q, curled_h = coupled_workspace[-1]
            curled_error = _fixed_contact_rod_error(
                curled_h, curled_contact, rod_length,
            )
            curled_loss = (curled_error / normalization) ** 2
            workspace_metrics = _workspace_collision(
                problem,
                objective,
                design_lengths,
                rod_length,
                q_schedule,
                hand_progress_schedule,
            )
            input_schedules[objective.id] = tuple(float(q) for q in q_schedule)
            hand_progress_schedules[objective.id] = tuple(
                float(progress) for progress in hand_progress_schedule
            )
        endpoint_reachability_loss = 0.5 * (horizontal_loss + curled_loss)
        reachability_loss = workspace_metrics.rod_closure_loss
        reachability_weight = float(config["guidance_weight"])
        collision_config = problem.component_config["hand_mechanism_non_collision"]
        collision_weight = float(collision_config["guidance_weight"])
        perpendicular_config = problem.component_config["output_link_perpendicularity"]
        perpendicular_weight = float(perpendicular_config["weight"])
        weight_sum = reachability_weight + collision_weight + perpendicular_weight
        weighted_loss = (
            reachability_weight * reachability_loss
            + collision_weight * workspace_metrics.collision_loss
            + perpendicular_weight * workspace_metrics.perpendicularity_loss
        )
        objective_loss = (
            weighted_loss / weight_sum
            if problem.normalize_objective_weights else weighted_loss
        )
        requested_collision_poses = int(collision_config["evaluation_poses"])
        collision_free = (
            workspace_metrics.evaluated_pose_count == requested_collision_poses
            and workspace_metrics.minimum_clearance_mm >= 0.0
        )
        closure_config = problem.constraint_config["closure_error"]
        closure_tolerance = float(closure_config["tolerance_mm"])
        rod_closure_feasible = (
            workspace_metrics.evaluated_pose_count
            == int(closure_config["evaluation_poses"])
            and workspace_metrics.maximum_rod_closure_error_mm
            <= closure_tolerance
        )
        motion_config = problem.constraint_config["hand_motion_admissibility"]
        progress_tolerance = float(motion_config["terminal_progress_tolerance"])
        hand_motion_feasible = (
            len(hand_progress_schedule) == requested_pose_count
            and math.isclose(float(hand_progress_schedule[0]), 0.0, abs_tol=1e-9)
            and float(hand_progress_schedule[-1]) >= 1.0 - progress_tolerance
            and bool(np.all(hand_progress_schedule >= -1e-9))
            and bool(np.all(hand_progress_schedule <= 1.0 + 1e-9))
            and bool(np.all(np.diff(hand_progress_schedule) >= -1e-9))
        )
        components = {
            "task_space_reachability": reachability_loss,
            "endpoint_reachability_diagnostic": endpoint_reachability_loss,
            "hand_mechanism_non_collision": workspace_metrics.collision_loss,
            "output_link_perpendicularity": workspace_metrics.perpendicularity_loss,
            "guidance_weight_task_space_reachability": reachability_weight,
            "guidance_weight_hand_mechanism_non_collision": collision_weight,
            "weight_output_link_perpendicularity": perpendicular_weight,
            "weighted_closure_constraint_guidance": (
                reachability_weight * reachability_loss / weight_sum
            ),
            "weighted_collision_constraint_guidance": (
                collision_weight * workspace_metrics.collision_loss / weight_sum
            ),
            "weighted_output_link_perpendicularity": (
                perpendicular_weight * workspace_metrics.perpendicularity_loss
                / weight_sum
            ),
            "minimum_signed_clearance_mm": workspace_metrics.minimum_clearance_mm,
            "collision_free_pose_fraction": (
                workspace_metrics.collision_free_pose_fraction
            ),
            "collision_evaluated_pose_count": float(
                workspace_metrics.evaluated_pose_count
            ),
            "collision_requested_pose_count": float(requested_collision_poses),
            "whole_intended_workspace_collision_free": float(collision_free),
            "maximum_perpendicular_deviation_deg": (
                workspace_metrics.maximum_perpendicular_deviation_deg
            ),
            "maximum_workspace_rod_closure_error_mm": (
                workspace_metrics.maximum_rod_closure_error_mm
            ),
            "rod_closure_within_tolerance_fraction": (
                workspace_metrics.rod_closure_within_tolerance_fraction
            ),
            "rod_closure_tolerance_mm": closure_tolerance,
            "whole_intended_workspace_rod_closure_feasible": float(
                rod_closure_feasible
            ),
            "hand_motion_admissible": float(hand_motion_feasible),
            "hand_motion_monotonic_non_decreasing": float(
                len(hand_progress_schedule) == requested_pose_count
                and bool(np.all(np.diff(hand_progress_schedule) >= -1e-9))
            ),
            "terminal_hand_curl_progress": (
                float(hand_progress_schedule[-1])
                if len(hand_progress_schedule) else math.nan
            ),
            "horizontal_r4_x_mm": horizontal_contact[0],
            "horizontal_r4_y_mm": horizontal_contact[1],
            "curled_r4_x_mm": curled_contact[0],
            "curled_r4_y_mm": curled_contact[1],
            "horizontal_fixed_contact_rod_error_mm": horizontal_error,
            "curled_fixed_contact_rod_error_mm": curled_error,
            "curled_best_input_deg": best_q,
        }
        objective_losses[objective.id] = objective_loss
        component_losses[objective.id] = components
        total += objective_loss
        minimum_clearance = min(
            minimum_clearance, workspace_metrics.minimum_clearance_mm,
        )
        all_collision_free = all_collision_free and collision_free
        maximum_rod_closure_error = max(
            maximum_rod_closure_error,
            workspace_metrics.maximum_rod_closure_error_mm,
        )
        all_rod_closure_feasible = (
            all_rod_closure_feasible and rod_closure_feasible
        )
        all_hand_motion_feasible = (
            all_hand_motion_feasible and hand_motion_feasible
        )
    if problem.normalize_objective_weights:
        total /= len(problem.objectives)
    return Evaluation(
        total,
        objective_losses,
        component_losses,
        all_collision_free,
        minimum_clearance,
        all_rod_closure_feasible,
        maximum_rod_closure_error,
        all_hand_motion_feasible,
        input_schedules,
        hand_progress_schedules,
    )


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


def _is_better_candidate(candidate: Evaluation, incumbent: Evaluation) -> bool:
    """Hard feasibility outranks every weighted objective value."""
    candidate_passes = sum((
        int(candidate.collision_free),
        int(candidate.rod_closure_feasible),
        int(candidate.hand_motion_feasible),
    ))
    incumbent_passes = sum((
        int(incumbent.collision_free),
        int(incumbent.rod_closure_feasible),
        int(incumbent.hand_motion_feasible),
    ))
    if candidate_passes != incumbent_passes:
        return candidate_passes > incumbent_passes
    return candidate.total_loss < incumbent.total_loss


def _tensorboard_scalars(
    problem: Problem,
    evaluation: Evaluation,
    values: np.ndarray,
    best_total_loss: float,
    learning_rate: float,
    gradient: np.ndarray | None = None,
    gradient_norm: float | None = None,
    step_norm: float | None = None,
) -> dict[str, float]:
    scalars = {
        "loss/total": evaluation.total_loss,
        "loss/best_total": best_total_loss,
        "optimizer/learning_rate": learning_rate,
        "constraints/collision_free": float(evaluation.collision_free),
        "constraints/minimum_signed_clearance_mm": evaluation.minimum_clearance_mm,
        "constraints/rod_closure_feasible": float(
            evaluation.rod_closure_feasible
        ),
        "constraints/hand_motion_feasible": float(
            evaluation.hand_motion_feasible
        ),
        "constraints/maximum_rod_closure_error_mm": (
            evaluation.maximum_rod_closure_error_mm
        ),
    }
    scalars.update({
        f"loss/objective/{objective_id}": loss
        for objective_id, loss in evaluation.objective_losses.items()
    })
    for objective_id, components in evaluation.component_losses.items():
        scalars.update({
            f"metrics/{objective_id}/{component_id}": value
            for component_id, value in components.items()
        })
        scalars.update({
            f"loss/component/{component_id}": components[component_id]
            for component_id in RECORDED_COMPONENT_IDS
        })
    for variable, value in zip(problem.variables, values):
        scalars[f"design_variables_mm/{variable.id}"] = float(value)
        scalars[f"bounds_min_mm/{variable.id}"] = variable.minimum
        scalars[f"bounds_max_mm/{variable.id}"] = variable.maximum
    if gradient is not None:
        scalars.update({
            f"gradients/{variable.id}": float(value)
            for variable, value in zip(problem.variables, gradient)
        })
    if gradient_norm is not None:
        scalars["optimizer/gradient_norm"] = gradient_norm
    if step_norm is not None:
        scalars["optimizer/step_norm"] = step_norm
    return scalars


def _history_components(evaluation: Evaluation) -> dict[str, float]:
    if len(evaluation.component_losses) != 1:
        return {}
    components = next(iter(evaluation.component_losses.values()))
    return {
        component_id: components[component_id]
        for component_id in RECORDED_COMPONENT_IDS
    }


def optimize(
    problem: Problem,
    iterations: int | None = None,
    learning_rate: float | None = None,
    tensorboard_logger: TensorBoardLogger | None = None,
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
        "collision_free": float(initial_evaluation.collision_free),
        "minimum_clearance_mm": initial_evaluation.minimum_clearance_mm,
        "rod_closure_feasible": float(initial_evaluation.rod_closure_feasible),
        "hand_motion_feasible": float(initial_evaluation.hand_motion_feasible),
        "maximum_rod_closure_error_mm": (
            initial_evaluation.maximum_rod_closure_error_mm
        ),
        **_history_components(initial_evaluation),
        **initial_evaluation.objective_losses,
    }]
    if tensorboard_logger is not None:
        tensorboard_logger.add_scalars(
            _tensorboard_scalars(
                problem,
                initial_evaluation,
                values,
                initial_evaluation.total_loss,
                rate,
                step_norm=0.0,
            ),
            step=0,
        )
    converged = False
    completed = 0
    for iteration in range(1, configured_iterations + 1):
        gradient = finite_difference_gradient(problem, values, relative_step)
        gradient_norm = float(np.linalg.norm(gradient))
        if gradient_norm <= tolerance:
            converged = (
                current_evaluation.collision_free
                and current_evaluation.rod_closure_feasible
                and current_evaluation.hand_motion_feasible
                and current_evaluation.total_loss < 1.0e5
            )
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
        if _is_better_candidate(current_evaluation, best_evaluation):
            best_values = values.copy()
            best_evaluation = current_evaluation
        history.append({
            "iteration": float(iteration),
            "total_loss": current_evaluation.total_loss,
            "gradient_norm": gradient_norm,
            "step_norm": step_norm,
            "collision_free": float(current_evaluation.collision_free),
            "minimum_clearance_mm": current_evaluation.minimum_clearance_mm,
            "rod_closure_feasible": float(
                current_evaluation.rod_closure_feasible
            ),
            "hand_motion_feasible": float(
                current_evaluation.hand_motion_feasible
            ),
            "maximum_rod_closure_error_mm": (
                current_evaluation.maximum_rod_closure_error_mm
            ),
            **_history_components(current_evaluation),
            **current_evaluation.objective_losses,
        })
        if tensorboard_logger is not None:
            tensorboard_logger.add_scalars(
                _tensorboard_scalars(
                    problem,
                    current_evaluation,
                    values,
                    best_evaluation.total_loss,
                    rate,
                    gradient,
                    gradient_norm,
                    step_norm,
                ),
                step=iteration,
            )
        completed = iteration
    return OptimizationResult(
        best_values,
        best_evaluation,
        initial_evaluation,
        completed,
        converged,
        tuple(history),
    )


def _single_finger(problem: Problem) -> str:
    if len(problem.objectives) != 1:
        raise OptimizationConfigError(
            "an optimization job must contain exactly one finger target"
        )
    return problem.objectives[0].finger


def _candidate_status(evaluation: Evaluation) -> str:
    if (evaluation.collision_free and evaluation.rod_closure_feasible
            and evaluation.hand_motion_feasible):
        return "hard_constraints_feasible_optimization_candidate"
    failures = []
    if not evaluation.collision_free:
        failures.append("collision")
    if not evaluation.rod_closure_feasible:
        failures.append("rod_closure")
    if not evaluation.hand_motion_feasible:
        failures.append("hand_motion")
    return "rejected_" + "_and_".join(failures)


def _result_document(problem: Problem, result: OptimizationResult) -> dict[str, Any]:
    finger = _single_finger(problem)
    return {
        "schema_version": 1,
        "status": _candidate_status(result.evaluation),
        "model": problem.model.get("id", "unspecified"),
        "optimization_scope": "independent_per_finger",
        "target_finger": finger,
        "initialization": {
            "source": str(problem.nominal_design_path),
            "shared_nominal_initial_vector": True,
            "design_variables_shared_with_other_fingers": False,
        },
        "initial_candidate_variables": {
            variable.id: {"value": variable.initial, "units": "mm"}
            for variable in problem.variables
        },
        "fixed_upstream_parameters": {
            "L_ad_mm": problem.fixed_mechanism_dimensions["L_ad"],
            "dorsal_clearance_mm": problem.dorsal_clearance_mm,
            "distal_phalanx_width_mm": problem.distal_phalanx_width_mm,
        },
        "warning": (
            "Output-rod perpendicularity is the sole design objective. Fixed-contact "
            "rod closure, downward monotone hand curl, and sampled hand-mechanism "
            "clearance are hard constraints. The crank is the sole prescribed input; "
            "the ideal compliant hand progress is solved from closure without a motion "
            "ratio. Collision is not continuously certified "
            "between samples. Energy, torque, transmission, singularity, joint-limit, "
            "smoothness, and structural objectives remain disabled."
        ),
        "constraint_acceptance": {
            "collision_free": result.evaluation.collision_free,
            "minimum_signed_clearance_mm": result.evaluation.minimum_clearance_mm,
            "rod_closure_feasible": result.evaluation.rod_closure_feasible,
            "maximum_rod_closure_error_mm": (
                result.evaluation.maximum_rod_closure_error_mm
            ),
            "rod_closure_tolerance_mm": float(
                problem.constraint_config["closure_error"]["tolerance_mm"]
            ),
            "hand_motion_feasible": result.evaluation.hand_motion_feasible,
            "hand_motion_rule": (
                "0 <= s <= 1, s[k+1] >= s[k], s[0]=0, and terminal s=1 "
                "within configured tolerance"
            ),
            "rules": [
                "all sampled intended-workspace poses require signed clearance >= 0",
                "all sampled intended-workspace poses require rod closure within tolerance",
                "the passive finger may only curl downward and may never reverse",
            ],
        },
        "scalarization": {
            "method": (
                "normalized_weighted_sum"
                if problem.normalize_objective_weights else "weighted_sum"
            ),
            "weights": {
                component_id: float(problem.component_config[component_id]["weight"])
                for component_id in ACTIVE_COMPONENT_IDS
            },
            "constraint_guidance_weights": {
                "fixed_contact_rod_closure": float(
                    problem.component_config["task_space_reachability"]["guidance_weight"]
                ),
                "hand_mechanism_clearance": float(
                    problem.component_config["hand_mechanism_non_collision"]["guidance_weight"]
                ),
            },
        },
        "optimizer": {
            "name": "adam",
            "iterations_completed": result.iterations_completed,
            "converged": result.converged,
            "initial_total_loss": result.initial_evaluation.total_loss,
            "final_total_loss": result.evaluation.total_loss,
        },
        "motion_solution": {
            objective_id: {
                "prescribed_input_crank_deg": list(
                    result.evaluation.input_schedules_deg[objective_id]
                ),
                "passive_hand_curl_progress": list(
                    result.evaluation.hand_progress_schedules[objective_id]
                ),
            }
            for objective_id in result.evaluation.input_schedules_deg
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


def _apply_target_finger(document: dict[str, Any], objective: Objective) -> None:
    """Materialize the candidate's own human geometry instead of nominal index data."""
    hand = document["human_hand_model"]
    lengths = objective.data["phalanx_lengths_mm"]
    finger = objective.finger
    hand["id"] = f"{finger}_finger"
    hand["reference_finger"] = finger
    hand["source_objective"] = str(objective.source_path)
    document["finger_analysis_targets"] = [{
        "objective": {
            "id": objective.id,
            "finger": finger,
        },
        "phalanx_lengths_mm": copy.deepcopy(
            objective.data["phalanx_lengths_mm"]
        ),
        "joint_flexion_ranges_deg": copy.deepcopy(
            objective.data["joint_flexion_ranges_deg"]
        ),
    }]
    segments = (
        ("proximal_phalanx", "proximal"),
        ("middle_phalanx", "middle"),
        ("distal_phalanx", "distal"),
    )
    phalanges = {row["id"]: row for row in hand["phalanges"]}
    for segment_id, length_id in segments:
        phalanges[segment_id]["length_mm"] = float(lengths[length_id])

    flexion = hand["nominal_flexion_deg"]
    headings = (
        -math.radians(float(flexion["mcp"])),
        -math.radians(float(flexion["mcp"]) + float(flexion["pip"])),
        -math.radians(
            float(flexion["mcp"]) + float(flexion["pip"]) + float(flexion["dip"])
        ),
    )
    segment_lengths = tuple(float(lengths[length_id]) for _, length_id in segments)
    points: list[Point] = [(0.0, 0.0)]
    for length, heading in zip(segment_lengths, headings):
        points.append((
            points[-1][0] + length * math.cos(heading),
            points[-1][1] + length * math.sin(heading),
        ))
    distal_midpoint = (
        (points[2][0] + points[3][0]) / 2.0,
        (points[2][1] + points[3][1]) / 2.0,
    )
    positions = {
        "hand_wrist_dorsal": (-float(hand["palm"]["length_mm"]), 0.0),
        "hand_mcp": points[0],
        "hand_pip": points[1],
        "hand_dip": points[2],
        "hand_distal_contact": distal_midpoint,
        "hand_tip": points[3],
    }
    for joint in hand["joints"]:
        joint["position_mm"] = list(positions[joint["id"]])

def _candidate_mechanism_document(
    problem: Problem, result: OptimizationResult, candidate_id: str,
) -> dict[str, Any]:
    finger = _single_finger(problem)
    document = copy.deepcopy(problem.nominal_design)
    document["mechanism"]["id"] = f"mechanism_2_{finger}"
    document["mechanism"]["name"] = f"Mechanism 2 — {finger} finger candidate"
    _apply_target_finger(document, problem.objectives[0])
    values = _candidate_lengths(problem, result.values)
    optimizable_ids = {variable.id for variable in problem.variables}
    for dimension in document["dimensions"]:
        dimension["value"] = values[dimension["id"]]
        if dimension["id"] in optimizable_ids:
            dimension["value_source"] = "co_optimization_candidate"
        else:
            dimension["value_source"] = "fixed_nominal_design"
    output_rod = next(
        row for row in document["exoskeleton_attachments"]
        if row["id"] == "distal_output_rod"
    )
    output_rod["assumed_length_mm"] = values[problem.model["tip_rod_variable"]]
    output_rod["value_source"] = "co_optimization_candidate"
    positions = _candidate_initial_positions(values)
    for node in document["nodes"]:
        node["initial_position_mm"] = list(positions[node["id"]])
    document["mechanism"]["status"] = (
        "co_optimization_candidate"
        if result.evaluation.collision_free
        and result.evaluation.rod_closure_feasible
        and result.evaluation.hand_motion_feasible
        else "rejected_hard_constraint_candidate"
    )
    objective_id = problem.objectives[0].id
    q_schedule = result.evaluation.input_schedules_deg[objective_id]
    hand_progress_schedule = result.evaluation.hand_progress_schedules[objective_id]
    terminal_input_deg = q_schedule[-1] if q_schedule else math.nan
    document["optimization_provenance"] = {
        "candidate_id": candidate_id,
        "target_finger": finger,
        "optimization_scope": "independent_per_finger",
        "parent_nominal_design": str(problem.nominal_design_path),
        "optimizer": "adam",
        "objective_status": "perpendicularity_objective_with_hard_geometry_constraints",
        "collision_free": result.evaluation.collision_free,
        "minimum_signed_clearance_mm": result.evaluation.minimum_clearance_mm,
        "rod_closure_feasible": result.evaluation.rod_closure_feasible,
        "maximum_rod_closure_error_mm": (
            result.evaluation.maximum_rod_closure_error_mm
        ),
        "rod_closure_tolerance_mm": float(
            problem.constraint_config["closure_error"]["tolerance_mm"]
        ),
        "hand_motion_feasible": result.evaluation.hand_motion_feasible,
        "intended_workspace_mapping": "crank_driven_passive_monotone_hand_curl",
        "prescribed_coordinate": "mechanism_input_crank_q",
        "passive_coordinate": "hand_curl_progress_s",
        "hand_crank_ratio": None,
        "prescribed_input_schedule_deg": list(q_schedule),
        "passive_hand_progress_schedule": list(hand_progress_schedule),
        "terminal_input_deg": terminal_input_deg,
        "collision_evaluation_poses": int(
            problem.component_config["hand_mechanism_non_collision"]["evaluation_poses"]
        ),
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
                "iteration", "total_loss", "gradient_norm", "step_norm",
                "collision_free", "minimum_clearance_mm", "rod_closure_feasible",
                "maximum_rod_closure_error_mm", "hand_motion_feasible",
                *RECORDED_COMPONENT_IDS, *objective_ids,
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
    parser.add_argument(
        "--tensorboard-dir",
        type=Path,
        default=None,
        help=(
            "TensorBoard root (default: <output-dir>/fingers/<finger>/tensorboard; "
            "an explicit root receives one <finger>/ child per job)"
        ),
    )
    parser.add_argument("--iterations", type=int, default=None)
    parser.add_argument("--learning-rate", type=float, default=None)
    parser.add_argument(
        "--finger",
        choices=FINGERS,
        action="append",
        help="Optimize only this finger; repeat to select several (default: all four)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    base_problem = load_problem(args.objectives.resolve(), args.variables.resolve())
    output_dir = args.output_dir.resolve()
    selected = set(args.finger or FINGERS)
    problems = [
        replace(base_problem, objectives=(objective,))
        for objective in base_problem.objectives
        if objective.finger in selected
    ]
    if not problems:
        raise OptimizationConfigError("no finger optimization problems selected")

    manifest = {
        "schema_version": 1,
        "optimization_scope": "independent_per_finger",
        "shared_nominal_design": str(base_problem.nominal_design_path),
        "design_variable_sharing": "none",
        "fingers": [problem.objectives[0].finger for problem in problems],
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "run_manifest.yaml").write_text(
        yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8"
    )

    for problem in problems:
        finger = _single_finger(problem)
        finger_dir = output_dir / "fingers" / finger
        tensorboard_dir = (
            args.tensorboard_dir.resolve() / finger
            if args.tensorboard_dir is not None
            else finger_dir / "tensorboard"
        )
        with TensorBoardLogger(tensorboard_dir) as tensorboard_logger:
            result = optimize(
                problem,
                args.iterations,
                args.learning_rate,
                tensorboard_logger,
            )
        write_outputs(finger_dir, problem, result)
        print(
            f"{finger}: independent Adam problem, "
            f"{len(problem.variables)} variables initialized from the shared nominal"
        )
        print(
            f"{finger} loss: {result.initial_evaluation.total_loss:.8g} -> "
            f"{result.evaluation.total_loss:.8g} in "
            f"{result.iterations_completed} iterations"
        )
        print(
            f"{finger} collision constraint: "
            f"{'PASS' if result.evaluation.collision_free else 'REJECTED'} · "
            f"minimum sampled clearance "
            f"{result.evaluation.minimum_clearance_mm:.6g} mm"
        )
        print(
            f"{finger} rod-closure constraint: "
            f"{'PASS' if result.evaluation.rod_closure_feasible else 'REJECTED'} · "
            f"maximum sampled error "
            f"{result.evaluation.maximum_rod_closure_error_mm:.6g} mm"
        )
        print(
            f"{finger} downward-curl constraint: "
            f"{'PASS' if result.evaluation.hand_motion_feasible else 'REJECTED'}"
        )
        print(f"Wrote {finger_dir / 'candidate_0001' / 'candidate.yaml'}")
        print(f"Wrote {finger_dir / 'candidate_0001' / 'mechanism.yaml'}")
        print(f"Wrote {finger_dir / 'history.csv'}")
        print(f"Wrote TensorBoard events to {tensorboard_dir}")


if __name__ == "__main__":
    main()
