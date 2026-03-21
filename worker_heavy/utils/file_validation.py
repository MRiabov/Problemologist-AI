"""
File validation utilities for agent handover files.

Validates the structure and content of:
- benchmark_definition.yaml: Central data exchange object
- assembly_definition.yaml: Cost risk management
- plan.md: Structured planning documents
- Review files: YAML frontmatter with decision field
"""

# T015: Hashing for immutability checks
import hashlib
import io
import re
import subprocess
import tokenize
from pathlib import Path

import structlog
import yaml
from pydantic import ValidationError

from shared.enums import AgentName, BenchmarkAttachmentMethod
from shared.models.schemas import (
    AssemblyDefinition,
    BenchmarkDefinition,
    PartConfig,
    PlanRefusalFrontmatter,
    ReviewFrontmatter,
    SubassemblyEstimate,
)
from shared.observability.events import emit_event
from shared.observability.schemas import LintFailureDocsEvent, LogicFailureEvent
from shared.simulation.schemas import SimulatorBackendType
from shared.workers.workbench_models import ManufacturingConfig
from worker_heavy.utils.dfm import (
    validate_declared_assembly_cost,
    validate_exact_declared_assembly_cost,
)
from worker_heavy.utils.validation import _validate_benchmark_definition_consistency
from worker_heavy.workbenches.config import load_config, load_merged_config

logger = structlog.get_logger(__name__)

# Required sections for plan.md validation
BENCHMARK_PLAN_REQUIRED_SECTIONS = [
    "Learning Objective",
    "Geometry",
    "Objectives",
]

ENGINEERING_PLAN_REQUIRED_SECTIONS = [
    "Solution Overview",
    "Parts List",
    "Assembly Strategy",
    "Cost & Weight Budget",
    "Risk Assessment",
]

TEMPLATE_PLACEHOLDERS = [
    "x_min",
    "x_max",
    "[x, y, z]",
    "y_min",
    "z_min",  # benchmark_definition.yaml
    "[implement here]",
    "TODO:",
    "...",  # generic
    "[x_min",
    "[x_max",  # generic
]


_MISSING_FILE_ERROR_RE = re.compile(
    r"^Error:\s*File\s+'(?P<path>[^']+)'\s+not found\.?$", re.IGNORECASE
)


def _find_template_placeholders(filename: str, content: str) -> list[str]:
    """Return template placeholder markers, with Python-aware handling for ellipses."""
    found_placeholders = [
        p for p in TEMPLATE_PLACEHOLDERS if p != "..." and p in content
    ]

    if "..." not in content:
        return found_placeholders

    # Python code often contains valid ellipses in strings or type hints. Treat
    # `...` as a placeholder only when it appears in comments for Python files.
    if filename.endswith(".py"):
        try:
            for token in tokenize.generate_tokens(io.StringIO(content).readline):
                if token.type == tokenize.COMMENT and "..." in token.string:
                    found_placeholders.append("...")
                    break
        except tokenize.TokenError:
            # Fail closed for malformed Python content that still contains
            # template-style ellipses.
            found_placeholders.append("...")
        return found_placeholders

    found_placeholders.append("...")
    return found_placeholders


def _is_missing_file_error(content: str, *, expected_path: str | None = None) -> bool:
    """Return True when a read_file 404 placeholder was mistaken for content."""
    match = _MISSING_FILE_ERROR_RE.match(content.strip())
    if not match:
        return False
    if expected_path is None:
        return True
    returned_path = match.group("path").strip().lstrip("/")
    normalized_expected = expected_path.strip().lstrip("/")
    return returned_path == normalized_expected


def validate_benchmark_definition_yaml(
    content: str, session_id: str | None = None
) -> tuple[bool, BenchmarkDefinition | list[str]]:
    """
    Parse and validate benchmark_definition.yaml content.

    Args:
        content: Raw YAML string content
        session_id: Optional session ID for logging

    Returns:
        (True, BenchmarkDefinition) if valid
        (False, list[str]) with error messages if invalid
    """
    try:
        data = yaml.safe_load(content)
        if data is None:
            return False, ["Empty or invalid YAML content"]

        benchmark_parts = data.get("benchmark_parts")
        if not isinstance(benchmark_parts, list) or not benchmark_parts:
            return False, [
                "benchmark_definition.yaml must declare at least one benchmark_parts entry"
            ]

        # 1. Enforce that file is not the template
        found_placeholders = _find_template_placeholders(
            "benchmark_definition.yaml", content
        )
        if found_placeholders:
            return False, [
                f"benchmark_definition.yaml still contains template placeholders: {found_placeholders}"
            ]

        objectives = BenchmarkDefinition(**data)

        material_id = objectives.moved_object.material_id
        manufacturing_config = load_config()
        known_material_ids = set(manufacturing_config.materials.keys())
        known_material_ids.update(manufacturing_config.cnc.materials.keys())
        known_material_ids.update(
            manufacturing_config.injection_molding.materials.keys()
        )
        known_material_ids.update(manufacturing_config.three_dp.materials.keys())
        if material_id not in known_material_ids:
            return False, [
                "moved_object.material_id must reference a known material from "
                f"manufacturing_config.yaml (got '{material_id}')"
            ]

        # WP2: Validate that fluids are NOT requested if using MuJoCo
        if objectives.physics.backend == SimulatorBackendType.MUJOCO:
            if objectives.fluids:
                return False, [
                    "MuJoCo backend does not support fluids. Use Genesis instead."
                ]

        objective_error = _validate_benchmark_definition_consistency(objectives)
        if objective_error:
            logger.error(
                "benchmark_definition_yaml_invalid",
                errors=[objective_error],
                session_id=session_id,
            )
            emit_event(
                LogicFailureEvent(
                    file_path="benchmark_definition.yaml",
                    constraint_name="objectives_consistency",
                    error_message=objective_error,
                )
            )
            return False, [objective_error]

        logger.info("benchmark_definition_yaml_valid", session_id=session_id)
        return True, objectives
    except yaml.YAMLError as e:
        logger.error(
            "benchmark_definition_yaml_parse_error", error=str(e), session_id=session_id
        )
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.error(
            "benchmark_definition_yaml_validation_error",
            errors=errors,
            session_id=session_id,
        )
        for error in errors:
            emit_event(
                LogicFailureEvent(
                    file_path="benchmark_definition.yaml",
                    constraint_name="pydantic_validation",
                    error_message=error,
                )
            )
        return False, errors


def validate_assembly_definition_yaml(
    content: str,
    session_id: str | None = None,
    manufacturing_config: ManufacturingConfig | None = None,
) -> tuple[bool, AssemblyDefinition | list[str]]:
    """
    Parse and validate assembly_definition.yaml content.

    Args:
        content: Raw YAML string content
        session_id: Optional session ID for logging

    Returns:
        (True, AssemblyDefinition) if valid
        (False, list[str]) with error messages if invalid
    """
    try:
        data = yaml.safe_load(content)
        if data is None:
            return False, ["Empty or invalid YAML content"]

        # 1. Check for template placeholders in cost estimation too
        found_placeholders = _find_template_placeholders(
            "assembly_definition.yaml", content
        )
        if found_placeholders:
            return False, [
                f"assembly_definition.yaml still contains template placeholders: {found_placeholders}"
            ]

        estimation = AssemblyDefinition(**data)
        effective_config = manufacturing_config or load_config()
        cost_errors = validate_declared_planner_cost_contract(
            assembly_definition=estimation,
            manufacturing_config=effective_config,
        )
        if cost_errors:
            logger.error(
                "cost_estimation_yaml_invalid",
                errors=cost_errors,
                session_id=session_id,
            )
            return False, cost_errors

        if estimation.electronics:
            from shared.models.schemas import PartConfig, SubassemblyEstimate

            all_part_names = {p.part_name for p in estimation.manufactured_parts}
            # Also check final_assembly
            for item in estimation.final_assembly:
                if isinstance(item, SubassemblyEstimate):
                    for p_config in item.parts:
                        all_part_names.add(p_config.name)
                elif isinstance(item, PartConfig):
                    all_part_names.add(item.name)

            for comp in estimation.electronics.components:
                if (
                    comp.assembly_part_ref
                    and comp.assembly_part_ref not in all_part_names
                ):
                    msg = f"Electronic component '{comp.component_id}' references unknown part '{comp.assembly_part_ref}'"
                    logger.error(
                        "electronics_reference_error",
                        error=msg,
                        session_id=session_id,
                    )
                    return False, [msg]

        logger.info("cost_estimation_yaml_valid", session_id=session_id)
        return True, estimation
    except yaml.YAMLError as e:
        logger.error(
            "cost_estimation_yaml_parse_error", error=str(e), session_id=session_id
        )
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.error(
            "cost_estimation_yaml_validation_error",
            errors=errors,
            session_id=session_id,
        )
        for error in errors:
            emit_event(
                LogicFailureEvent(
                    file_path="assembly_definition.yaml",
                    constraint_name="pydantic_validation",
                    error_message=error,
                )
            )
        return False, errors


def validate_environment_drilling_contract(
    *,
    benchmark_definition: BenchmarkDefinition,
    assembly_definition: AssemblyDefinition,
) -> list[str]:
    """Validate planner-declared drilling intent against benchmark fixture policy."""
    errors: list[str] = []
    if not assembly_definition.environment_drill_operations:
        return errors

    benchmark_parts = {
        part.part_id: part for part in benchmark_definition.benchmark_parts
    }
    drill_counts: dict[str, int] = {}

    for operation in assembly_definition.environment_drill_operations:
        target = benchmark_parts.get(operation.target_part_id)
        if target is None:
            errors.append(
                "environment_drill_operations: target_part_id "
                f"'{operation.target_part_id}' is not declared in benchmark_parts"
            )
            continue

        attachment_policy = target.metadata.attachment_policy
        drill_policy = (
            attachment_policy.drill_policy if attachment_policy is not None else None
        )
        if drill_policy is None or not drill_policy.allowed:
            errors.append(
                "environment_drill_operations: drilling is not allowed for "
                f"benchmark part '{operation.target_part_id}'"
            )
            continue

        if drill_policy.diameter_range_mm is not None:
            diameter_min, diameter_max = drill_policy.diameter_range_mm
            if not (diameter_min <= operation.diameter_mm <= diameter_max):
                errors.append(
                    "environment_drill_operations: diameter_mm "
                    f"({operation.diameter_mm}) for hole '{operation.hole_id}' "
                    f"must be within [{diameter_min}, {diameter_max}] on "
                    f"benchmark part '{operation.target_part_id}'"
                )

        if (
            drill_policy.max_depth_mm is not None
            and operation.depth_mm > drill_policy.max_depth_mm
        ):
            errors.append(
                "environment_drill_operations: depth_mm "
                f"({operation.depth_mm}) for hole '{operation.hole_id}' exceeds "
                f"max_depth_mm ({drill_policy.max_depth_mm}) on benchmark part "
                f"'{operation.target_part_id}'"
            )

        drill_counts[operation.target_part_id] = (
            drill_counts.get(operation.target_part_id, 0) + operation.quantity
        )

    for part_id, count in drill_counts.items():
        target = benchmark_parts.get(part_id)
        if target is None or target.metadata.attachment_policy is None:
            continue
        drill_policy = target.metadata.attachment_policy.drill_policy
        if (
            drill_policy is not None
            and drill_policy.allowed
            and drill_policy.max_hole_count is not None
            and count > drill_policy.max_hole_count
        ):
            errors.append(
                "environment_drill_operations: total planned hole count "
                f"({count}) exceeds max_hole_count ({drill_policy.max_hole_count}) "
                f"for benchmark part '{part_id}'"
            )

    return errors


def validate_environment_attachment_contract(
    *,
    benchmark_definition: BenchmarkDefinition,
    assembly_definition: AssemblyDefinition,
) -> list[str]:
    """Validate assembly joints and drilling against benchmark attachment policy."""
    errors = validate_environment_drilling_contract(
        benchmark_definition=benchmark_definition,
        assembly_definition=assembly_definition,
    )

    benchmark_parts = {
        part.part_id: part for part in benchmark_definition.benchmark_parts
    }

    engineer_part_refs = {
        part.part_id for part in assembly_definition.manufactured_parts
    } | {part.part_id for part in assembly_definition.cots_parts}

    for item in assembly_definition.final_assembly:
        if isinstance(item, SubassemblyEstimate):
            engineer_part_refs.update(part.name for part in item.parts)
            joints = item.joints
        elif isinstance(item, PartConfig):
            engineer_part_refs.add(item.name)
            joints = []
        else:
            joints = []

        for joint in joints:
            for part_ref in joint.parts:
                if (
                    part_ref not in engineer_part_refs
                    and part_ref not in benchmark_parts
                ):
                    errors.append(
                        "final_assembly.joints: joint "
                        f"'{joint.joint_id}' references undeclared part '{part_ref}'"
                    )

            benchmark_refs = [
                part_ref for part_ref in joint.parts if part_ref in benchmark_parts
            ]
            if not benchmark_refs:
                continue

            if joint.type != "fastener_joint":
                errors.append(
                    "final_assembly.joints: benchmark attachments must use "
                    f"'fastener_joint'; joint '{joint.joint_id}' uses '{joint.type}'"
                )

            if len(benchmark_refs) != 1:
                errors.append(
                    "final_assembly.joints: each benchmark attachment joint must "
                    f"reference exactly one benchmark part; joint '{joint.joint_id}' "
                    f"references {benchmark_refs}"
                )
                continue

            benchmark_ref = benchmark_refs[0]
            target = benchmark_parts[benchmark_ref]
            attachment_policy = target.metadata.attachment_policy
            if (
                attachment_policy is None
                or BenchmarkAttachmentMethod.FASTENER
                not in attachment_policy.attachment_methods
            ):
                errors.append(
                    "final_assembly.joints: benchmark part "
                    f"'{benchmark_ref}' does not permit fastener attachment"
                )

            non_benchmark_refs = [
                part_ref for part_ref in joint.parts if part_ref != benchmark_ref
            ]
            if not non_benchmark_refs:
                errors.append(
                    "final_assembly.joints: benchmark attachment joint "
                    f"'{joint.joint_id}' must include at least one engineer-owned part"
                )
                continue

            for part_ref in non_benchmark_refs:
                if part_ref in benchmark_parts:
                    errors.append(
                        "final_assembly.joints: benchmark attachment joint "
                        f"'{joint.joint_id}' must not connect benchmark part "
                        f"'{benchmark_ref}' to another benchmark part '{part_ref}'"
                    )
                elif part_ref not in engineer_part_refs:
                    errors.append(
                        "final_assembly.joints: joint "
                        f"'{joint.joint_id}' references undeclared engineer part "
                        f"'{part_ref}'"
                    )

    return errors


def validate_declared_planner_cost_contract(
    *,
    assembly_definition: AssemblyDefinition,
    manufacturing_config: ManufacturingConfig,
) -> list[str]:
    """Validate that planner totals include deterministic declared costs."""
    return validate_declared_assembly_cost(assembly_definition, manufacturing_config)


def validate_exact_planner_cost_contract(
    *,
    assembly_definition: AssemblyDefinition,
    manufacturing_config: ManufacturingConfig,
) -> list[str]:
    """Validate that planner totals exactly match deterministic declared costs."""
    return validate_exact_declared_assembly_cost(
        assembly_definition, manufacturing_config
    )


def validate_planner_handoff_cross_contract(
    *,
    benchmark_definition: BenchmarkDefinition,
    assembly_definition: AssemblyDefinition,
    manufacturing_config: ManufacturingConfig,
) -> list[str]:
    """Validate copied benchmark caps and normalized planner totals exactly."""
    errors: list[str] = []

    cap_pairs = (
        (
            "benchmark_definition.constraints.max_unit_cost",
            benchmark_definition.constraints.max_unit_cost,
            "assembly_definition.constraints.benchmark_max_unit_cost_usd",
            assembly_definition.constraints.benchmark_max_unit_cost_usd,
        ),
        (
            "benchmark_definition.constraints.max_weight_g",
            benchmark_definition.constraints.max_weight_g,
            "assembly_definition.constraints.benchmark_max_weight_g",
            assembly_definition.constraints.benchmark_max_weight_g,
        ),
    )
    for expected_label, expected_value, observed_label, observed_value in cap_pairs:
        if expected_value is None:
            errors.append(
                f"{expected_label} is missing; cannot validate copied planner caps"
            )
            continue
        if round(observed_value, 2) != round(expected_value, 2):
            errors.append(
                f"{observed_label} ({observed_value:.2f}) must equal "
                f"{expected_label} ({expected_value:.2f})"
            )

    errors.extend(
        validate_exact_planner_cost_contract(
            assembly_definition=assembly_definition,
            manufacturing_config=manufacturing_config,
        )
    )
    return errors


def validate_review_frontmatter(
    content: str, cad_agent_refused: bool = False, session_id: str | None = None
) -> tuple[bool, ReviewFrontmatter | list[str]]:
    """
    Parse and validate review markdown frontmatter.

    Args:
        content: Raw markdown content with YAML frontmatter
        cad_agent_refused: Whether the CAD agent refused the plan
            (determines if refusal decisions are valid)
        session_id: Optional session ID for logging

    Returns:
        (True, ReviewFrontmatter) if valid
        (False, list[str]) with error messages if invalid
    """
    # Extract YAML frontmatter
    frontmatter_match = re.search(r"^---\s*\n(.*?)\n---\s*\n", content, re.DOTALL)
    if not frontmatter_match:
        return False, [
            "Missing YAML frontmatter (must start with --- and end with ---)"
        ]

    try:
        data = yaml.safe_load(frontmatter_match.group(1))
        if data is None:
            return False, ["Empty frontmatter"]

        frontmatter = ReviewFrontmatter(**data)

        # Context-specific validation: refusal decisions
        is_refusal_decision = frontmatter.decision in (
            "confirm_plan_refusal",
            "reject_plan_refusal",
        )
        if is_refusal_decision and not cad_agent_refused:
            return False, [
                f"Decision '{frontmatter.decision}' is only valid "
                "when CAD agent refused the plan"
            ]

        logger.info(
            "review_frontmatter_valid",
            decision=frontmatter.decision,
            session_id=session_id,
        )
        return True, frontmatter
    except yaml.YAMLError as e:
        logger.error(
            "review_frontmatter_parse_error", error=str(e), session_id=session_id
        )
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.error(
            "review_frontmatter_validation_error", errors=errors, session_id=session_id
        )
        return False, errors


def validate_plan_refusal(
    content: str, session_id: str | None = None
) -> tuple[bool, PlanRefusalFrontmatter | list[str]]:
    """
    Parse and validate plan_refusal.md content.
    Requires structured frontmatter and evidence in the body.
    """
    # Extract YAML frontmatter
    frontmatter_match = re.search(r"^---\s*\n(.*?)\n---\s*\n", content, re.DOTALL)
    if not frontmatter_match:
        return False, ["Missing YAML frontmatter in plan_refusal.md"]

    body = content[frontmatter_match.end() :].strip()
    if not body:
        return False, ["plan_refusal.md must include evidence in the body"]

    try:
        data = yaml.safe_load(frontmatter_match.group(1))
        if data is None:
            return False, ["Empty frontmatter in plan_refusal.md"]

        frontmatter = PlanRefusalFrontmatter(**data)
        logger.info(
            "plan_refusal_valid",
            role=frontmatter.role,
            reasons=frontmatter.reasons,
            session_id=session_id,
        )
        return True, frontmatter
    except yaml.YAMLError as e:
        logger.error("plan_refusal_parse_error", error=str(e), session_id=session_id)
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.warning("plan_refusal_validation_error", errors=errors)
        return False, errors


def validate_node_output(
    node_type: str,
    files_content_map: dict[str, str],
    session_id: str | None = None,
    manufacturing_config: ManufacturingConfig | None = None,
) -> tuple[bool, list[str]]:
    """
    Universally validate node output for required files and template placeholders.

    Args:
        node_type: 'planner', 'coder', 'electronics_engineer', etc.
        files_content_map: Mapping of filename to string content.
        session_id: Optional session ID for logging

    Returns:
        (True, []) if valid
        (False, list[str]) with error messages if invalid
    """
    errors = []
    benchmark_definition_model: BenchmarkDefinition | None = None
    assembly_definition_models: dict[str, AssemblyDefinition] = {}
    effective_config = manufacturing_config

    def _missing_file(path: str) -> bool:
        content = files_content_map.get(path)
        if content is None or not content.strip():
            return True
        return _is_missing_file_error(content, expected_path=path)

    # 1. Required files check
    # If plan_refusal.md is present and valid, skip regular required files check
    if "plan_refusal.md" in files_content_map:
        is_refusal_valid, _ = validate_plan_refusal(
            files_content_map["plan_refusal.md"], session_id=session_id
        )
        if is_refusal_valid:
            # Only validate plan_refusal.md and skip others
            required_files = ["plan_refusal.md"]
        else:
            # If refusal is invalid, we still want regular files or a better refusal
            required_files = {
                AgentName.ENGINEER_PLANNER: [
                    "plan.md",
                    "todo.md",
                    "benchmark_definition.yaml",
                    "assembly_definition.yaml",
                ],
                AgentName.BENCHMARK_PLANNER: [
                    "plan.md",
                    "todo.md",
                    "benchmark_definition.yaml",
                    "benchmark_assembly_definition.yaml",
                ],
                AgentName.ENGINEER_CODER: [
                    "plan.md",
                    "todo.md",
                    "benchmark_definition.yaml",
                    "script.py",
                ],
                AgentName.ELECTRONICS_PLANNER: [
                    "plan.md",
                    "todo.md",
                    "benchmark_definition.yaml",
                    "assembly_definition.yaml",
                ],
                AgentName.BENCHMARK_CODER: [
                    "plan.md",
                    "todo.md",
                    "benchmark_definition.yaml",
                ],
                AgentName.ELECTRONICS_ENGINEER: [
                    "plan.md",
                    "todo.md",
                    "assembly_definition.yaml",
                ],
            }.get(node_type, [])
    else:
        required_files = {
            AgentName.ENGINEER_PLANNER: [
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "assembly_definition.yaml",
            ],
            AgentName.BENCHMARK_PLANNER: [
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "benchmark_assembly_definition.yaml",
            ],
            AgentName.ENGINEER_CODER: [
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "script.py",
            ],
            AgentName.ELECTRONICS_PLANNER: [
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
                "assembly_definition.yaml",
            ],
            AgentName.BENCHMARK_CODER: [
                "plan.md",
                "todo.md",
                "benchmark_definition.yaml",
            ],
            AgentName.ELECTRONICS_ENGINEER: [
                "plan.md",
                "todo.md",
                "assembly_definition.yaml",
            ],
        }.get(node_type, [])

    for req_file in required_files:
        if _missing_file(req_file):
            errors.append(f"Missing required file: {req_file}")

    # 2. Template placeholder check
    for filename, content in files_content_map.items():
        if _is_missing_file_error(content, expected_path=filename):
            continue
        found_placeholders = _find_template_placeholders(filename, content)
        if found_placeholders:
            placeholder_list = ", ".join(found_placeholders)
            errors.append(
                f"File '{filename}' contains template placeholders: {placeholder_list}"
            )

    # 3. Specific validation for known formats
    for filename, content in files_content_map.items():
        if _is_missing_file_error(content, expected_path=filename):
            continue
        if filename == "plan.md":
            plan_type = "engineering"  # Default to engineering for most nodes
            if "benchmark" in node_type or "# Learning Objective" in content:
                plan_type = "benchmark"

            is_valid, plan_errors = validate_plan_md_structure(
                content, plan_type, session_id=session_id
            )
            if not is_valid:
                errors.extend([f"plan.md: {e}" for e in plan_errors])
        elif filename == "todo.md":
            from shared.workers.markdown_validator import validate_todo_md

            res = validate_todo_md(content)
            if not res.is_valid:
                errors.extend([f"todo.md: {e}" for e in res.violations])
        elif filename == "benchmark_definition.yaml":
            is_valid, obj_res = validate_benchmark_definition_yaml(
                content, session_id=session_id
            )
            if not is_valid:
                # obj_res is list[str] on failure
                errors.extend([f"benchmark_definition.yaml: {e}" for e in obj_res])
            else:
                benchmark_definition_model = obj_res
        elif filename in {
            "assembly_definition.yaml",
            "benchmark_assembly_definition.yaml",
        }:
            if effective_config is None:
                if "manufacturing_config.yaml" in files_content_map:
                    custom_config = yaml.safe_load(
                        files_content_map["manufacturing_config.yaml"]
                    )
                    effective_config = load_merged_config(
                        override_data=custom_config or {}
                    )
                else:
                    effective_config = load_config()
            is_valid, asm_res = validate_assembly_definition_yaml(
                content,
                session_id=session_id,
                manufacturing_config=effective_config,
            )
            if not is_valid:
                # asm_res is list[str] on failure
                errors.extend([f"{filename}: {e}" for e in asm_res])
            else:
                assembly_definition_models[filename] = asm_res
        elif filename == "plan_refusal.md":
            is_valid, refusal_res = validate_plan_refusal(
                content, session_id=session_id
            )
            if not is_valid:
                if isinstance(refusal_res, list):
                    errors.extend([f"plan_refusal.md: {e}" for e in refusal_res])
                else:
                    # Should not happen based on validate_plan_refusal return type
                    errors.append("plan_refusal.md: Invalid structure")

    if benchmark_definition_model is not None and assembly_definition_models:
        if effective_config is None:
            effective_config = load_config()
        for filename, assembly_definition_model in assembly_definition_models.items():
            cross_contract_errors = validate_planner_handoff_cross_contract(
                benchmark_definition=benchmark_definition_model,
                assembly_definition=assembly_definition_model,
                manufacturing_config=effective_config,
            )
            if cross_contract_errors:
                errors.extend(
                    [f"{filename}: {message}" for message in cross_contract_errors]
                )

    return len(errors) == 0, errors


def validate_plan_md_structure(
    content: str, plan_type: str = "benchmark", session_id: str | None = None
) -> tuple[bool, list[str]]:
    """
    Validate plan.md has required sections.

    Args:
        content: Raw markdown content
        plan_type: "benchmark" or "engineering"
        session_id: Optional session ID for logging

    Returns:
        (True, []) if valid
        (False, list[str]) with missing section names if invalid
    """
    if plan_type == "engineering":
        from shared.workers.markdown_validator import validate_plan_md

        result = validate_plan_md(content)
        if not result.is_valid:
            logger.error(
                "plan_md_missing_sections",
                missing=result.violations,
                session_id=session_id,
            )
            for error in result.violations:
                emit_event(LintFailureDocsEvent(file_path="plan.md", errors=[error]))
            return False, result.violations

        logger.info("plan_md_valid", plan_type=plan_type, session_id=session_id)
        return True, []

    required_sections = BENCHMARK_PLAN_REQUIRED_SECTIONS

    # Extract all headings from markdown
    heading_pattern = r"^#{1,3}\s+(.+)$"
    headings = re.findall(heading_pattern, content, re.MULTILINE)
    headings_lower = [h.lower().strip() for h in headings]

    missing = []
    for section in required_sections:
        # Check if any heading contains the required section name
        if not any(section.lower() in h for h in headings_lower):
            missing.append(section)

    if missing:
        logger.error("plan_md_missing_sections", missing=missing, session_id=session_id)
        errors = [f"Missing required section: {s}" for s in missing]
        for error in errors:
            emit_event(LintFailureDocsEvent(file_path="plan.md", errors=[error]))
        return False, errors

    logger.info("plan_md_valid", plan_type=plan_type, session_id=session_id)
    return True, []


def calculate_file_hash(path: Path) -> str:
    """Calculate SHA-256 hash of a file."""
    if not path.exists():
        return ""
    sha256_hash = hashlib.sha256()
    with path.open("rb") as f:
        # Read and update hash string value in blocks of 4K
        for byte_block in iter(lambda: f.read(4096), b""):
            sha256_hash.update(byte_block)
    return sha256_hash.hexdigest()


def validate_immutability(
    path: Path, session_id: str | None = None
) -> tuple[bool, str | None]:
    """
    Verify that a file has not changed since the initial commit (or baseline).

    Strategy:
    1. Check if git is available and repo is initialized.
    2. Get the hash of the file from the *first* commit (benchmark baseline).
    3. Compare with current hash.

    Returns:
        (True, None) if immutable or cannot verify.
        (False, error_message) if changed.
    """
    if not path.exists():
        return True, None

    try:
        # Check if inside a git repo
        subprocess.check_output(
            ["git", "rev-parse", "--is-inside-work-tree"],
            cwd=path.parent,
            stderr=subprocess.DEVNULL,
        )

        # Get the first commit hash (root commit where benchmark was generated)
        # We assume the benchmark generator commits the initial state.
        root_commit = subprocess.check_output(
            ["git", "rev-list", "--max-parents=0", "HEAD"], cwd=path.parent, text=True
        ).strip()

        if not root_commit:
            # No commits yet, can't verify against baseline
            return True, None

        # Get the hash of the file at the root commit
        # git show <commit>:<path>
        try:
            original_content = subprocess.check_output(
                ["git", "show", f"{root_commit}:{path.name}"],
                cwd=path.parent,
                stderr=subprocess.DEVNULL,
            )
            original_hash = hashlib.sha256(original_content).hexdigest()
            current_hash = calculate_file_hash(path)

            if original_hash != current_hash:
                msg = (
                    f"Immutability violation: {path.name} has been modified "
                    "from the benchmark baseline."
                )
                logger.error(
                    "immutability_violation", path=str(path), session_id=session_id
                )
                emit_event(
                    LogicFailureEvent(
                        file_path=path.name,
                        constraint_name="immutability_check",
                        error_message=msg,
                    )
                )
                return False, msg

        except subprocess.CalledProcessError:
            # File might not have existed in root commit (e.g. created later)
            # In that case, immutability check might not apply or is ambiguous.
            # Ideally benchmark_definition.yaml SHOULD exist in root commit.
            pass

    except (subprocess.CalledProcessError, FileNotFoundError):
        # Git not installed or not a repo, skip check
        pass
    except Exception as e:
        logger.error(
            "immutability_check_failed_internal",
            error=str(e),
            session_id=session_id,
        )

    return True, None
