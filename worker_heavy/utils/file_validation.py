"""
File validation utilities for agent handover files.

Validates the structure and content of:
- objectives.yaml: Central data exchange object
- assembly_definition.yaml: Cost risk management
- plan.md: Structured planning documents
- Review files: YAML frontmatter with decision field
"""

# T015: Hashing for immutability checks
import hashlib
import re
import subprocess
from pathlib import Path

import structlog
import yaml
from pydantic import ValidationError

from shared.models.schemas import (
    AssemblyDefinition,
    ObjectivesYaml,
    ReviewFrontmatter,
)
from shared.observability.events import emit_event
from shared.observability.schemas import LintFailureDocsEvent, LogicFailureEvent
from shared.simulation.schemas import SimulatorBackendType

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
    "z_min",  # objectives.yaml
    "[implement here]",
    "TODO:",
    "...",  # generic
    "[x_min",
    "[x_max",  # generic
]


def validate_objectives_yaml(content: str) -> tuple[bool, ObjectivesYaml | list[str]]:
    """
    Parse and validate objectives.yaml content.

    Args:
        content: Raw YAML string content

    Returns:
        (True, ObjectivesYaml) if valid
        (False, list[str]) with error messages if invalid
    """
    try:
        data = yaml.safe_load(content)
        if data is None:
            return False, ["Empty or invalid YAML content"]

        # 1. Enforce that file is not the template
        found_placeholders = [p for p in TEMPLATE_PLACEHOLDERS if p in content]
        if found_placeholders:
            return False, [
                f"objectives.yaml still contains template placeholders: {found_placeholders}"
            ]

        objectives = ObjectivesYaml(**data)

        # WP2: Validate that fluids are NOT requested if using MuJoCo
        if objectives.physics.backend == SimulatorBackendType.MUJOCO:
            if objectives.fluids:
                return False, [
                    "MuJoCo backend does not support fluids. Use Genesis instead."
                ]

        logger.info("objectives_yaml_valid")
        return True, objectives
    except yaml.YAMLError as e:
        logger.error("objectives_yaml_parse_error", error=str(e))
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.error("objectives_yaml_validation_error", errors=errors)
        for error in errors:
            emit_event(
                LogicFailureEvent(
                    file_path="objectives.yaml",
                    constraint_name="pydantic_validation",
                    error_message=error,
                )
            )
        return False, errors


def validate_assembly_definition_yaml(
    content: str,
) -> tuple[bool, AssemblyDefinition | list[str]]:
    """
    Parse and validate assembly_definition.yaml content.

    Args:
        content: Raw YAML string content

    Returns:
        (True, AssemblyDefinition) if valid
        (False, list[str]) with error messages if invalid
    """
    try:
        data = yaml.safe_load(content)
        if data is None:
            return False, ["Empty or invalid YAML content"]

        # 1. Check for template placeholders in cost estimation too
        found_placeholders = [p for p in TEMPLATE_PLACEHOLDERS if p in content]
        if found_placeholders:
            return False, [
                f"assembly_definition.yaml still contains template placeholders: {found_placeholders}"
            ]

        estimation = AssemblyDefinition(**data)

        # T003 (WP01): Cross-reference electronics component references
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
                    logger.error("electronics_reference_error", error=msg)
                    return False, [msg]

        logger.info("cost_estimation_yaml_valid")
        return True, estimation
    except yaml.YAMLError as e:
        logger.error("cost_estimation_yaml_parse_error", error=str(e))
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.error("cost_estimation_yaml_validation_error", errors=errors)
        for error in errors:
            emit_event(
                LogicFailureEvent(
                    file_path="assembly_definition.yaml",
                    constraint_name="pydantic_validation",
                    error_message=error,
                )
            )
        return False, errors


def validate_review_frontmatter(
    content: str, cad_agent_refused: bool = False
) -> tuple[bool, ReviewFrontmatter | list[str]]:
    """
    Parse and validate review markdown frontmatter.

    Args:
        content: Raw markdown content with YAML frontmatter
        cad_agent_refused: Whether the CAD agent refused the plan
            (determines if refusal decisions are valid)

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

        logger.info("review_frontmatter_valid", decision=frontmatter.decision)
        return True, frontmatter
    except yaml.YAMLError as e:
        logger.error("review_frontmatter_parse_error", error=str(e))
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.error("review_frontmatter_validation_error", errors=errors)
        return False, errors


def validate_node_output(
    node_type: str, files_content_map: dict[str, str]
) -> tuple[bool, list[str]]:
    """
    Universally validate node output for required files and template placeholders.

    Args:
        node_type: 'planner', 'coder', 'electronics_engineer', etc.
        files_content_map: Mapping of filename to string content.

    Returns:
        (True, []) if valid
        (False, list[str]) with error messages if invalid
    """
    errors = []

    # 1. Required files check
    required_files = {
        "planner": ["plan.md", "todo.md", "assembly_definition.yaml"],
        "benchmark_planner": ["plan.md", "todo.md", "objectives.yaml"],
        "coder": [
            "plan.md",
            "todo.md",
            "objectives.yaml",
        ],  # Coder should maintain these
        "benchmark_coder": [
            "plan.md",
            "todo.md",
            "objectives.yaml",
        ],
        "electronics_engineer": [
            "plan.md",
            "todo.md",
            "assembly_definition.yaml",
        ],
    }.get(node_type, [])

    for req_file in required_files:
        if req_file not in files_content_map or not files_content_map[req_file].strip():
            errors.append(f"Missing required file: {req_file}")

    # 2. Template placeholder check
    for filename, content in files_content_map.items():
        found_placeholders = [p for p in TEMPLATE_PLACEHOLDERS if p in content]
        if found_placeholders:
            placeholder_list = ", ".join(found_placeholders)
            errors.append(
                f"File '{filename}' contains template placeholders: {placeholder_list}"
            )

    # 3. Specific validation for known formats
    for filename, content in files_content_map.items():
        if filename == "plan.md":
            plan_type = "engineering"  # Default to engineering for most nodes
            if "benchmark" in node_type or "# Learning Objective" in content:
                plan_type = "benchmark"

            is_valid, plan_errors = validate_plan_md_structure(content, plan_type)
            if not is_valid:
                errors.extend([f"plan.md: {e}" for e in plan_errors])
        elif filename == "todo.md":
            from shared.workers.markdown_validator import validate_todo_md

            res = validate_todo_md(content)
            if not res.is_valid:
                errors.extend([f"todo.md: {e}" for e in res.violations])
        elif filename == "objectives.yaml":
            is_valid, obj_res = validate_objectives_yaml(content)
            if not is_valid:
                # obj_res is list[str] on failure
                errors.extend([f"objectives.yaml: {e}" for e in obj_res])
        elif filename == "assembly_definition.yaml":
            is_valid, asm_res = validate_assembly_definition_yaml(content)
            if not is_valid:
                # asm_res is list[str] on failure
                errors.extend([f"assembly_definition.yaml: {e}" for e in asm_res])

    return len(errors) == 0, errors


def validate_plan_md_structure(
    content: str, plan_type: str = "benchmark"
) -> tuple[bool, list[str]]:
    """
    Validate plan.md has required sections.

    Args:
        content: Raw markdown content
        plan_type: "benchmark" or "engineering"

    Returns:
        (True, []) if valid
        (False, list[str]) with missing section names if invalid
    """
    if plan_type == "engineering":
        from shared.workers.markdown_validator import validate_plan_md

        result = validate_plan_md(content)
        if not result.is_valid:
            logger.error("plan_md_missing_sections", missing=result.violations)
            for error in result.violations:
                emit_event(LintFailureDocsEvent(file_path="plan.md", errors=[error]))
            return False, result.violations

        logger.info("plan_md_valid", plan_type=plan_type)
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
        logger.error("plan_md_missing_sections", missing=missing)
        errors = [f"Missing required section: {s}" for s in missing]
        for error in errors:
            emit_event(LintFailureDocsEvent(file_path="plan.md", errors=[error]))
        return False, errors

    logger.info("plan_md_valid", plan_type=plan_type)
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


def validate_immutability(path: Path) -> tuple[bool, str | None]:
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
                logger.error("immutability_violation", path=str(path))
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
            # Ideally objectives.yaml SHOULD exist in root commit.
            pass

    except (subprocess.CalledProcessError, FileNotFoundError):
        # Git not installed or not a repo, skip check
        pass
    except Exception as e:
        logger.error("immutability_check_failed_internal", error=str(e))

    return True, None
