"""
File validation utilities for agent handover files.

Validates the structure and content of:
- objectives.yaml: Central data exchange object
- preliminary_cost_estimation.yaml: Cost risk management
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
    ObjectivesYaml,
    PreliminaryCostEstimation,
    ReviewFrontmatter,
)
from shared.observability.events import emit_event
from shared.observability.schemas import LintFailureDocsEvent, LogicFailureEvent

logger = structlog.get_logger(__name__)


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
        placeholders = ["x_min", "x_max", "[x_min", "[x, y, z]", "y_min", "z_min"]
        if any(p in content for p in placeholders):
            return False, ["objectives.yaml still contains template placeholders"]

        objectives = ObjectivesYaml(**data)
        logger.info("objectives_yaml_valid")
        return True, objectives
    except yaml.YAMLError as e:
        logger.warning("objectives_yaml_parse_error", error=str(e))
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.warning("objectives_yaml_validation_error", errors=errors)
        for error in errors:
            emit_event(
                LogicFailureEvent(
                    file_path="objectives.yaml",
                    constraint_name="pydantic_validation",
                    error_message=error,
                )
            )
        return False, errors


def validate_preliminary_cost_estimation_yaml(
    content: str,
) -> tuple[bool, PreliminaryCostEstimation | list[str]]:
    """
    Parse and validate preliminary_cost_estimation.yaml content.

    Args:
        content: Raw YAML string content

    Returns:
        (True, PreliminaryCostEstimation) if valid
        (False, list[str]) with error messages if invalid
    """
    try:
        data = yaml.safe_load(content)
        if data is None:
            return False, ["Empty or invalid YAML content"]

        # 1. Check for template placeholders in cost estimation too
        placeholders = ["ramp_main", "guide_clip"]
        # Only check if it looks EXACTLY like the template example
        if any(p in content for p in placeholders):
            return False, [
                "preliminary_cost_estimation.yaml still contains template placeholders"
            ]

        estimation = PreliminaryCostEstimation(**data)

        logger.info("cost_estimation_yaml_valid")
        return True, estimation
    except yaml.YAMLError as e:
        logger.warning("cost_estimation_yaml_parse_error", error=str(e))
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.warning("cost_estimation_yaml_validation_error", errors=errors)
        for error in errors:
            emit_event(
                LogicFailureEvent(
                    file_path="preliminary_cost_estimation.yaml",
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
        logger.warning("review_frontmatter_parse_error", error=str(e))
        return False, [f"YAML parse error: {e}"]
    except ValidationError as e:
        errors = [f"{err['loc']}: {err['msg']}" for err in e.errors()]
        logger.warning("review_frontmatter_validation_error", errors=errors)
        return False, errors


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
        from worker.utils.markdown_validator import validate_plan_md

        result = validate_plan_md(content)
        if not result.is_valid:
            logger.warning("plan_md_missing_sections", missing=result.violations)
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
        logger.warning("plan_md_missing_sections", missing=missing)
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
    with open(path, "rb") as f:
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
                logger.warning("immutability_violation", path=str(path))
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
        logger.warning("immutability_check_failed_internal", error=str(e))

    return True, None
