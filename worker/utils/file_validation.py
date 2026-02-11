"""
File validation utilities for agent handover files.

Validates the structure and content of:
- objectives.yaml: Central data exchange object
- preliminary_cost_estimation.yaml: Cost risk management
- plan.md: Structured planning documents
- Review files: YAML frontmatter with decision field
"""

import re

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
        if "x_min" in content or "x_max" in content or "goal_zone: [x_min" in content:
            return False, [
                "objectives.yaml still contains template placeholders (x_min, etc.)"
            ]

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
