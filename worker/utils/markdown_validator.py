"""
Markdown validation utilities for plan.md and todo.md files.

Validates structural requirements as per desired_architecture.md.
"""

import re

from pydantic import BaseModel, Field


class ValidationResult(BaseModel):
    """Result of markdown validation."""

    is_valid: bool
    violations: list[str] = Field(default_factory=list)


# Required sections for plan.md (engineering plan)
PLAN_REQUIRED_SECTIONS = [
    "## 1. Solution Overview",
    "## 2. Parts List",
    "## 3. Assembly Strategy",
    "## 4. Cost & Weight Budget",
    "## 5. Risk Assessment",
]

# Alternative forms that are also acceptable
PLAN_SECTION_PATTERNS = [
    r"##\s*1\.\s*Solution\s+Overview",
    r"##\s*2\.\s*Parts\s+List",
    r"##\s*3\.\s*Assembly\s+Strategy",
    r"##\s*4\.\s*Cost\s*[&and]+\s*Weight\s+Budget",
    r"##\s*5\.\s*Risk\s+Assessment",
]

# Valid checkbox patterns for todo.md
TODO_CHECKBOX_PATTERN = re.compile(r"^\s*-\s*\[([ xX/])\]", re.MULTILINE)
INVALID_CHECKBOX_PATTERN = re.compile(r"^\s*-\s*\[[^\]]*\]", re.MULTILINE)


def validate_plan_md(content: str) -> ValidationResult:
    """
    Validate plan.md structure against required sections.

    Args:
        content: The markdown content to validate

    Returns:
        ValidationResult with is_valid flag and list of violations
    """
    violations = []

    for section, pattern in zip(
        PLAN_REQUIRED_SECTIONS, PLAN_SECTION_PATTERNS, strict=True
    ):
        if not re.search(pattern, content, re.IGNORECASE):
            violations.append(f"Missing required section: {section}")

    return ValidationResult(is_valid=len(violations) == 0, violations=violations)


def validate_todo_md(content: str) -> ValidationResult:
    """
    Validate todo.md structure for proper checkbox format.

    Valid checkboxes: [ ], [x], [X], [/]
    Invalid: anything else inside brackets

    Args:
        content: The markdown content to validate

    Returns:
        ValidationResult with is_valid flag and list of violations
    """
    violations = []

    # Find all checkbox-like patterns
    all_checkboxes = INVALID_CHECKBOX_PATTERN.findall(content)
    valid_checkboxes = TODO_CHECKBOX_PATTERN.findall(content)

    # Count invalid checkboxes
    invalid_count = len(all_checkboxes) - len(valid_checkboxes)
    if invalid_count > 0:
        violations.append(
            f"Found {invalid_count} invalid checkbox(es). "
            "Valid formats: [ ], [x], [X], [/]"
        )

    # Check that there's at least one checkbox (it's a TODO list)
    if len(all_checkboxes) == 0:
        violations.append("No checkboxes found. TODO list should have checkbox items.")

    return ValidationResult(is_valid=len(violations) == 0, violations=violations)


def validate_markdown_file(path: str, content: str) -> ValidationResult:
    """
    Dispatch validation based on filename.

    Args:
        path: File path (used to determine which validator to use)
        content: The markdown content to validate

    Returns:
        ValidationResult
    """
    if path.endswith("plan.md"):
        return validate_plan_md(content)
    if path.endswith("todo.md"):
        return validate_todo_md(content)
    # No validation for other markdown files
    return ValidationResult(is_valid=True, violations=[])
