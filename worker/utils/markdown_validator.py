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

# Strict heading patterns (numbered headings required)
PLAN_SECTION_PATTERNS = [
    r"^##\s*1\.\s*Solution\s+Overview\s*$",
    r"^##\s*2\.\s*Parts\s+List\s*$",
    r"^##\s*3\.\s*Assembly\s+Strategy\s*$",
    r"^##\s*4\.\s*Cost\s*[&and]+\s*Weight\s+Budget\s*$",
    r"^##\s*5\.\s*Risk\s+Assessment\s*$",
]

# Valid checkbox patterns for todo.md
TODO_CHECKBOX_PATTERN = re.compile(r"^\s*-\s*\[([ xX/\-])\]", re.MULTILINE)
INVALID_CHECKBOX_PATTERN = re.compile(r"^\s*-\s*\[[^\]]*\]", re.MULTILINE)

# List/table detection helpers
BULLET_LIST_PATTERN = re.compile(r"^\s*[-*+]\s+\S+", re.MULTILINE)
NUMBERED_LIST_PATTERN = re.compile(r"^\s*\d+\.\s+\S+", re.MULTILINE)


def _extract_sections(content: str) -> dict[str, list[str]]:
    """Return a map of section title to body lines (excluding the heading)."""
    lines = content.splitlines()
    heading_matches: list[tuple[int, str]] = []

    for idx, line in enumerate(lines):
        for section, pattern in zip(
            PLAN_REQUIRED_SECTIONS, PLAN_SECTION_PATTERNS, strict=True
        ):
            if re.match(pattern, line, re.IGNORECASE):
                heading_matches.append((idx, section))
                break

    if not heading_matches:
        return {}

    heading_matches.sort(key=lambda x: x[0])
    sections: dict[str, list[str]] = {}
    for i, (start_idx, section) in enumerate(heading_matches):
        end_idx = (
            heading_matches[i + 1][0] if i + 1 < len(heading_matches) else len(lines)
        )
        body_lines = lines[start_idx + 1 : end_idx]
        sections[section] = body_lines
    return sections


def _has_table(lines: list[str]) -> bool:
    for i in range(len(lines) - 1):
        if "|" in lines[i] and re.match(r"^\s*\|?[\s:-]+\|?[\s|:-]*$", lines[i + 1]):
            return True
    return False


def validate_plan_md(content: str) -> ValidationResult:
    """
    Validate plan.md structure against required sections and list/table requirements.

    Args:
        content: The markdown content to validate

    Returns:
        ValidationResult with is_valid flag and list of violations
    """
    violations: list[str] = []

    for section, pattern in zip(
        PLAN_REQUIRED_SECTIONS, PLAN_SECTION_PATTERNS, strict=True
    ):
        if not re.search(pattern, content, re.IGNORECASE | re.MULTILINE):
            violations.append(f"Missing required section: {section}")

    sections = _extract_sections(content)

    # Enforce list/table requirements for specific sections
    parts_lines = sections.get("## 2. Parts List", [])
    if "## 2. Parts List" in sections and not (
        _has_table(parts_lines) or BULLET_LIST_PATTERN.search("\n".join(parts_lines))
    ):
        violations.append("Parts List must contain a bullet list or table of parts.")

    assembly_lines = sections.get("## 3. Assembly Strategy", [])
    if "## 3. Assembly Strategy" in sections and not NUMBERED_LIST_PATTERN.search(
        "\n".join(assembly_lines)
    ):
        violations.append("Assembly Strategy must contain a numbered list of steps.")

    budget_lines = sections.get("## 4. Cost & Weight Budget", [])
    if "## 4. Cost & Weight Budget" in sections and not (
        _has_table(budget_lines) or BULLET_LIST_PATTERN.search("\n".join(budget_lines))
    ):
        violations.append(
            "Cost & Weight Budget must contain a bullet list or table of items."
        )

    risk_lines = sections.get("## 5. Risk Assessment", [])
    if "## 5. Risk Assessment" in sections and not (
        _has_table(risk_lines) or BULLET_LIST_PATTERN.search("\n".join(risk_lines))
    ):
        violations.append(
            "Risk Assessment must contain a bullet list or table of risks."
        )

    return ValidationResult(is_valid=len(violations) == 0, violations=violations)


def validate_todo_md(
    content: str, require_completion: bool = False, baseline_content: str | None = None
) -> ValidationResult:
    """
    Validate todo.md structure for proper checkbox format.

    Valid checkboxes: [ ], [x], [X], [/], [-]
    Invalid: anything else inside brackets

    Args:
        content: The markdown content to validate
        require_completion: If True, no unchecked items are allowed.
        baseline_content: Optional previous version to ensure no deletions.

    Returns:
        ValidationResult with is_valid flag and list of violations
    """
    violations = []

    # 1. Check for deletions if baseline is provided
    if baseline_content:
        # Extract task text (everything after the checkbox)
        task_pattern = re.compile(r"^\s*-\s*\[[ xX/\-]\]\s*(.*)$", re.MULTILINE)
        baseline_tasks = set(task_pattern.findall(baseline_content))
        current_tasks = set(task_pattern.findall(content))

        missing_tasks = baseline_tasks - current_tasks
        if missing_tasks:
            violations.append(
                f"Found {len(missing_tasks)} deleted TODO items. "
                "You must not delete items from the TODO list."
            )

    # 2. Find all checkbox-like patterns
    all_checkboxes = INVALID_CHECKBOX_PATTERN.findall(content)
    valid_checkboxes = TODO_CHECKBOX_PATTERN.findall(content)

    # Count invalid checkboxes
    invalid_count = len(all_checkboxes) - len(valid_checkboxes)
    if invalid_count > 0:
        violations.append(
            f"Found {invalid_count} invalid checkbox(es). "
            "Valid formats: [ ], [x], [X], [/], [-]"
        )

    # Check that there's at least one checkbox (it's a TODO list)
    if len(all_checkboxes) == 0:
        violations.append("No checkboxes found. TODO list should have checkbox items.")

    if require_completion:
        # Only [x] (completed) or [-] (skipped) are acceptable at submission
        invalid_states = [s for s in valid_checkboxes if s not in ("x", "-")]
        if invalid_states:
            violations.append(
                "All TODO items must be completed or skipped before submission "
                "(only [x] or [-] allowed)."
            )

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
