from __future__ import annotations

import re
from dataclasses import dataclass

import yaml

from shared.enums import AgentName, ReviewDecision
from shared.models.schemas import (
    AssemblyDefinition,
    PartConfig,
    ReviewResult,
    SubassemblyEstimate,
)

CANONICAL_DOF_CHECKLIST_KEYS = {
    "engineering_plan_reviewer": "dof_minimality",
    "engineering_execution_reviewer": "dof_deviation_justified",
}
DOF_MINIMALITY_THRESHOLD = 3
_DOF_JUSTIFICATION_LINE_RE = re.compile(
    r"^\s*(?:[-*+]\s+|\d+\.\s+)?"
    r"(?P<marker>DOF_JUSTIFICATION_ACCEPTED|DOF_JUSTIFICATION:(?P<part_id>[^\s]+))"
    r"\s*$"
)


@dataclass(frozen=True)
class ExcessiveDofFinding:
    part_id: str
    dofs: list[str]
    dof_count: int


def expected_minimal_dofs(finding: ExcessiveDofFinding) -> list[str]:
    """Return the canonical minimal DOF set expected by the reviewer gate."""
    return finding.dofs[:DOF_MINIMALITY_THRESHOLD]


def expected_minimal_engineering_dofs(finding: ExcessiveDofFinding) -> list[str]:
    """
    Return the canonical engineering DOF set used in observability payloads.

    Keep the alias separate so downstream consumers can key off a stable,
    explicit field name while the legacy `expected_minimal_dofs` field remains
    available for compatibility.
    """

    return expected_minimal_dofs(finding)


def build_excessive_dof_event_payload(
    finding: ExcessiveDofFinding, *, reviewer_stage: str
) -> dict[str, object]:
    """Build the canonical excessive-DOF observability payload."""
    return {
        "reviewer_stage": reviewer_stage,
        "part_id": finding.part_id,
        "proposed_dofs": finding.dofs,
        "expected_minimal_engineering_dofs": expected_minimal_engineering_dofs(finding),
        "expected_minimal_dofs": expected_minimal_dofs(finding),
        "dof_count": finding.dof_count,
        "dof_count_gt_3": finding.dof_count > DOF_MINIMALITY_THRESHOLD,
    }


def collect_excessive_dof_findings(
    assembly_definition_content: str, *, threshold: int = DOF_MINIMALITY_THRESHOLD
) -> list[ExcessiveDofFinding]:
    """Return parts whose DOF count exceeds deterministic reviewer threshold."""
    raw = yaml.safe_load(assembly_definition_content) or {}
    assembly = AssemblyDefinition.model_validate(raw)
    findings: list[ExcessiveDofFinding] = []

    for item in assembly.final_assembly:
        if isinstance(item, SubassemblyEstimate):
            for part in item.parts:
                _append_if_excessive(findings, part, threshold=threshold)
            continue
        if isinstance(item, PartConfig):
            _append_if_excessive(findings, item, threshold=threshold)

    return findings


def has_accepted_dof_justification(plan_markdown: str, *, part_id: str) -> bool:
    """
    Lightweight, deterministic escape hatch for justified high-DOF mechanisms.

    Accepted markers in `engineering_plan.md`:
    - `DOF_JUSTIFICATION_ACCEPTED` (global)
    - `DOF_JUSTIFICATION:<part_id>` (part-specific)
    """
    normalized_part_id = part_id.strip().lower()
    if not normalized_part_id:
        return False

    for line in plan_markdown.splitlines():
        match = _DOF_JUSTIFICATION_LINE_RE.match(line)
        if not match:
            continue
        marker = match.group("marker")
        if marker == "DOF_JUSTIFICATION_ACCEPTED":
            return True
        line_part_id = (match.group("part_id") or "").lower()
        if line_part_id == normalized_part_id:
            return True
    return False


def canonical_dof_checklist_key(reviewer_stage: str) -> str:
    try:
        return CANONICAL_DOF_CHECKLIST_KEYS[reviewer_stage]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported reviewer_stage for canonical DOF checklist: {reviewer_stage}"
        ) from exc


def apply_canonical_dof_checklist(
    review: ReviewResult,
    *,
    reviewer_stage: str,
    checklist_value: str | None = None,
    overwrite: bool = False,
) -> ReviewResult:
    """
    Ensure review traces and persisted comments use the canonical DOF checklist key.

    The key is stage-specific so downstream evals can distinguish minimality
    checks from justified deviation checks without relying on freeform prose.
    """

    checklist = dict(review.checklist)
    key = canonical_dof_checklist_key(reviewer_stage)
    if review.decision == ReviewDecision.APPROVED:
        checklist[key] = "pass"
        if reviewer_stage == AgentName.ENGINEER_EXECUTION_REVIEWER:
            checklist.setdefault("dof_minimality", "pass")
        elif reviewer_stage == AgentName.ENGINEER_PLAN_REVIEWER:
            checklist.setdefault("dof_deviation_justified", "pass")
    elif overwrite or key not in checklist:
        if checklist_value is None:
            checklist_value = "not_applicable"
        checklist[key] = checklist_value
    return review.model_copy(update={"checklist": checklist})


def _append_if_excessive(
    findings: list[ExcessiveDofFinding], part: PartConfig, *, threshold: int
) -> None:
    dofs = list(part.config.dofs or [])
    if len(dofs) <= threshold:
        return
    findings.append(
        ExcessiveDofFinding(part_id=part.name, dofs=dofs, dof_count=len(dofs))
    )
