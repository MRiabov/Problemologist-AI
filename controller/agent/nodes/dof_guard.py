from __future__ import annotations

from dataclasses import dataclass

import yaml

from shared.enums import ReviewDecision
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


@dataclass(frozen=True)
class ExcessiveDofFinding:
    part_id: str
    dofs: list[str]
    dof_count: int


def collect_excessive_dof_findings(
    assembly_definition_content: str, *, threshold: int = 3
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

    Accepted markers in `plan.md`:
    - `DOF_JUSTIFICATION_ACCEPTED` (global)
    - `DOF_JUSTIFICATION:<part_id>` (part-specific)
    """
    lowered = plan_markdown.lower()
    if "dof_justification_accepted" in lowered:
        return True
    token = f"dof_justification:{part_id.lower()}"
    return token in lowered


def canonical_dof_checklist_key(reviewer_stage: str) -> str:
    try:
        return CANONICAL_DOF_CHECKLIST_KEYS[reviewer_stage]
    except KeyError as exc:
        raise ValueError(
            f"Unsupported reviewer_stage for canonical DOF checklist: {reviewer_stage}"
        ) from exc


def apply_canonical_dof_checklist(
    review: ReviewResult, *, reviewer_stage: str
) -> ReviewResult:
    """
    Ensure review traces and persisted comments use the canonical DOF checklist key.

    The key is stage-specific so downstream evals can distinguish minimality
    checks from justified deviation checks without relying on freeform prose.
    """

    checklist = dict(review.checklist)
    key = canonical_dof_checklist_key(reviewer_stage)
    checklist.setdefault(
        key,
        "pass" if review.decision == ReviewDecision.APPROVED else "fail",
    )
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
