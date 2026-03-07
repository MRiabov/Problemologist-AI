from __future__ import annotations

from dataclasses import dataclass

import yaml

from shared.models.schemas import AssemblyDefinition, PartConfig, SubassemblyEstimate


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


def _append_if_excessive(
    findings: list[ExcessiveDofFinding], part: PartConfig, *, threshold: int
) -> None:
    dofs = list(part.config.dofs or [])
    if len(dofs) <= threshold:
        return
    findings.append(
        ExcessiveDofFinding(part_id=part.name, dofs=dofs, dof_count=len(dofs))
    )
