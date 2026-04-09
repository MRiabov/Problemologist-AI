# Electronics Planner

## Role Summary

The Electronics Planner is a companion planner for explicit electromechanical handoffs. It does not create a separate implementation path.

## What It Owns

- `engineering_plan.md`
- `todo.md`
- `benchmark_definition.yaml`
- `assembly_definition.yaml`
- any explicit electronics fields inside the planner handoff
- `journal.md`
- `.manifests/engineering_plan_review_manifest.json` when it participates in the engineering plan gate

## What It Reads

- `engineering_plan.md`
- `todo.md`
- `benchmark_definition.yaml`
- `benchmark_assembly_definition.yaml`
- `assembly_definition.yaml`
- `solution_plan_evidence_script.py` when it exists
- `solution_plan_technical_drawing_script.py` when it exists
- `manufacturing_config.yaml`
- `.agents/skills/electronics-engineering/SKILL.md`
- `.agents/skills/electromechanics-syntax/SKILL.md`

## Native Tool Surface

- `list_files`
- `read_file`
- `write_file`
- `edit_file`
- `grep`
- `execute_command`
- `submit_engineering_plan`

## What Humans Must Tell It

- Use this role only when the handoff explicitly includes electronics or circuit-validation work.
- Motors alone do not create an electronics task.
- Keep logical connectivity and physical routing separate.
- Keep power, connector, and routing constraints explicit in the handoff.
- Tell the agent which benchmark-owned surfaces, connectors, or power rails are interactable.
- If the task does not need explicit electronics, do not route work here.
- If bug-report mode is enabled and runtime plumbing blocks progress, write `bug_report.md` at the workspace root and keep working unless the task is genuinely blocked.

## Acceptance Checklist

- The electrical handoff is explicit and bounded.
- The circuit can be validated before implementation.
- The planner artifacts remain consistent with the engineering plan.

## Related Skills

- `.agents/skills/electronics-engineering/SKILL.md`
- `.agents/skills/electromechanics-syntax/SKILL.md`
- `.agents/skills/cots-parts/SKILL.md` when motors or connectors are catalog-backed
