# Electronics Reviewer

## Role Summary

The Electronics Reviewer checks explicit electromechanical implementation work after coding.

## What It Owns

- `reviews/electronics-review-decision-round-<n>.yaml`
- `reviews/electronics-review-comments-round-<n>.yaml`

## What It Reads

- `engineering_plan.md`
- `todo.md`
- `benchmark_definition.yaml`
- `benchmark_assembly_definition.yaml`
- `assembly_definition.yaml`
- `benchmark_script.py` when benchmark-side electronics exist
- `benchmark_plan_evidence_script.py` when it exists
- `benchmark_plan_technical_drawing_script.py` when it exists
- `plan_refusal.md` when present
- `solution_script.py`
- `solution_plan_evidence_script.py` when it exists
- `solution_plan_technical_drawing_script.py` when it exists
- `validation_results.json`
- `simulation_result.json`
- `renders/current-episode/**` when it exists
- `renders/**`
- `.manifests/electronics_review_manifest.json`

## Native Tool Surface

- `list_files`
- `read_file`
- `inspect_media`
- `write_file`
- `grep`
- `execute_command`
- `inspect_topology`
- `invoke_cots_search_subagent`

## What Humans Must Tell It

- Use this reviewer only when explicit electronics work exists.
- Verify the circuit, the power budget, and the physical routing separately.
- Inspect media when it exists.
- If the task has moving parts, check the latest dynamic evidence before approval.
- Keep the review read-only and stage-scoped.
- If bug-report mode is enabled and runtime plumbing blocks the review, write `bug_report.md` at the workspace root and continue unless the reviewer is actually blocked.
- Write only the stage-owned decision/comments YAML pair, then finish with `bash scripts/submit_review.sh`.

## Acceptance Checklist

- The electrical implementation matches the handoff.
- The circuit is valid.
- The routed wires or connectors are physically plausible.
- The review output is limited to the stage-owned YAML pair.

## Related Skills

- `.agents/skills/electronics-engineering/SKILL.md`
- `.agents/skills/electromechanics-syntax/SKILL.md`
- `.agents/skills/render-evidence/SKILL.md`
- `.agents/skills/cots-parts/SKILL.md` when motors or connectors are catalog-backed
- `.agents/skills/runtime-script-contract/SKILL.md`
