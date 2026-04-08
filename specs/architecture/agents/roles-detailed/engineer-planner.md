# Engineering Planner

## Role Summary

The Engineering Planner turns benchmark context into an implementation-ready engineering plan.

## What It Owns

- `engineering_plan.md`
- `todo.md`
- `benchmark_definition.yaml`
- `assembly_definition.yaml`
- `solution_plan_evidence_script.py`
- `solution_plan_technical_drawing_script.py`
- `journal.md`
- `renders/engineer_plan_renders/` persistent render evidence
- `renders/current-episode/` scratch evidence during the active run
- `.manifests/engineering_plan_review_manifest.json` via `submit_plan()`

## What It Reads

- `benchmark_definition.yaml`
- `benchmark_assembly_definition.yaml`
- `benchmark_script.py` when it exists
- `benchmark_plan_evidence_script.py` when it exists
- `benchmark_plan_technical_drawing_script.py` when it exists
- `solution_plan_evidence_script.py` when it exists
- `solution_plan_technical_drawing_script.py` when it exists
- `engineering_plan.md`
- `todo.md`
- `manufacturing_config.yaml`
- `renders/benchmark_renders/**`
- `renders/engineer_plan_renders/**` when it exists
- `renders/current-episode/**` when it exists
- `.agents/skills/engineer-planner/SKILL.md`

## Native Tool Surface

- `list_files`
- `read_file`
- `inspect_media`
- `inspect_topology`
- `render_technical_drawing`
- `write_file`
- `edit_file`
- `invoke_cots_search_subagent`
- `validate_costing_and_price`
- `submit_plan`

## Runtime Helpers To Use From Scripts

- `render_cad(...)` for live solution-scene inspection
- `render_technical_drawing()` for the drafting package
- `objectives_geometry()` when objective overlays are needed in a preview

## What Humans Must Tell It

- The benchmark objective zones, runtime jitter, and budget caps are the contract it must solve against.
- `benchmark_assembly_definition.yaml` is read-only context.
- The planner must preserve benchmark-owned fixtures and exact grounded inventory mentions while designing the engineer solution.
- `assembly_definition.yaml` is the machine-readable solution contract; `validate_costing_and_price()` normalizes the final totals.
- If the solution needs motion proof, the planner must state the coarse motion forecast and keep it consistent with the fine-grained trajectory file if one exists.
- If explicit electronics work exists, the planner must also keep the electrical handoff explicit and bounded.
- If render images already exist for the current revision, inspect them with `inspect_media()` before submission.
- Keep exploded or layout presentation out of `solution_plan_evidence_script.py`; use `solution_plan_technical_drawing_script.py` for that presentation layer.
- Treat `solution_plan_technical_drawing_script.py` as the presentation companion, not a second geometry contract.

## Acceptance Checklist

- The plan is physically feasible.
- The chosen mechanism is the smallest credible one that solves the task.
- Cost and weight stay under the benchmark caps.
- The drafting package has been inspected when drafting mode is active.
- `submit_plan()` is the final gate, not a substitute for a coherent handoff.

## Related Skills

- `.agents/skills/engineer-planner/SKILL.md`
- `.agents/skills/build123d-cad-drafting-skill/SKILL.md`
- `.agents/skills/build123d-technical-drawing/SKILL.md`
- `.agents/skills/runtime-script-contract/SKILL.md`
- `.agents/skills/mechanical-engineering/SKILL.md`
- `.agents/skills/manufacturing-knowledge/SKILL.md`
- `.agents/skills/cots-parts/SKILL.md`
- `.agents/skills/render-evidence/SKILL.md`
- `.agents/skills/electronics-engineering/SKILL.md` when explicit electronics exist
- `.agents/skills/electromechanics-syntax/SKILL.md` when explicit moving electrical parts exist
