# Benchmark Planner

## Role Summary

The Benchmark Planner turns a problem brief into a benchmark handoff package. It designs the task, not the solution.

## What It Owns

- `plan.md`
- `todo.md`
- `benchmark_definition.yaml`
- `benchmark_assembly_definition.yaml`
- `benchmark_plan_evidence_script.py`
- `benchmark_plan_technical_drawing_script.py`
- `journal.md`
- `renders/benchmark_renders/` persistent preview evidence
- `renders/current-episode/` scratch evidence during the active run
- `.manifests/benchmark_plan_review_manifest.json` via `submit_plan()`

## What It Reads

- `specs/architecture/agents/roles.md`
- `.agents/skills/benchmark-planner/SKILL.md`
- `benchmark_definition.yaml`
- `benchmark_assembly_definition.yaml`
- `manufacturing_config.yaml`
- `renders/benchmark_renders/**` when they exist
- `renders/current-episode/**` when it exists
- the shared benchmark-runtime docs for objectives, motion, and file ownership

## Native Tool Surface

- `list_files`
- `read_file`
- `inspect_media`
- `write_file`
- `edit_file`
- `grep`
- `execute_command`
- `inspect_topology`
- `invoke_cots_search_subagent`
- `preview_drawing`
- `submit_plan`

## Runtime Helpers To Use From Scripts

- `preview(...)` for a live benchmark scene with objective overlays
- `objectives_geometry()` for reconstructed objective bodies
- `preview_drawing()` when inspecting the drafted drawing package

## What Humans Must Tell It

- The benchmark objective geometry, build zone, forbid zones, and runtime jitter are authoritative.
- Benchmark-owned fixtures, input objects, and objective markers are read-only context, not engineer-owned deliverables.
- The benchmark handoff must stay exact across `plan.md`, the YAML files, and both planner scripts.
- `moved_object.material_id` must resolve to a known material from `manufacturing_config.yaml`.
- Moving benchmark-owned fixtures need explicit, reviewer-visible motion facts.
- If render images already exist for the current revision, inspect them with `inspect_media()` before submission.
- Do not expect `benchmark_script.py` in the workspace until after plan approval.
- Treat `benchmark_plan_technical_drawing_script.py` as the presentation companion, not a second geometry contract.
- `invoke_cots_search_subagent` is for engineer-side candidate parts, not for benchmark-owned fixtures.
- `submit_plan()` is the only completion gate; do not hand off before the package is internally consistent.

## Acceptance Checklist

- The benchmark is solvable in principle.
- The geometry is valid and the objective bodies do not overlap the forbidden or spawn regions.
- The planner scripts preserve the same labels, repeated quantities, and COTS identities as the plan.
- Drafting mode, when enabled, has been inspected with `preview_drawing()`.
- The planner wrote realistic estimated cost and weight values before submission.

## Related Skills

- `.agents/skills/benchmark-planner/SKILL.md`
- `.agents/skills/build123d-cad-drafting-skill/SKILL.md`
- `.agents/skills/build123d-technical-drawing/SKILL.md`
- `.agents/skills/runtime-script-contract/SKILL.md`
- `.agents/skills/mechanical-engineering/SKILL.md`
- `.agents/skills/manufacturing-knowledge/SKILL.md`
- `.agents/skills/cots-parts/SKILL.md`
- `.agents/skills/render-evidence/SKILL.md`
