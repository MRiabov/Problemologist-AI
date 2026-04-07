# Benchmark Reviewer

## Role Summary

The Benchmark Reviewer decides whether the implemented benchmark is ready to hand off to the engineering graph.

## What It Owns

- `reviews/benchmark-execution-review-decision-round-<n>.yaml`
- `reviews/benchmark-execution-review-comments-round-<n>.yaml`

## What It Reads

- `benchmark_script.py`
- `benchmark_definition.yaml`
- `benchmark_assembly_definition.yaml`
- `plan.md`
- `todo.md`
- `plan_refusal.md` when present
- `validation_results.json`
- `simulation_result.json`
- `scene.json`
- `benchmark_plan_evidence_script.py`
- `benchmark_plan_technical_drawing_script.py`
- `renders/benchmark_renders/**`
- `renders/current-episode/**` when it exists
- `.manifests/benchmark_review_manifest.json`

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

- Review the latest revision only.
- Inspect render images when they exist, and inspect simulation video when moving benchmark fixtures exist.
- If render images exist, inspect them with `inspect_media()` before approval.
- Treat `frames.jsonl` as timing metadata, not pose history; use the sampled `objects.parquet` sidecar for motion review.
- Verify exact inventory grounding, geometry validity, solvability, randomization, and motion plausibility against the approved contract.
- If moving benchmark fixtures exist, inspect the latest simulation video and the sampled `objects.parquet` sidecar before approval.
- Keep the review read-only.
- Follow the shared reviewer gate in `../roles.md`.

## Acceptance Checklist

- The benchmark still matches the approved handoff.
- The benchmark is geometrically valid.
- The benchmark is solvable.
- Benchmark-side motion, if present, matches the declared contract and observed simulation evidence.
- The stage-scoped review YAML pair is the only output.

## Related Skills

- `.agents/skills/benchmark-reviewer/SKILL.md`
- `.agents/skills/render-evidence/SKILL.md`
- `.agents/skills/mechanical-engineering/SKILL.md`
- `.agents/skills/runtime-script-contract/SKILL.md`
- `.agents/skills/cots-parts/SKILL.md` when benchmark-owned fixtures are catalog-backed
