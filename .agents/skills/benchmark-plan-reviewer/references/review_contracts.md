# Benchmark Plan Review Contracts

## Core checklist

- Review only the latest revision in the active workspace.
- Treat planner-owned artifacts as read-only.
- Fail closed on missing, stale, invalid, or cross-revision artifacts.
- Use `inspect_media(...)` for render, image, or video evidence. Do not rely on filenames or text summaries alone.
- If drafting evidence must be materialized for the current revision, call `preview_drawing()` first and inspect the persisted output.
- Keep the decision YAML as the routing source of truth. Keep the comments YAML explanatory and evidence-based.
- Treat `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` as the inspectable source of the approved benchmark contract.

## Required planner handoff

- `.manifests/benchmark_plan_review_manifest.json`
- `plan.md`
- `todo.md`
- `benchmark_definition.yaml`
- `benchmark_assembly_definition.yaml`
- `benchmark_plan_evidence_script.py`
- `benchmark_plan_technical_drawing_script.py`
- `renders/**` when planner-side previews already exist

## Cross-artifact checklist

- Match all object names, labels, repeated quantities, and COTS identities across `plan.md`, the YAML files, and both planner scripts.
- Ensure every planner-declared inventory label and selected COTS `part_id` appears in `plan.md` as an exact identifier mention.
- Reject nonexistent objects, relabeled inventory, silent renames, or missing repeated entries.
- Require `benchmark_assembly_definition.yaml` to be a schema-valid full `AssemblyDefinition`.
- Require `moved_object.material_id` to be present and to reference a known material from `manufacturing_config.yaml`.
- Require zone geometry, randomization, runtime jitter, and build-zone bounds to stay mutually consistent.
- Reject planner drafting scripts that drift from the approved inventory or omit a real `TechnicalDrawing` construction path.

## Motion checklist

- Reject hidden, unsupported, or contradictory benchmark-side motion.
- Require any moving benchmark fixture to have a fully explicit motion contract that the reviewer can reconstruct from the handoff and evidence.
- Require enough motion-visible facts for downstream engineering intake, including the declared motion type, axis or path reference when applicable, and operating limits or envelope.
- Reject motion that is impossible, unstable, or not grounded in the declared benchmark artifacts.

## Comments checklist

- Use the stage-specific checklist keys from the handover contract in the comments YAML.
- Plan-review keys include `cross_artifact_consistency`, `feasible_mechanism`, `budget_realism`, and `dof_minimality`.
- Treat checklist values as `pass`, `fail`, or `not_applicable` only.
- Keep the summary factual and the required fixes concrete.

## Evidence fields

- `latest_revision_verified`
- `review_manifest_revision`
- `render_count`
- `inspected_render_count`
- `visual_inspection_min_images`
- `visual_inspection_satisfied`
- `deterministic_error_count`
- `deterministic_refusal_reason`
- `solvability_summary`
- `attachment_policy_summary`

## Output and submission

- Write only the stage-scoped benchmark-plan review decision YAML and comments YAML under `reviews/`.
- Keep comments factual, specific, and tied to evidence.
- Submit the review with `bash scripts/submit_review.sh` after the files are written.
