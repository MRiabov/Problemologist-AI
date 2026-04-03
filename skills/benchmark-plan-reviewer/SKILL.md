---
name: benchmark-plan-reviewer
description: Review benchmark planner handoffs before coding starts. Use when validating `plan.md`, `todo.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `benchmark_plan_evidence_script.py`, `benchmark_plan_technical_drawing_script.py`, render evidence, or the benchmark plan-review manifest; when applying the benchmark-plan review checklist for cross-artifact consistency, motion visibility, and geometry feasibility; or when writing the stage-scoped benchmark-plan review YAML pair under `reviews/`.
---

# Benchmark Plan Reviewer

## Review Checklist

- [ ] Confirm the latest planner revision and matching `.manifests/benchmark_plan_review_manifest.json`.
- [ ] Read `plan.md`, `todo.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `benchmark_plan_evidence_script.py`, and `benchmark_plan_technical_drawing_script.py` as read-only context.
- [ ] If render or drawing evidence exists, inspect it with `inspect_media(...)`. If drafting evidence must be materialized first, call `preview_drawing()` and inspect the persisted output.
- [ ] Treat `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` as the inspectable source of the approved benchmark solution, not just helper files.
- [ ] Verify cross-artifact consistency for labels, repeated quantities, COTS identities, zone geometry, randomization, bounds, runtime jitter, and motion facts. See `references/review_contracts.md`.
- [ ] Verify `moved_object.material_id` resolves to a known material and `benchmark_assembly_definition.yaml` is a schema-valid full `AssemblyDefinition`.
- [ ] Reject planner drafts that depend on free-form XYZ placement as the primary positioning mechanism; require selector-driven placement, explicit mates/joints, or clearly bounded absolute anchors.
- [ ] Verify benchmark-side motion is explicit, reconstructable, and feasible; reject hidden motion, unsupported motion, or any drafting script that lacks a real `TechnicalDrawing` construction path.
- [ ] Reject invented defaults, placeholder geometry, unlabeled inventory drift, or mismatched object references across planner artifacts.
- [ ] Write only `reviews/benchmark-plan-review-decision-round-<n>.yaml` and `reviews/benchmark-plan-review-comments-round-<n>.yaml`.
- [ ] Use the decision YAML as the routing source of truth, then run `bash scripts/submit_review.sh`.

## Comment Checklist

- [ ] Populate the stage keys from the handover contract: `cross_artifact_consistency`, `feasible_mechanism`, `budget_realism`, and `dof_minimality`.
- [ ] Use `pass`, `fail`, or `not_applicable` values only.
- [ ] Keep the summary factual and the required fixes concrete.

## References

- `references/review_contracts.md` for the detailed cross-artifact, motion, and media rules.
- `specs/architecture/agents/handover-contracts.md` for the canonical checklist keys and output contract.
