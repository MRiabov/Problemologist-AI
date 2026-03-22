# Story 1.1: Benchmark Geometry and Objectives

Status: review

## Story

As a human operator, I want to define benchmark goals, forbid zones, build zones, and runtime randomization so that the benchmark has a clear, bounded, solvable contract.

## Acceptance Criteria

1. Given valid geometry inputs, when benchmark validation runs, then the system accepts only non-intersecting, in-bounds objective geometry.
1. Given a valid benchmark package, then the package preserves the declared `goal_zone`, `forbid_zones`, and `build_zone` in `benchmark_definition.yaml`.
1. Given runtime randomization values, when the benchmark is saved, then those ranges are persisted and visible in the benchmark package.
1. Given a moved object with static randomization and runtime jitter, then the full runtime envelope stays inside `build_zone` and does not intersect any forbid zone across the declared randomization range.
1. Given invalid geometry, overlapping objective zones, or out-of-bounds randomization, then validation fails closed with an explicit reason rather than silently correcting the benchmark.
1. Given a benchmark definition for this story, then it remains schema-valid with at least one `benchmark_parts` entry, unique top-level part labels, and a `moved_object.material_id` that resolves to a known material in `manufacturing_config.yaml`.

## Tasks / Subtasks

- [x] Verify the benchmark geometry contract in `shared/models/schemas.py` still matches the story shape: `ObjectivesSection`, `MovedObject`, `Constraints`, and `RandomizationMeta`.
- [x] Confirm `worker_heavy/utils/file_validation.py` rejects template placeholders, invalid materials, and malformed `benchmark_definition.yaml` payloads for this geometry story.
- [x] Confirm `worker_heavy/utils/validation.py::_validate_benchmark_definition_consistency` enforces AABB objective rules, build-zone containment, and randomization-envelope containment.
- [x] Keep the benchmark planner prompt contract in `config/prompts.yaml` aligned with this story's exact geometry and randomization requirements.
- [x] Preserve a minimal but valid `benchmark_parts` fixture entry in the benchmark package so the schema passes while the story remains focused on geometry and objectives.
- [x] Add or refresh integration coverage for objective geometry validation, randomization persistence, and benchmark workflow success/failure behavior.
- [x] Run the integration tests that cover benchmark-definition validation and the benchmark generation workflow before marking the story done.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 1, Story 1.1 is the source of truth for the human-facing requirement.
  - Benchmark objectives are AABBs, and touching a goal/forbid zone counts as failure.
  - Runtime randomization is smaller than static randomization, but the benchmark package must persist both clearly.
  - The benchmark planner handoff is fail-closed: missing artifacts, placeholders, invalid geometry, or unknown materials must reject the run.
  - `moved_object.material_id` is mandatory and must come from `manufacturing_config.yaml`.
  - Top-level labels must remain unique and must not be `environment` or start with `zone_`.
  - The benchmark planner must still produce a valid `benchmark_assembly_definition.yaml`, but this story should not invent moving benchmark fixtures unless the geometry truly needs them.
- Source tree components to touch:
  - `shared/models/schemas.py`
  - `worker_heavy/utils/file_validation.py`
  - `worker_heavy/utils/validation.py`
  - `config/prompts.yaml`
  - `tests/integration/architecture_p0/test_planner_gates.py`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py`
- Testing standards summary:
  - Use integration tests only; do not add unit-test-only coverage for this story.
  - Prefer the existing planner-gate and benchmark-workflow tests as the regression anchor.
  - If render evidence is involved during review, the reviewer path must use `inspect_media(...)`; file listing is not visual inspection.

### Project Structure Notes

- Keep the geometry/objective contract in the shared schema and worker validation layers. Do not create a new benchmark-geometry subsystem.
- Keep the benchmark planner prompt and schema aligned. Prompt text should not drift from the actual `BenchmarkDefinition` fields or the validation rules.
- If a minimal benchmark fixture is needed to satisfy schema requirements, keep it minimal and read-only. Do not expand into fixture-motion or engineering-attachment concerns in this story.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 1.1: Benchmark Geometry and Objectives]
- [Source: \_bmad-output/planning-artifacts/prd.md, Functional Requirements FR1-FR6 and benchmark-definition sections]
- [Source: specs/architecture/agents/roles.md, Benchmark generator agent and benchmark-reviewer responsibilities]
- [Source: specs/architecture/handover-contracts.md, Benchmark Planner and Benchmark Plan Reviewer contract]
- [Source: specs/architecture/simulation-and-dod.md, objective AABB rules, build-zone rules, and randomization contract]
- \[Source: shared/models/schemas.py, `BenchmarkDefinition`, `ObjectivesSection`, `MovedObject`, `Constraints`, and `RandomizationMeta`\]
- \[Source: worker_heavy/utils/file_validation.py, `validate_benchmark_definition_yaml` and placeholder/material validation\]
- \[Source: worker_heavy/utils/validation.py, `_validate_benchmark_definition_consistency`\]
- [Source: config/prompts.yaml, benchmark planner prompt contract and mandatory output files]
- [Source: docs/backend-reference.md, benchmark planner, validation, and CAD/workbench rules]
- [Source: specs/integration-tests.md, INT-008, INT-024, INT-031, and INT-188]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

- `2026-03-22T08:02:04Z`: Ran the focused integration slice covering `test_int_008_objectives_validation.py`, the benchmark planner gate subset, and `test_benchmark_workflow.py`.
- `2026-03-22T08:05:39Z`: Ran `tests/integration/architecture_p0/test_int_008_objectives_validation.py` directly; all objective-validation tests passed.
- `2026-03-22T08:04:13Z`: Reran the benchmark workflow slice after adding the benchmark-definition persistence assertions; it passed.
- `2026-03-22T08:06:41Z`: Ran `uv run ruff check tests/integration/architecture_p1/test_benchmark_workflow.py`; the modified file is lint-clean.

### Completion Notes List

- The shared schema and worker validation layers already matched the story contract, so no production code changes were needed for the benchmark geometry/objective rules themselves.
- Refreshed `tests/integration/architecture_p1/test_benchmark_workflow.py` so the workflow test now reads back `benchmark_definition.yaml` and asserts persisted `benchmark_parts`, unique `part_id`/`label` values, `moved_object.material_id`, and serialized randomization fields.
- Confirmed the geometry failure paths remain fail-closed through the existing INT-008 coverage for overlapping zones, template placeholders, legacy attachment methods, missing `benchmark_parts`, and missing reviewer stage.
- Verified the benchmark generation workflow still completes and emits the expected review manifests and artifact bundle.

### File List

- `_bmad-output/implementation-artifacts/1-1-benchmark-geometry-and-objectives.md`
- `tests/integration/architecture_p1/test_benchmark_workflow.py`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
