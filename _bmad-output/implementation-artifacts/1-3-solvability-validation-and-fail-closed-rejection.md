# Story 1.3: Solvability Validation and Fail-Closed Rejection

Status: done

## Story

As a human operator, I want the system to validate solvability and reject ambiguous, impossible, or unsupported benchmark setups so that only benchmark candidates that can actually enter the solution workflow progress past planning.

## Acceptance Criteria

1. Given benchmark objective geometry that is contradictory or obstructed, when validation runs, then the benchmark is rejected with a deterministic reason and never transitions to `PLANNED` or `ACCEPTED`.
2. Given a moved object plus static randomization and runtime jitter that leaves the `build_zone` or intersects a `forbid_zone`, when validation runs, then the system fails closed and names the violated boundary in the error message.
3. Given benchmark-owned moving fixtures or motion metadata that is missing, contradictory, or unsupported by reviewer-visible bounds, axes, controller facts, or declared motion topology, when the benchmark plan reviewer inspects the handoff, then the reviewer rejects it with an explicit solvability reason rather than inferring motion.
4. Given a benchmark candidate that is schema-valid but logically unsolvable, when planner handoff validation or reviewer entry validation runs, then the controller records a machine-readable rejection reason and failure class and blocks downstream coder/reviewer entry.
5. Given a rejected benchmark candidate, when the workflow is queried later, then the persisted traces, review artifacts, and validation logs show the exact reason code, failing boundary, and stage that rejected the benchmark.

## Tasks / Subtasks

- [x] Tighten solvability checks in `worker_heavy/utils/validation.py::_validate_benchmark_definition_consistency` and related helpers so rejection reasons are explicit for goal/forbid intersections, build-zone envelope violations, runtime-envelope collisions, and other impossible geometry conditions.
  - [x] Reuse the existing benchmark geometry and randomization models; do not add a parallel validator.
  - [x] Use the existing benchmark refusal vocabulary (`INVALID_OBJECTIVES`, `CONTRADICTORY_CONSTRAINTS`, `UNSOLVABLE_SCENARIO`, `AMBIGUOUS_TASK`) and fail closed instead of auto-correcting.
- [x] Propagate solvability failures through `worker_heavy/utils/file_validation.py`, `controller/agent/benchmark_handover_validation.py`, and `controller/agent/review_handover.py` so benchmark plan review and benchmark coder entry both stop before downstream solution work.
  - [x] Keep the benchmark plan-review manifest and latest-revision checks strict; do not fall back to stale or inferred approval.
  - [x] Ensure benchmark-side motion metadata in `benchmark_assembly_definition.yaml` is rejected when it cannot be explained with reviewer-visible motion facts.
- [x] Update benchmark planner / reviewer guidance in `controller/agent/benchmark/nodes.py` and, if needed, the benchmark starter template so the LLM is instructed to reject impossible or unsupported benchmark setups instead of brute-forcing `submit_plan()`.
  - [x] Keep the guidance aligned with `specs/architecture/simulation-and-dod.md` and the existing fail-closed planner handoff contract.
- [x] Extend integration coverage in `tests/integration/architecture_p0/test_int_008_objectives_validation.py` and `tests/integration/architecture_p0/test_planner_gates.py` for negative solvability cases.
  - [x] Add explicit rejection cases for obstructed goals, build-zone violations, runtime-envelope collisions, and unsupported benchmark motion.
  - [x] Assert that the error text names the failing boundary and that controller traces record the rejection.
- [x] Add an end-to-end rejection test in `tests/integration/architecture_p1/test_benchmark_workflow.py` for a benchmark prompt that would otherwise enter the workflow but must now be stopped by solvability validation.
  - [x] Add a deterministic mock-response scenario under `tests/integration/mock_responses/` if the workflow test needs a seeded transcript to reach the rejection path.
- [x] Run the integration slices that cover benchmark-definition validation, planner gates, and benchmark workflow before marking the story complete.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 1, Story 1.3 is the source of truth for the human-facing requirement.
  - The system must fail closed before an unsolvable benchmark reaches the solution workflow; silent correction, inferred success, or partial acceptance are regressions.
  - Benchmark objectives are AABBs, touching a goal/forbid zone counts as failure, and runtime jitter must be considered when validating the full moved-object envelope.
  - Benchmark-owned moving fixtures are benchmark-only exceptions, but they still must have an explicit motion contract that is reviewable and consistent with the handoff artifacts.
  - Existing enums already provide the refusal vocabulary: `BenchmarkRefusalReason` (`INVALID_OBJECTIVES`, `CONTRADICTORY_CONSTRAINTS`, `UNSOLVABLE_SCENARIO`, `AMBIGUOUS_TASK`), `TerminalReason.HANDOFF_INVARIANT_VIOLATION`, and `FailureClass.AGENT_SEMANTIC_FAILURE`.
  - `worker_heavy/utils/validation.py::_validate_benchmark_definition_consistency` already checks goal/forbid overlap and runtime-envelope containment; extend that source instead of introducing a second geometry validator.
  - `validate_benchmark_definition_yaml`, `validate_node_output`, and the benchmark handover validators are the propagation points that keep the rejection visible at controller/worker boundaries.
  - `benchmark_definition.yaml` and `benchmark_assembly_definition.yaml` remain benchmark-owned, read-only handoff artifacts. Do not mutate them to encode solution-side workarounds.
  - If render images exist for the current revision, benchmark-reviewer/plan-reviewer visual-inspection policy still applies; use `inspect_media(...)` rather than file listing.
- Source tree components to touch:
  - `worker_heavy/utils/validation.py`
  - `worker_heavy/utils/file_validation.py`
  - `controller/agent/benchmark_handover_validation.py`
  - `controller/agent/review_handover.py`
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/benchmark/nodes.py`
  - `tests/integration/architecture_p0/test_int_008_objectives_validation.py`
  - `tests/integration/architecture_p0/test_planner_gates.py`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py`
  - `tests/integration/mock_responses/`
- Testing standards summary:
  - Use integration tests only; do not add unit-test-only coverage for this story.
  - Assert against HTTP responses, persisted artifacts, traces, and emitted validation/review events.
  - If the rejection path produces render evidence, the review path must use `inspect_media(...)`; reading `renders/` paths is not visual inspection.
  - Keep the failure assertions specific: verify the rejected boundary, the refusal reason, and the absence of downstream `PLANNED`/`ACCEPTED` progression.

### Project Structure Notes

- Keep this story in the existing validation/handover pipeline. Do not create a separate solvability engine or duplicate the benchmark geometry rules in controller and worker code.
- Prefer deterministic boundary text and enum-backed refusal reasons over generic "invalid benchmark" messages.
- This story should make impossible or unsupported benchmark definitions stop early, while preserving the current benchmark geometry, fixture, and attachment contracts from Stories 1.1 and 1.2.
- Keep the benchmark planner and reviewer guidance aligned with the actual schema fields and the existing `submit_plan()` gate behavior.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 1.3: Solvability Validation and Fail-Closed Rejection]
- [Source: \_bmad-output/planning-artifacts/prd.md, Functional Requirements FR1-FR6 and failure/robustness requirements]
- [Source: specs/architecture/agents/roles.md, Benchmark generator reviewer responsibilities and fail-closed handoff behavior]
- [Source: specs/architecture/handover-contracts.md, planner/reviewer handoff contracts and reviewer routing]
- [Source: specs/architecture/simulation-and-dod.md, objective AABB rules, build-zone rules, runtime-jitter containment, and benchmark fixture motion contract]
- [Source: specs/architecture/evals-and-gates.md, fail-closed terminalization and seeded preflight contract]
- [Source: specs/architecture/CAD-and-other-infra.md, strict schema, benchmark-owned fixture metadata, and read-only benchmark context]
- [Source: specs/architecture/observability.md, validation/rejection event and terminal-reason logging contract]
- \[Source: shared/enums.py, `BenchmarkRefusalReason`, `TerminalReason`, `FailureClass`, and `SessionStatus`\]
- \[Source: shared/models/schemas.py, `BenchmarkDefinition`, `ObjectivesSection`, `MovedObject`, `Constraints`, `AssemblyDefinition`, and `ReviewResult`\]
- \[Source: worker_heavy/utils/validation.py, `_validate_benchmark_definition_consistency`, `validate_environment_attachment_contract`, and cross-contract checks\]
- [Source: worker_heavy/utils/file_validation.py, benchmark-definition parsing and planner-handoff validation entry points]
- [Source: controller/agent/benchmark_handover_validation.py, benchmark planner handoff propagation]
- [Source: controller/agent/review_handover.py, benchmark plan-review entry validation]
- [Source: controller/agent/node_entry_validation.py, benchmark node-entry guards and custom checks]
- [Source: controller/agent/benchmark/graph.py, planner-router rejection flow and terminal status mapping]
- [Source: tests/integration/architecture_p0/test_int_008_objectives_validation.py, objective validation integration coverage]
- [Source: tests/integration/architecture_p0/test_planner_gates.py, benchmark planner submit_plan trace and handoff gating]
- [Source: tests/integration/architecture_p1/test_benchmark_workflow.py, benchmark workflow artifact and manifest checks]

## Dev Agent Record

### Agent Model Used

GPT-5

### Debug Log References

- 2026-03-23T00:07:12Z: Tightened benchmark solvability validation in `worker_heavy/utils/validation.py` so contradictory geometry, build-zone envelope violations, runtime-envelope collisions, and related impossible setups fail closed with explicit refusal reasons.
- 2026-03-23T00:07:12Z: Added benchmark motion-contract validation in `worker_heavy/utils/file_validation.py` and propagated benchmark reviewer handoff checks through `controller/agent/review_handover.py` and `controller/agent/benchmark_handover_validation.py`.
- 2026-03-23T00:07:12Z: Fixed controller persistence by falling back to `review_feedback` in `controller/api/tasks.py`, which preserved rejection metadata, and ensured the accepted benchmark bundle includes `script.py`.
- 2026-03-23T00:07:12Z: Verified the rejection path with focused integration slices for objective validation, planner gates, and benchmark workflow rejection.

### Completion Notes List

- Implemented fail-closed solvability validation for benchmark geometry and runtime envelopes, with explicit boundary naming for invalid, contradictory, and unsolvable benchmark setups.
- Rejected unsupported benchmark-owned motion unless reviewer-visible bounds, axes, and controller facts are present, and blocked downstream benchmark reviewer/coder entry on handoff validation failure.
- Persisted rejection metadata through the controller path and kept the approved benchmark bundle complete by copying `script.py` into the artifact bundle.
- Added and updated integration coverage for negative objective cases, planner gate rejections, and end-to-end benchmark workflow rejection.
- Verified with integration slices:
  - `tests/integration/architecture_p0/test_int_008_objectives_validation.py -k "test_int_008_"`
  - `tests/integration/architecture_p0/test_planner_gates.py -k "test_int_113 or test_int_200 or test_int_201"`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py -k "test_int_200 or test_int_202"`

### File List

- `_bmad-output/implementation-artifacts/1-3-solvability-validation-and-fail-closed-rejection.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
- `controller/agent/benchmark/graph.py`
- `controller/agent/benchmark/nodes.py`
- `controller/agent/benchmark_handover_validation.py`
- `controller/agent/graph.py`
- `controller/agent/node_entry_validation.py`
- `controller/agent/nodes/electronics_planner.py`
- `controller/agent/nodes/planner.py`
- `controller/agent/review_handover.py`
- `controller/api/tasks.py`
- `shared/assets/template_repos/benchmark_generator/benchmark_assembly_definition.yaml`
- `shared/assets/template_repos/benchmark_generator/benchmark_definition.yaml`
- `shared/assets/template_repos/benchmark_generator/plan.md`
- `shared/assets/template_repos/benchmark_generator/todo.md`
- `tests/integration/architecture_p0/test_int_008_objectives_validation.py`
- `tests/integration/architecture_p0/test_planner_gates.py`
- `tests/integration/architecture_p1/test_benchmark_workflow.py`
- `tests/integration/mock_responses/INT-005.yaml`
- `tests/integration/mock_responses/INT-113.yaml`
- `tests/integration/mock_responses/INT-005/benchmark_coder/entry_01/01__script.py`
- `tests/integration/mock_responses/INT-005/benchmark_planner/entry_01/01__plan.md`
- `tests/integration/mock_responses/INT-005/benchmark_planner/entry_01/02__todo.md`
- `tests/integration/mock_responses/INT-005/benchmark_planner/entry_01/03__benchmark_assembly_definition.yaml`
- `tests/integration/mock_responses/INT-005/benchmark_planner/entry_01/04__benchmark_definition.yaml`
- `tests/integration/mock_responses/INT-113/benchmark_coder/entry_01/01__script.py`
- `tests/integration/mock_responses/INT-113/benchmark_planner/entry_01/01__plan.md`
- `tests/integration/mock_responses/INT-113/benchmark_planner/entry_01/02__todo.md`
- `tests/integration/mock_responses/INT-113/benchmark_planner/entry_01/03__benchmark_assembly_definition.yaml`
- `tests/integration/mock_responses/INT-113/benchmark_planner/entry_01/04__benchmark_definition.yaml`
- `worker_heavy/utils/file_validation.py`
- `worker_heavy/utils/validation.py`

### Change Log

- 2026-03-23: Implemented solvability validation and fail-closed rejection for benchmark planning, including geometry checks, benchmark motion-contract enforcement, controller persistence of rejection metadata, template updates, and integration coverage.

### Status

review
