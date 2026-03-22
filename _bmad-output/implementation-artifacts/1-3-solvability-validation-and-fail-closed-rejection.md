# Story 1.3: Solvability Validation and Fail-Closed Rejection

Status: ready-for-dev

## Story

As a human operator, I want the system to validate solvability and reject ambiguous, impossible, or unsupported benchmark setups so that only benchmark candidates that can actually enter the solution workflow progress past planning.

## Acceptance Criteria

1. Given benchmark objective geometry that is contradictory or obstructed, when validation runs, then the benchmark is rejected with a deterministic reason and never transitions to `PLANNED` or `ACCEPTED`.
1. Given a moved object plus static randomization and runtime jitter that leaves the `build_zone` or intersects a `forbid_zone`, when validation runs, then the system fails closed and names the violated boundary in the error message.
1. Given benchmark-owned moving fixtures or motion metadata that is underconstrained, unsupported, or missing reviewer-visible bounds, axes, or controller facts, when the benchmark plan reviewer inspects the handoff, then the reviewer rejects it with an explicit solvability reason rather than inferring motion.
1. Given a benchmark candidate that is schema-valid but logically unsolvable, when planner handoff validation or reviewer entry validation runs, then the controller records a machine-readable rejection reason and failure class and blocks downstream coder/reviewer entry.
1. Given a rejected benchmark candidate, when the workflow is queried later, then the persisted traces, review artifacts, and validation logs show the exact reason code, failing boundary, and stage that rejected the benchmark.

## Tasks / Subtasks

- [ ] Tighten solvability checks in `worker_heavy/utils/validation.py::_validate_benchmark_definition_consistency` and related helpers so rejection reasons are explicit for goal/forbid intersections, build-zone envelope violations, runtime-envelope collisions, and other impossible geometry conditions.
  - [ ] Reuse the existing benchmark geometry and randomization models; do not add a parallel validator.
  - [ ] Use the existing benchmark refusal vocabulary (`INVALID_OBJECTIVES`, `CONTRADICTORY_CONSTRAINTS`, `UNSOLVABLE_SCENARIO`, `AMBIGUOUS_TASK`) and fail closed instead of auto-correcting.
- [ ] Propagate solvability failures through `worker_heavy/utils/file_validation.py`, `controller/agent/benchmark_handover_validation.py`, and `controller/agent/review_handover.py` so benchmark plan review and benchmark coder entry both stop before downstream solution work.
  - [ ] Keep the benchmark plan-review manifest and latest-revision checks strict; do not fall back to stale or inferred approval.
  - [ ] Ensure benchmark-side motion metadata in `benchmark_assembly_definition.yaml` is rejected when it cannot be explained with reviewer-visible motion facts.
- [ ] Update benchmark planner / reviewer guidance in `controller/agent/benchmark/nodes.py` and, if needed, the benchmark starter template so the LLM is instructed to reject impossible or unsupported benchmark setups instead of brute-forcing `submit_plan()`.
  - [ ] Keep the guidance aligned with `specs/architecture/simulation-and-dod.md` and the existing fail-closed planner handoff contract.
- [ ] Extend integration coverage in `tests/integration/architecture_p0/test_int_008_objectives_validation.py` and `tests/integration/architecture_p0/test_planner_gates.py` for negative solvability cases.
  - [ ] Add explicit rejection cases for obstructed goals, build-zone violations, runtime-envelope collisions, and unsupported benchmark motion.
  - [ ] Assert that the error text names the failing boundary and that controller traces record the rejection.
- [ ] Add an end-to-end rejection test in `tests/integration/architecture_p1/test_benchmark_workflow.py` for a benchmark prompt that would otherwise enter the workflow but must now be stopped by solvability validation.
  - [ ] Add a deterministic mock-response scenario under `tests/integration/mock_responses/` if the workflow test needs a seeded transcript to reach the rejection path.
- [ ] Run the integration slices that cover benchmark-definition validation, planner gates, and benchmark workflow before marking the story complete.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 1, Story 1.3 is the source of truth for the human-facing requirement.
  - The system must fail closed before an unsolvable benchmark reaches the solution workflow; silent correction, inferred success, or partial acceptance are regressions.
  - Benchmark objectives are AABBs, touching a goal/forbid zone counts as failure, and runtime jitter must be considered when validating the full moved-object envelope.
  - Benchmark-owned moving fixtures are benchmark-only exceptions, but they still must not be underconstrained, contradictory, or impossible to reason about from the handoff artifacts.
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

TBD

### Debug Log References

### Completion Notes List

### File List
