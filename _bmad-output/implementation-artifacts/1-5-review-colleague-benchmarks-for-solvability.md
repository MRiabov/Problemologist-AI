# Story 1.5: Review Colleague Benchmarks for Solvability

Status: ready-for-dev

## Story

As a human operator, I want to review a colleague's benchmark for solvability by inspecting the rendered CAD model and simulation preview so that I can reject technically valid but logically unsolvable problems before they are handed to engineering.

## Acceptance Criteria

1. Given a benchmark that passes schema and geometry checks but is logically unsolvable, when the Benchmark Plan Reviewer reviews it, then the reviewer can reject it explicitly with a solvability reason and the benchmark does not advance to downstream solution work.
1. Given a benchmark that is solvable, when the Benchmark Plan Reviewer reviews it, then the reviewer can approve it for downstream solution work and the workflow transitions to `PLANNED`.
1. Given render evidence exists for the latest revision under review, when the Benchmark Plan Reviewer approves or rejects the benchmark, then the reviewer must inspect the current revision's rendered CAD model and simulation preview through `inspect_media(...)`, and file listing alone is not sufficient.
1. Given the review decision is persisted, when I inspect the episode later, then the latest-revision review manifest and stage-specific decision/comments YAML capture the solvability rationale and checklist, and the revision/hash linkage remains traceable without relying on stale or inferred state.

## Tasks / Subtasks

- [ ] Tighten the benchmark plan reviewer prompt and reasoning contract in `controller/agent/benchmark/nodes.py` so the reviewer explicitly evaluates solvability, not just object consistency.
  - [ ] Keep the prompt aligned with the existing benchmark reviewer handoff contract and with the benchmark fixture motion rules in `specs/architecture/simulation-and-dod.md`.
  - [ ] Make sure the reviewer can cite explicit failure boundaries such as obstructed goals, impossible geometry, unsupported motion, or underconstrained fixture behavior.
- [ ] Thread solvability evidence through the benchmark handoff validation path in `controller/agent/benchmark_handover_validation.py`, `controller/agent/review_handover.py`, `controller/agent/node_entry_validation.py`, and `controller/agent/benchmark/graph.py`.
  - [ ] Keep latest-revision manifest validation strict and fail closed on stale or missing plan-review artifacts.
  - [ ] Preserve the existing `ReviewDecision` routing and stage-specific review file names; do not introduce a new reviewer artifact family.
- [ ] Reuse the deterministic benchmark-definition validation source of truth in `worker_heavy/utils/validation.py` and `worker_heavy/utils/file_validation.py` so the reviewer can explain why a benchmark is unsolvable without duplicating geometry logic.
  - [ ] Keep the rejection vocabulary explicit and machine-readable rather than falling back to generic "invalid benchmark" messages.
  - [ ] Preserve benchmark-owned read-only context boundaries and the benchmark fixture motion exception from the architecture docs.
- [ ] Extend integration coverage in `tests/integration/architecture_p0/test_planner_gates.py`, `tests/integration/architecture_p0/test_int_008_objectives_validation.py`, `tests/integration/architecture_p1/test_benchmark_workflow.py`, `tests/integration/architecture_p1/test_handover.py`, and `tests/integration/architecture_p1/test_reviewer_evidence.py`.
  - [ ] Prove that a logically unsolvable benchmark is rejected explicitly by the benchmark plan reviewer even when schema validation passes.
  - [ ] Prove that a solvable benchmark can be approved and advanced to `PLANNED`.
  - [ ] Prove that render assets trigger `inspect_media(...)` before approval and that the evidence is tied to the latest revision.
- [ ] Run the integration slices covering benchmark-definition validation, benchmark planner/reviewer gating, reviewer evidence, and benchmark workflow before marking the story complete.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 1, Story 1.5 is the source of truth for the human-facing requirement.
  - This is the benchmark plan-review gate, not the later benchmark execution reviewer. The reviewer must decide whether the benchmark should proceed to coding and solution work.
  - Story 1.3 already established fail-closed solvability validation; reuse its boundary/reason conventions rather than inventing a second solver or a new refusal vocabulary.
  - Story 1.4 already established latest-revision-only review bundles and `inspect_media(...)` as mandatory visual inspection when render evidence exists; keep that contract intact here.
  - Benchmark-owned fixtures may be benchmark-only exceptions, but they still must be minimum-motion, review-visible, and defensible from the handoff artifacts.
  - `BenchmarkPlanReviewerNode` already persists stage-specific review files; solvability rejections must remain visible in `review_decision`, `review_comments`, and `review_feedback` outputs.
  - `validate_reviewer_handover` and `benchmark_plan_reviewer_handover_custom_check` are the fail-closed entry points for the reviewer stage; do not allow stale or inferred approval.
  - `ReviewDecisionEvent` and observability traces should carry the solvability rationale so the decision is reconstructable later.
- Source tree components to touch:
  - `controller/agent/benchmark/nodes.py`
  - `controller/agent/benchmark/graph.py`
  - `controller/agent/benchmark_handover_validation.py`
  - `controller/agent/review_handover.py`
  - `controller/agent/node_entry_validation.py`
  - `worker_heavy/utils/validation.py`
  - `worker_heavy/utils/file_validation.py`
  - `tests/integration/architecture_p0/test_planner_gates.py`
  - `tests/integration/architecture_p0/test_int_008_objectives_validation.py`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py`
  - `tests/integration/architecture_p1/test_handover.py`
  - `tests/integration/architecture_p1/test_reviewer_evidence.py`
- Testing standards summary:
  - Use integration tests only; do not add unit-only coverage.
  - Assert against HTTP responses, episode assets, review manifests, review YAML files, and traceable review/evidence events.
  - If render evidence exists, reviewer approval must be backed by `inspect_media(...)`; file listing or text summaries do not count.
  - Keep assertions specific: unsolvable benchmarks must show the explicit solvability reason and solvable ones must reach the approved downstream state.

### Project Structure Notes

- Keep benchmark solvability review inside the existing benchmark graph and worker validation layers. Do not introduce a separate solver or a separate benchmark-review artifact family.
- Preserve the stage-scoped `.manifests/benchmark_plan_review_manifest.json` and `reviews/benchmark-plan-review-*.yaml` pair; the planner-reviewer gate remains the source of truth for approval routing.
- If a solvability issue is already captured by the worker-side benchmark-definition validators, surface that same reason in the reviewer decision rather than re-deriving a parallel explanation.
- The reviewer is allowed to reject based on logical unsolvability even when schema validation passes; that distinction is the point of this story.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 1.5: Review Colleague Benchmarks for Solvability]
- [Source: \_bmad-output/implementation-artifacts/1-3-solvability-validation-and-fail-closed-rejection.md, previous solvability gate behavior and reason vocabulary]
- [Source: \_bmad-output/implementation-artifacts/1-4-handoff-artifacts-versioning-and-reproducibility.md, latest-revision manifest and render evidence contract]
- [Source: specs/architecture/agents/roles.md, benchmark plan reviewer responsibilities and visual-inspection policy]
- [Source: specs/architecture/agents/handover-contracts.md, benchmark planner/plan-reviewer handoff contract and strict reviewer persistence naming]
- \[Source: specs/architecture/agents/artifacts-and-filesystem.md, benchmark plan reviewer read/write scope and `.manifests` rules\]
- \[Source: specs/architecture/agents/tools.md, `inspect_media`, `submit_plan`, and `submit_review` contracts\]
- [Source: specs/architecture/simulation-and-dod.md, benchmark fixture motion exception, DOF minimality, and review-visible motion facts]
- [Source: specs/architecture/CAD-and-other-infra.md, benchmark-owned read-only context and render/preview contracts]
- [Source: specs/architecture/evals-and-gates.md, fail-closed terminalization and seeded preflight contract]
- [Source: specs/architecture/observability.md, review decision and media inspection event lineage]
- \[Source: controller/agent/benchmark/nodes.py, `BenchmarkPlanReviewerNode` and render-inspection gate\]
- [Source: controller/agent/benchmark/graph.py, benchmark plan-reviewer entry guard and review-decision routing]
- [Source: controller/agent/benchmark_handover_validation.py, benchmark planner handoff semantic checks]
- [Source: controller/agent/review_handover.py, latest-revision reviewer-manifest validation]
- [Source: controller/agent/node_entry_validation.py, benchmark plan reviewer entry guard]
- [Source: worker_heavy/utils/validation.py, benchmark objective consistency and solvability checks]
- [Source: worker_heavy/utils/file_validation.py, benchmark-definition parsing and cross-contract validation]
- [Source: tests/integration/architecture_p0/test_planner_gates.py, benchmark submit_plan and plan-review gating]
- [Source: tests/integration/architecture_p0/test_int_008_objectives_validation.py, benchmark-definition validation coverage]
- [Source: tests/integration/architecture_p1/test_benchmark_workflow.py, end-to-end benchmark handoff and review artifacts]
- [Source: tests/integration/architecture_p1/test_handover.py, benchmark-to-engineer handoff package expectations]
- [Source: tests/integration/architecture_p1/test_reviewer_evidence.py, review evidence and inspect_media requirement]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
