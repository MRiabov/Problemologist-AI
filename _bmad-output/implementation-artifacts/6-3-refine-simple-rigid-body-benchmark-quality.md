# Story 6.3: Refine Simple Rigid-Body Benchmark Quality

Status: ready-for-dev

## Story

As a human operator, I want benchmark generator output to improve based on reviewer feedback so that generated simple rigid-body benchmarks remain solvable, unambiguous, and suitable for engineering intake.

## Acceptance Criteria

1. Given reviewer feedback on a simple rigid-body benchmark, when the benchmark is regenerated, then the next revision preserves the solvable contract and resolves the flagged issue.
2. Given a technically valid but poor simple rigid-body benchmark, when review runs, then it can be rejected as unsuitable for engineering intake.
3. Given repeated simple rigid-body benchmark generations, when I inspect the run set, then rejected examples are preserved for debugging and dataset analysis.

## Tasks / Subtasks

- [ ] Tighten the benchmark planner/reviewer feedback loop so `required_fixes`, review decision, and latest-revision evidence are carried forward to the next regeneration attempt instead of being overwritten. (AC: 1, 2)
  - [ ] Update the benchmark planner/reviewer prompt blocks in `config/prompts.yaml` so a quality-only rejection is explicit, machine-readable, and still preserves the solvable-contract checks from Story 6.1.
  - [ ] Keep the review YAML pair and review trace content aligned with the `review_decision` event so the next revision can be explained from persisted artifacts alone.
- [ ] Preserve rejected benchmark attempts for debugging and dataset analysis. (AC: 3)
  - [ ] If the current benchmark pipeline drops or overwrites rejected attempts, fix the persistence/export path so rejected rows remain visible in the generated manifest/provenance output instead of being replaced by the later accepted revision.
  - [ ] Keep rejected-example provenance keyed by source episode and reason list, using the existing manifest style (`rejected`, `dropped_lineage`, `bucket_counts`, `counts`) instead of inventing a second taxonomy.
- [ ] Extend integration coverage for reject/retry quality-loop behavior. (AC: 1-3)
  - [ ] Refresh `tests/integration/mock_responses/INT-203.yaml` and add a separate quality-only technically valid but unsuitable benchmark fixture if needed, so the suite covers both unsolvable and technically valid but unsuitable simple rigid-body cases.
  - [ ] Update `tests/integration/architecture_p1/test_benchmark_workflow.py` and `tests/integration/architecture_p1/test_reviewer_evidence.py` to assert the rejection reason, required fixes, latest-revision evidence, and preserved retry lineage remain inspectable after regeneration.
  - [ ] Keep `INT-200`, `INT-201`, `INT-202`, and `INT-204` as regression anchors for hidden motion, underbounded motion, unsupported motion, and render-inspection gating.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Story 6.3 refines benchmark quality, not benchmark scope. Keep the simple rigid-body / gravity-only contract from Story 6.1 and the curated coverage from Story 6.2 unchanged.
  - Reviewers may reject a schema-valid benchmark if it is poor for engineering intake, but the rejection must be explicit and explainable. Keep `REJECT_PLAN` and `required_fixes` stable and machine-readable.
  - Preserve the benchmark review artifact contract:
    - `.manifests/benchmark_plan_review_manifest.json`
    - `reviews/benchmark-plan-review-decision-round-<n>.yaml`
    - `reviews/benchmark-plan-review-comments-round-<n>.yaml`
  - If a rejected benchmark is later corrected, both the rejected and accepted attempts must remain inspectable in traces and artifacts so a dataset analyst can reconstruct the evolution.
  - When renders exist, visual inspection through `inspect_media(...)` remains mandatory for the review roles; quality rejection does not waive render review.
  - If you need to record drops or rejected groups, use the existing provenance pattern from generated manifests such as `dataset/data/generated/component_seeded/v0.0.1/manifest.json` and `dataset/data/generated/workflow/v0.0.1/manifest.json`.
- Source tree components to touch:
  - `config/prompts.yaml`
  - `controller/agent/benchmark/nodes.py`
  - `controller/api/routes/datasets.py`
  - `controller/api/tasks.py`
  - `evals/logic/runner.py`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py`
  - `tests/integration/architecture_p1/test_reviewer_evidence.py`
  - `tests/integration/mock_responses/INT-203.yaml`
  - `tests/integration/mock_responses/INT-200.yaml`
  - `tests/integration/mock_responses/INT-201.yaml`
  - `tests/integration/mock_responses/INT-202.yaml`
  - `tests/integration/mock_responses/INT-204.yaml`
- Testing standards summary:
  - Use integration tests only; do not add unit-only coverage for this story.
  - Assert against HTTP responses, persisted review artifacts, review traces, manifests, and emitted events.
  - Keep failures deterministic and fail-closed.
  - Preserve earlier rejected attempts as inspectable evidence instead of replacing them with the later accepted revision.

### Previous Story Intelligence

- Story 6.1 established the simple rigid-body gravity-only family and the fail-closed benchmark submission/review path. Do not reopen that scope here.
- Story 6.2 established the curated validation-set coverage and the role-based seed/review bundles. Reuse that coverage instead of creating a new family or duplicate taxonomy.
- The existing benchmark generator fixtures already include passive gravity-only and negative motion cases (`INT-114`, `INT-200`, `INT-201`, `INT-202`, `INT-204`). Use them as regression anchors rather than inventing unrelated scenarios.

### Project Structure Notes

- Keep the benchmark-review loop on the existing planner/reviewer contract. Do not add a second benchmark review pipeline.
- Preserve the latest-revision policy on review artifacts. A regenerated benchmark may supersede the accepted state, but the rejected revision must still be discoverable from the persisted history.
- If current code already persists rejected examples, keep that record intact. If it discards them, fix the persistence layer rather than adding a sidecar cache.
- Keep any new quality-only rejection reason compatible with the current machine-readable review schema and the existing comments YAML shape.

### References

- \[Source: `_bmad-output/planning-artifacts/epics.md`, Epic 6: Gravity: Benchmarks, Story 6.3\]
- \[Source: `_bmad-output/implementation-artifacts/6-1-generate-simple-rigid-body-benchmarks.md`, Story 6.1 baseline scope and fail-closed benchmark submission path\]
- \[Source: `_bmad-output/implementation-artifacts/6-2-produce-a-representative-simple-rigid-body-validation-set.md`, Story 6.2 curated coverage and rejection-aware seed set\]
- \[Source: `specs/desired_architecture.md`, architecture index for agent and evaluation contracts\]
- \[Source: `specs/architecture/agents/overview.md`, benchmark generator workflow split\]
- \[Source: `specs/architecture/agents/roles.md`, benchmark plan reviewer responsibilities and output expectations\]
- \[Source: `specs/architecture/agents/handover-contracts.md`, strict handoff and reviewer-contract rules\]
- \[Source: `specs/architecture/agents/artifacts-and-filesystem.md`, read-only mounts, latest-revision artifacts, and filesystem contract\]
- \[Source: `specs/architecture/agents/tools.md`, planner/reviewer submission gates and validation utilities\]
- \[Source: `specs/architecture/evals-and-gates.md`, fail-closed gates and quality-tier expectations\]
- \[Source: `specs/architecture/observability.md`, lineage, review, and media-inspection events\]
- \[Source: `specs/integration-tests.md`, integration-only verification and artifact-based assertions\]
- \[Source: `config/prompts.yaml`, benchmark planner/reviewer prompt contracts and review output shape\]
- \[Source: `controller/agent/benchmark/nodes.py`, benchmark review state, persistence, and retry handling\]
- \[Source: `controller/api/routes/datasets.py`, dataset/export views that surface rejected/provenance records\]
- \[Source: `controller/api/tasks.py`, benchmark task persistence and latest-revision handoff coordination\]
- \[Source: `evals/logic/runner.py`, manifest generation and rejected-row lineage handling\]
- \[Source: `tests/integration/architecture_p1/test_benchmark_workflow.py`, benchmark review rejection and retry coverage\]
- \[Source: `tests/integration/architecture_p1/test_reviewer_evidence.py`, review evidence completeness and latest-revision inspection\]
- \[Source: `tests/integration/mock_responses/INT-203.yaml`, unsolvable benchmark rejection fixture\]
- \[Source: `tests/integration/mock_responses/INT-200.yaml`, hidden-motion rejection fixture\]
- \[Source: `tests/integration/mock_responses/INT-201.yaml`, underbounded-motion rejection fixture\]
- \[Source: `tests/integration/mock_responses/INT-202.yaml`, unsupported-motion rejection fixture\]
- \[Source: `tests/integration/mock_responses/INT-204.yaml`, latest-revision render-inspection gate fixture\]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
