# Story 4.1: Persist Immutable Run and Release Bundles

Status: ready-for-dev

## Story

As a human operator, I want validated benchmark artifacts, preview evidence, and immutable run or release manifests persisted so that downstream solution workflows can consume a stable, inspectable benchmark package and I have no questions about reproducibility of the experiment when I revisit the same episode later.

## Acceptance Criteria

1. Given a valid benchmark or official evaluation run, when it is persisted, then the system stores the required manifest bundle, environment version, and revision identifiers for the latest revision only.
1. Given the persisted bundle, when I inspect it later, then I can trace the benchmark revision, solution revision, environment version, joinable session and episode identifiers, and preview evidence from the same bundle without relying on undocumented runtime state.
1. Given render evidence exists, when the benchmark or run is archived, then the latest-revision preview artifacts remain linked to the bundle and remain inspectable.
1. Given stale or superseded run/release artifacts exist, when the bundle is resolved, then only the latest-revision bundle is considered active and older artifacts are rejected or ignored.
1. Given a persisted benchmark or run bundle, when downstream export or replay logic reads it, then the bundle provides the metadata needed to reconstruct the same episode lineage later.

## Tasks / Subtasks

- [ ] Verify the run/release persistence contract keeps the latest revision only.
  - [ ] Confirm the manifest bundle includes benchmark revision, solution revision, environment version, episode/session identifiers, and preview evidence paths.
  - [ ] Confirm stale or cross-revision artifacts are not treated as active bundle inputs.
- [ ] Ensure persisted bundle metadata is sufficient for later replay or export.
  - [ ] Keep the persisted identifiers joinable to the source episode and session records.
  - [ ] Preserve render/preview evidence links when media exists so the bundle remains inspectable later.
- [ ] Extend integration coverage for bundle persistence and latest-revision resolution.
  - [ ] Add or refresh workflow assertions that validate the active bundle metadata and preview evidence are present in persisted artifacts.
  - [ ] Add a regression test that proves stale bundle material does not satisfy latest-revision resolution.
  - [ ] Assert the episode record exposes the metadata needed to reconstruct the benchmark package and later dataset row.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 4, Story 4.1 is the source of truth for immutable run/release persistence and reproducibility.
  - Story 4.1 is a persistence and traceability story, not a new simulation or CAD validation story.
  - Keep the bundle latest-revision-only: stale manifests, stale previews, or cross-revision artifacts must not be treated as active handoff data.
  - If render evidence exists for the latest revision, the bundle must keep those preview artifacts linked and inspectable; text-only references to render paths are not sufficient by themselves.
  - The persisted metadata must remain joinable to the source session and episode records so downstream export/replay can reconstruct lineage without undocumented state.
  - Do not invent a second immutable bundle family if the existing review/manifest artifacts already satisfy the reproducibility contract.
- Source tree components to touch:
  - `worker_heavy/utils/handover.py`
  - `worker_heavy/utils/rendering.py`
  - `controller/agent/review_handover.py`
  - `controller/agent/benchmark/graph.py`
  - `controller/middleware/remote_fs.py`
  - `controller/persistence/models.py`
  - `shared/workers/schema.py`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py`
  - `tests/integration/architecture_p1/test_handover.py`
  - `tests/integration/architecture_p1/test_reviewer_evidence.py`
  - `tests/integration/architecture_p1/test_engineering_loop.py`
- Testing standards summary:
  - Use integration tests only; do not add unit-test-only coverage for this story.
  - Assert against persisted manifests, episode/session identifiers, render artifacts, and latest-revision resolution behavior.
  - If render images exist for the latest revision, the review path must still use `inspect_media(...)`; file listing is not visual inspection.

### Project Structure Notes

- Keep the bundle in the existing worker/controller asset pipeline. Do not build a separate release-packaging subsystem for this story.
- Preserve the current split between benchmark-side persistence and downstream replay/export readers. The persisted bundle is evidence and traceability metadata, not a substitute for episode simulation or review correctness.
- If later work needs a stronger dataset-export contract, that belongs in Story 4.2 or a follow-on change; this story should close the reproducibility gap using the existing manifest and artifact storage surfaces.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 4.1: Persist Immutable Run and Release Bundles]
- [Source: specs/architecture/agents/handover-contracts.md, latest-revision manifest gate, render evidence contract, and benchmark-to-engineer handoff rules]
- [Source: specs/architecture/observability.md, revision/hash/lineage and render-evidence observability fields]
- [Source: specs/architecture/evals-and-gates.md, latest-revision-only persistence and fail-closed artifact validation expectations]
- [Source: specs/architecture/CAD-and-other-infra.md, render and preview evidence assumptions]
- [Source: specs/integration-tests.md, INT-015, INT-032, INT-034, INT-039, INT-114, and INT-188]
- \[Source: shared/workers/schema.py, `ReviewManifest`, `RenderManifest`, and lineage-bearing schema fields\]
- [Source: worker_heavy/utils/handover.py, latest git revision, benchmark review manifest creation, and render-path persistence]
- [Source: worker_heavy/utils/rendering.py, render manifest generation and preview artifact persistence]
- [Source: controller/agent/review_handover.py, latest-revision hash and simulation gate validation]
- [Source: controller/agent/benchmark/graph.py, benchmark asset persistence and render/review artifact syncing]
- [Source: controller/middleware/remote_fs.py, persisted asset discovery and manifest loading]
- [Source: controller/persistence/models.py, episode/session metadata persistence]
- [Source: tests/integration/architecture_p1/test_benchmark_workflow.py, benchmark workflow and review-manifest assertions]
- [Source: tests/integration/architecture_p1/test_handover.py, benchmark-to-engineer handoff package expectations]
- [Source: tests/integration/architecture_p1/test_reviewer_evidence.py, reviewer evidence completeness and media-inspection requirements]
- [Source: tests/integration/architecture_p1/test_engineering_loop.py, persisted run metadata and replay-adjacent workflow assertions]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
