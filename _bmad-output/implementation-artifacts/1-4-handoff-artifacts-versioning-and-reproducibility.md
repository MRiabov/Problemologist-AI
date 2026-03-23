# Story 1.4: Handoff Artifacts, Versioning, and Reproducibility

Status: done

## Story

As a human operator, I want validated benchmark artifacts and preview evidence persisted so that downstream solution workflows can consume a stable, inspectable benchmark package, and I want the benchmark package to be reproducible later from persisted hashes and revision metadata when I inspect the rendered CAD model and simulation preview again.

## Acceptance Criteria

1. Given a valid benchmark, when it is submitted, then the required handoff artifacts, revision metadata, and manifest are persisted for the latest revision only, together with preview artifacts for the rendered CAD model and simulation preview.
1. Given render evidence exists, when the benchmark is reviewed, then the latest-revision preview artifacts, including the rendered CAD model and simulation preview, are available for inspection and traceable in the episode record.
1. Given the persisted benchmark package, when it is reloaded later, then the revision identifier and artifact hashes make the experiment reproducible without relying on unstored runtime state.
1. Given stale or superseded preview/review artifacts exist, when the benchmark is reloaded or reviewed, then only the latest-revision bundle is considered part of the handoff.

## Tasks / Subtasks

- [x] Verify the benchmark review handoff persists and surfaces the latest-revision bundle end to end.
  - [x] Confirm `ReviewManifest.revision` is populated from the current workspace git revision, not from stale or inferred state.
  - [x] Keep the benchmark reviewer stage working only from the latest revision's `script.py`, `validation_results.json`, `simulation_result.json`, and `renders/*` bundle.
- [x] Ensure render evidence is persisted as a reproducible package.
  - [x] Keep `renders/render_manifest.json` synchronized with the preview image bundle and include it in episode asset discovery.
  - [x] Preserve the review manifest's `renders` list so each render path can be traced back to the current revision.
- [x] Keep the benchmark handoff replayable from stored artifacts alone.
  - [x] Make sure the benchmark handoff artifacts surfaced through the episode record include the versioning/hash data needed to reload the same revision later.
  - [x] Reject stale or cross-revision artifacts instead of falling back to older renders or manifests.
- [x] Extend integration coverage for latest-revision persistence and replayability.
  - [x] Add or refresh benchmark workflow/handoff assertions that verify the latest revision's manifest, render manifest, and render files are present in the episode assets.
  - [x] Add a stale-artifact regression test that proves older preview or manifest files do not satisfy the handoff.
  - [x] Assert the episode record exposes the manifest and asset paths needed to reconstruct the benchmark package later.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 1, Story 1.4 is the source of truth for benchmark handoff persistence and replayability.
  - Keep the benchmark package latest-revision-only: stale renders, stale manifests, or cross-revision artifacts must not be treated as the active handoff.
  - The current versioning anchor is the existing git revision stored on `ReviewManifest.revision`; do not invent a second ad hoc "environment version" field for this story.
  - `ReviewManifest.renders` is part of the reproducibility contract and should remain aligned with the actual latest render bundle.
  - `renders/render_manifest.json` is the structured companion to the render images and should stay discoverable through the episode asset surface.
  - `inspect_media(...)` is still the required visual-evidence path when render images exist; listing files is not visual inspection.
  - Do not introduce a new immutable run/release manifest family in this story; reuse the existing benchmark review manifest, render manifest, and artifact hashes as the reproducibility anchor.
- Source tree components to touch:
  - `worker_heavy/utils/handover.py`
  - `controller/agent/review_handover.py`
  - `controller/agent/benchmark/graph.py`
  - `controller/agent/benchmark/nodes.py`
  - `controller/clients/worker.py`
  - `controller/middleware/remote_fs.py`
  - `shared/workers/schema.py`
  - `tests/integration/architecture_p0/test_int_188_validation_preview.py`
  - `tests/integration/architecture_p1/test_handover.py`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py`
  - `tests/integration/architecture_p1/test_reviewer_evidence.py`
- Testing standards summary:
  - Use integration tests only; do not add unit-only coverage for this story.
  - Assert against HTTP responses, episode assets, manifests, render artifacts, and traceable revision/hash fields.
  - If render images exist for the latest revision, the reviewer path must still use `inspect_media(...)`; text-only file listing does not satisfy the review contract.

### Project Structure Notes

- Keep the benchmark handoff bundle in the existing worker/controller asset pipeline. Do not build a separate benchmark-packaging subsystem for this story.
- Preserve the current split between validation preview and simulation proof. `renders/render_manifest.json` and the review manifest are evidence surfaces, not a substitute for simulation correctness.
- If future work wants a true immutable run or release bundle, that belongs in a follow-on epic or architecture update. This story should close the reproducibility gap using the existing benchmark review manifest and render manifest.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 1.4: Handoff Artifacts, Versioning, and Reproducibility]
- [Source: specs/architecture/agents/handover-contracts.md, latest-revision manifest gate, render evidence contract, and benchmark-to-engineer handoff rules]
- \[Source: specs/architecture/CAD-and-other-infra.md, `renders/render_manifest.json` and instance-aware segmentation legend requirements\]
- [Source: specs/architecture/simulation-and-dod.md, MuJoCo validation-preview default, Genesis proof split, and 24-view render policy]
- [Source: specs/architecture/observability.md, revision/hash/lineage and render-evidence observability fields]
- [Source: specs/integration-tests.md, INT-015, INT-032, INT-034, INT-039, INT-114, and INT-188]
- \[Source: shared/workers/schema.py, `ReviewManifest` and `RenderManifest` schemas\]
- [Source: worker_heavy/utils/handover.py, latest git revision, benchmark review manifest creation, and render-path persistence]
- [Source: controller/agent/review_handover.py, latest-revision hash and simulation gate validation]
- [Source: controller/agent/benchmark/graph.py, benchmark asset persistence and render/review artifact syncing]
- [Source: tests/integration/architecture_p0/test_int_188_validation_preview.py, render manifest and preview artifact contract]
- [Source: tests/integration/architecture_p1/test_handover.py, benchmark-to-engineer handoff package expectations]
- [Source: tests/integration/architecture_p1/test_benchmark_workflow.py, benchmark workflow and review-manifest assertions]
- [Source: tests/integration/architecture_p1/test_reviewer_evidence.py, reviewer evidence completeness and media-inspection requirements]

## Dev Agent Record

### Agent Model Used

GPT-5

### Debug Log References

- `2026-03-23T00:57:57Z`: Focused rerun of `tests/integration/architecture_p1/test_benchmark_workflow.py::test_benchmark_planner_cad_reviewer_path` passed with the latest `ReviewManifest.revision` and `renders/render_manifest.json` persisted.
- `2026-03-23T01:01:12Z`: First handover rerun failed on a test assertion bug, comparing a string manifest session ID to a UUID and referencing a nonexistent `EpisodeStatus.REJECTED` member.
- `2026-03-23T01:02:52Z`: Final rerun of `tests/integration/architecture_p1/test_handover.py::test_benchmark_to_engineer_handoff` passed after normalizing the session ID and keeping confirm single-shot.

### Completion Notes List

- Added a repo-root git revision helper and used it for latest-revision review manifests so the benchmark handoff is tied to the repository HEAD, not an empty session repo.
- Validated `renders/render_manifest.json` against the static preview bundle and kept benchmark-reviewer manifest fields benchmark-stage appropriate.
- Refreshed integration assertions so the benchmark workflow and handoff tests now check manifest revision, session linkage, and render-manifest traceability.
- Reused the existing stale-manifest regression coverage in INT-188 rather than introducing a second stale-artifact path.

### File List

- `_bmad-output/implementation-artifacts/1-4-handoff-artifacts-versioning-and-reproducibility.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
- `shared/git_utils.py`
- `worker_heavy/utils/handover.py`
- `controller/agent/review_handover.py`
- `controller/api/tasks.py`
- `controller/agent/node_entry_validation.py`
- `tests/integration/agent/helpers.py`
- `tests/integration/architecture_p1/test_benchmark_workflow.py`
- `tests/integration/architecture_p1/test_handover.py`

### Change Log

- 2026-03-23: Enforced latest-revision review-manifest validation, synchronized render manifests with the preview bundle, scoped benchmark-reviewer manifests to benchmark-stage evidence, and refreshed integration coverage for reproducible handoff artifacts.

### Status

review
