# Story 2.2: Inspect Solution Evidence and Terminal Outcome

Status: done

## Story

As a human engineer, I want to inspect rendered CAD previews, simulation previews, render evidence, and terminal reasons so that I know why a candidate solution passed or failed.

## Acceptance Criteria

1. Given a completed engineer episode, when I open the run, then the workspace exposes the latest validation or simulation evidence and the terminal outcome metadata from `EpisodeMetadata` (`detailed_status`, `terminal_reason`, `failure_class`).
1. Given render or video evidence exists, when I inspect the run, then the latest-revision media is visible, traceable to the exact asset path, and the viewport opens the relevant evidence rather than the planner draft.
1. Given preview artifacts exist, when I inspect the run, then I can view the rendered CAD model and simulation preview for the latest engineer revision without manually hunting through raw files.
1. Given the attempt failed, when I inspect the outcome, then failure classification is explicit and not inferred from generic `FAILED` status or `validation_logs` alone.
1. Given evidence is missing, stale, or unavailable, when I inspect the run, then the UI shows an explicit missing-artifact state instead of synthesizing success.

## Tasks / Subtasks

- [x] Update solution-run status rendering in `frontend/src/components/workspace/ChatWindow.tsx` so the failed-run banner shows structured terminal metadata from `selectedEpisode.metadata_vars` and uses `validation_logs` only as supplemental detail.
  - [x] Surface `terminal_reason`, `failure_class`, and `detailed_status` in a compact summary block.
  - [x] Keep the existing free-form failure text as fallback context only when the structured fields are absent.
- [x] Update `frontend/src/components/workspace/ArtifactView.tsx` so engineer episodes default to the latest solution evidence artifact instead of always preferring `plan.md`.
  - [x] Keep benchmark-generation sessions plan-first.
  - [x] For engineer runs, prefer `simulation_result.json` first, then `validation_results.json`, then raw render/video evidence, then fallback to plan/file browsing if no solution evidence exists.
  - [x] Keep raw file browsing available for `validation_results.json`, `simulation_result.json`, and render assets.
- [x] Add stable, testable terminal/evidence hooks in `frontend/src/components/workspace/UnifiedGeneratorView.tsx` and any supporting view component needed for the artifact pane.
  - [x] Extend the hidden debug payload with the episode terminal metadata used by the Playwright tests.
  - [x] Add a deterministic test hook for the selected solution evidence artifact so the UI does not need brittle DOM scraping.
- [x] Keep `frontend/src/components/visualization/DesignViewer.tsx` and `frontend/src/components/visualization/SimulationResults.tsx` aligned with the new default evidence selection so video, 3D preview, and summary data remain discoverable without extra clicks.
  - [x] Do not replace the existing video/3D/heatmap modes; only change what is selected first for engineer completion/failure states.
- [x] Extend `tests/integration/architecture_p1/test_engineering_loop.py` to assert the episode metadata exposes `terminal_reason`/`failure_class` after a real solve attempt and that the expected evidence assets are present.
  - [x] Keep asserting `validation_results.json`, `simulation_result.json`, and reviewer manifest presence as the evidence bundle.
  - [x] Assert the final metadata is explicit rather than inferred from status alone.
- [x] Add or extend a Playwright integration test under `tests/integration/frontend/p0/` to prove the finished engineer run opens the solution evidence and terminal summary by default.
  - [x] Assert the terminal summary is visible in the workspace.
  - [x] Assert the artifact pane lands on the solution-evidence file, not `plan.md`, for engineer runs.
- [x] If the failure path cannot be driven reliably from the live stack, add a deterministic `tests/integration/mock_responses/INT-2xx.yaml` scenario instead of weakening the assertion or using a fake unit test.
- [x] Keep the story fail-closed: do not infer a pass/fail explanation from status alone; always derive it from persisted metadata and the latest evidence assets.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 2 is the human solution workflow. Story 2.1 already established benchmark-to-engineer handoff and read-only benchmark context; this story must not mutate benchmark-owned assets.
  - `EpisodeMetadata` already carries `detailed_status`, `terminal_reason`, `failure_class`, and `validation_logs`. The frontend should render those fields directly instead of inventing a second terminal-state contract.
  - `controller/api/routes/episodes.py` already returns `EpisodeResponse.metadata_vars`, `assets`, `traces`, and `last_trace_id`; prefer those existing fields over a new endpoint.
  - `shared/workers/schema.py::ValidationResultRecord` and `shared/models/simulation.py::SimulationResult` are the persisted evidence artifacts that correspond to validation and simulation outcomes.
  - `frontend/src/components/workspace/ArtifactView.tsx` currently defaults to `plan.md`; that behavior is correct for planning states but wrong for completed/failed solution inspection.
  - `frontend/src/components/visualization/DesignViewer.tsx` already defaults to video when a video asset is present; preserve that behavior and feed it the right asset choice from the artifact pane.
  - `frontend/src/components/workspace/ChatWindow.tsx` currently treats `validation_logs` as the only failure narrative; keep them as fallback context, not the primary terminal explanation.
  - `frontend/src/components/workspace/UnifiedGeneratorView.tsx` exposes a stable hidden debug payload used by integration tests; extend it rather than inventing brittle DOM scraping.
  - `selectedEpisode.metadata_vars?.episode_type` distinguishes benchmark from engineer flows. Use it so benchmark sessions remain plan-first while engineer sessions become evidence-first after completion/failure.
  - If render images exist, the required reviewer/vision roles still must use `inspect_media(...)`; this story does not replace media inspection with text-only summaries.
  - Story 2.3 owns revision/retry behavior. Story 2.4 owns peer-solution stability review under jitter. Keep this story focused on evidence and terminal outcome surfaces.
- Source tree components to touch:
  - `frontend/src/components/workspace/ChatWindow.tsx`
  - `frontend/src/components/workspace/ArtifactView.tsx`
  - `frontend/src/components/workspace/UnifiedGeneratorView.tsx`
  - `frontend/src/components/visualization/DesignViewer.tsx`
  - `frontend/src/components/visualization/SimulationResults.tsx`
  - `tests/integration/architecture_p1/test_engineering_loop.py`
  - `tests/integration/frontend/p0/test_solution_evidence.py` or an equivalent new Playwright integration test file
  - `tests/integration/mock_responses/INT-2xx.yaml` only if a deterministic failure path is required
- Testing standards summary:
  - Use integration tests only; do not add unit-test-only coverage for this story.
  - Assert against HTTP responses, episode assets, terminal metadata, render/video artifacts, and traceable review/evidence events.
  - Prefer the live engineer workflow for the success path. Use a deterministic mock response only if the failure path cannot be reproduced reliably through the real stack.
  - Keep assertions on the episode payload and visible UI state rather than on internal helper calls.

### Project Structure Notes

- Keep this as a frontend-inspection story, not a new evidence backend.
- Do not add a second evidence store or a new terminal-outcome API if the existing episode payload already carries the needed metadata.
- Preserve benchmark-plan behavior for benchmark episodes. The evidence-first default applies only to engineer solution episodes.
- If you need a stable selector for the new summary state, add a dedicated `data-testid` to the terminal-summary block and the active artifact header instead of relying on broad text matching.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 2.2: Inspect Solution Evidence and Terminal Outcome]
- [Source: \_bmad-output/planning-artifacts/prd.md, Journey 1 and Journey 2, plus the MVP scope for the solution workflow]
- [Source: specs/architecture/agents/overview.md, engineering workflow split and visual-inspection policy]
- [Source: specs/architecture/agents/roles.md, Engineering Execution Reviewer responsibilities and evidence requirements]
- [Source: specs/architecture/agents/handover-contracts.md, latest-revision evidence, reviewer manifest, and benchmark-to-engineer handoff rules]
- \[Source: specs/architecture/agents/artifacts-and-filesystem.md, file ownership, artifact surfaces, and `inspect_media(...)` contract\]
- [Source: specs/architecture/observability.md, episode metadata fields, terminal reasons, and evidence observability]
- [Source: specs/architecture/evals-and-gates.md, solution workflow gates and visual-inspection requirements]
- [Source: docs/backend-reference.md, episode payload, artifact surfaces, and frontend inspection surface]
- \[Source: docs/component-inventory.md, `ArtifactView`, `ChatWindow`, `UnifiedGeneratorView`, and `DesignViewer` responsibilities\]
- [Source: specs/integration-tests.md, INT-033, INT-034, INT-039, INT-058, and INT-188]
- [Source: controller/api/routes/episodes.py, episode detail payload and asset proxy endpoint]
- \[Source: shared/models/schemas.py, `EpisodeMetadata` fields for `detailed_status`, `terminal_reason`, `failure_class`, and `validation_logs`\]
- \[Source: shared/workers/schema.py, `ValidationResultRecord` and `ReviewManifest` evidence fields\]
- \[Source: shared/models/simulation.py, `SimulationResult` summary and failure payload\]
- [Source: frontend/src/components/workspace/ChatWindow.tsx, existing failure banner and validation-log fallback]
- [Source: frontend/src/components/workspace/ArtifactView.tsx, current plan-first default selection and raw artifact rendering]
- [Source: frontend/src/components/workspace/UnifiedGeneratorView.tsx, hidden debug payload used by tests]
- [Source: frontend/src/components/visualization/DesignViewer.tsx, video/3D/heatmap evidence display]
- [Source: tests/integration/architecture_p1/test_engineering_loop.py, current engineering flow evidence assertions]
- [Source: tests/integration/architecture_p1/test_reviewer_evidence.py, reviewer evidence and media-inspection assertions]
- [Source: tests/integration/frontend/p0/test_frontend_p0.py, shared workspace integration patterns]

## Dev Agent Record

### Agent Model Used

GPT-5.4

### Debug Log References

- Frontend integration validation: `tests/integration/frontend/p0/test_solution_evidence.py::test_int_181_engineer_run_defaults_to_solution_evidence`
- Backend integration validation: `tests/integration/architecture_p1/test_engineering_loop.py::test_engineering_full_loop`
- Successful frontend run archived under `logs/integration_tests/runs/run_20260323_071305/`
- Successful backend run archived under `logs/integration_tests/runs/run_20260323_071818/`

### Completion Notes List

- Engineer episodes now persist explicit terminal metadata on completion/failure, including `detailed_status`, `terminal_reason`, and `failure_class`.
- Artifact selection now defaults engineer runs to solution evidence first, with benchmark runs remaining plan-first.
- Simulation evidence now renders a summary overview when only summary/render-path metadata is available, and the UI exposes stable test hooks for the selected artifact and terminal summary.
- Verified with live integration tests covering both the frontend workspace and the backend engineering loop.

### File List

- \_bmad-output/implementation-artifacts/2-2-inspect-solution-evidence-and-terminal-outcome.md
- controller/api/tasks.py
- frontend/src/components/workspace/artifactSelection.ts
- frontend/src/components/workspace/ArtifactView.tsx
- frontend/src/components/workspace/ChatWindow.tsx
- frontend/src/components/workspace/UnifiedGeneratorView.tsx
- frontend/src/components/visualization/DesignViewer.tsx
- frontend/src/components/visualization/SimulationResults.tsx
- tests/integration/architecture_p1/test_engineering_loop.py
- tests/integration/frontend/p0/test_solution_evidence.py

## Change Log

- 2026-03-23: Implemented evidence-first solution inspection for engineer runs, surfaced explicit terminal metadata in the workspace, and added deterministic integration coverage for default evidence selection and terminal outcome rendering.
