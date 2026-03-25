# Story 5.1: Inspect Session History and Run Timeline

Status: review

## Story

As a human operator, I want a session history and run timeline so that I can inspect the progress and outcome of benchmark and solution runs in one place.

## Acceptance Criteria

1. Given a session, when I open it in the UI, then I can see the episode history, status, terminal reason, and progress of the active or completed run.
2. Given backend traces and tool calls exist, when I inspect the run, then the UI renders those persisted records rather than fabricating placeholders.
3. Given an interrupted run, when I reopen the session, then I can resume from the last persisted state.
4. Given context usage telemetry exists, when I inspect the run, then the UI shows it.
5. Given a failed or cancelled session, when I revisit it later, then the summary still shows the persisted terminal reason and failure class instead of collapsing to a generic failure state.
6. Given an active run with phase metadata, when I view the session rail or run header, then I can distinguish the current episode phase/progress from raw status without inventing new run state.

## Tasks / Subtasks

- [x] Keep `EpisodeContext` as the single source of truth for selected episode restoration. Preserve the localStorage restore path and the `fetchEpisode(...)` hydration path on reload/reopen so interrupted sessions repopulate `traces`, `assets`, and episode metadata from the latest persisted API state. (AC: 1, 3)
  - [x] Keep the `selectedEpisodeId` restore behavior intact in `frontend/src/context/EpisodeContext.tsx`.
  - [x] Make sure the websocket/poll hydration path continues to repopulate the full episode record after status changes.
- [x] Extend the session history rail so each history item exposes persisted run state, not just title and icon. Show `status`, `episode_phase`/`detailed_status`, `terminal_reason`, and a small progress indicator derived from `EpisodeMetadata` when present. (AC: 1, 5, 6)
  - [x] Keep the list view backed by `GET /episodes`; do not fabricate a second history source.
  - [x] Keep the selected episode highlight tied to the actual persisted episode id.
- [x] Extend the run header / failure banner in `ChatWindow` so a selected run can show terminal reason, failure class, and context usage telemetry directly from persisted metadata. (AC: 1, 4, 5)
  - [x] Continue to derive context usage from `metadata_vars.additional_info.context_usage`.
  - [x] Keep the failure panel’s detailed logs, but add persisted terminal metadata alongside them.
- [x] Keep the run timeline strictly trace-driven in `TraceList` and its existing child cards. Render only persisted `TOOL_START`, `LLM_END`, `EVENT`, `LOG`, and `ERROR` rows from backend traces, and keep empty-state behavior explicit rather than synthetic. (AC: 2)
  - [x] Do not synthesize placeholder trace rows from phase labels or local UI state.
  - [x] Preserve existing reasoning/tool/event row behavior and ordering.
- [x] If the UI needs a typed field that is not already exposed cleanly, add the minimal schema change to `EpisodeListItem` / `EpisodeResponse` instead of parsing ad hoc JSON in components. (AC: 1, 4, 6)
  - [x] Prefer the shared `EpisodeMetadata` shape over custom frontend-only state.
- [x] Add or extend integration coverage for session selection, reload restore, terminal-reason/progress display, and persisted trace timeline rendering. Use live-browser integration tests only. (AC: 1-6)
  - [x] Anchor the regression coverage on `INT-157`, `INT-160`, and `INT-178`.
  - [x] Add a focused frontend integration slice only if the existing coverage cannot be extended cleanly.

## Dev Notes

- Relevant architecture patterns and constraints:
  - `controller/api/routes/episodes.py` already returns persisted episode metadata, traces, and assets through `GET /episodes` and `GET /episodes/{id}`. The story should not introduce a second history endpoint or synthesize run summaries from component state.
  - `EpisodeMetadata.additional_info.context_usage` is the persisted source for the context usage indicator. Compute and display the percentage from that data rather than storing a duplicate progress field in the UI.
  - `EpisodeMetadata.episode_phase`, `EpisodeMetadata.detailed_status`, and `EpisodeMetadata.terminal_reason` are already available in the shared model and generated frontend types; use them for session/run summaries and terminal-state badges.
  - `EpisodeContext` already restores the selected episode from localStorage and hydrates the full record after status changes. Preserve that contract and make interrupted-reopen behavior work with the persisted episode state instead of replaying the run.
  - `TraceList` is the canonical timeline renderer. It already handles reasoning spans, tool activity, and event rows; the remaining work is to make the surrounding summary surfaces describe the same persisted run state.
  - For failed runs, keep the validation-log display as the detailed failure text and add terminal reason/failure class as metadata, not as a replacement.
- Source tree components to touch:
  - `frontend/src/components/layout/Sidebar.tsx`
  - `frontend/src/context/EpisodeContext.tsx`
  - `frontend/src/components/workspace/ChatWindow.tsx`
  - `frontend/src/components/workspace/TraceList.tsx`
  - `frontend/src/components/workspace/UnifiedGeneratorView.tsx`
  - `frontend/src/components/workspace/ArtifactView.tsx`
  - `controller/api/schemas.py` only if a typed response field is actually missing
  - `tests/integration/frontend/p0/test_frontend_p0.py`
  - `tests/integration/frontend/p1/test_int_160.py`
  - `tests/integration/frontend/p1/test_int_178.py`
- Testing standards summary:
  - Use integration tests only; avoid adding unit-test-only coverage for this story.
  - Prefer live browser assertions plus live API assertions against `/episodes/{id}` and `/episodes/`.
  - Keep assertions tied to persisted backend traces, episode metadata, and restore behavior.

### Project Structure Notes

- `Sidebar` is the session-history rail; `ChatWindow` is the run summary / reasoning / input column; `TraceList` is the chronological timeline; `UnifiedGeneratorView` only composes those pieces. Keep the story changes in those existing layers instead of inventing a parallel history/timeline subsystem.
- Avoid a backend-only solution that stores duplicate history summaries. If a new UI element needs a derived field, derive it in the UI from the persisted episode payload first.
- Preserve the reload and selection semantics already used by `AppLayout` and `EpisodeContext` so the session history continues to repopulate after browser refreshes.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Epic 5: UI, Visualization, and Demo, Story 5.1]
- [Source: \_bmad-output/planning-artifacts/prd.md, Phase 1 UI and debuggability requirements]
- [Source: specs/frontend-specs.md, shared workflow, session history, reasoning, and timeline requirements]
- [Source: specs/architecture/observability.md, episode/session linkage, context usage, and trace requirements]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, persisted traces as the source of truth and no fabricated placeholders]
- \[Source: controller/api/routes/episodes.py, `GET /episodes` and `GET /episodes/{episode_id}` episode payloads\]
- \[Source: controller/api/schemas.py, `EpisodeListItem`, `EpisodeResponse`, `TraceResponse`, and `EpisodeMetadata` typing\]
- \[Source: shared/models/schemas.py, `EpisodeMetadata`, `EpisodePhase`, `TerminalReason`, and `FailureClass`\]
- [Source: frontend/src/context/EpisodeContext.tsx, selected-episode restoration and hydration logic]
- [Source: frontend/src/components/layout/Sidebar.tsx, session history rail]
- [Source: frontend/src/components/workspace/ChatWindow.tsx, run summary, context usage, and failure banner]
- [Source: frontend/src/components/workspace/TraceList.tsx, canonical trace timeline renderer]
- [Source: frontend/src/components/workspace/ArtifactView.tsx, persisted trace-backed timeline usage]
- [Source: specs/integration-tests.md, INT-157, INT-160, INT-161, and INT-178]

## Dev Agent Record

### Agent Model Used

GPT-5.4

### Debug Log References

- Frontend build: `npm run build` in `frontend/`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_solution_evidence.py::test_int_189_persisted_event_metadata_renders_when_content_is_empty_json`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_solution_evidence.py::test_int_189_engineer_run_defaults_to_solution_evidence tests/integration/frontend/p0/test_int_205.py::test_int_205_failed_engineer_retry_revises_same_benchmark`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_frontend_p0.py -k test_int_157_session_history`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_frontend_p0.py -k test_int_162_interrupt_ux_propagation`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_solution_evidence.py`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p1/test_int_160.py tests/integration/frontend/p1/test_int_178.py`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p1/test_int_160.py tests/integration/frontend/p1/test_int_161.py tests/integration/frontend/p1/test_int_178.py`
- Prior validation from this work session also covered `tests/integration/frontend/p1/test_int_160.py` with a skip when the live backend did not persist reasoning traces.
- OpenAPI/client regeneration: `uv run python scripts/generate_openapi.py`
- Frontend client regeneration: `npm run gen:api` in `frontend/`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_int_205.py -k test_int_205_failed_engineer_retry_revises_same_benchmark`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_solution_evidence.py tests/integration/frontend/p1/test_int_161.py tests/integration/frontend/p1/test_int_178.py tests/integration/frontend/p1/test_int_160.py -k 'test_int_189_engineer_run_defaults_to_solution_evidence or test_int_161_tool_activity_and_reasoning_visibility or test_int_178_session_restore_continuity or test_int_160_reasoning_default_hidden_and_expandable'`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_frontend_p0.py::test_int_157_session_history`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p1/test_int_178.py::test_int_178_session_restore_continuity`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_solution_evidence.py::test_int_189_engineer_run_defaults_to_solution_evidence`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p1/test_int_160.py::test_int_160_reasoning_default_hidden_and_expandable` (skipped in the live backend run because no persisted reasoning traces were available)
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_frontend_p0.py::test_int_157_session_history tests/integration/frontend/p1/test_int_178.py::test_int_178_session_restore_continuity`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p1/test_int_160.py::test_int_160_reasoning_default_hidden_and_expandable` (live backend emitted a fail-fast node-entry validation error, so the test was skipped by the backend-error gate on the rerun)
- Integration: `uv run pytest -n0 -m integration_p1 .autopilot/tests/integration/architecture_p1/test_bmad_autopilot_story_statuses.py::test_int_autopilot_story_worktree_restore_reuses_branch_and_location`

### Completion Notes List

- Resolved the remaining review finding by treating empty JSON `EVENT` bodies as absent in `TraceList` so the renderer falls back to persisted metadata instead of showing `{}`.
- Added a deterministic browser regression that injects a generic `EVENT` row with empty content and persisted motor-state metadata, then verifies the row renders the metadata payload in the browser.
- Revalidated the solution-evidence flow and the retry-lineage event-row path with live-browser integration tests after the trace renderer change.
- Added persisted session-history metadata to the sidebar: raw status, detailed status, episode phase, terminal reason, failure class, and a derived progress indicator.
- Kept the run summary aligned with persisted metadata by showing raw status separately from detailed status, and by showing validation logs alongside persisted terminal metadata for failed sessions.
- Restored trace-driven rendering for persisted `EVENT` rows so backend review, validation, and simulation records appear in the run timeline instead of disappearing.
- Fixed the circuit timeline data path so it reads persisted motor states from trace metadata and backed that contract with regenerated OpenAPI/client artifacts.
- Extended live-browser coverage for session history, retry lineage failure handling, persisted trace events, context usage telemetry, reasoning visibility, and reload continuity.
- Verified the frontend bundle and the targeted live-browser slices after the schema and render updates.
- Restored persisted `running` state from the selected episode on reload, selection changes, websocket updates, and poll hydration so reopened active runs stay in stop/steer mode.
- Aligned the artifact timeline to the same resolved media episode that supplies the asset bundle, and exposed that episode id in the artifact debug payload for regression coverage.
- Tightened the live-browser regression coverage so INT-178 asserts the restored active-run affordance and INT-189 asserts the artifact timeline source matches the resolved media episode.
- Verified the frontend build and the targeted live-browser slices after the state-restoration and artifact-source fixes.
- Added a selection-intent/request guard in `EpisodeContext` so late episode hydration responses can no longer overwrite a newer selection, and removed the fallback that cleared the persisted selection id on transient restore failures.
- Coalesced same-target episode hydrate requests so the refresh loop cannot invalidate an in-flight restore or selection load, then verified the updated behavior with the session-history and session-restore live-browser slices.
- Seeded the INT-160 reasoning-visibility regression through the API with a deterministic mock scenario so the live-browser check now verifies persisted reasoning traces instead of timing out on backend setup.
- Normalized story worktree `sprint-status.yaml` materialization so reopened story worktrees validate their persisted `story_location` against the active checkout instead of the original repo path.
- Switched story worktree restoration to reuse an existing branch with `git worktree add -f`, which keeps interrupted-session recovery working even after the temp directory is removed.
- Added a focused architecture-p1 regression that proves a deleted story worktree can be recreated, revalidated, and reopened with the normalized story-location contract intact.

### File List

- `shared/models/schemas.py`
- `controller_openapi.json`
- `frontend/src/api/generated/models/TraceMetadata.ts`
- `frontend/src/context/EpisodeContext.tsx`
- `frontend/src/components/workspace/ChatWindow.tsx`
- `frontend/src/components/workspace/ArtifactView.tsx`
- `frontend/src/components/workspace/UnifiedGeneratorView.tsx`
- `frontend/src/components/workspace/TraceList.tsx`
- `frontend/src/components/visualization/CircuitTimeline.tsx`
- `tests/integration/frontend/p0/test_int_205.py`
- `tests/integration/frontend/p0/test_frontend_p0.py`
- `tests/integration/frontend/p0/test_solution_evidence.py`
- `tests/integration/frontend/p1/test_int_178.py`
- `tests/integration/frontend/p1/test_int_160.py`
- `tests/integration/mock_responses/INT-160.yaml`
- `.autopilot/scripts/internal/runner_environment.py`
- `.autopilot/scripts/internal/runner_state_worktree.py`
- `.autopilot/tests/integration/architecture_p1/test_bmad_autopilot_story_statuses.py`

## Change Log

- 2026-03-25: Treated empty JSON event bodies as absent in the timeline renderer, added a deterministic browser regression for metadata-backed event rows, and reran the live-browser solution-evidence and retry-lineage slices.
- 2026-03-25: Restored persisted event rows in the timeline, added failure-log rendering alongside terminal metadata, wired circuit-timeline motor states to the trace contract, and regenerated the OpenAPI/client artifacts.
- 2026-03-25: Verified the updated frontend bundle and reran the targeted live-browser slices for session history, retry lineage, reasoning visibility, solution evidence, and reload continuity.
- 2026-03-25: Restored selected-episode running state on reload and aligned the artifact timeline with the resolved media episode, then reran the live-browser session-history, restore, and solution-evidence regressions.
- 2026-03-25: Added request-guarded episode hydration in `EpisodeContext` so stale restore/select responses cannot overwrite a newer session and transient restore failures keep the persisted episode pointer intact.
- 2026-03-25: Revalidated the story with the live-browser session-history and session-restore slices after the context fix; INT-160 was still backend-gated by a fail-fast node-entry validation error and was skipped by the backend-error fixture on rerun.
- 2026-03-25: Reworked INT-160 into an API-seeded reasoning-visibility regression, added the deterministic mock scenario needed for the engineer flow, and verified the live-browser coverage passed end to end.
- 2026-03-25: Normalized story-worktree sprint-status materialization, enabled existing-branch reuse for deleted story worktrees, and added a regression covering reopen/resume of interrupted story sessions.
