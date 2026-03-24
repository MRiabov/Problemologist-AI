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
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_frontend_p0.py -k test_int_157_session_history`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p0/test_solution_evidence.py`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p1/test_int_178.py`
- Integration: `./scripts/run_integration_tests.sh tests/integration/frontend/p1/test_int_160.py tests/integration/frontend/p1/test_int_178.py`
- Prior validation from this work session also covered `tests/integration/frontend/p1/test_int_160.py` with a skip when the live backend did not persist reasoning traces.

### Completion Notes List

- Added persisted session-history metadata to the sidebar: raw status, detailed status, episode phase, terminal reason, failure class, and a derived progress indicator.
- Kept the run summary and reasoning surfaces aligned with persisted metadata and trace-driven rendering.
- Extended live-browser coverage for session history, terminal metadata, context usage telemetry, and reload continuity.
- Tightened the `INT-178` demo-mode assertion to use an exact text match so the reload continuity check targets the status badge instead of colliding with the exit button.

### File List

- `frontend/src/components/layout/Sidebar.tsx`
- `frontend/src/components/workspace/ChatWindow.tsx`
- `frontend/src/components/workspace/TraceList.tsx`
- `frontend/src/components/workspace/__tests__/ChatWindow.test.tsx`
- `frontend/src/components/workspace/__tests__/FeedbackSystem.test.tsx`
- `specs/frontend-specs.md`
- `tests/integration/frontend/p0/test_frontend_p0.py`
- `tests/integration/frontend/p0/test_solution_evidence.py`
- `tests/integration/frontend/p1/test_int_160.py`
- `tests/integration/frontend/p1/test_int_178.py`
