# Story 5.4: Collect Feedback and Present Demos

Status: in-progress

## Story

As a human operator, I want to submit feedback and present completed runs through the UI so that I can capture review input and demonstrate system capability without leaving the browser.

## Acceptance Criteria

1. Given a completed model output, when I open feedback from the final assistant turn or run history, then I can choose thumbs up/down, select a common topic, and enter a short explanation before submitting.
2. Given feedback has already been submitted for a trace, when I reopen the same episode later, then the persisted score and comment are visible in the run UI and the feedback modal opens with the saved state preloaded so it can be revised.
3. Given a completed run, when I switch into demo/presentation mode, then the UI keeps the session history, chat, CAD/simulation viewport, and artifact browser visible in a stable layout tied to the selected episode.
4. Given I reload or revisit the completed run, when demo mode is active, then the selected episode, layout state, and feedback state restore from persisted API/local state without manual re-selection.

## Tasks / Subtasks

- [x] Keep `FeedbackSystem` as the single global modal host in `AppLayout`, but pass it enough trace/episode context to prefill the selected trace's score/comment and resubmit edited feedback cleanly. (AC: 1, 2, 4)
  - [x] Reuse the existing topic chips and `submitTraceFeedback(...)` flow; do not create a second feedback form or a separate persistence path.
- [x] Surface persisted trace feedback back into the episode UI so later review is visible. Render `TraceResponse.feedback_score` and `feedback_comment` on the relevant trace or summary row, and keep the final assistant turn as the only place where new feedback can be launched. (AC: 2)
  - [x] Keep the combined `[topic] comment` format unless a new typed topic field is intentionally added to the controller/OpenAPI schema.
  - [x] Do not infer topics from UI state after the episode has already been persisted.
- [x] Add a presentation/demo layout mode to the shared workspace shell. Reuse `UnifiedGeneratorView` plus the existing page wrappers so demo mode is a shell composition change, not a new route or duplicated workspace. (AC: 3, 4)
  - [x] Keep session history, chat, viewport, and artifact panels visible and stable.
  - [x] Preserve selected episode and layout state across reloads.
- [x] Keep episode restore and feedback modal state tied to `EpisodeContext`. Clear modal state only on explicit close or successful submission. (AC: 2, 4)
  - [x] Do not add a parallel presentation-state store.
  - [x] Reuse the existing controller-first episode polling/hydration path.
- [x] Extend live-browser integration coverage for completion-gated feedback, edited feedback persistence, and demo-layout stability. (AC: 1-4)
  - [x] Anchor on `INT-170`, `INT-171`, `INT-177`, and `INT-178`.
  - [x] Extend `tests/integration/frontend/p0/test_frontend_p0.py`, `tests/integration/frontend/p0/test_int_177.py`, and `tests/integration/frontend/p1/test_int_178.py`; add a focused `test_int_171.py` only if the layout regression needs its own slice.

## Dev Notes

- Relevant architecture patterns and constraints:
  - The frontend remains controller-first. Feedback submission must go through `POST /api/episodes/{episode_id}/traces/{trace_id}/feedback` via the generated client, not through direct persistence writes or worker access.
  - `FeedbackSystem` already owns the modal UX, topic chips, and score editing. Extend the existing component and its global host in `AppLayout` instead of building a second feedback subsystem.
  - `TraceResponse.feedback_score` and `TraceResponse.feedback_comment` already exist in the controller schema and generated frontend types, so later review can be rendered from persisted trace data without a new backend model unless you deliberately normalize feedback topics.
  - `TraceList` and `Sidebar` are the existing feedback entry points. Keep feedback tied to the final assistant output and the selected episode's latest completed trace.
  - Demo/presentation layout should be a composition concern in `UnifiedGeneratorView` and the page wrappers. Reuse the existing `EngineerWorkspace` and `BenchmarkGeneration` shells; do not create a separate demo page with duplicated workspace state.
  - `EpisodeContext` owns selected-episode restoration, run hydration, and transient modal state. Keep it as the source of truth for reload and reopen behavior.
  - If a normalized topic field becomes necessary, add it to the typed controller/OpenAPI models and regenerate the frontend client instead of parsing ad hoc text after persistence.

### Project Structure Notes

- `FeedbackSystem` is the modal host and submitter; `AppLayout` is the global mount point.
- `Sidebar` and `TraceList` are the trigger surfaces; `ChatWindow` controls when the final assistant turn can be reviewed.
- `UnifiedGeneratorView` and the two page wrappers own the stable presentation layout. Keep demo mode inside that shell instead of adding another route tree.
- Persisted feedback should remain trace-scoped inside the episode payload so it can be reviewed later from the same run.
- Avoid creating a duplicate feedback history store or a second presentation state machine.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Epic 5: UI, Visualization, and Demo, Story 5.4]
- [Source: specs/frontend-specs.md, Collecting feedback from users]
- [Source: specs/architecture/observability.md, User Reviews and trace feedback fields]
- \[Source: controller/api/schemas.py, `TraceResponse.feedback_score` and `feedback_comment`\]
- [Source: frontend/src/components/workspace/FeedbackSystem.tsx, modal UI, topic chips, and score editing]
- [Source: frontend/src/components/workspace/TraceList.tsx, feedback entry points on completed traces]
- [Source: frontend/src/components/layout/Sidebar.tsx, episode-level hover feedback entry points]
- [Source: frontend/src/components/layout/AppLayout.tsx, global feedback modal host]
- [Source: frontend/src/components/workspace/UnifiedGeneratorView.tsx, shared workspace shell]
- [Source: frontend/src/pages/EngineerWorkspace.tsx, engineer presentation shell]
- [Source: frontend/src/pages/BenchmarkGeneration.tsx, benchmark presentation shell]
- [Source: frontend/src/context/EpisodeContext.tsx, selected episode restore and transient UI state]
- [Source: specs/integration-tests.md, INT-170, INT-171, INT-177, and INT-178]

## Dev Agent Record

### Agent Model Used

GPT-5.4

### Debug Log References

- `npm run build -- --mode development` in `frontend/` completed successfully after a single syntax fix in `TraceList.tsx`.
- Integration tests were not run in this turn per user request because a parallel suite is already active.

### Completion Notes List

- Added feedback prefill plumbing from `EpisodeContext`/selected trace into the global `FeedbackSystem` host.
- Preserved the existing `submitTraceFeedback(...)` controller flow and kept the combined `[topic] comment` format.
- Rendered persisted trace feedback back into the chat trace UI so reopened runs show score/topic/comment context.
- Added a persisted demo/presentation mode flag in `UISettingsContext` and surfaced it in both workspace shells.
- Extended the browser integration coverage files for feedback recall and demo-mode persistence.

### File List

- `_bmad-output/implementation-artifacts/5-4-collect-feedback-and-present-demos.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
- `frontend/src/components/layout/AppLayout.tsx`
- `frontend/src/components/workspace/FeedbackSystem.tsx`
- `frontend/src/components/workspace/TraceList.tsx`
- `frontend/src/components/workspace/UnifiedGeneratorView.tsx`
- `frontend/src/components/workspace/feedbackUtils.ts`
- `frontend/src/context/UISettingsContext.tsx`
- `frontend/src/pages/BenchmarkGeneration.tsx`
- `frontend/src/pages/EngineerWorkspace.tsx`
- `tests/integration/frontend/p0/test_int_177.py`
- `tests/integration/frontend/p1/test_int_178.py`

## Change Log

- 2026-03-24: Implemented persisted feedback prefill/display, demo-mode shell persistence, and live-browser coverage updates for INT-170/171/177/178.
