# Story 5.5: Inspect Live Agent Output and Interrupt Execution

Status: ready-for-dev

<!-- Note: Validation is optional. Run validate-create-story for quality check before dev-story. -->

## Story

As a human operator, I want to see the agent's output and reasoning in real time, and interrupt execution when it goes off track, so that I can catch logical errors early.

## Acceptance Criteria

1. Given an active agent run, when I inspect the session, then I can see tool-call activity and reasoning traces as they arrive from the backend.
2. Given I need to stop a run, when I press the stop button, then the controller interrupts the active agent execution and the UI stops showing the run as actively streaming.
3. Given reasoning traces are required but unavailable, when I inspect the run, then the UI shows an explicit missing-trace state instead of pretending success.

## Tasks / Subtasks

- [ ] Preserve the live trace stream contract in the shared workspace shell. Keep `EpisodeContext` as the source of truth for selected episode hydration, websocket event ingestion, polling refresh, and per-episode scoping. Do not add a second trace store or synthetic local echo for reasoning/tool rows. (AC: 1, 3)
  - [ ] Continue to append backend `new_trace` websocket events into the selected episode payload only when the event belongs to the active `episode_id`.
  - [ ] Keep the fallback polling refresh after terminal status changes so the timeline ends from the latest persisted episode payload.
- [ ] Keep the stop path controller-first and visible in the existing input control. `ChatInput` should continue to switch from `Send Message` to `Stop Agent`, and the stop action should call `EpisodeContext.interruptAgent`, which maps to `POST /api/episodes/{episode_id}/interrupt` through the generated client. (AC: 2)
  - [ ] Preserve the existing `aria-label="Stop Agent"` and Playwright-visible button state contract.
  - [ ] Do not mark the run as stopped purely from local component state; wait for backend status updates or refetch.
- [ ] Preserve the reasoning/tool rendering contract in `TraceList`, `ActionCard`, and `ThoughtBlock`. Reasoning rows must come only from persisted `LLM_END` traces with names, tool rows must come from `TOOL_START`, and missing reasoning in reasoning-required runs must surface an explicit warning state instead of a fabricated placeholder. (AC: 1, 3)
  - [ ] Keep `reasoning_step_index` and `reasoning_source` ordering metadata intact when present.
  - [ ] Do not infer hidden reasoning from phase labels, prompt text, or chat placeholders.
- [ ] Extend the real integration coverage for live output and interrupt behavior. Use the existing Playwright/live-stack tests as the source of truth and only add assertions where the current coverage is too weak. (AC: 1, 2, 3)
  - [ ] Keep `tests/integration/frontend/p0/test_frontend_p0.py` aligned with INT-158 and INT-162 expectations.
  - [ ] Keep `tests/integration/frontend/p1/test_int_160.py` aligned with hidden-by-default reasoning visibility.
  - [ ] Keep `tests/integration/frontend/p1/test_int_161.py` aligned with live tool-call rows and reasoning traces.
  - [ ] Reuse `tests/integration/frontend/p1/test_int_178.py` reload and rehydration patterns only if the interrupted or live run state needs an explicit restore assertion.

## Dev Notes

- `EpisodeContext` owns the live episode state. It already handles selected-episode hydration, websocket trace ingestion, periodic refetch after status changes, and the interrupt call. Keep that as the single source of truth for active run state.
- `ChatWindow` composes the shared timeline, reasoning toggle, activity feed, and stop control. Preserve the existing data flow from `selectedEpisode` into `TraceList` and `ChatInput`.
- `TraceList` renders reasoning only from persisted backend traces. It already has the explicit `reasoning-telemetry-warning` state for reasoning-required runs that have tool activity but no reasoning traces yet.
- `ActionCard` is the live tool-call renderer. It turns `TOOL_START` traces into typed rows such as `Edited`, `Viewed`, and `Read`; do not replace that with a synthetic activity feed.
- `ThoughtBlock` is the expandable reasoning primitive. Keep it as the single reasoning presentation component instead of introducing a second reasoning UI.
- `frontend/src/api/client.ts` already exposes `interruptEpisode(...)` and routes it to `POST /api/episodes/{episode_id}/interrupt` via the generated controller client. Reuse that path rather than adding a parallel stop API wrapper.
- Missing reasoning is a backend telemetry failure, not a UI success case. The frontend should fail closed with an explicit warning state when reasoning is required but absent.
- Keep the stop button contract stable for Playwright and other integration assertions. The existing `Stop Agent` label is part of the user-facing and test-facing API.

### Non-Goals

- Do not add a new feedback flow, demo mode, or artifact browser changes in this story.
- Do not add a synthetic stream or placeholder reasoning content to make the UI look complete.
- Do not change the shared workspace layout unless it is strictly required to preserve live trace/interrupt behavior.

### Project Structure Notes

- `frontend/src/context/EpisodeContext.tsx` owns episode selection, trace hydration, websocket updates, and interrupt wiring.
- `frontend/src/components/workspace/ChatWindow.tsx` owns the shared live timeline and input composition.
- `frontend/src/components/Chat/ChatInput.tsx` owns the send/stop button state and the user-facing interrupt action.
- `frontend/src/components/workspace/TraceList.tsx` owns live trace rendering, reasoning visibility, and the missing-trace warning.
- `frontend/src/components/workspace/ActionCard.tsx` owns tool activity rows and artifact jump behavior.
- `frontend/src/components/workspace/ThoughtBlock.tsx` owns expandable reasoning rows.
- `frontend/src/components/layout/AppLayout.tsx` hosts the global feedback modal, but it should not absorb live-stream or interrupt logic for this story.
- `tests/integration/frontend/p0/test_frontend_p0.py` is the primary interrupt regression guard for INT-162.
- `tests/integration/frontend/p1/test_int_160.py` and `tests/integration/frontend/p1/test_int_161.py` cover the reasoning and tool-call rendering contract in the live stack.
- `tests/integration/frontend/p1/test_int_178.py` shows the reload and rehydration pattern to reuse if the interrupted state needs an explicit restore check.

### References

- [Source: _bmad-output/planning-artifacts/epics.md, Epic 5: UI, Visualization, and Demo, Story 5.5]
- [Source: specs/frontend-specs.md, Reasoning trace contract, Context and compaction telemetry contract, Interrupting the worker, Collecting feedback from users]
- [Source: specs/architecture/observability.md, Reasoning trace capture and missing-trace reconstruction]
- [Source: specs/integration-tests.md, INT-158, INT-160, INT-161, INT-162, and INT-178]
- [Source: frontend/src/context/EpisodeContext.tsx, websocket ingestion, hydration, and interrupt plumbing]
- [Source: frontend/src/components/workspace/ChatWindow.tsx, shared live trace composition]
- [Source: frontend/src/components/Chat/ChatInput.tsx, Send/Stop button and interrupt callback]
- [Source: frontend/src/components/workspace/TraceList.tsx, backend-backed reasoning and tool rendering]
- [Source: frontend/src/components/workspace/ActionCard.tsx, tool activity labels and artifact jump behavior]
- [Source: frontend/src/components/workspace/ThoughtBlock.tsx, expandable reasoning rows]
- [Source: frontend/src/api/client.ts, `interruptEpisode(...)` and trace feedback client paths]
- [Source: tests/integration/frontend/p0/test_frontend_p0.py, INT-162 interrupt UX baseline]
- [Source: tests/integration/frontend/p1/test_int_160.py, INT-160 reasoning visibility]
- [Source: tests/integration/frontend/p1/test_int_161.py, INT-161 live tool activity and reasoning]
- [Source: tests/integration/frontend/p1/test_int_178.py, reload restoration pattern]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
