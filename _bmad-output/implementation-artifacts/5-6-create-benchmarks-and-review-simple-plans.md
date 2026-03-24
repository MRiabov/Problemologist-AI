# Story 5.6: Create Benchmarks and Review Simple Plans

Status: review

## Story

As a human operator, I want to create benchmark drafts and accept or reject simple plan artifacts in the UI so that I can handle lightweight workflow steps without leaving the browser.

## Acceptance Criteria

1. Given I am in the benchmark workspace, when I submit a benchmark draft prompt and objective inputs, then the UI calls the controller-backed benchmark generation flow, persists the new benchmark episode, and keeps the benchmark route selected.
2. Given a benchmark planner has produced a simple plan, when I review it, then I can approve it from either the chat card or the file explorer, optionally add a comment, and see that comment persisted in the episode trace/history.
3. Given I choose to reject or request changes to a benchmark plan, when I do so from the UI, then the reason is recorded explicitly through the controller-backed comment/chat path and remains visible in the episode trace/history instead of being handled by local-only UI state.
4. Given I edit benchmark objectives while drafting, when I update them, then the controller persists the values and the selected episode or creation mode remains stable across refresh and route changes.
5. Given I switch between benchmark and engineer workspaces, then benchmark draft and plan-approval controls remain benchmark-only and do not leak into engineer runs.

## Tasks / Subtasks

- [x] Keep the benchmark draft lifecycle controller-backed in `EpisodeContext`, `BenchmarkGeneration`, `ChatWindow`, and `ObjectivesForm`. Preserve `createNewBenchmark`, `startAgent(..., objectives)`, `updateObjectives`, and `confirmBenchmark` as the only paths for creating a benchmark draft, persisting objective edits, and continuing after plan approval. (AC: 1, 4, 5)
  - [x] Keep `/benchmark` entering benchmark creation mode via `createNewBenchmark(true)` and `generateBenchmark(...)`, not the engineer run path.
  - [x] Preserve the `BenchmarkObjectives` to `CustomObjectives` field mapping (`max_cost`/`max_weight`/`target_quantity` on the frontend, `max_unit_cost`/`max_weight`/`target_quantity` in persisted episode metadata).
  - [x] Do not add a second local benchmark draft store or route-specific shadow state.
- [x] Keep the plan-approval surfaces in `ChatWindow` and `ArtifactView` aligned with the backend planner state. `chat-confirm-button` and `file-explorer-confirm-button` should remain the visible approval entry points, with optional comments or reasons persisted via the existing controller-backed confirmation/comment trace flow. (AC: 2, 3)
  - [x] Keep `Request Changes` explicit and user-visible; it should route to a reason-bearing comment/message path rather than a silent no-op.
  - [x] Preserve the existing `user_confirmation` trace behavior and do not replace it with a synthetic frontend-only approval record.
- [x] Keep benchmark/workspace routing and shell composition consistent. `Sidebar`, `BenchmarkGeneration`, and `UnifiedGeneratorView` should continue to route benchmark sessions to `/benchmark`, engineer sessions to `/`, and keep the benchmark create-new button behavior route-aware. (AC: 1, 5)
  - [x] Preserve the benchmark-specific header, subtitle, and badge configuration in `BenchmarkGeneration`.
  - [x] Keep benchmark session selection tied to persisted episode metadata, not page-local state.
- [x] Extend live-browser integration coverage for benchmark draft creation, objective persistence, approval/comment persistence, and control placement. Use the existing benchmark frontend P0 coverage as the source of truth and add only the minimal assertions needed. (AC: 1-5)
  - [x] Keep `INT-158` for benchmark/solution workflow parity.
  - [x] Keep `INT-159` for approval/comment persistence.
  - [x] Keep `INT-172` for control placement.
  - [x] Keep `INT-114` for planner submission gating, because the UI approval flow depends on the backend benchmark plan handoff.

## Dev Notes

- The benchmark workspace is controller-first. Draft creation, objective updates, and plan approval must continue to flow through `frontend/src/api/client.ts` and the controller benchmark endpoints:
  - `POST /api/benchmark/generate`
  - `POST /api/benchmark/{sessionId}/objectives`
  - `POST /api/benchmark/{sessionId}/confirm`
- `EpisodeContext` is the source of truth for benchmark creation mode, selected episode hydration, and confirmation wiring. Preserve the current route-aware `createNewBenchmark(true)` behavior instead of introducing a separate benchmark-draft store.
- `ChatWindow` already owns the benchmark approval card, the optional comment textarea, the objectives panel, and the `Request Changes` affordance. Keep those controls bound to persisted episode state rather than local-only UI state.
- `ArtifactView` already exposes the file-explorer approval button. Keep that entry point aligned with the chat approval control so benchmark confirmation is available in both places.
- `ObjectivesForm` edits the benchmark draft objective fields. Keep the existing three-field contract intact and preserve the frontend/backend naming bridge between `max_cost` and `max_unit_cost`.
- The backend `confirm` route currently persists confirmation comments as a trace. Preserve that trace-backed comment contract so approval and request-change reasons are visible in the episode history.
- The benchmark route shell in `BenchmarkGeneration` and `UnifiedGeneratorView` should remain the benchmark-only entrypoint. Do not let the engineer route inherit benchmark draft state or approval controls.
- If a new typed field is needed for a benchmark decision/reason, add the minimal schema change to the controller model and regenerate the client rather than parsing ad hoc JSON in the UI.

### Non-Goals

- Do not add a separate benchmark plan editor or a second approval workflow.
- Do not add advanced steering, multiphysics visualization, or new review states in this story.
- Do not change the shared workspace layout unless it is strictly required to preserve benchmark draft or approval behavior.

### Project Structure Notes

- `frontend/src/pages/BenchmarkGeneration.tsx` owns the benchmark route shell.
- `frontend/src/components/workspace/UnifiedGeneratorView.tsx` composes the shared benchmark/engineer workspace layout.
- `frontend/src/components/workspace/ChatWindow.tsx` owns the benchmark approval card, comment entry, and objectives panel.
- `frontend/src/components/workspace/ArtifactView.tsx` owns the file-explorer approval entry point.
- `frontend/src/components/workspace/ObjectivesForm.tsx` owns the draft objective inputs.
- `frontend/src/context/EpisodeContext.tsx` owns benchmark creation mode, episode hydration, objective persistence, and confirmation plumbing.
- `frontend/src/components/layout/Sidebar.tsx` owns route-aware session selection and benchmark creation entry.
- `frontend/src/api/client.ts` owns the benchmark draft/confirm API bridge.
- `controller/api/routes/benchmark.py` owns the benchmark generate/objectives/confirm backend contract.
- `tests/integration/frontend/p0/test_frontend_p0.py` is the main benchmark workflow parity and approval/comment regression file.
- `tests/integration/frontend/p0/test_int_172.py` covers the control placement contract.
- `tests/integration/frontend/p1/test_int_178.py` covers session restore continuity.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Epic 5: UI, Visualization, and Demo, Story 5.6]
- [Source: \_bmad-output/planning-artifacts/prd.md, Phase 1 UI and plan approval requirements]
- [Source: \_bmad-output/planning-artifacts/ux-design-specification.md, sections 6, 7, 8, and 9]
- [Source: specs/frontend-specs.md, benchmark generation workflow, plan approval, chat UI, and planning controls]
- [Source: specs/architecture/agents/handover-contracts.md, benchmark planner handoff and approval contracts]
- \[Source: controller/api/routes/benchmark.py, `/generate`, `/objectives`, and `/confirm` endpoints\]
- [Source: frontend/src/context/EpisodeContext.tsx, benchmark draft state and confirmation plumbing]
- [Source: frontend/src/components/workspace/ChatWindow.tsx, benchmark approval card and objectives panel]
- [Source: frontend/src/components/workspace/ArtifactView.tsx, file-explorer approval entry point]
- [Source: frontend/src/components/workspace/ObjectivesForm.tsx, draft objective editor]
- [Source: frontend/src/components/layout/Sidebar.tsx, route-aware benchmark creation and episode selection]
- \[Source: frontend/src/api/client.ts, `generateBenchmark`, `updateBenchmarkObjectives`, and `confirmBenchmark`\]
- [Source: tests/integration/frontend/p0/test_frontend_p0.py, INT-158 and INT-159]
- [Source: tests/integration/frontend/p0/test_int_172.py, control placement contract]
- [Source: tests/integration/frontend/p1/test_int_178.py, restore continuity contract]
- [Source: specs/integration-tests.md, INT-114, INT-158, INT-159, INT-172, and INT-178]

## Dev Agent Record

### Agent Model Used

GPT-5.4

### Debug Log References

- 2026-03-24: Updated benchmark-only draft/approval control flow in `EpisodeContext`, `ChatWindow`, `ArtifactView`, `ChatInput`, `BenchmarkGeneration`, and `Sidebar`.
- 2026-03-24: Verified frontend production build with `npm run build` in `frontend/`.

### Completion Notes List

- Kept benchmark draft creation controller-backed through the existing `generateBenchmark`, `updateBenchmarkObjectives`, and `confirmBenchmark` paths.
- Moved benchmark approval comment handling into shared episode context so chat and file-explorer approval surfaces use the same persisted comment value.
- Gated benchmark planning/approval controls to the benchmark route and hid them from engineer sessions.
- Updated integration coverage so benchmark approval/comment persistence and control placement are exercised through the existing P0 browser suite.
- Confirmed the frontend production build passes after the implementation changes.

### File List

- `_bmad-output/implementation-artifacts/5-6-create-benchmarks-and-review-simple-plans.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
- `frontend/src/context/EpisodeContext.tsx`
- `frontend/src/components/workspace/ChatWindow.tsx`
- `frontend/src/components/workspace/ArtifactView.tsx`
- `frontend/src/components/Chat/ChatInput.tsx`
- `frontend/src/components/layout/Sidebar.tsx`
- `frontend/src/pages/BenchmarkGeneration.tsx`
- `tests/integration/frontend/p0/test_frontend_p0.py`
- `tests/integration/frontend/p0/test_int_172.py`

### Change Log

- 2026-03-24: Completed the benchmark draft and plan-review UI wiring, routed approval comments through shared controller-backed episode state, tightened benchmark-only control visibility, and extended the browser coverage for approval/comment persistence and control placement.

### Status

review
