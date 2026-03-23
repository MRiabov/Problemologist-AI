# Story 2.3: Revise and Retry Against the Same Benchmark

Status: done

## Story

As a human engineer, I want to revise a failed solution and try again against the same benchmark so that I can converge on a passing design without redefining the problem.

## Acceptance Criteria

1. Given a failed engineer solution episode, when I request a revision or retry, then the new engineer episode preserves the original `benchmark_id`, records `prior_episode_id`, marks `is_reused`, and leaves the benchmark definition and environment version unchanged.
2. Given a revised solution is submitted, when validation runs again, then the new attempt is evaluated against the same benchmark contract and the copied benchmark-owned assets resolve to the same benchmark package as the original episode.
3. Given multiple revisions exist, when I inspect the workspace or episode history, then I can see the revision chain and compare the outcomes of each revision against the same benchmark package.
4. Given the currently selected episode is not a failed engineer episode, when the workspace renders actions, then the revise/retry control is hidden or disabled so the system fails closed.

## Tasks / Subtasks

- [x] Add a revise/retry action in `frontend/src/components/workspace/ChatWindow.tsx` for failed engineer episodes.
  - [x] Launch a fresh engineer episode through `startAgent(...)` using the selected episode lineage metadata (`benchmark_id`, `prior_episode_id`, `is_reused: true`) and do not mutate the failed episode in place.
  - [x] Keep the action hidden for benchmark episodes and for any episode state that is not a failure.
  - [x] Add stable `data-testid` hooks for the retry control and the failure-summary container so integration tests can target them directly.
- [x] Surface revision lineage in the workspace history view, either in `frontend/src/components/layout/Sidebar.tsx` or a small shared revision-summary component.
  - [x] Show the shared benchmark linkage, parent episode, and reuse state for each revision.
  - [x] Make it easy to switch between revisions and compare their terminal outcomes against the same benchmark package.
- [x] Extend `frontend/src/components/workspace/UnifiedGeneratorView.tsx` with a hidden debug payload for the retry flow.
  - [x] Include `benchmarkId`, `priorEpisodeId`, `isReused`, and a revision-count or related-episode hook that Playwright can assert without brittle text matching.
- [x] Reuse the existing `/agent/run` metadata propagation and benchmark asset copy path rather than introducing a second benchmark store or retry endpoint.
  - [x] Confirm the new revision inherits the same benchmark package and environment version through the existing controller/task bootstrap path in `controller/api/main.py` and `controller/api/tasks.py`.
- [x] Extend `tests/integration/architecture_p1/test_engineering_loop.py` to prove a retry creates a distinct engineer episode with the same benchmark linkage.
  - [x] Assert persisted `benchmark_id`, `prior_episode_id`, `is_reused`, and the expected benchmark-owned assets for both attempts.
- [x] Add a Playwright integration test under `tests/integration/frontend/p0/` that exercises retry from a failed engineer episode and verifies both revisions are visible and inspectable.
  - [x] Assert the retry control appears only on the failed engineer episode, the second episode is created, and the workspace can compare both outcomes against the same benchmark package.
- [x] Validate the retry path with deterministic seeded episodes; no dedicated `INT-205` mock-response transcript is required.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Story 2.1 established benchmark-to-engineer handoff and read-only benchmark context. This story must keep the benchmark package immutable and only create a new engineer episode.
  - `EpisodeMetadata` already has `benchmark_id`, `prior_episode_id`, `is_reused`, `terminal_reason`, and `failure_class`; use those fields instead of inventing a revision-specific schema.
  - `controller/api/tasks.py` already copies benchmark assets when `metadata.benchmark_id` is present. Retry should reuse that path so the revised run sees the same benchmark package.
  - `controller/api/routes/episodes.py` already exposes `EpisodeResponse.metadata_vars`; use that existing surface for lineage/history display rather than a new API.
  - `frontend/src/components/workspace/ChatWindow.tsx` is the natural place for the retry affordance because it already renders failure-state guidance.
  - `frontend/src/components/layout/Sidebar.tsx` can surface the episode chain for comparison without changing the benchmark/solution split.
  - `frontend/src/components/workspace/UnifiedGeneratorView.tsx` already carries a hidden debug payload for integration tests; extend it instead of scraping visible DOM text.
- Story 2.2 owns evidence and terminal-outcome inspection. This story should add revision lineage and retry only, not a second evidence model.
- Do not infer a valid retry path from a benchmark episode or a non-failed state; the UI and tests should stay fail closed.
- The final implementation seeds the retry UI test with deterministic failed engineer episodes through the integration test API rather than depending on a dedicated `INT-205` transcript.
- Source tree components to touch:
  - `frontend/src/components/workspace/ChatWindow.tsx`
  - `frontend/src/components/layout/Sidebar.tsx`
  - `frontend/src/components/workspace/UnifiedGeneratorView.tsx`
  - `controller/api/main.py`
  - `controller/api/tasks.py`
  - `tests/integration/architecture_p1/test_engineering_loop.py`
  - `tests/integration/frontend/p0/test_int_205.py`
  - `tests/integration/mock_responses/INT-205.yaml` was not needed after seeding deterministic episodes through the test API
- Testing standards summary:
  - Use integration tests only; verify via HTTP responses, persisted episode metadata, episode assets, and visible workspace state.
  - Keep assertions on lineage and benchmark package reuse rather than on internal helper calls.
  - Prefer the live engineer workflow for the success path. Use a deterministic mock response only if the retry path cannot be reproduced reliably through the real stack.

### Project Structure Notes

- Keep this as a retry-and-lineage story, not a new benchmark persistence subsystem.
- Reuse the existing benchmark handoff and episode metadata fields; do not add a second benchmark package store or a new revision API unless the current metadata propagation path drops fields.
- If a stable UI selector is needed, add explicit `data-testid` hooks for the retry action and revision summary instead of relying on generic text matching.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 2.3: Revise and Retry Against the Same Benchmark]
- [Source: \_bmad-output/implementation-artifacts/2-1-start-solution-episode-from-approved-benchmark.md, benchmark-to-engineer handoff and immutable benchmark context]
- [Source: \_bmad-output/implementation-artifacts/2-2-inspect-solution-evidence-and-terminal-outcome.md, terminal metadata and failure-state inspection]
- [Source: specs/desired_architecture.md, split architecture entrypoint and workflow boundaries]
- [Source: specs/architecture/primary-system-objectives.md, human engineer workflow goals and expected outputs]
- [Source: specs/architecture/agents/overview.md, engineer workflow topology and episode flow]
- [Source: specs/architecture/agents/roles.md, planner/implementer/reviewer role boundaries]
- [Source: specs/architecture/agents/handover-contracts.md, benchmark-to-engineer handoff rules and immutable asset requirements]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, filesystem ownership and read-only benchmark context]
- [Source: specs/architecture/observability.md, episode metadata lineage and traceability fields]
- [Source: specs/architecture/evals-and-gates.md, fail-closed workflow gates for solution revisions]
- [Source: specs/integration-tests.md, integration-only verification standard]
- \[Source: controller/api/main.py, `/agent/run` episode bootstrap and `EpisodeMetadata` linkage\]
- [Source: controller/api/tasks.py, benchmark asset copy into engineer workspace and status progression]
- \[Source: controller/api/routes/episodes.py, episode detail payload and `metadata_vars` exposure\]
- [Source: frontend/src/components/workspace/ChatWindow.tsx, existing failure banner and workspace actions]
- [Source: frontend/src/components/layout/Sidebar.tsx, episode history and navigation]
- [Source: frontend/src/components/workspace/UnifiedGeneratorView.tsx, hidden debug payload used by tests]
- [Source: tests/integration/architecture_p1/test_engineering_loop.py, engineer flow evidence and lineage assertions]
- [Source: tests/integration/frontend/p0/test_frontend_p0.py, existing Playwright workspace integration patterns]
- [Source: tests/integration/mock_responses/, deterministic scenario coverage when needed]

## Dev Agent Record

### Agent Model Used

GPT-5.4

### Debug Log References

- `logs/integration_tests/runs/run_20260323_080904/controller_errors.log`
- `logs/integration_tests/runs/run_20260323_080904/session*/worker-light.log`
- `logs/integration_tests/runs/run_20260323_081723/controller_errors.log`

### Completion Notes List

- Added a shared `RevisionLineageSummary` workspace component and mounted it in the sidebar.
- Added retry controls and failure-summary hooks in `ChatWindow`, plus hidden lineage/debug payloads in `UnifiedGeneratorView`.
- Added integration coverage for retry lineage in both `tests/integration/architecture_p1/test_engineering_loop.py` and `tests/integration/frontend/p0/test_int_205.py`.
- The browser test now seeds failed engineer episodes deterministically through the test-episode API and verifies the retry request metadata by intercepting the POST payload.
- Removed the temporary `INT-205` mock-response file after the deterministic test path no longer needed a dedicated transcript.

### File List

- \_bmad-output/implementation-artifacts/2-3-revise-and-retry-against-the-same-benchmark.md
- frontend/src/components/layout/Sidebar.tsx
- frontend/src/components/workspace/ChatWindow.tsx
- frontend/src/components/workspace/RevisionLineageSummary.tsx
- frontend/src/components/workspace/UnifiedGeneratorView.tsx
- specs/integration-tests.md
- tests/integration/architecture_p1/test_engineering_loop.py
- tests/integration/frontend/p0/test_int_205.py
- tests/integration/mock_responses/INT-205.yaml
