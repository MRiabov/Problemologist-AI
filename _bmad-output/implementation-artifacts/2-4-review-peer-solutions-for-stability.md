# Story 2.4: Review Peer Solutions for Stability

Status: done

## Story

As a human engineer, I want to review a colleague's solution under runtime jitter and inspect the rendered CAD model and simulation preview so that I can reject solutions that only work in a lucky run.

## Acceptance Criteria

1. Given a solution has jittered evaluation results, when I review it, then the workspace exposes pass/fail across the required variations, including batch width, success count, success rate, consistency, and per-scene failures.
2. Given the solution is flaky or unstable, when I review it, then I can reject it with an explicit reason that is persisted with the review decision and the system fails closed if the stability summary is missing.
3. Given the solution is robust across the required jitter cases, when I review it, then I can approve it for the next workflow stage.
4. Given a solution under review, when I inspect it, then the rendered CAD model and simulation preview are available alongside the jittered pass/fail results.

## Tasks / Subtasks

- [x] Expose the peer-review workflow in the workspace UI.
  - [x] Add a peer-review card to `frontend/src/components/workspace/ChatWindow.tsx` for reviewable engineer episodes with stable `data-testid` hooks and explicit approve/reject actions.
  - [x] Wire review submission through `frontend/src/api/client.ts` and `frontend/src/context/EpisodeContext.tsx` to `/api/episodes/{episode_id}/review` using the existing `ReviewDecision` frontmatter contract and `review_content` payload string.
  - [x] Require a rejection reason and include the stability summary in the visible review copy so the user is not guessing why the solution was rejected.
- [x] Render the stability evidence next to the CAD/simulation preview.
  - [x] Extend `frontend/src/components/workspace/ArtifactView.tsx` and `frontend/src/components/visualization/SimulationResults.tsx` to render the batched jitter summary when `verification_result` is present in `validation_results.json`.
  - [x] Keep the existing CAD and simulation preview surfaces intact; only add the runtime-jitter summary alongside them.
- [x] Surface lineage and review context in the workspace chrome.
  - [x] Extend `frontend/src/components/workspace/UnifiedGeneratorView.tsx` with hidden debug payload fields for benchmark linkage, revision lineage, and verification summary data.
  - [x] Add benchmark/revision lineage badges in `frontend/src/components/layout/Sidebar.tsx` so peer solutions can be compared at a glance.
- [x] Add integration coverage for the review/stability flow.
  - [x] Extend `tests/integration/architecture_p1/test_reviewer_evidence.py` or `tests/integration/architecture_p1/test_engineering_loop.py` to assert the persisted stability summary and the review decision trail from a live run.
  - [x] Add a focused Playwright test under `tests/integration/frontend/p1/` (for example `test_int_206.py`) that proves the review card, CAD preview, simulation preview, and jitter summary render together and that approve/reject actions hit the review endpoint.
  - [x] Add deterministic mock-response coverage in `tests/integration/mock_responses/INT-206.yaml` only if the live stable/unstable cases cannot be driven reliably.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Story 2.1 owns benchmark-to-engineer handoff, Story 2.2 owns evidence and terminal-outcome surfaces, and Story 2.3 owns revision/retry lineage. Keep this story focused on peer stability review and do not mutate benchmark-owned assets.
  - Use the existing runtime-jitter robustness threshold from `specs/architecture/evals-and-gates.md` when deciding whether a solution is approvable. The system should not approve solutions that only pass on a lucky seed.
  - `EpisodeMetadata` already carries the lineage fields needed for this screen: `benchmark_id`, `prior_episode_id`, `is_reused`, `seed_id`, `seed_dataset`, `seed_match_method`, `generation_kind`, and `parent_seed_id`.
  - Keep render inspection fail-closed: when renders exist, approval still requires `inspect_media(...)`.
  - The existing handover/sync contract already carries `verification_result` inside `validation_results.json`; consume that field rather than introducing a second verification transport or review artifact.
  - Review decisions should use the existing `ReviewDecision` frontmatter contract and persist an explicit rejection reason.
  - The review endpoint already exists at `/api/episodes/{episode_id}/review`; reuse it rather than creating a separate review transport.
- Source tree components to touch:
  - `frontend/src/components/workspace/ChatWindow.tsx`
  - `frontend/src/components/workspace/ArtifactView.tsx`
  - `frontend/src/components/visualization/SimulationResults.tsx`
  - `frontend/src/components/workspace/UnifiedGeneratorView.tsx`
  - `frontend/src/components/layout/Sidebar.tsx`
  - `frontend/src/api/client.ts`
  - `frontend/src/context/EpisodeContext.tsx`
  - `tests/integration/architecture_p1/test_reviewer_evidence.py`
  - `tests/integration/architecture_p1/test_engineering_loop.py`
  - `tests/integration/frontend/p1/test_int_206.py`
  - `tests/integration/mock_responses/INT-206.yaml`
- Testing standards summary:
  - Use integration tests only; verify via HTTP responses, persisted episode metadata, assets, and visible workspace state.
  - Assert against the review decision and the persisted stability evidence, not internal helper calls.
  - Keep assertions on the episode payload and visible UI state rather than on brittle DOM text scraping.

### Project Structure Notes

- Keep the stability summary backwards compatible with existing `validation_results.json` consumers.
- Do not add a new evidence store if the existing episode asset sync can carry the runtime-jitter summary.
- Keep new selectors explicit and stable; add `data-testid` hooks for the review card, reason field, and stability summary rather than relying on broad text matching.
- The workspace should continue to use the current episode/asset polling model.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 2.4: Review Peer Solutions for Stability]
- [Source: \_bmad-output/planning-artifacts/prd.md, Journey 2: Mechanical Engineer Inspects Validation and Simulation]
- [Source: specs/architecture/agents/roles.md, engineering execution reviewer responsibilities and visual-inspection policy]
- [Source: specs/architecture/evals-and-gates.md, runtime-jitter robustness threshold and approval criteria]
- [Source: specs/architecture/simulation-and-dod.md, runtime randomization and batched verification behavior]
- [Source: specs/architecture/observability.md, lineage and review observability fields]
- [Source: specs/integration-tests.md, INT-021, INT-026, and INT-073]
- [Source: controller/agent/review_handover.py, latest-revision handover validation]
- [Source: controller/agent/nodes/execution_reviewer.py, reviewer decision flow and render-inspection gate]
- [Source: controller/clients/worker.py, handover artifact sync]
- [Source: controller/middleware/remote_fs.py, controller-side worker proxy and filesystem mediation]
- [Source: controller/api/routes/script_tools.py, controller script-tool proxy surface]
- [Source: worker_heavy/api/routes.py, verification response and artifact persistence path]
- \[Source: shared/workers/schema.py, `ValidationResultRecord`, `VerificationRequest`, and `MultiRunResult`\]
- [Source: shared/workers/persistence.py, validation result persistence]
- [Source: frontend/src/components/workspace/ChatWindow.tsx, failure/banner and action placement]
- [Source: frontend/src/components/workspace/ArtifactView.tsx, artifact selection and preview rendering]
- [Source: frontend/src/components/visualization/SimulationResults.tsx, simulation result rendering]
- [Source: frontend/src/components/workspace/UnifiedGeneratorView.tsx, hidden debug payload used by tests]
- [Source: frontend/src/components/layout/Sidebar.tsx, episode history and navigation]
- [Source: frontend/src/api/client.ts, frontend API wrappers]
- [Source: frontend/src/context/EpisodeContext.tsx, episode state and action orchestration]
- [Source: tests/integration/architecture_p1/test_reviewer_evidence.py, reviewer evidence assertions]
- [Source: tests/integration/architecture_p1/test_engineering_loop.py, engineer loop artifact assertions]
- [Source: tests/integration/frontend/p1/test_int_179.py, Playwright debug-hook pattern]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

- `./scripts/run_integration_tests.sh tests/integration/frontend/p1/test_int_180.py::test_int_180_peer_review_card_surfaces_stability_and_review_actions --maxfail=1`
- `./scripts/run_integration_tests.sh tests/integration/architecture_p1/test_engineering_loop.py tests/integration/architecture_p1/test_reviewer_evidence.py --maxfail=1`
- `./scripts/run_integration_tests.sh tests/integration/architecture_p1/test_reviewer_evidence.py::test_reviewer_evidence_completeness --maxfail=1`

### Completion Notes List

- Peer-review UI, review submission, stability summary rendering, and lineage chrome are present and exercised by the frontend story test.
- Live backend stability review flow is exercised by `test_engineering_full_loop`.
- `test_reviewer_evidence_completeness` still fails on an unrelated benchmark-generation regression (`worker_heavy.utils.validation: _metadata_cots_id is not defined`) outside this story's review/stability scope.

### File List

- \_bmad-output/implementation-artifacts/2-4-review-peer-solutions-for-stability.md
- \_bmad-output/implementation-artifacts/sprint-status.yaml
