# Story 4.3: Replay Failed Episodes and Surface Deterministic Failures

Status: ready-for-dev

## Story

As a maintainer, I want failed episodes to replay from persisted artifacts and traces so that I can reproduce and diagnose failures without rerunning the solver or depending on live workspace state.

## Acceptance Criteria

1. Given a failed episode, when replay mode loads it, then the replay bundle reconstructs the failure state using persisted artifacts only.
2. Given a terminal episode, when I inspect the replay payload, then the terminal reason, failure classification, and explicit fallback or unsupported-feature mismatch are surfaced from persisted metadata and trace records rather than inferred from generic `FAILED` status alone.
3. Given missing artifacts, stale manifests, unsupported mechanisms, or missing review evidence, when replay runs, then it fails closed with a concrete reason and does not return a success-like replay bundle.
4. Given the same episode is replayed twice without artifact changes, when the replay bundle is resolved, then it returns the same episode/session linkage and artifact hashes instead of fabricating new runtime state.
5. Given replayable evidence exists, when I inspect the replay payload, then it includes the latest-revision `validation_results.json`, `simulation_result.json`, review manifest(s), and the trace IDs for the relevant simulation, review, and entry-validation events.

## Tasks / Subtasks

- [ ] Define the replay response contract and strict schema models. (AC: 1-5)
  - [ ] Add typed replay bundle models in `shared/models/schemas.py` and `controller/api/schemas.py` instead of using ad hoc dict payloads.
  - [ ] Carry `user_session_id`, `episode_id`, `simulation_run_id`, `cots_query_id`, `review_id`, `terminal_reason`, `failure_class`, `detailed_status`, `validation_logs`, and `additional_info.entry_validation` through the replay response.
  - [ ] Reuse `EpisodeMetadata`, `TraceMetadata`, `EntryValidationContext`, `ValidationResultRecord`, `SimulationResult`, `ReviewManifest`, and `ReviewDecisionEvent` where those structures already exist.
- [ ] Implement the backend replay surface from persisted state. (AC: 1-4)
  - [ ] Extend `controller/api/routes/episodes.py` with a read-only `GET /episodes/{episode_id}/replay` endpoint, or extract a tiny dedicated replay route only if separation materially improves clarity.
  - [ ] Load the `Episode` row, trace rows, and worker-session artifacts from the persisted workspace/object-storage boundary rather than from live solver state.
  - [ ] Fail closed when `validation_results.json`, `simulation_result.json`, required review manifests, or required evidence files are missing, invalid, stale, or not aligned to the latest revision.
  - [ ] Do not assemble replay data from `list_episodes()` normalization, `EpisodeListItem`, or other UI/read-model projections.
- [ ] Surface deterministic failure classification and replay diagnostics. (AC: 2-4)
  - [ ] Map `EpisodeMetadata.terminal_reason`, `failure_class`, `detailed_status`, `validation_logs`, and `additional_info.entry_validation` into the replay payload.
  - [ ] Preserve explicit fallback or unsupported-feature mismatch signals as machine-readable fields or reason strings.
  - [ ] Keep replay read-only; do not mutate episode state or create a new episode/revision while reconstructing the bundle.
- [ ] Add integration coverage for replay reconstruction and fail-closed behavior. (AC: 1-5)
  - [ ] Add a dedicated `tests/integration/architecture_p1/test_episode_replay.py` slice, or extend the nearest episode/observability integration file only if that keeps the coverage focused.
  - [ ] Cover a successful reconstruction of a known failed episode from persisted artifacts and traces.
  - [ ] Cover fail-closed paths for missing artifacts, stale manifests, unsupported mechanisms, and missing review evidence.
  - [ ] Assert against HTTP responses, persisted episode metadata, traces, and artifact contents only.

## Dev Notes

- Epic 4, Story 4.3 is the source of truth for replaying failed episodes.
- Story 4.1 already established immutable run/release bundle persistence, and Story 4.2 established dataset export boundaries. Replay must consume those persisted artifacts, not create a second bundle family or a hidden export path.
- The controller already persists the core failure signals needed for replay: `EpisodeMetadata.terminal_reason`, `EpisodeMetadata.failure_class`, `EpisodeMetadata.detailed_status`, `EpisodeMetadata.validation_logs`, trace IDs, and the `additional_info.entry_validation` block used by node-entry validation.
- Use the persisted `Episode`, `Trace`, and asset records plus the worker-session file surface (`validation_results.json`, `simulation_result.json`, review manifests, `renders/`, `plan.md`, `todo.md`, `journal.md`) as the replay source.
- `events.jsonl` and Postgres are observability/replay inputs; replay must not depend on a live solver process or on undocumented controller memory.
- The replay surface should treat missing artifacts, invalid schemas, stale manifests, or missing review evidence as terminal, deterministic failures rather than auto-healed success.
- Do not infer replay success from a generic `FAILED` status. The failure summary must come from persisted terminal metadata, traces, and artifact evidence.
- Keep the replay path backend-first. Epic 5 owns any future browser UI for replay inspection, but this story should stay in the controller/data layer.
- `controller/api/routes/episodes.py` already exposes the canonical episode read surface; reuse it as the starting point rather than introducing a parallel read model.
- `controller/agent/benchmark/graph.py` and `controller/api/tasks.py` already populate terminal metadata and entry-validation context. Replay should read those persisted fields rather than re-computing them from scratch.

### Source tree components to touch

- `controller/api/routes/episodes.py`
- `controller/api/schemas.py`
- `shared/models/schemas.py`
- `tests/integration/architecture_p1/test_episode_replay.py`
- `controller/api/main.py` only if the replay endpoint is extracted into a new route module

### Testing standards summary

- Use integration tests only; do not add unit-test-only coverage for this story.
- Drive replay over HTTP against the live controller and the persisted worker-session artifact boundary.
- Assert against persisted episode metadata, trace rows, worker-session artifacts, and fail-closed HTTP responses.
- Keep the replay assertions deterministic and replay-state focused; do not couple them to implementation-only helper calls.

### Project Structure Notes

- Keep replay inside the existing controller + worker artifact pipeline. Do not introduce a separate replay database, a second run manifest store, or a frontend-only approximation of failure state.
- Reuse the strict Pydantic schema pattern already used by the controller and shared models. Unknown or inferred fields should fail closed.
- The implementation should prefer raw persisted episode artifacts over any normalized list response that rewrites plan or journal text for UI convenience.
- If a new endpoint module is extracted, keep it small and register it through the existing FastAPI router pattern in `controller/api/main.py`.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Epic 4: Dataset Export & Replay and Story 4.3: Replay Failed Episodes and Surface Deterministic Failures]
- [Source: \_bmad-output/planning-artifacts/prd.md, FR25-FR35 and the replayability / deterministic terminal reason requirements]
- [Source: specs/desired_architecture.md, architecture index for agents, observability, simulation, and eval gates]
- [Source: specs/architecture/primary-system-objectives.md, product-level benchmark and dataset goals]
- [Source: specs/architecture/agents/overview.md, backend agent graph responsibilities and review split]
- [Source: specs/architecture/agents/roles.md, role responsibilities and failure/review expectations]
- [Source: specs/architecture/agents/handover-contracts.md, strict handoff contracts and fail-closed review routing]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, artifact ownership, read-only workspace contracts, and replay-adjacent path rules]
- [Source: specs/architecture/agents/tools.md, worker/tool boundaries and submission gates]
- [Source: specs/architecture/distributed-execution.md, controller/worker split and heavy-path routing]
- [Source: specs/architecture/simulation-and-dod.md, failure taxonomy, validation-preview split, and replayability assumptions]
- [Source: specs/architecture/CAD-and-other-infra.md, render artifacts, benchmark/replay evidence, and strict schema assumptions]
- [Source: specs/architecture/observability.md, joinable IDs, event families, terminal reason, failure class, and replay support requirements]
- [Source: specs/architecture/evals-and-gates.md, fail-closed validation, deterministic terminal failure, and replay-oriented eval expectations]
- [Source: specs/architecture/spec-reviews/round-1.md, immutable run/release bundle and replayable manifest gap]
- [Source: docs/backend-reference.md, backend topology, handoff rules, observability, and replay contract summary]
- \[Source: shared/models/schemas.py, `EpisodeMetadata`, `TraceMetadata`, `EntryValidationContext`, and `ReviewResult`\]
- \[Source: shared/models/simulation.py, `SimulationResult` and structured failure payloads\]
- \[Source: shared/observability/schemas.py, `ReviewDecisionEvent`, `NodeEntryValidationFailedEvent`, and media/trace event models\]
- \[Source: shared/workers/schema.py, `ValidationResultRecord`, `ReviewManifest`, and worker artifact response contracts\]
- [Source: controller/api/routes/episodes.py, existing episode read surface and normalization behavior to avoid for replay assembly]
- [Source: controller/api/schemas.py, controller response model patterns for typed replay payloads]
- \[Source: controller/persistence/models.py, persisted `Episode`, `Trace`, and `Asset` records\]
- [Source: controller/agent/benchmark/graph.py, terminal metadata persistence and entry-validation context recording]
- [Source: controller/api/tasks.py, terminal episode updates and failure-class assignment]
- [Source: controller/clients/worker.py, artifact sync path for persisted validation/simulation/review files]
- [Source: worker_heavy/utils/handover.py, latest-revision handoff manifest persistence and artifact naming]
- [Source: worker_heavy/api/routes.py, persisted validation/simulation artifact materialization]
- [Source: tests/integration/architecture_p0/test_int_071_to_073.py, observability linkage and persisted metadata assertions]
- [Source: tests/integration/architecture_p0/test_observability.py, episode failure-state persistence patterns]
- [Source: tests/integration/architecture_p1/test_reviewer_evidence.py, review evidence and trace inspection patterns]
- [Source: tests/integration/architecture_p1/test_benchmark_workflow.py, latest-revision review-manifest assertions]
- [Source: tests/integration/architecture_p1/test_engineering_loop.py, latest-revision engineering execution evidence]
- [Source: tests/integration/architecture_p1/test_handover.py, persisted handoff bundle expectations]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
