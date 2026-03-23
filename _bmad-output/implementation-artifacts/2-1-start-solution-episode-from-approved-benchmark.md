# Story 2.1: Start Solution Episode from Approved Benchmark

Status: done

## Story

As a human engineer, I want to start solution work from an approved benchmark so that I can solve against the exact benchmark context that was validated, inspect the benchmark's rendered CAD model and simulation preview, and keep the benchmark package immutable during the solution run.

## Acceptance Criteria

1. Given an approved benchmark revision, when solution work starts, then the new engineer episode is linked to that benchmark via `EpisodeMetadata.benchmark_id` and the engineer workspace contains the latest approved benchmark bundle unchanged, including `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, the latest review manifests, validation or simulation evidence, and render assets from the same revision, including preview artifacts for the rendered CAD model and simulation preview.
2. Given a simple rigid-body benchmark, when solution work starts, then the engineer run enters the engineering workflow and evaluates the solution in physically correct rigid-body simulation against the same approved benchmark contract.
3. Given an unapproved, stale, or revision-mismatched benchmark bundle, when solution work starts, then the system fails closed before the engineer graph starts and records a deterministic handoff rejection reason.
4. Given benchmark-owned files in the engineer workspace, when they are consumed by planner, coder, or reviewer nodes, then they remain read-only context and are not rewritten as engineer-owned artifacts.
5. Given the benchmark episode is retried, when the engineer run is started again, then the benchmark linkage stays deterministic through the same benchmark episode ID and the copied assets still resolve to the latest approved benchmark bundle.

## Tasks / Subtasks

- [x] Extend engineer episode bootstrap in `controller/api/main.py` and `controller/api/tasks.py` so `benchmark_id` is treated as a required approved benchmark linkage for solution launches, not just a free-form metadata hint.
  - [x] Carry `EpisodeMetadata.benchmark_id` through the engineer episode record and preserve the existing `worker_session_id`, `user_session_id`, and episode lineage fields.
  - [x] Copy benchmark-owned assets into the engineer workspace as a system-owned handoff only after the benchmark revision is verified as approved and latest-revision.
  - [x] Reject stale, missing, or revision-mismatched benchmark bundles with deterministic handoff metadata and no downstream agent start.
- [x] Tighten node-entry validation in `controller/agent/node_entry_validation.py` and `controller/agent/review_handover.py` so the engineer planner cannot start from benchmark context unless the benchmark review bundle is complete and latest-revision valid.
  - [x] Keep the engineer planner contract aligned with the existing `benchmark_assembly_definition.yaml` read-only requirement.
  - [x] Reuse the existing reviewer and manifest validation helpers instead of inventing a second approval mechanism.
- [x] Preserve read-only benchmark context boundaries in the engineer graph and nodes.
  - [x] Keep benchmark files copied by the controller system path, not rewritten by the agent.
  - [x] Do not let the engineer planner, coder, or reviewers mutate benchmark-owned assets, review manifests, or render evidence in place.
- [x] Add or refresh integration coverage for benchmark-to-engineer start conditions.
  - [x] Extend `tests/integration/architecture_p1/test_handover.py` to prove the engineer receives the approved benchmark bundle intact.
  - [x] Extend `tests/integration/architecture_p1/test_engineering_loop.py` to prove the engineer flow starts from an approved benchmark and retains the benchmark ID linkage.
  - [x] Extend `tests/integration/architecture_p0/test_node_entry_validation.py` to reject unapproved or stale benchmark handoffs.
  - [x] Extend `tests/integration/architecture_p1/test_benchmark_workflow.py` if needed so the latest approved benchmark bundle is still the source of truth for engineering intake.
- [x] Add deterministic mock-response coverage in `tests/integration/mock_responses/` only if the existing integration flow cannot reach the stale or unapproved rejection path without it.
- [x] Run the integration slices for handoff, planner gating, and engineer loop before marking the story ready for implementation or done.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Epic 2 is the human solution workflow: benchmarks and solutions are separate but connected.
  - Story 1.4 already established latest-revision-only benchmark bundles and manifest-based reproducibility; reuse that contract here rather than creating another bundle format.
  - `controller/api/main.py` seeds engineer runs with `EpisodeMetadata.episode_type = EpisodeType.ENGINEER` and `metadata.worker_session_id = request.session_id`; preserve those fields and add the benchmark linkage only through validated metadata.
  - `controller/api/tasks.py` copies benchmark assets into the engineer session when `metadata_vars.benchmark_id` is present. This story should make that copy conditional on an approved latest-revision benchmark, not on a bare ID.
  - `controller/agent/node_entry_validation.py` currently requires `benchmark_assembly_definition.yaml` for `ENGINEER_PLANNER`; keep that as the read-only context baseline and add approval or revision checks there or in the task bootstrap path.
  - `controller/agent/review_handover.py` already validates review manifests against the latest script revision; reuse the same latest-revision logic for benchmark intake.
  - Do not infer approval from a benchmark episode being merely present or `COMPLETED`; the handoff must prove it came from the approved latest revision.
  - Unapproved or stale benchmark handoffs should map to the existing `HANDOFF_INVARIANT_VIOLATION` / `AGENT_SEMANTIC_FAILURE` route rather than a new rejection taxonomy.
  - The engineer workspace must consume benchmark-owned artifacts read-only. Any fixes belong in engineer-owned `plan.md`, `todo.md`, `assembly_definition.yaml`, and `script.py`.
- Source tree components to touch:
  - `controller/api/main.py`
  - `controller/api/tasks.py`
  - `controller/agent/node_entry_validation.py`
  - `controller/agent/review_handover.py`
  - `controller/agent/graph.py`
  - `controller/agent/nodes/planner.py`
  - `controller/agent/nodes/coder.py`
  - `controller/agent/nodes/plan_reviewer.py`
  - `controller/agent/nodes/execution_reviewer.py`
  - `tests/integration/architecture_p1/test_handover.py`
  - `tests/integration/architecture_p1/test_engineering_loop.py`
  - `tests/integration/architecture_p1/test_benchmark_workflow.py`
  - `tests/integration/architecture_p0/test_node_entry_validation.py`
  - `tests/integration/mock_responses/INT-033.yaml` or a new deterministic scenario if needed
- Testing standards summary:
  - Use integration tests only; verify via HTTP responses, persisted episode metadata, asset bundles, manifests, and traces.
  - Keep assertions on the benchmark bundle and episode lineage rather than on internal helper calls.
  - If render evidence is part of the handoff, any reviewer path must use `inspect_media(...)`; file listing alone is not visual inspection.
  - The engineer flow should fail closed before any agent node starts when the benchmark bundle is stale or unapproved.

### Project Structure Notes

- Keep benchmark-to-engineer handoff as a controller-owned copy and validation step, not a new agent-owned synchronization layer.
- Reuse the existing episode asset sync and manifest handling in `controller/api/tasks.py`; do not add a second artifact-copy pipeline.
- Treat `benchmark_id` as the canonical cross-workflow linkage field for the solution episode. If the benchmark is rerun, the solution episode must not silently follow the old bundle.
- The engineer run should remain compatible with the current planner, coder, and reviewer graph and the `start_node` routing rules in `controller/agent/graph.py`.
- Story 2.2 owns solution evidence inspection and terminal outcome surfaces.
- Story 2.3 owns retrying against the same benchmark without changing the problem definition.
- Story 2.4 owns peer solution stability review under runtime jitter.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 2.1: Start Solution Episode from Approved Benchmark]
- [Source: \_bmad-output/planning-artifacts/prd.md, Journey 1 and Journey 2, plus the MVP scope for the solution workflow]
- [Source: specs/architecture/agents/overview.md, engineering workflow split between planner, coder, and reviewers]
- [Source: specs/architecture/agents/handover-contracts.md, benchmark-to-engineer handoff rules and latest-revision manifest gates]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, benchmark-owned artifacts are read-only context and controller-owned copy boundaries]
- [Source: specs/architecture/CAD-and-other-infra.md, benchmark-owned fixture metadata and latest preview render contracts]
- [Source: specs/architecture/simulation-and-dod.md, benchmark context read-only contract and backend split]
- [Source: specs/architecture/observability.md, episode/session lineage and immutable bundle traceability]
- [Source: specs/integration-tests.md, INT-032, INT-033, INT-034, INT-184]
- \[Source: controller/api/main.py, `/agent/run` episode bootstrap and `EpisodeMetadata` linkage\]
- [Source: controller/api/tasks.py, benchmark asset copy into engineer workspace and status progression]
- [Source: controller/agent/node_entry_validation.py, engineer planner benchmark-context requirements]
- [Source: controller/agent/review_handover.py, latest-revision review-manifest validation]
- [Source: controller/agent/graph.py, engineer start-node routing]
- [Source: tests/integration/architecture_p1/test_handover.py, benchmark-to-engineer handoff assertions]
- [Source: tests/integration/architecture_p1/test_engineering_loop.py, full engineer loop from benchmark]
- [Source: tests/integration/architecture_p1/test_benchmark_workflow.py, benchmark bundle and manifest assertions]
- [Source: tests/integration/architecture_p0/test_node_entry_validation.py, engineer planner entry gating]
- [Source: tests/integration/mock_responses/INT-033.yaml, deterministic engineer loop scenario]

## Dev Agent Record

### Agent Model Used

GPT-5

### Debug Log References

- `controller/api/main.py`: normalized `benchmark_id` for engineer launches so approved benchmark linkage is deterministic.
- `controller/api/tasks.py`: validated approved benchmark bundles before graph start, copied benchmark-owned artifacts into the engineer workspace, and failed closed on stale or invalid bundles.
- `controller/agent/node_entry_validation.py`, `controller/agent/graph.py`, `controller/agent/review_handover.py`: tightened benchmark-handoff validation and benchmark reviewer revision checks.
- `shared/utils/agent/__init__.py`: deferred `submit_for_review()` until validation and simulation evidence exist to avoid premature backend gate errors.
- `config/prompts.yaml`: aligned planner contracts with the current benchmark/assembly schema and read-only benchmark-context rules.
- Integration reruns of `tests/integration/architecture_p1/test_engineering_loop.py` reached the harness health gate but were blocked by a startup stability failure before scenario execution.
- `worker_heavy/utils/handover.py`: stopped populating benchmark-only path fields for `engineering_execution_reviewer`, so reviewer handoff validation no longer looks for temp-workspace-only files.
- `tests/integration/mock_responses/INT-033.yaml`: added `inspect_media` calls for `engineer_planner`, `engineer_plan_reviewer`, and `engineer_execution_reviewer` against `renders/render_e45_a45.png`.
- Targeted integration rerun `logs/integration_tests/runs/run_20260323_055025/` passed `tests/integration/architecture_p1/test_handover.py::test_benchmark_to_engineer_handoff`, `tests/integration/architecture_p1/test_engineering_loop.py::test_engineering_full_loop`, and `tests/integration/architecture_p0/test_node_entry_validation.py::test_int_184_engineer_planner_rejects_stale_benchmark_bundle` in 156.01s.

### Completion Notes List

- Implemented benchmark-to-engineer bundle linkage, read-only benchmark context handling, and fail-closed approval checks.
- Added render-inspection coverage to the INT-033 mock transcript so the benchmark planner and both engineer reviewers exercise the same visible evidence path.
- Verified the fix with the targeted integration slice in `logs/integration_tests/runs/run_20260323_055025/`; all three selected tests passed and the story is ready for review.

### File List

- \_bmad-output/implementation-artifacts/2-1-start-solution-episode-from-approved-benchmark.md
- `controller/api/main.py`
- `controller/api/tasks.py`
- `controller/agent/node_entry_validation.py`
- `controller/agent/graph.py`
- `controller/agent/review_handover.py`
- `shared/utils/agent/__init__.py`
- `shared/workers/schema.py`
- `worker_heavy/utils/handover.py`
- `worker_heavy/api/routes.py`
- `config/prompts.yaml`
- `tests/integration/mock_responses/INT-033.yaml`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`

### Change Log

- 2026-03-23: Removed execution-review manifest path fields that pointed at temp-workspace-only assets and validated the benchmark-to-engineer start flow with the INT-033 render-inspection transcript.
