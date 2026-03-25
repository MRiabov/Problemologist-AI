# Story 3.4: Prefer the Simpler Valid Solution

Status: review

## Story

As a human operator, I want the system to identify unnecessary or unjustified degrees of freedom and prefer the valid candidate with the fewest moving parts, actuators, and DOFs so that the simplest workable solution is favored over an over-actuated one.

## Acceptance Criteria

1. Given two or more valid candidate solutions for the same benchmark, when the engineering planner compares them, then it prefers the candidate with fewer unnecessary degrees of freedom, actuators, and parts.
2. Given a solution with unjustified extra movement, when the plan reviewer or execution reviewer inspects it, then the reviewer can reject or flag it as over-actuated or unnecessarily complex with an explicit reason.
3. Given a solution whose motion is required by the benchmark objective or explicitly justified in the plan, when the reviewer evaluates it, then the system does not flag it as over-actuated merely because it moves.
4. Given a part with `len(dofs) > 3`, when the plan reviewer or execution reviewer evaluates the handoff, then the run fails closed unless an accepted `DOF_JUSTIFICATION_ACCEPTED` or `DOF_JUSTIFICATION:<part_id>` marker is present.
5. Given a reviewer decision, when the episode trace is persisted, then the event and checklist payload use the canonical DOF keys so downstream evals can distinguish minimality from justified deviation.

## Tasks / Subtasks

- [x] Keep `controller/agent/nodes/dof_guard.py` as the canonical deterministic over-actuation helper.
  - [x] Preserve the existing `len(dofs) > 3` threshold and the accepted justification markers (`DOF_JUSTIFICATION_ACCEPTED` and `DOF_JUSTIFICATION:<part_id>`).
  - [x] Do not introduce a second DOF-scoring heuristic or silently change the threshold without a spec update.
- [x] Tighten the engineer planner prompt so the preference is explicit.
  - [x] Update `config/prompts.yaml` so the planner is told to minimize motion complexity and choose the smallest DOF set, fewest actuators, and fewest parts necessary for a valid mechanism.
  - [x] Keep the existing warning/review split: `validate_and_price()` may warn on unusual motion complexity, but the reviewer gates remain the hard rejection path.
- [x] Keep reviewer gating and telemetry aligned.
  - [x] Ensure `controller/agent/nodes/plan_reviewer.py` and `controller/agent/nodes/execution_reviewer.py` continue to emit `excessive_dof_detected` events with stage-specific `reviewer_stage` payloads.
  - [x] Keep reviewer checklist keys aligned with the canonical contract in `specs/architecture/agents/artifacts-and-filesystem.md` and `config/reward_config.yaml` (`dof_minimality` for plan review, `dof_deviation_justified` for execution review).
  - [x] Make sure the review comments/YAML surface the reason clearly enough for downstream evals to distinguish justified motion from over-actuation.
- [x] Keep benchmark-owned motion separate from engineer-owned motion.
  - [x] Apply this story's simpler-valid-solution rule to `assembly_definition.yaml` only.
  - [x] Do not classify benchmark-owned fixture motion in `benchmark_assembly_definition.yaml` as engineer over-actuation; that file stays governed by the benchmark motion contract.
- [x] Extend integration coverage for the acceptance cases.
  - [x] Refresh `tests/integration/architecture_p0/test_int_074.py` so it proves both the unjustified rejection path and the justified-motion path.
  - [x] Extend `tests/integration/architecture_p1/test_reviewer_evidence.py` or `tests/integration/architecture_p1/test_engineering_loop.py` to assert the checklist payload and persisted review artifact reflect the canonical DOF keys.
  - [x] Keep the assertions HTTP-, trace-, and artifact-based; do not add unit-only coverage.
- [x] Run the DOF review regression slices before closing the story.
  - [x] At minimum cover `INT-074`, `INT-075`, and the reviewer-evidence slice that checks persisted checklist/event data.

## Dev Notes

- Epic 3, Story 3.4 is the source of truth for the human-facing requirement.
- The hard gate already lives in `controller/agent/nodes/dof_guard.py`, `controller/agent/nodes/plan_reviewer.py`, and `controller/agent/nodes/execution_reviewer.py`; treat that helper as canonical and keep the reviewer nodes thin.
- `worker_heavy/utils/dfm.py::validate_and_price()` currently warns on compounds with `>=4` DOFs. That warning should stay warning-only; do not move the hard rejection there unless the architecture changes.
- The canonical DOF markers are already part of the live contract: `DOF_JUSTIFICATION_ACCEPTED` and `DOF_JUSTIFICATION:<part_id>`.
- `ReviewResult` / `ReviewDecisionEvent` already carry checklist payloads. Keep `dof_minimality` and `dof_deviation_justified` stable so reward shaping and evals continue to line up.
- `config/prompts.yaml` currently has a single reviewer-scrutiny warning for `>=4` DOFs. This story should strengthen the planner instruction, not invent a different scoring system.
- `config/reward_config.yaml` already includes `benchmark_dof_minimality`, `dof_minimality`, and `dof_deviation_justified`; any wording or checklist updates should remain consistent with those keys.
- The relevant seeded over-actuation cases are already present in `tests/integration/mock_responses/INT-074.yaml` and `tests/integration/mock_responses/INT-075.yaml`.

### Project Structure Notes

- Keep the over-actuation rule in the shared guard + reviewer nodes. Do not create a parallel motion-complexity subsystem or infer over-actuation from freeform prose alone.
- Preserve the explicit warning/rejection split: planner validation may warn, reviewers enforce.
- Apply the simpler-valid-solution rule only to engineer-owned solution assemblies. Benchmark-owned fixtures and benchmark motion stay governed by the benchmark-side contract.
- Keep the event names and checklist keys stable because downstream traces, reviewer evidence checks, and reward config already depend on them.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Story 3.4: Prefer the Simpler Valid Solution]
- [Source: \_bmad-output/planning-artifacts/prd.md, FR22-FR24 and review-quality success criteria]
- [Source: specs/architecture/agents/roles.md, Engineering Planner motion-complexity guidance and reviewer responsibilities]
- \[Source: specs/architecture/agents/handover-contracts.md, deterministic DOF suspicion threshold and `DOF_JUSTIFICATION` markers\]
- [Source: specs/architecture/agents/artifacts-and-filesystem.md, plan-review and execution-review checklist examples]
- [Source: specs/architecture/evals-and-gates.md, INT-074 and INT-075 expectations for DOF rejection and over-actuation flags]
- [Source: specs/architecture/simulation-and-dod.md, benchmark-owned fixture read-only boundary and benchmark motion contract]
- [Source: specs/architecture/observability.md, checklist/event observability and metric collection]
- [Source: config/prompts.yaml, planner DOF warning line]
- \[Source: config/reward_config.yaml, `benchmark_dof_minimality`, `dof_minimality`, and `dof_deviation_justified`\]
- [Source: controller/agent/nodes/dof_guard.py, canonical deterministic DOF helper]
- [Source: controller/agent/nodes/plan_reviewer.py, excessive DOF review event emission and rejection path]
- [Source: controller/agent/nodes/execution_reviewer.py, over-actuation deviation flagging and review persistence]
- \[Source: worker_heavy/utils/dfm.py, DOF warning in `validate_and_price()`\]
- [Source: tests/integration/architecture_p0/test_int_074.py, seeded plan/execution DOF regression coverage]
- [Source: tests/integration/architecture_p1/test_reviewer_evidence.py, persisted review checklist/event evidence]
- [Source: tests/integration/mock_responses/INT-074.yaml, over-actuated plan seed]
- [Source: tests/integration/mock_responses/INT-075.yaml, over-actuated execution seed]

## Dev Agent Record

### Agent Model Used

GPT-5.4

### Debug Log References

- Tightened `controller/agent/nodes/dof_guard.py` to require standalone `DOF_JUSTIFICATION_ACCEPTED` and `DOF_JUSTIFICATION:<part_id>` marker lines instead of free substring matches.
- Made `controller/agent/nodes/plan_reviewer.py` fail closed when deterministic DOF validation cannot parse the handoff, instead of falling through to the LLM review path.
- Made `controller/agent/nodes/execution_reviewer.py` fail closed when deterministic DOF validation errors during execution review, preserving rejected review persistence.
- Extended `tests/integration/architecture_p0/test_int_074.py` with strict-marker and malformed-assembly regression coverage for the simpler-valid-solution gate.
- Verified with `./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_int_074.py::test_int_074_engineering_dof_minimization_review_gate`.
- Fixed worker-light execution env to expose `REPO_REVISION` for seeded mock scripts.
- Fixed INT-075 execution seed to use the justified `planner_link` label for the over-actuated part so the canonical DOF marker matches the reviewed part id.
- Added `seed_execution_reviewer_handover` import to `tests/integration/architecture_p1/test_reviewer_evidence.py`.
- Added re-fetch helpers for persisted review evidence in the DOF regression tests.
- Verified with `./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_int_074.py::test_int_074_engineering_dof_minimization_review_gate tests/integration/architecture_p1/test_reviewer_evidence.py::test_engineering_dof_review_evidence_uses_canonical_keys`.
- Verified with `./scripts/run_integration_tests.sh --maxfail=1 tests/integration/architecture_p0/test_int_074.py::test_int_074_engineering_dof_minimization_review_gate tests/integration/architecture_p1/test_reviewer_evidence.py::test_engineering_dof_review_evidence_uses_canonical_keys tests/integration/architecture_p1/test_reviewer_evidence.py::test_engineer_execution_reviewer_rejects_over_actuated_dofs_after_render_inspection`.
- Fixed `controller/migrations/versions/b0f5e3c1d2a4_add_dataset_row_archives.py` so the new archive table reuses the seed-lineage enum types instead of trying to recreate them during fresh integration runs.
- Seeded benchmark reviewer preview renders at `renders/cad_preview.png` and `renders/simulation_preview.png` so `inspect_media()` can attach real media during benchmark plan review.
- Aligned `tests/integration/architecture_p1/test_reviewer_evidence.py` with the benchmark review manifest contract and current-revision media inspection evidence for benchmark episodes.
- Verified with `./scripts/run_integration_tests.sh tests/integration/architecture_p1/test_reviewer_evidence.py::test_benchmark_plan_reviewer_rejection_persists_latest_revision_evidence`.
- Bound engineer execution reviewer render approval to the current revision's render manifest so unrelated images in `/renders` no longer satisfy the visual-evidence gate.
- Re-ran the DOF regression slice after the render-manifest fix and kept the targeted integration tests green.

### Completion Notes List

- The simpler-valid-solution gate now uses explicit marker-line parsing for accepted DOF justifications, so prose mentions no longer satisfy the bypass.
- Plan review and execution review now fail closed when deterministic DOF validation cannot parse the handoff, which keeps malformed assemblies out of the fallback path.
- The INT-074 regression slice now covers unjustified rejection, justified motion, strict-marker parsing, and malformed-assembly fail-closed behavior.
- Comprehensive story context assembled from epic, PRD, architecture, reviewer gate code, reward config, and seeded DOF regression coverage.
- Canonical DOF checklist keys now persist through review events and review artifacts for both the rejection and justified-motion paths.
- Reviewer-stage visual inspection now emits persisted tool traces even when the gate exits early on DOF policy checks.
- The canonical excessive-DOF event now persists for justified and rejected over-actuation cases, keeping the observability breadcrumb stable across both paths.
- Integration regression slices passed after aligning the INT-075 execution seed with the justification marker expected by the reviewer helper.
- Benchmark plan reviewer now receives canonical preview renders early enough to complete multimodal inspection and persist the benchmark review manifest.
- The benchmark reviewer evidence test now checks the recorded `media_inspection` payload and the persisted review manifest without over-constraining fields that are not present in this path.

### File List

- \_bmad-output/implementation-artifacts/3-4-prefer-the-simpler-valid-solution.md
- `controller/agent/nodes/dof_guard.py`
- `controller/agent/nodes/plan_reviewer.py`
- `controller/agent/nodes/execution_reviewer.py`
- `config/prompts.yaml`
- `tests/integration/architecture_p0/test_int_074.py`
- `controller/migrations/versions/b0f5e3c1d2a4_add_dataset_row_archives.py`
- `worker_light/runtime/executor.py`
- `tests/integration/mock_responses/INT-075.yaml`
- `tests/integration/mock_responses/INT-075/engineer_coder/entry_01/01__assembly_definition.yaml`
- `tests/integration/architecture_p0/test_int_074.py`
- `tests/integration/architecture_p1/test_reviewer_evidence.py`
- `controller/agent/benchmark/graph.py`
- `controller/agent/benchmark/nodes.py`
- `controller/middleware/remote_fs.py`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
