---
title: Current Role JSON Source of Truth
status: investigation
agents_affected:
  - benchmark_planner
  - benchmark_plan_reviewer
  - benchmark_coder
  - benchmark_reviewer
  - engineer_planner
  - engineer_plan_reviewer
  - engineer_coder
  - engineer_execution_reviewer
  - electronics_planner
  - electronics_reviewer
added_at: '2026-04-09T07:41:41Z'
---

# Current Role JSON Source of Truth

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration makes `.manifests/current_role.json` the single source of
truth for the role that is actively running in a workspace. The file replaces
role guessing from `AGENT_NAME`, prompt markers, or workspace filename
presence in the CLI-backend submission helpers and in node-entry validation.

The target architecture is described in
[agent-harness.md](../../architecture/agents/agent-harness.md),
[artifacts-and-filesystem.md](../../architecture/agents/artifacts-and-filesystem.md),
[tools.md](../../architecture/agents/tools.md), and
[handover-contracts.md](../../architecture/agents/handover-contracts.md).
The same contract also affects the eval materialization path described in
[desired_architecture.md](../../desired_architecture.md).

The intended steady state is simple: if a workspace is currently running as
`benchmark_planner`, `engineer_coder`, `benchmark_reviewer`, or any other
supported role, `.manifests/current_role.json` names that role directly. When
a workspace directory is copied to a new node, the controller or workspace
materializer rewrites the file before the next node starts so the file always
describes the role that is about to run, not the role that last touched the
workspace.

## Problem Statement

The current role contract is inferred from a mix of environment variables,
prompt text, and file-name heuristics:

1. CLI submission helpers still inspect `AGENT_NAME`, `prompt.md`, and the
   presence of benchmark or engineer filenames to guess the active role.
2. Shared runtime helpers do the same inference again so preview, review, and
   submission paths stay in sync.
3. Node-entry validation has the target node, but it does not consume a
   durable role manifest that it can compare against the entered node.
4. A workspace directory can contain both benchmark and engineer artifacts at
   different times, so file presence alone is not a reliable role signal.
5. Copying a workspace to another node can preserve stale filenames and stale
   environment state even though the active node has changed.

That behavior is brittle and duplicates the same decision in several places.
The result is exactly the failure mode you observed: a benchmark workspace can
be misclassified as engineer-shaped when the fallback logic sees the wrong
combination of files.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `shared/utils/agent/__init__.py` | `_script_agent_role()` falls back to `AGENT_NAME` and workspace file presence; `_default_reviewer_stage()` repeats the same role guess. | The shared helper layer should read one authoritative role file instead of re-deriving the current role from workspace shape. |
| `shared/agent_templates/codex/scripts/submit_plan.py` | Planner role detection uses `AGENT_NAME`, `prompt.md` markers, and plan or handoff filenames. | Planner submission should read the active role from `.manifests/current_role.json` and fail closed when the file is missing or malformed. |
| `shared/agent_templates/codex/scripts/submit_review.py` | Reviewer role detection uses `AGENT_NAME` and reviewer-manifest presence. | Reviewer submission should use the current-role file as the source of truth, not manifest sniffing. |
| `shared/agent_templates/codex/scripts/submit_for_review.py` and `worker_heavy/utils/handover.py` | Coder submission and backend handover still infer role from benchmark or engineer assembly files and from legacy `plan.md` fallback behavior. | Submission routing must stop guessing from file layout and instead trust the active role file. |
| `worker_heavy/utils/validation.py`, `worker_heavy/utils/preview.py`, `worker_heavy/utils/rendering.py` | Preview and validation helpers still read `AGENT_NAME` or script names to decide which role-specific rendering path to use. | These helpers need the same authoritative role source so they do not drift from the submission helpers. |
| `controller/agent/node_entry_validation.py` | Entry validation knows the target node but does not yet compare it to a durable current-role manifest. | Node entry should fail closed when the recorded role does not match the node being entered. |
| `controller/agent/initialization.py`, `controller/agent/benchmark/graph.py`, `controller/middleware/remote_fs.py`, `controller/clients/worker.py` | Controller-side initialization and transition plumbing currently communicate role through headers, env, and workspace shape. | These transition points must refresh `.manifests/current_role.json` whenever the active node changes. |
| `evals/logic/codex_workspace.py`, `evals/logic/specs.py`, `shared/eval_artifacts.py` | Eval workspaces and seed contracts do not materialize a current-role artifact. | Seeded workspaces and eval contracts need to match the runtime contract exactly. |
| `tests/integration/architecture_p0/test_codex_runner_mode.py`, `tests/integration/architecture_p0/test_node_entry_validation.py`, `tests/integration/architecture_p0/test_planner_gates.py`, `tests/integration/architecture_p0/test_int_188_validation_preview.py`, `tests/integration/architecture_p1/test_benchmark_workflow.py`, `tests/integration/architecture_p1/test_engineering_loop.py`, `tests/integration/architecture_p1/test_episode_replay.py`, `tests/integration/mock_responses/**`, `dataset/data/seed/role_based/**` | The integration surface still asserts the old inference behavior and omits the current-role artifact from several fixtures. | The migration must refresh the visible contract in tests, mock responses, and seeded eval rows. |

## Proposed Target State

1. Every materialized workspace contains `.manifests/current_role.json` as a
   backend-owned artifact.
2. The file contains the active role directly. The minimum contract is a JSON
   object with one required string field, `agent_name`, whose value is an
   `AgentName` enum string such as `benchmark_coder` or `engineer_planner`.
3. CLI submission helpers and shared runtime helpers read
   `.manifests/current_role.json` first and do not infer the current role
   from `AGENT_NAME`, prompt markers, plan filenames, assembly filenames, or
   reviewer-manifest presence.
4. Node-entry validation compares the recorded role against the node that is
   about to run and rejects the entry when the roles do not match.
5. When a workspace is handed from one node to another, the backend rewrites
   `.manifests/current_role.json` at the transition boundary so copied
   directories cannot carry stale role state.
6. The file remains system-owned metadata under `.manifests/**` and is not
   writable by LLM agent roles.
7. Seeded evals, controller-backed sessions, and replay material all use the
   same role file so local CLI runs and controller-managed runs agree on the
   current role contract.

## Required Work

### 1. Add the role artifact contract

- Define `.manifests/current_role.json` as a first-class system-owned
  metadata artifact in the filesystem and artifact-contract docs.
- Add a narrow artifact-criteria doc for the file under
  `specs/architecture/agents/agent-artifacts/`.
- State the file schema explicitly and keep it minimal.
- Treat a missing, malformed, or stale current-role file as a contract
  failure, not as a hint to infer the role some other way.

### 2. Materialize the file in workspace setup

- Write `.manifests/current_role.json` during CLI workspace materialization in
  `evals/logic/codex_workspace.py`.
- Write or refresh the same file in controller-backed initialization paths in
  `controller/agent/initialization.py` and the benchmark graph bootstrap
  points that prepare a node workspace.
- Ensure any workspace reuse path rewrites the file when the active node
  changes.
- Keep the file in the backend-owned `.manifests/` bucket rather than in a
  role-authored workspace path.

### 3. Route role inference through the file

- Update `shared/utils/agent/__init__.py` so the role helper reads
  `.manifests/current_role.json` instead of guessing from `AGENT_NAME` or
  workspace filenames.
- Update `shared/agent_templates/codex/scripts/submit_plan.py`,
  `shared/agent_templates/codex/scripts/submit_review.py`, and
  `shared/agent_templates/codex/scripts/submit_for_review.py` to read the
  role file and fail closed when it is absent or malformed.
- Update `worker_heavy/utils/handover.py`, `worker_heavy/utils/validation.py`,
  `worker_heavy/utils/preview.py`, and `worker_heavy/utils/rendering.py` so
  preview, validation, and handover behavior stays aligned with the same
  current-role source.
- Remove the filename and prompt-marker heuristics instead of keeping them as
  hidden compatibility paths.

### 4. Enforce role matching at node entry

- Update `controller/agent/node_entry_validation.py` so the current role file
  is compared with the node being entered.
- Fail closed when the node role and current-role manifest disagree.
- Make stale copied workspaces invalid until the backend refreshes the file
  for the next node.
- Keep the node-entry check authoritative even when the workspace already
  contains files for multiple roles.

### 5. Refresh evals, tests, and fixtures

- Update `evals/logic/specs.py` and `shared/eval_artifacts.py` so seeded
  workspaces and eval-side contracts include the current-role artifact.
- Update the integration tests that assert materialized workspace contents or
  role inference.
- Refresh `tests/integration/mock_responses/**` and
  `dataset/data/seed/role_based/**` so they include the role file and no
  longer depend on filename sniffing.
- Add a regression case for a mixed benchmark/engineer workspace where the
  current-role file must win over file presence.

### 6. Update docs and role guidance

- Update `specs/architecture/agents/artifacts-and-filesystem.md` to record
  the new system-owned metadata artifact.
- Update `specs/architecture/agents/agent-harness.md` so the CLI-provider
  contract names the current-role file as the source of truth.
- Update `specs/architecture/agents/tools.md` so the submission helpers are
  described as file-driven rather than filename-driven.
- Update `specs/architecture/agents/handover-contracts.md` and the role
  sheets under `specs/architecture/agents/roles-detailed/` so they no longer
  describe role inference through workspace shape.

## Non-Goals

- Do not rename `submit_plan`, `submit_review`, or `submit_for_review`.
- Do not change reviewer-manifest filenames or handover-manifest filenames.
- Do not rename planner artifacts as part of this migration.
- Do not broaden `.manifests/**` write access to LLM agent roles.
- Do not keep `AGENT_NAME`, prompt markers, or filename heuristics as a
  fallback contract once the current-role file is in place.
- Do not turn this into a prompt-management rewrite or a workspace-template
  rename.

## Risks

1. A copied workspace can carry a stale current-role file if the backend does
   not refresh it at the handoff boundary.
2. Mixed benchmark and engineer artifacts can still exist in one workspace
   during transitions, so the node-entry check must be based on the role file
   rather than on the visible file set.
3. If any helper keeps a fallback heuristic, the migration remains brittle and
   the source-of-truth contract is not actually fixed.

## Sequencing

The safe order is:

1. Define the artifact contract and add the writer in workspace
   materialization paths.
2. Refresh the current-role file in controller-managed handoff and node-entry
   setup paths.
3. Route shared helper inference and submission logic through the file.
4. Update eval contracts, seed materialization, mock responses, and
   integration tests.
5. Remove the old fallback heuristics only after the new file is written in
   every supported workspace path.

## Acceptance Criteria

1. Every supported workspace materialization path writes
   `.manifests/current_role.json`.
2. The file value matches the node role that is actually running.
3. Node-entry validation rejects missing, malformed, or mismatched role
   metadata before the node starts.
4. CLI submission helpers no longer infer the current role from `AGENT_NAME`,
   prompt markers, plan filenames, assembly filenames, or reviewer-manifest
   presence.
5. A workspace copied from one role to another is only valid after the
   backend refreshes the current-role file for the new node.
6. Representative integration tests and seeded eval fixtures assert the
   current-role file explicitly and no longer rely on role sniffing.

## Migration Checklist

### Artifact contract

- [ ] Define the `.manifests/current_role.json` schema.
- [ ] Add an artifact-criteria doc for `current_role.json`.
- [ ] Update `specs/architecture/agents/artifacts-and-filesystem.md` to list
  the artifact as system-owned metadata.

### Workspace materialization

- [ ] Write `current_role.json` in `evals/logic/codex_workspace.py`.
- [ ] Refresh `current_role.json` in controller-backed initialization and
  node-transition paths.
- [ ] Ensure copied workspaces are rewritten before the next node starts.

### Role routing

- [ ] Update `shared/utils/agent/__init__.py` to read the file first.
- [ ] Update `shared/agent_templates/codex/scripts/submit_plan.py`.
- [ ] Update `shared/agent_templates/codex/scripts/submit_review.py`.
- [ ] Update `shared/agent_templates/codex/scripts/submit_for_review.py`.
- [ ] Update `worker_heavy/utils/handover.py`.
- [ ] Update `worker_heavy/utils/validation.py`.
- [ ] Update `worker_heavy/utils/preview.py`.
- [ ] Update `worker_heavy/utils/rendering.py`.
- [ ] Remove the old filename and prompt-marker heuristics.

### Validation and evals

- [ ] Update `controller/agent/node_entry_validation.py` to enforce the role
  match.
- [ ] Update `evals/logic/specs.py` and `shared/eval_artifacts.py`.
- [ ] Update the role-sensitive integration tests in
  `tests/integration/architecture_p0/` and `tests/integration/architecture_p1/`.
- [ ] Refresh `tests/integration/mock_responses/**` and
  `dataset/data/seed/role_based/**`.
- [ ] Add a mixed-workspace regression test that proves the role file wins
  over file presence.

### Docs

- [ ] Update the agent-harness, tools, handover-contracts, and role-sheet
  docs.

## Test Impact

The highest-impact test surfaces are the ones that currently assert workspace
contents or role inference:

- `tests/integration/architecture_p0/test_codex_runner_mode.py`
- `tests/integration/architecture_p0/test_node_entry_validation.py`
- `tests/integration/architecture_p0/test_planner_gates.py`
- `tests/integration/architecture_p0/test_int_188_validation_preview.py`
- `tests/integration/architecture_p1/test_benchmark_workflow.py`
- `tests/integration/architecture_p1/test_engineering_loop.py`
- `tests/integration/architecture_p1/test_episode_replay.py`
- `tests/integration/architecture_p1/test_reviewer_evidence.py`

The mock-response corpus and seeded role-based datasets will also need the new
file in the workspace snapshots so the tests reflect the real contract.

## Seed and Fixture Updates

1. Add `.manifests/current_role.json` to the role-based seed workspaces.
2. Update mock-response entries that currently depend on `AGENT_NAME` or file
   presence to indicate the active role explicitly.
3. Ensure copied benchmark-to-engineer transitions rewrite the file instead
   of reusing the previous node's value.
4. Keep the fixture content minimal so the new role artifact stays easy to
   diff and reason about.

## File-Level Change Set

The smallest realistic file set for this migration is:

- `evals/logic/codex_workspace.py`
- `evals/logic/specs.py`
- `shared/eval_artifacts.py`
- `shared/utils/agent/__init__.py`
- `shared/agent_templates/codex/scripts/submit_plan.py`
- `shared/agent_templates/codex/scripts/submit_review.py`
- `shared/agent_templates/codex/scripts/submit_for_review.py`
- `worker_heavy/utils/handover.py`
- `worker_heavy/utils/validation.py`
- `worker_heavy/utils/preview.py`
- `worker_heavy/utils/rendering.py`
- `controller/agent/node_entry_validation.py`
- `controller/agent/initialization.py`
- `controller/agent/benchmark/graph.py`
- `controller/middleware/remote_fs.py`
- `controller/clients/worker.py`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/agents/tools.md`
- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/agent-artifacts/README.md`
- `specs/architecture/agents/agent-artifacts/current_role_json_acceptance_criteria.md`
- `specs/devtools.md`
- `tests/integration/architecture_p0/test_codex_runner_mode.py`
- `tests/integration/architecture_p0/test_node_entry_validation.py`
- `tests/integration/architecture_p0/test_planner_gates.py`
- `tests/integration/architecture_p0/test_int_188_validation_preview.py`
- `tests/integration/architecture_p1/test_benchmark_workflow.py`
- `tests/integration/architecture_p1/test_engineering_loop.py`
- `tests/integration/architecture_p1/test_episode_replay.py`
- `tests/integration/architecture_p1/test_reviewer_evidence.py`
- `tests/integration/mock_responses/**`
- `dataset/data/seed/role_based/**`
