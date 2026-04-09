---
title: Role Scoped Submission Tool Split
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
added_at: '2026-04-09T08:17:50Z'
---

# Role Scoped Submission Tool Split

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration splits the public completion-tool names used by the benchmark
and engineering-side graphs, including electronics roles, so the code no
longer presents one shared
`submit_plan()`, `validate()`, `simulate()`, or `submit_for_review()` surface
for both workflows.

The migration assumes the current-role migration has landed first:
[current-role-json-source-of-truth.md](./current-role-json-source-of-truth.md)
must already make `.manifests/current_role.json` the authoritative active-role
source. The role-scoped helpers in this migration must fail closed when that
file is missing, malformed, or names a different role.

The naming contract in this migration uses `benchmark` and `engineering` as
the canonical public graph labels. `solution` remains the helper suffix and
file-name compatibility concept for the engineering-side submission path
(`submit_solution_for_review()`), not the public graph label.

This migration is intentionally narrower than a full contract rewrite. The
shared validation, simulation, and handover logic can stay shared internally.
What changes is the public function name, the workspace helper name, the
controller/worker route label, and the prompt/docs surface that teaches the
agent which helper to call.

## Problem Statement

The current surface has three separate problems:

1. Planner completion is shared under one `submit_plan()` name even though the
   benchmark and engineering graphs already have different planner contracts.
2. Coder completion helpers still expose generic names through the shared
   submission import surface, so the role split is hidden from the code that
   an agent actually sees.
3. Engineer-side simulation and handover still post through `/benchmark/*`
   transport paths, so benchmark terminology leaks into the engineering path.

That shape is confusing and it is easy to misread as "the benchmark coder is
solving the benchmark" instead of "the role is verifying and handing off the
current graph's work product". The role boundary should be explicit in the
symbol names and route names, not only in the backend branch logic.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `controller/agent/tools.py` | Engineering/electronics planner tools export a nested `submit_plan()` helper. | The planner completion gate should be role-scoped so the public tool name matches the graph. |
| `controller/agent/benchmark/tools.py` | Benchmark planner tools also export a nested `submit_plan()` helper. | The benchmark planner path needs its own public name, not a shared one. |
| `shared/utils/agent/__init__.py` | The shared import surface exports role-scoped helpers (`validate_benchmark`, `simulate_benchmark`, `submit_benchmark_for_review`, `validate_engineering`, `simulate_engineering`, `submit_solution_for_review`) plus generic aliases. | The public script import surface must advertise role-scoped names instead of one shared set. |
| `utils/submission.py` | The agent-facing compatibility import surface re-exports the same role-scoped helpers plus the legacy generic aliases. | The public script import surface must advertise role-scoped names instead of one shared set. |
| `shared/agent_templates/codex/scripts/submit_plan.py` | Planner submission is current-role aware, but the exported helper name is still generic. | The helper needs a role-scoped public entrypoint. |
| `shared/agent_templates/codex/scripts/submit_for_review.py` | Coder handoff is current-role aware and dispatches to the role-scoped benchmark or solution review helper. | Benchmark and engineering coders need distinct names in the public script contract. |
| `controller/clients/worker.py` | `validate()`, `simulate()`, and `submit()` still post through benchmark-branded worker routes. | Engineer-side transport should stop using benchmark terminology. |
| `worker_heavy/api/routes.py` | The worker exposes `/benchmark/validate`, `/benchmark/simulate`, and `/benchmark/submit`. | The endpoint labels should follow the graph label used by the caller. |
| `controller/middleware/remote_fs.py` | The controller-side Temporal-backed path also drives benchmark-branded validation and simulation calls. | The controller middleware should follow the same role-scoped route split as the client. |
| `evals/logic/codex_workspace.py` | Workspace materialization still copies generic helper script names. | New workspaces should expose role-scoped helper names directly. |
| `config/prompts.yaml` | Prompt text still teaches `submit_plan()`, `simulate()`, and `submit_for_review()` as shared names. | The prompt surface should teach the role-scoped helper names only. |
| `specs/architecture/agents/tools.md` | The canonical tool docs still describe generic submission helpers. | The docs need to match the renamed public helpers and route labels. |
| `specs/architecture/agents/agent-harness.md` | The workspace contract still names generic shell helpers. | The harness docs must teach the role-scoped shell names. |
| `docs/backend-reference.md` | The backend reference still mentions `submit_plan()` as a shared planner gate. | The public reference should track the role split. |
| `specs/integration-test-list.md` and seeded fixtures | The integration catalog and mock responses still assert the generic names. | The visible contract must be refreshed so the new names are not only runtime aliases. |

## File-Level Change Set

| File group | Change needed |
| -- | -- |
| `controller/agent/tools.py`, `controller/agent/benchmark/tools.py` | Rename the nested planner completion tool to the role-scoped public name and keep the shared implementation intact. |
| `shared/utils/agent/__init__.py`, `utils/submission.py` | Add role-scoped public helpers for validate, simulate, and coder handover; keep generic aliases only as compatibility paths. |
| `shared/agent_templates/codex/scripts/submit_plan.py`, `shared/agent_templates/codex/scripts/submit_for_review.py` | Export role-scoped helper names and reject wrong-role calls using the current-role manifest. |
| `shared/agent_templates/codex/scripts/submit_plan.sh`, `shared/agent_templates/codex/scripts/submit_for_review.sh`, and `evals/logic/codex_workspace.py` | Materialize role-scoped shell entrypoints in new workspaces and keep the old names only as transitional aliases. |
| `controller/clients/worker.py`, `controller/middleware/remote_fs.py`, `controller/api/routes/script_tools.py`, `worker_heavy/api/routes.py` | Split the route labels so engineer-side validation, simulation, and handover no longer traverse `/benchmark/*`. |
| `controller/agent/benchmark/graph.py`, `controller/agent/benchmark/nodes.py`, `controller/agent/nodes/base.py`, `controller/agent/review_handover.py`, `controller/agent/benchmark_handover_validation.py` | Update the tool-trace strings and handover validation messages so they name the role-scoped helpers instead of the generic ones. |
| `config/prompts.yaml`, `specs/architecture/agents/tools.md`, `specs/architecture/agents/agent-harness.md`, `docs/backend-reference.md` | Replace generic tool names with the role-scoped names in the canonical prompt and docs surface. |
| `specs/architecture/agents/artifacts-and-filesystem.md`, role sheets, integration catalog, mock responses, seed corpora | Refresh the visible contract so the new names appear in docs, fixtures, and seeded examples. |

## Proposed Target State

1. Planner public helpers are split into `submit_benchmark_plan()` and
   `submit_engineering_plan()`.
2. Coder public helpers are split into
   `validate_benchmark()`, `simulate_benchmark()`,
   `submit_benchmark_for_review()`, and the engineering-side counterparts
   `validate_engineering()`, `simulate_engineering()`,
   `submit_solution_for_review()`.
3. The backend keeps the shared implementation behind the new names. The split
   is an exported-name and route-label change first, not a rewrite of the
   validation logic itself.
4. Each role-scoped helper reads `.manifests/current_role.json` and fails
   closed when the active role does not match the helper family being called.
5. Engineer-side validation, simulation, and handover stop posting through
   `/benchmark/*` routes. The worker and controller layers use role-scoped
   route labels instead.
6. Legacy generic names remain compatibility aliases during the migration
   window only. They are not the canonical names taught to prompts or skills.
7. Reviewer tools stay generic in this first tranche unless a concrete stale
   benchmark/engineering leak appears in the reviewer path.
8. The transport labels, shell script names, prompt text, docs, fixtures, and
   controller traces all point at the same role-scoped names so the public
   surface is self-consistent.

## Required Work

### 1. Split the planner completion gate

- Rename the controller-side planner tool exports from `submit_plan()` to
  `submit_benchmark_plan()` and `submit_engineering_plan()`.
- Keep the existing planner behavior, validation rules, and manifest writes
  unchanged.
- Keep a compatibility alias only if needed for old traces or seeded fixtures.
- Update the planner prompt text and handover docs so they advertise the new
  role-scoped name, not the generic one.

### 2. Split the coder submission helpers

- Add role-scoped public helpers in `shared/utils/agent/__init__.py` and
  `utils/submission.py`.
- Make the canonical public import surface role-scoped instead of generic.
- Preserve the shared implementation internally so the logic is not copied
  into two divergent branches.
- Reject a mismatched current role instead of silently dispatching to the
  other graph.
- Keep the current-role manifest as the only role selector for these
  wrappers; do not add new inference heuristics.

### 3. Split the transport labels

- Add role-scoped worker endpoints for validation, simulation, and handover
  submission.
- Stop routing engineering simulation and engineer handover through
  `/benchmark/*`.
- Update `controller/clients/worker.py`,
  `controller/middleware/remote_fs.py`,
  `controller/api/routes/script_tools.py`, and
  `worker_heavy/api/routes.py` together so the route split is end to end.
- Keep the old benchmark-branded routes as compatibility aliases until the
  new path is fully adopted by prompts, workspaces, and tests.

### 4. Rename the workspace helper entrypoints

- Update the Codex workspace templates so the helper scripts match the
  role-scoped public names.
- Update `evals/logic/codex_workspace.py` so new workspaces expose the new
  helper script names directly.
- Introduce `scripts/submit_benchmark_plan.sh` and
  `scripts/submit_engineering_plan.sh` for the planner gate. If coder
  completion stays shell-launched in this branch, add matching
  `scripts/submit_benchmark_for_review.sh` and
  `scripts/submit_solution_for_review.sh` wrappers.
- Keep the helper scripts thin; the shell layer should remain a wrapper over
  the Python implementation.
- Leave reviewer script naming alone in this tranche unless the reviewer path
  also leaks the wrong graph label.

### 5. Refresh prompts, docs, and examples

- Update `config/prompts.yaml`, `specs/architecture/agents/tools.md`,
  `specs/architecture/agents/agent-harness.md`,
  `specs/architecture/agents/artifacts-and-filesystem.md`,
  `docs/backend-reference.md`, and the role sheets so the canonical names are
  the role-scoped ones.
- Update the integration catalog, mock responses, and seed corpora to use the
  new names.
- Add an explicit regression for a mixed workspace where both benchmark and
  engineering helpers are present: the current-role file must decide which
  helper is valid, and the wrong-role helper must fail closed.

### 6. Keep the compatibility window bounded

- Keep the old generic names only long enough to migrate seeded workspaces,
  prompt text, and integration coverage.
- Remove the old names once the new names are the only ones taught to the
  agent-facing surface.
- Do not let the compatibility aliases become the new permanent contract.

## Non-Goals

- Do not rename `solution_script.py` or `benchmark_script.py` in this
  migration.
- Do not rename reviewer tools in the first tranche unless they become a real
  source of stale terminology.
- Do not change physics semantics, manifest filenames, or the planner/coder
  validation rules beyond the role-scoped exported names.
- Do not replace `engineering` with `solution` as the new public graph label.
- Do not remove the compatibility aliases before the prompts, docs, and
  seeded fixtures have moved to the new names.
- Do not broaden `.manifests/**` write access to LLM roles.

## Risks

1. The compatibility layer can hide the rename if prompts or seeded fixtures
   continue to advertise the old names.
2. If transport labels are renamed without controller-side client updates, the
   CLI-provider path and the controller-backed path will diverge.
3. If the role-scoped helpers do not read `.manifests/current_role.json`,
   the split becomes cosmetic and wrong-role calls can still succeed.
4. Leaving reviewer tools generic is acceptable only if they do not leak the
   wrong graph label into engineer workflows.

## Sequencing

The safe order is:

1. Add the role-scoped wrappers and keep the existing names as aliases.
2. Update the controller/worker route labels and the controller clients.
3. Update workspace materialization so the new script names are visible to
   the agent.
4. Refresh prompts, docs, and examples so the new names are the canonical
   contract.
5. Refresh integration coverage and seed corpora.
6. Remove the compatibility aliases only after the new names are stable.

## Cleanup Checklist

- [ ] Remove the generic wrappers from `shared/utils/agent/__init__.py` and
  `utils/submission.py` after every internal caller has been moved to the
  role-scoped helpers.
- [ ] Remove the `submit_plan` alias from the planner tool surfaces in
  `controller/agent/tools.py`, `controller/agent/benchmark/tools.py`, and
  `shared/agent_templates/codex/scripts/submit_plan.py` once the role-scoped
  planner names are the only ones taught to prompts and fixtures.
- [ ] Remove any remaining generic submission wording from
  `config/prompts.yaml`, `docs/backend-reference.md`,
  `specs/architecture/agents/tools.md`, and
  `specs/architecture/agents/agent-harness.md`.
- [ ] Update seeded workspaces and mock responses under `dataset/data/seed/`
  and `tests/integration/mock_responses/` so they call only the role-scoped
  helpers.
- [ ] Remove compatibility aliases for benchmark-branded transport paths only
  after `controller/clients/worker.py`,
  `controller/middleware/remote_fs.py`,
  `controller/api/routes/script_tools.py`, and `worker_heavy/api/routes.py`
  all agree on the final route labels.
- [ ] Verify the cleanup with the narrowest integration slice that covers the
  changed surface, then widen only if a regression points at another layer.
- [ ] Re-run the repo-wide search for `submit_plan()`, `validate()`,
  `simulate()`, and `submit_for_review()` before deleting the last alias.

## Acceptance Criteria

1. Benchmark and engineering planner tools have distinct public names.
2. Benchmark and engineering coder helpers have distinct public names.
3. Engineer-side validation, simulation, and handover no longer post through
   `/benchmark/*` in the canonical path.
4. A wrong-role call to a role-scoped helper fails closed using the current
   role manifest.
5. Workspace materialization and prompt text teach the new names, not the old
   generic ones.
6. The compatibility aliases still exist long enough to keep historical
   fixtures and seeded runs functioning.
7. The migration is compatible with the current-role source-of-truth contract
   and does not reintroduce role guessing from workspace shape or prompt
   markers.

## Resolved Questions

1. Engineer-side transport prefix: `engineering`.
   - Use `engineering` as the canonical route prefix and controller label.
   - Keep `solution` only as a legacy file-name compatibility concept.
2. Shared response schema names: remain unchanged for this migration.
   - Keep `BenchmarkToolResponse`, `PlannerSubmissionResult`, and similar
     internal schema names as compatibility-only types.
   - Do not rename response schemas just to match the new helper names.
3. Reviewer tools: do not rename them in this tranche.
   - Keep reviewer helpers generic unless a concrete stale-name leak appears.
   - Revisit reviewer naming only if the reviewer path becomes a separate
     migration need.
