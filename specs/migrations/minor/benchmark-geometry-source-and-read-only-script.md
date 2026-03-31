# Benchmark Geometry Source, Split Authored Scripts, and Node Entry Validation

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration formalizes the authored source filenames that the runtime and
seed validator must understand.

The target contract is:

1. `benchmark_script.py` is the benchmark-owned, read-only geometry source.
2. `objectives_geometry()` is the canonical geometry utility for rendering and
   inspection. It takes no arguments because there is only one benchmark
   assembly-definition path per benchmark workspace.
3. `solution_script.py` is the engineer-authored source file.
4. There is no longer a user-authored, bare `script.py` in seeded workspaces.
   The authored workspace surface is the split pair of
   `benchmark_script.py` and `solution_script.py`.
5. Seed validation and node entry validation must fail closed when a benchmark
   row claims geometry but does not include `benchmark_script.py`.
6. The on-demand preview-rendering migration owns image generation. This
   migration only defines the geometry-source and authored-file naming contract.

This is a contract fix, not a convenience refactor. The benchmark package must
contain geometry that can be inspected, measured, and rendered from a visible
source file, and engineer-authored code must stop colliding with the legacy
single-file contract.

## Problem Statement

The current eval and agent flow still mixes three different concerns into the
same single-file `script.py` contract:

1. authored engineer solution code,
2. authored benchmark geometry, and
3. runtime-owned execution wrappers.

That creates ambiguity in prompts, templates, handoff checks, and validation
gates. It also lets benchmark rows look valid when they only carry YAML handoff
files and no geometry source.

The system already has a seeded node-entry validation path through
`scripts/validate_eval_seed.py` and `controller/agent/node_entry_validation.py`,
so the right fix is to make that path fail closed on missing benchmark geometry
instead of letting the legacy naming blur the contract.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `shared/agent_templates/common/script.py` | Provides the common starter file copied into workspaces as `script.py`. | This is engineer-authored code in the current contract, but the filename is ambiguous and collides with the runtime wrapper. |
| `controller/agent/nodes/coder.py` | Tells engineer coder nodes to implement the handoff in `script.py`. | Engineer-authored work needs to move to `solution_script.py`. |
| `evals/logic/codex_workspace.py` | Prompts engineer workspaces to edit `script.py` and treat it as the canonical execution path. | The prompt language must stop teaching the legacy filename as the authored source. |
| `shared/assets/template_repos/benchmark_generator/result.py` | Contains benchmark geometry assembly logic under a legacy internal name. | Benchmark geometry must be surfaced under `benchmark_script.py` instead of hidden behind `result.py`. |
| `shared/assets/template_repos/engineer/result.py` | Contains a placeholder engineer template. | It is not a stable authored-source contract and should not define the engineer file naming surface. |
| `controller/agent/benchmark/nodes.py` | Benchmark coder nodes still instruct creation of `script.py`. | Benchmark-owned geometry should be authored in `benchmark_script.py`. |
| `controller/agent/benchmark/storage.py` | Uploads benchmark geometry under `script.py`. | The storage key must match the benchmark-owned source filename. |
| `controller/agent/node_entry_validation.py` | Validates seeded handoffs and reviewer entry. | It is the right fail-closed gate for missing benchmark geometry, but it still reasons about the legacy naming shape. |
| `scripts/validate_eval_seed.py` | Runs `seed_eval_workspace()` and then `preflight_seeded_entry_contract()`. | This is the canonical seed validator, so invalid benchmark rows must be rejected here. |
| `controller/agent/handover_constants.py` | Treats `script.py` as the generic reviewer handoff file. | The contract needs to distinguish benchmark geometry, engineer solution code, and legacy runtime naming. |

## Target State

1. Every benchmark seed that is meant to be solved by engineering carries a
   read-only `benchmark_script.py` file alongside the benchmark YAML handoff
   artifacts.
2. `objectives_geometry()` materializes the objective geometry from the canonical
   benchmark definition path. The implementation may use build123d primitives
   directly or inline mesh-backed geometry inside that function, but it stays
   inside Python source rather than becoming a separate mesh-file contract.
3. Engineer-authored code lives in `solution_script.py`. Prompts, templates,
   handoff checks, and reviewer guidance should use that name instead of a bare
   `script.py`.
4. There is no user-authored bare `script.py` in the seeded workspace surface.
5. Benchmark and engineer workspaces that need to preview or inspect benchmark
   geometry receive `benchmark_script.py` as read-only context. The benchmark
   execution reviewer receives it as well.
6. Seed validation and node entry validation fail closed when a benchmark row
   claims geometry but does not carry `benchmark_script.py`. YAML-only rows are
   invalid for benchmark-backed evals.
7. The on-demand preview-rendering migration remains the owner of preview image
   generation and must consume the geometry utility from this contract instead
   of inventing a second geometry source.

## Required Work

### 1. Define `benchmark_script.py` as the benchmark-owned geometry source

- Introduce `benchmark_script.py` as the visible benchmark geometry file in the
  benchmark handoff surface and in seeded benchmark workspaces.
- Treat it as read-only context for downstream engineer roles and the benchmark
  execution reviewer.
- Use `objectives_geometry()` as the canonical geometry helper for rendering and
  inspection.
- Keep the geometry source simple and Python-native. Inline mesh-backed geometry
  is allowed inside the function, but do not add a new mesh-file contract.

### 2. Rename engineer-authored source to `solution_script.py`

- Replace the current engineer-facing starter `script.py` contract with
  `solution_script.py`.
- Update the common template loader, prompt text, and any agent instructions so
  engineer coder workspaces edit `solution_script.py` rather than a bare
  `script.py`.
- Update reviewer and submission helpers so they refer to the authored solution
  file explicitly and stop treating `script.py` as the human-authored source
  name.
- Keep the authored solution source in `solution_script.py`; do not reintroduce
  a bare `script.py` as the human-authored file.

### 3. Wire node entry validation to the new authored-file contract

- Make `scripts/validate_eval_seed.py` fail closed on benchmark rows that claim
  geometry but do not include `benchmark_script.py`.
- Update `controller/agent/node_entry_validation.py` so the seeded-entry
  preflight logic enforces the benchmark geometry source contract instead of
  silently accepting YAML-only rows.
- Preserve the existing seeded validation flow; this migration changes the
  contract enforced by the gate, not the fact that the gate exists.

### 4. Retire legacy naming assumptions

- Stop using `shared/assets/template_repos/benchmark_generator/result.py` as a
  contract surface. If a compatibility alias is kept temporarily, it must not be
  the source of truth.
- Remove benchmark/engineer documentation that tells agents to edit a bare
  `script.py` when the authored file should be `solution_script.py` or
  `benchmark_script.py`.
- Update tests and seed fixtures so they assert the new filenames instead of
  expecting `result.py` or legacy `script.py` authoring.

## Non-Goals

- Do not introduce prerendered benchmark images in this migration.
- Do not change the on-demand preview-rendering contract beyond the geometry
  source it consumes.
- Do not move benchmark geometry authoring into a separate non-Python mesh
  pipeline.
- Do not reintroduce a bare authored `script.py` in the workspace surface.

## Sequencing

This migration is intentionally staged before implementation work.

The safe order is:

1. Add the `benchmark_script.py` and `solution_script.py` contracts to the
   architecture/spec layer.
2. Update the template loaders, prompts, and handoff docs so the authored source
   filenames are unambiguous.
3. Update node entry validation and seed validation to fail closed on missing
   benchmark geometry.
4. Retrofit the benchmark seed rows to carry the visible geometry source.
5. Remove or de-emphasize any legacy `result.py` and bare-`script.py` authoring
   assumptions only after the new contracts are in place.

## Acceptance Criteria

1. Benchmark workspaces contain a visible `benchmark_script.py` when the seed
   claims to define benchmark geometry.
2. `objectives_geometry()` is available with no arguments and is the geometry
   helper that the preview path will later consume.
3. Engineer workspaces and prompts refer to `solution_script.py` instead of a
   bare authored `script.py`.
4. `scripts/validate_eval_seed.py` rejects benchmark rows that are missing
   benchmark geometry rather than letting them pass as YAML-only seeds.
5. Node entry validation fails closed on the same missing-geometry condition in
   the runtime preflight path.
6. The integration suite proves that benchmark geometry is present, readable,
   and named consistently across the seeded workspace path.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce
the new contract:

- `shared/agent_templates/common/script.py` -> `shared/agent_templates/common/solution_script.py`
- `shared/assets/template_repos/benchmark_generator/benchmark_script.py`
- `shared/assets/template_repos/benchmark_generator/result.py` if the migration
  keeps it as a temporary compatibility alias
- `shared/agent_templates/__init__.py`
- `evals/logic/codex_workspace.py`
- `evals/logic/workspace.py`
- `scripts/validate_eval_seed.py`
- `controller/agent/node_entry_validation.py`
- `controller/agent/benchmark/nodes.py`
- `controller/agent/benchmark/storage.py`
- `controller/agent/nodes/coder.py`
- `controller/agent/handover_constants.py`
- `shared/agent_templates/codex/scripts/submit_for_review.py`
- `tests/integration/architecture_p0/test_shared_agent_templates.py`
- `tests/integration/architecture_p0/test_codex_runner_mode.py`
- `tests/integration/architecture_p1/test_benchmark_workflow.py`
- `tests/integration/architecture_p1/test_engineering_loop.py`
- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/evals-architecture.md`
- `specs/integration-test-list.md`
- `dataset/data/seed/role_based/*.json`
- `dataset/data/seed/artifacts/**`

## Open Questions

None for the migration plan. The remaining work is implementation detail.

## Migration Checklist

Use this checklist to track the migration from spec edits through runtime gates, seed fixtures, and integration verification. Do not close the migration until every unchecked item is either completed or explicitly waived with a written rationale.

### Contract and docs

- [ ] Update the architecture docs so `benchmark_script.py` is the benchmark-owned, read-only geometry source and `solution_script.py` is the engineer-authored source.
- [ ] Keep `objectives_geometry()` as the only canonical geometry helper for rendering and inspection, with no positional arguments.
- [ ] Remove or replace any remaining architecture wording that tells agents to edit a bare `script.py` when the authored source should be `benchmark_script.py` or `solution_script.py`.
- [ ] Retire `shared/assets/template_repos/benchmark_generator/result.py` as a source-of-truth contract surface.
- [ ] Keep this migration aligned with the on-demand preview-rendering migration; do not add prerender-image ownership here.
- [ ] Update `specs/integration-test-list.md` and any adjacent guidance if the migration changes any INT scope notes or test-name expectations.

### Templates and workspace materialization

- [ ] Rename the shared engineer starter from `shared/agent_templates/common/script.py` to `shared/agent_templates/common/solution_script.py`.
- [ ] Create or update `shared/assets/template_repos/benchmark_generator/benchmark_script.py` as the benchmark-owned visible geometry script.
- [ ] Ensure benchmark planner workspaces do not receive `benchmark_script.py` before benchmark plan approval.
- [ ] Ensure benchmark coder and benchmark reviewer workspaces do receive `benchmark_script.py` as read-only geometry context after plan approval.
- [ ] Ensure engineering workspaces receive `benchmark_script.py` as read-only context when they need to preview or reason about benchmark geometry.
- [ ] Ensure engineering workspaces use `solution_script.py` as the writable authored implementation source.
- [ ] Update prompt generation in `evals/logic/codex_workspace.py` and related helpers so the prompt text names `solution_script.py` and conditionally names `benchmark_script.py`.
- [ ] Update any submission helper, reviewer helper, or workspace bootstrapper that still assumes a bare `script.py` is the authored source file.
- [ ] Make sure `objectives_geometry()` is available from the benchmark script and returns the geometry object directly from Python source.
- [ ] Keep mesh-backed benchmark geometry inline in Python only; do not introduce a separate mesh-file contract or a new authoring pipeline.

### Validation gates

- [ ] Make `scripts/validate_eval_seed.py` fail closed when a benchmark row claims geometry but does not include `benchmark_script.py`.
- [ ] Make `controller/agent/node_entry_validation.py` enforce the same missing-geometry failure for seeded-entry preflight.
- [ ] Preserve the existing seed-validation flow so the migration changes the contract, not the existence of the gate.
- [ ] Verify benchmark plan review still blocks missing or contradictory geometry, motion, or material metadata.
- [ ] Verify benchmark coder entry still blocks if the benchmark handoff is incomplete or the benchmark geometry source is missing.
- [ ] Verify reviewer entry still uses the latest-revision manifest and fails closed on stale or missing artifacts.
- [ ] Verify `benchmark_assembly_definition.yaml` remains a required benchmark-owned handoff artifact for downstream engineering intake.
- [ ] Verify `benchmark_script.py` is read-only once copied into downstream engineering or benchmark-review workspaces.

### Seed and fixture updates

- [ ] Add `benchmark_script.py` to every benchmark-backed seed row that claims visible geometry.
- [ ] Populate `objectives_geometry()` in every benchmark script that needs to render or inspect objective geometry.
- [ ] Keep `solution_script.py` as the only authored engineer source in engineer seed artifacts.
- [ ] Rename any seed artifacts or seed JSON metadata that still refer to bare `script.py` as the authored source.
- [ ] Remove any seed rows that are YAML-only but still claim benchmark geometry.
- [ ] Confirm that benchmark seeds remain readable, measurable, and renderable from the visible source file.
- [ ] Confirm that the seeded workspace copies match the new file-name contract without aliasing `result.py` or bare `script.py`.

### Integration tests and fixtures to update

- [ ] Update [tests/integration/architecture_p0/test_shared_agent_templates.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_shared_agent_templates.py).
- [ ] Update [tests/integration/architecture_p0/test_codex_runner_mode.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_codex_runner_mode.py).
- [ ] Update [tests/integration/architecture_p0/test_codex_session_trace_capture.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_codex_session_trace_capture.py).
- [ ] Update [tests/integration/architecture_p0/test_int_008_objectives_validation.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_008_objectives_validation.py).
- [ ] Update [tests/integration/architecture_p0/test_int_018_multipart_dfm_gate.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_018_multipart_dfm_gate.py).
- [ ] Update [tests/integration/architecture_p0/test_int_024_runtime_execute.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_024_runtime_execute.py).
- [ ] Update [tests/integration/architecture_p0/test_int_026_030.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_026_030.py).
- [ ] Update [tests/integration/architecture_p0/test_int_071_to_073.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_071_to_073.py).
- [ ] Update [tests/integration/architecture_p0/test_int_102_111.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_102_111.py).
- [ ] Update [tests/integration/architecture_p0/test_int_110_gpu_oom.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_110_gpu_oom.py).
- [ ] Update [tests/integration/architecture_p0/test_int_120_electronics.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_120_electronics.py).
- [ ] Update [tests/integration/architecture_p0/test_int_188_validation_preview.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_188_validation_preview.py).
- [ ] Update [tests/integration/architecture_p0/test_int_190_benchmark_coder_permissions.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_190_benchmark_coder_permissions.py).
- [ ] Update [tests/integration/architecture_p0/test_physics_fluids.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_physics_fluids.py).
- [ ] Update [tests/integration/architecture_p0/test_planner_gates.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_planner_gates.py).
- [ ] Update [tests/integration/architecture_p0/test_architecture_p0.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_architecture_p0.py) if it still asserts the old authored-source contract anywhere.
- [ ] Update [tests/integration/architecture_p1/test_electronics_full.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_electronics_full.py).
- [ ] Update [tests/integration/architecture_p1/test_engineering_loop.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_engineering_loop.py).
- [ ] Update [tests/integration/architecture_p1/test_episode_replay.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_episode_replay.py).
- [ ] Update [tests/integration/architecture_p1/test_infrastructure.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_infrastructure.py).
- [ ] Update [tests/integration/architecture_p1/test_int_064_to_069.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_int_064_to_069.py).
- [ ] Update [tests/integration/architecture_p1/test_manufacturing.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_manufacturing.py).
- [ ] Update [tests/integration/architecture_p1/test_physics_fluids_extended.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_physics_fluids_extended.py).
- [ ] Update [tests/integration/architecture_p1/test_physics_fluids_full.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_physics_fluids_full.py).
- [ ] Update [tests/integration/architecture_p1/test_reviewer_evidence.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_reviewer_evidence.py).
- [ ] Update [tests/integration/architecture_p1/test_script_tools_proxy.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p1/test_script_tools_proxy.py) if it still treats bare `script.py` as canonical rather than arbitrary file I/O.
- [ ] Update [tests/integration/frontend/p0/test_solution_evidence.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/frontend/p0/test_solution_evidence.py).
- [ ] Update [tests/integration/frontend/test_int_164.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/frontend/test_int_164.py).
- [ ] Update [tests/integration/frontend/p1/test_int_179.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/frontend/p1/test_int_179.py).
- [ ] Update [tests/integration/frontend/p1/test_int_180.py](/home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/frontend/p1/test_int_180.py).
- [ ] Rewrite the mock-response fixture trees for the affected INTs: `INT-002`, `INT-003`, `INT-005`, `INT-010`, `INT-014`, `INT-016`, `INT-017`, `INT-025`, `INT-027`, `INT-033`, `INT-034`, `INT-035`, `INT-036`, `INT-037`, `INT-038`, `INT-039`, `INT-040`, `INT-042`, `INT-043`, `INT-045`, `INT-053`, `INT-055`, `INT-058`, `INT-059`, `INT-060`, `INT-075`, `INT-113`, `INT-115`, `INT-129`, `INT-142`, `INT-160`, `INT-161`, `INT-167`, `INT-173`, `INT-174`, `INT-176`, `INT-177`, `INT-179`, `INT-180`, `INT-181`, `INT-182`, `INT-183`, `INT-184`, `INT-185`, `INT-186`, `INT-189`, and `INT-205`.
- [ ] Update any remaining fixture files that still name `script.py` or `result.py` as the authored source.

### Rerun matrix

- [ ] Rerun the benchmark planner and benchmark reviewer flows after the fixture rewrite.
- [ ] Rerun the engineer planner, engineer coder, electronics reviewer, and engineer execution reviewer flows after the fixture rewrite.
- [ ] Rerun the preview/render path tests that validate `objectives_geometry()` and the bucketed render layout.
- [ ] Rerun the frontend code-viewer and `@path` mention tests after the rename.
- [ ] Rerun the template-loader and Codex workspace-contract tests after the authored-source rename.
- [ ] Rerun the seed-validation and node-entry-validation paths after the fail-closed geometry check lands.
- [ ] Rerun the full affected integration slice once the targeted tests are green.

### Final verification

- [ ] Confirm benchmark planner workspaces never receive `benchmark_script.py` before plan approval.
- [ ] Confirm benchmark coder and benchmark reviewer workspaces do receive `benchmark_script.py` after plan approval.
- [ ] Confirm engineer workspaces receive `benchmark_script.py` as read-only context and `solution_script.py` as writable authored code.
- [ ] Confirm no seeded workspace treats bare `script.py` as the authored source file.
- [ ] Confirm `scripts/validate_eval_seed.py` rejects benchmark-backed rows that are YAML-only.
- [ ] Confirm `controller/agent/node_entry_validation.py` rejects the same missing-geometry condition.
- [ ] Confirm `result.py` is not treated as a source-of-truth benchmark artifact anywhere.
- [ ] Confirm the integration-test catalog, seed fixtures, and runtime gates all agree on the new filenames.
