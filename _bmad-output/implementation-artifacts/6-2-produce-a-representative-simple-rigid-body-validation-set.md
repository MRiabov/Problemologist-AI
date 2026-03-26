# Story 6.2: Produce a Representative Simple Rigid-Body Validation Set

Status: ready-for-review

## Story

As a human operator, I want a representative simple rigid-body validation set so that benchmark generator runs are exercised across repeatable variations instead of a single toy case.

## Acceptance Criteria

1. Given the simple rigid-body benchmark family, when seeds are generated, then the set includes multiple variants across object shape, placement, size, and objective arrangement.
2. Given the validation set, when I inspect it, then it is broad enough to exercise the benchmark generator against repeatable simple rigid-body cases.
3. Given a family member is duplicated too closely, when the set is curated, then the duplicate is excluded or marked as redundant.

## Tasks / Subtasks

- [x] Curate the canonical simple rigid-body seed corpus in `dataset/data/seed/role_based/benchmark_planner.json` and the paired benchmark review/coder/reviewer seed bundles so the set spans the required coverage axes. (AC: 1, 2)
  - [x] Keep the current coverage families explicit: shape, placement, size, and objective arrangement.
  - [x] Keep the seed phrasing close to the way a human operator would describe a benchmark request, so the validation set reflects real intake rather than only taxonomy labels.
  - [x] Preserve the existing simple rigid-body / gravity-only contract from Story 6.1; do not widen the set into actuators, FEM, or fluids.
  - [x] Treat the current baseline rows as intentional coverage anchors, including the `bp-001` through `bp-010` benchmark-planner variants and the paired benchmark-plan-reviewer approved/reject fixtures.
- [x] Add or refresh the matching seed artifact bundles under `dataset/data/seed/artifacts/benchmark_plan_reviewer/`, `dataset/data/seed/artifacts/benchmark_coder/`, and `dataset/data/seed/artifacts/benchmark_reviewer/` so each curated row has deterministic, reusable evidence. (AC: 1, 2)
  - [x] Keep the artifact directories aligned with the role-based JSON rows and avoid introducing a separate seed loader or a second dataset format.
  - [x] If a new row is added, keep its `plan.md`, `todo.md`, and YAML artifacts internally consistent with the simple rigid-body gravity-only family.
  - [x] Reuse the existing approved and rejected simple rigid-body edge cases instead of inventing a new prompt taxonomy.
- [x] Introduce or preserve deterministic redundancy handling for near-duplicate seeds. (AC: 3)
  - [x] Prefer stable row IDs, artifact-dir identity, and manifest/provenance output over ad hoc duplicate flags.
  - [x] If a row is excluded as redundant, keep the reason explainable in the curation/validation output instead of silently mutating the row text.
  - [x] Reuse the existing manifest/provenance pattern seen in generated dataset manifests when you need to record drops or redundant groups.
- [x] Extend integration coverage for the curated simple rigid-body seed set and seed-workspace materialization. (AC: 1-3)
  - [x] Update `tests/integration/architecture_p0/test_codex_runner_mode.py` to materialize representative benchmark-planner and benchmark-plan-reviewer rows from the curated set and assert that the workspace contents stay workspace-relative and deterministic.
  - [x] Add or refresh a seed-validation integration slice around `scripts/validate_eval_seed.py` and/or `dataset/evals/materialize_seed_workspace.py` so duplicate or near-duplicate rows fail closed or are marked redundant deterministically.
  - [x] Keep all assertions integration-only and artifact-based; do not add unit-test-only coverage for the seed set.

## Dev Notes

- Relevant architecture patterns and constraints:
  - Story 6.2 is a dataset/seed curation story, not a benchmark planner runtime redesign. Keep the work on the seed corpus and its validation path.
  - The canonical simple rigid-body baseline already exists in `dataset/data/seed/role_based/benchmark_planner.json`; it currently spans forbid-zone, sideways transfer, gap bridge, low bar, raised shelf, lower bin, through-window, ring/post, S-curve, and speed-bump style variants.
  - The paired benchmark review set in `dataset/data/seed/role_based/benchmark_plan_reviewer.json` already contains both approved and rejected edge cases. Use that as the source of truth for review-oriented coverage, especially the hidden-DOF and obstruction failures.
  - `dataset/data/seed/role_based/benchmark_coder.json` and `dataset/data/seed/role_based/benchmark_reviewer.json` should stay aligned with the same simple rigid-body family so the end-to-end generator graph remains coherent.
  - `dataset/evals/materialize_seed_workspace.py` and `scripts/validate_eval_seed.py` are the existing fail-closed entrypoints for inspecting and validating seeded rows. Prefer extending those paths and their integration coverage rather than introducing a parallel loader.
  - `evals/logic/workspace.py::resolve_seed_artifact_dir()` already resolves role-based seed artifact directories under `dataset/data/seed/artifacts/`. Keep the folder layout compatible with that resolver.
  - If you need to explain why a row was dropped or treated as redundant, keep that explanation in the existing manifest/provenance style instead of inventing a second duplicate-tracking taxonomy.
  - Preserve the simple rigid-body gravity-only contract from Story 6.1. This story widens coverage; it does not add new modalities, motion systems, or prompt-layer behavior.
  - The seed loader contract is `EvalDatasetItem` in `evals/logic/models.py`. Keep any row changes valid under that model and avoid a second seed schema.

### Previous Story Intelligence

- Story 6.1 established the simple rigid-body gravity-only family as the baseline benchmark-generator contract and already refreshed the starter planner/reviewer fixtures. Reuse that contract here; do not reopen prompt design unless a seed row genuinely needs a wording fix.
- Story 6.1 also established that the benchmark planner should keep using the existing fail-closed validation and `submit_plan()` handoff path. This story should stay on the dataset/seed side and avoid runtime contract changes unless a test exposes a seed-materialization bug.
- The existing benchmark generator fixtures already include a passive gravity-only example (`INT-114`) and a negative unsupported-motion case (`INT-202`). Use them as regression anchors rather than inventing new ad hoc scenarios.

### Project Structure Notes

- `dataset/data/seed/role_based/benchmark_planner.json` is the canonical row list for benchmark generator validation coverage.
- `dataset/data/seed/role_based/benchmark_plan_reviewer.json` and `dataset/data/seed/artifacts/benchmark_plan_reviewer/` carry the paired review-fixture coverage, including approved and rejected edge cases.
- `dataset/data/seed/role_based/benchmark_coder.json`, `dataset/data/seed/role_based/benchmark_reviewer.json`, and their artifact folders should stay aligned with the same simple rigid-body family so end-to-end benchmark generation coverage remains coherent.
- Keep `scripts/validate_eval_seed.py` and `dataset/evals/materialize_seed_workspace.py` as the entrypoints for validating and inspecting seeded rows. If the validation set changes, update the assertions there or in the integration tests rather than adding a separate loader.
- Any new coverage or redundancy marker should stay compatible with `EvalDatasetItem` and the existing JSON row loader in `evals/logic/models.py`.

### References

- \[Source: `_bmad-output/planning-artifacts/epics.md`, Epic 6: Gravity: Benchmarks and Story 6.2\]
- \[Source: `dataset/data/seed/role_based/benchmark_planner.json`, canonical simple rigid-body benchmark planner validation rows\]
- \[Source: `dataset/data/seed/role_based/benchmark_plan_reviewer.json`, paired plan-reviewer validation rows and reject cases\]
- \[Source: `dataset/data/seed/role_based/benchmark_coder.json`, paired benchmark-coder validation rows\]
- \[Source: `dataset/data/seed/role_based/benchmark_reviewer.json`, paired benchmark-reviewer validation rows\]
- \[Source: `dataset/data/seed/artifacts/benchmark_plan_reviewer/`, approved and rejected simple rigid-body review fixtures\]
- \[Source: `dataset/data/seed/artifacts/benchmark_coder/`, benchmark-coder seed fixtures\]
- \[Source: `dataset/data/seed/artifacts/benchmark_reviewer/`, benchmark-reviewer seed fixtures\]
- \[Source: `dataset/data/seed/readme.md`, curated seed dataset intent\]
- \[Source: `dataset/evals/materialize_seed_workspace.py`, seed workspace materialization entrypoint\]
- \[Source: `evals/logic/workspace.py`, seed artifact directory resolution and workspace seeding\]
- \[Source: `evals/logic/models.py`, `EvalDatasetItem` seed loader contract\]
- \[Source: `scripts/validate_eval_seed.py`, fail-closed seeded entry validation\]
- \[Source: `tests/integration/architecture_p0/test_codex_runner_mode.py`, seed workspace materialization regression coverage\]
- \[Source: `tests/integration/mock_responses/INT-114.yaml`, passive rigid-body gravity-only benchmark planner transcript\]
- \[Source: `tests/integration/mock_responses/INT-202.yaml`, unsupported-motion fail-closed transcript\]
- \[Source: `dataset/data/generated/component_seeded/v0.0.1/manifest.json`, dedupe/provenance manifest shape for dropped rows\]

## Dev Agent Record

### Agent Model Used

GPT-5.4

### Debug Log References

- `./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_codex_runner_mode.py::test_validate_eval_seed_accepts_curated_rows_and_preserves_redundancy_metadata`
- `./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_codex_runner_mode.py`

### Completion Notes List

- Added fail-closed seed validation support for the curated benchmark-reviewer row by restoring a feasible benchmark geometry and copied planner caps in `benchmark_definition.yaml`.
- Stabilized the seed validator path for temporary workspaces by setting `COTS_DB_PATH` in the Codex workspace environment.
- Kept duplicate-reason normalization deterministic by deduping normalized rejection reasons instead of failing on repeated variants.
- Extended integration coverage for representative benchmark planner, coder, and reviewer seed rows plus seed-validation/redundancy checks.
- Normalized generated curation-manifest `dropped_lineage` ordering so the redundancy provenance assertion stays deterministic.
- Verified the story with `./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_codex_runner_mode.py`.

### File List

- `_bmad-output/implementation-artifacts/6-2-produce-a-representative-simple-rigid-body-validation-set.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`
- `shared/models/schemas.py`
- `evals/logic/codex_workspace.py`
- `tests/integration/architecture_p0/test_codex_runner_mode.py`
- `dataset/data/seed/artifacts/benchmark_plan_reviewer/bpr-001-raised-shelf/benchmark_assembly_definition.yaml`
- `dataset/data/seed/artifacts/benchmark_plan_reviewer/bpr-012-gap-bridge-hidden-dof/.manifests/benchmark_plan_review_manifest.json`
- `dataset/data/seed/artifacts/benchmark_plan_reviewer/bpr-012-gap-bridge-hidden-dof/benchmark_definition.yaml`
- `dataset/data/seed/artifacts/benchmark_plan_reviewer/bpr-012-gap-bridge-hidden-dof/benchmark_assembly_definition.yaml`
- `dataset/data/seed/artifacts/benchmark_coder/bc-011-sideways-ball/benchmark_definition.yaml`
- `dataset/data/seed/artifacts/benchmark_coder/bc-011-sideways-ball/benchmark_assembly_definition.yaml`
- `dataset/data/seed/artifacts/benchmark_reviewer/br-011-sideways-ball-review/benchmark.xml`
- `dataset/data/seed/artifacts/benchmark_reviewer/br-012-sideways-ball-infeasible-goal/.manifests/benchmark_review_manifest.json`
- `dataset/data/seed/artifacts/benchmark_reviewer/br-012-sideways-ball-infeasible-goal/assembly_definition.yaml`
- `dataset/data/seed/artifacts/benchmark_reviewer/br-012-sideways-ball-infeasible-goal/benchmark_definition.yaml`
- `dataset/data/seed/artifacts/benchmark_reviewer/br-012-sideways-ball-infeasible-goal/benchmark_assembly_definition.yaml`
- `dataset/data/seed/artifacts/benchmark_reviewer/br-012-sideways-ball-infeasible-goal/benchmark.xml`
- `dataset/data/seed/artifacts/benchmark_reviewer/br-013-sideways-ball-overlapping-goal-forbid/benchmark.xml`
- `dataset/data/seed/artifacts/engineer_plan_reviewer/epr-001-sideways-transfer/assembly_definition.yaml`
- `dataset/data/seed/artifacts/engineer_plan_reviewer/epr-001-sideways-transfer/plan.md`
- `dataset/data/generated/component_seeded/v0.0.1/manifest.json`
- `dataset/data/generated/workflow/v0.0.1/manifest.json`

## Change Log

- 2026-03-26: Validated the curated seed set with the integration runner, fixed reviewer-seed contract gaps, and normalized redundant lineage ordering.
