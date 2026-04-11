---
title: Shared Scene Builder for Static Preview and Physics
status: completed
agents_affected:
  - benchmark_planner
  - benchmark_coder
  - benchmark_reviewer
  - engineer_planner
  - engineer_coder
  - engineer_execution_reviewer
added_at: '2026-04-11T07:16:39Z'
---

# Shared Scene Builder for Static Preview and Physics

<!-- Implemented. Shared scene-builder contract now backs preview, physics, and validation. -->

## Purpose

This migration removes the scene-parity gap that makes static preview omit the
benchmark payload when the payload only exists through runtime insertion in the
physics path. The target contract is that one shared typed scene-builder module
owns assembly traversal, payload geometry materialization, and payload scene
naming for both preview bundles and simulation scenes, so preview evidence
matches the physics scene instead of reconstructing a second, weaker model.

Relevant architecture context:

- [Simulation and Rendering](../../architecture/simulation-and-rendering.md)
- [CAD and other infrastructure](../../architecture/CAD-and-other-infra.md)

The migration only normalizes scene construction. It does not change render
backend selection, physics backend selection, or motion semantics.

## Problem Statement

The current codebase splits the scene contract across multiple modules:

1. `worker_renderer/utils/scene_builder.py` traverses authored `Compound`
   geometry and resolves metadata for preview export.
2. `worker_heavy/simulation/builder.py` duplicates the same traversal and
   metadata logic, then appends the benchmark payload body inline.
3. `worker_heavy/utils/validation.py` rebuilds the payload geometry again for
   start-clearance validation.
4. `worker_renderer/utils/build123d_rendering.py` consumes the renderer-side
   scene builder and therefore only sees the authored assembly unless the
   runtime insertion logic is duplicated there too.

That split is the root cause of the static-preview bug. The preview bundle is
missing the payload because preview scene export is not consuming the same
scene-insertion contract that physics uses. This is a contract mismatch, not a
rendering backend issue.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `worker_renderer/utils/scene_builder.py` | Owns preview-oriented assembly traversal, metadata resolution, and mesh export helpers. | The renderer path has the right traversal primitives, but it is isolated from the physics-side insertion path. |
| `worker_renderer/utils/build123d_rendering.py` | Builds `preview_scene.json` from authored assembly traversal only. | Static preview can omit the benchmark payload unless the same insertion logic is shared. |
| `worker_heavy/simulation/builder.py` | Reimplements assembly traversal and inserts the benchmark payload body inline. | Physics owns the payload insertion path today, so preview and physics can drift. |
| `worker_heavy/utils/validation.py` | Rebuilds the payload geometry for start-clearance validation. | The payload shape contract is defined more than once, which increases drift risk. |
| `worker_heavy/simulation/naming.py` | Owns the benchmark payload scene-name helper for physics-side code. | The naming contract is part of the scene builder and should not live as a worker-specific detail if rendering also needs it. |
| `tests/integration/architecture_p1/test_engineering_loop.py` | Asserts final preview bundle contents from the submitted solution path. | Final handoff preview content will change once the payload is inserted into the shared scene model. |
| `tests/integration/architecture_p0/test_int_188_validation_preview.py` | Exercises preview rendering and bundle materialization. | This is the narrowest place to assert that the preview bundle now includes the payload entity. |
| `tests/worker_heavy/simulation/test_builder.py` and `tests/worker_heavy/simulation/test_builder_mock.py` | Verifies physics builder behavior and payload-related scene construction. | The builder refactor must preserve payload naming and start pose while removing duplicated scene logic. |

## Proposed Target State

1. A shared typed scene-builder module under `shared/` owns payload geometry
   materialization, payload scene naming, and assembly traversal helpers.
2. `worker_heavy/simulation/builder.py` and
   `worker_renderer/utils/build123d_rendering.py` both consume that shared
   module instead of each materializing the payload independently.
3. Static preview bundles include the benchmark payload when
   `BenchmarkDefinition.payload` is present, using the same start position and
   scene name that physics uses.
4. Validation helpers consume the same payload geometry constructor so
   start-clearance checks and scene insertion cannot drift.
5. The insertion contract remains fail-closed when payload metadata is missing
   or ambiguous.
6. The change does not alter the preview manifest format, render modality
   selection, or simulation backend selection.

## Required Work

### 1. Introduce a shared scene-builder module

- Add a shared typed helper module for assembly traversal, payload geometry,
  payload scene naming, and mesh export.
- Keep the payload shape constructor and the payload scene-name helper in one
  place.
- Return typed records or typed helper results rather than ad hoc dictionaries.
- Prefer a shared module under `shared/` so both worker-heavy and
  worker-renderer import the same code.

### 2. Route preview scene export through the shared contract

- Update `worker_renderer/utils/build123d_rendering.py` to build preview scenes
  from the shared scene-builder module.
- Keep `preview_scene.json` as the render-bundle snapshot, but include the
  benchmark payload entity whenever the benchmark definition declares one.
- Preserve the current render-manifest shape apart from the newly present
  payload entity.

### 3. Route physics scene construction through the same helper

- Update `worker_heavy/simulation/builder.py` to consume the shared scene
  builder instead of recreating traversal and payload insertion inline.
- Keep the MuJoCo body naming convention stable.
- Preserve existing physics behavior, including the runtime payload start
  position and material resolution.

### 4. Share the payload geometry constructor with validation

- Update `worker_heavy/utils/validation.py` so the payload start-clearance
  helper uses the shared payload geometry constructor.
- Keep the clearance check fail-closed when the benchmark payload cannot be
  materialized.

### 5. Preserve compatibility during the transition

- Keep `worker_renderer/utils/scene_builder.py` and
  `worker_heavy/simulation/naming.py` as compatibility shims only if the
  migration needs them during rollout.
- Do not let a compatibility shim become a second source of truth.

## Non-Goals

- Do not change simulation backend selection.
- Do not change render backend selection.
- Do not change objective-zone overlays.
- Do not change the benchmark payload contract itself.
- Do not add a preview-only fallback or a synthetic payload placeholder.
- Do not change agent prompts or handoff files in this migration.

## Sequencing

1. Introduce the shared scene-builder module and route validation to it.
2. Update the physics builder and preview exporter to consume it.
3. Add the preview bundle parity regression.
4. Verify the narrow render and validation integration slice.
5. Widen only if the targeted slice exposes collateral regressions.

## Acceptance Criteria

1. A benchmark with a declared payload produces a static preview bundle whose
   `preview_scene.json` contains the payload entity at the declared start
   position.
2. The physics builder and the preview exporter derive the payload geometry
   from the same shared constructor.
3. The validation start-clearance helper and the scene builder no longer
   duplicate payload shape logic.
4. No new fallback or placeholder payload is introduced.
5. The change is limited to scene construction, preview export, physics scene
   construction, validation parity, and the associated tests.

## Test Impact

- `tests/integration/architecture_p0/test_int_188_validation_preview.py` needs
  a preview-bundle assertion that checks for the payload entity in
  `preview_scene.json`.
- `tests/integration/architecture_p1/test_engineering_loop.py` needs to accept
  the payload entity in the final solution submission preview scene.
- `tests/worker_heavy/simulation/test_builder.py` and
  `tests/worker_heavy/simulation/test_builder_mock.py` need to preserve payload
  naming and start pose after the shared-module refactor.

## Risks

- The shared module must stay import-safe in both worker processes and must not
  pull in worker-only HTTP clients or lifecycle state.
- Preview bundle entity counts and ordering will change for benchmarks that
  declare a payload, so tests should assert payload presence rather than exact
  entity list equality unless ordering is the contract being checked.
- Compatibility shims must stay thin. If the shim starts diverging from the
  shared module, the migration has recreated the same parity bug in a second
  place.

## Migration Checklist

### Contract

- [x] Add the shared scene-builder module.
- [x] Move payload geometry construction to the shared module.
- [x] Move payload scene naming to the shared module.

### Runtime

- [x] Update preview scene export to include the payload entity.
- [x] Update the physics builder to consume the shared scene builder.
- [x] Update validation to consume the same payload geometry constructor.

### Tests

- [x] Add preview bundle parity coverage.
- [x] Add physics-builder regression coverage.
- [x] Add validation geometry parity coverage.

## File-Level Change Set

- `shared/simulation/scene_builder.py`
- `worker_renderer/utils/build123d_rendering.py`
- `worker_renderer/utils/scene_builder.py`
- `worker_heavy/simulation/builder.py`
- `worker_heavy/simulation/naming.py`
- `worker_heavy/utils/validation.py`
- `tests/integration/architecture_p0/test_int_188_validation_preview.py`
- `tests/integration/architecture_p1/test_engineering_loop.py`
- `tests/worker_heavy/simulation/test_builder_mock.py`
