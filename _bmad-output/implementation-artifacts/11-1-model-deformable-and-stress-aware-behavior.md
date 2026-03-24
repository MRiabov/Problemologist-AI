# Story 11.1: Model Deformable and Stress-Aware Behavior

Status: ready-for-dev

## Story

As a human operator,
I want deformable and stress-aware simulation to be explicit,
so that tasks beyond rigid-body mechanics are evaluated correctly.

## Acceptance Criteria

1. Given a benchmark that requests FEM or soft-material behavior, when simulation runs, then the backend uses the configured deformable or stress-aware path rather than a rigid-only approximation.
2. Given a stress-aware run, when it completes, then the system records structured stress summaries for the inspected parts.
3. Given a rigid-only benchmark, when it is simulated, then the FEM path is not activated accidentally.

## Tasks / Subtasks

- [ ] Keep FEM enablement explicit in the simulation path. Preserve the Genesis-backed deformable branch for `physics.fem_enabled=true` and material-class-driven soft/elastomer handling, and do not let rigid-only runs enter the FEM path. (AC: 1, 3)
  - [ ] Touch only the existing simulation and validation modules unless a schema field is actually missing.
  - [ ] Keep `/benchmark/validate` on the MuJoCo-backed static preview path; do not turn this story into a Genesis parity gate.
- [ ] Thread structured stress summaries through the simulation result surface so they are persisted in `SimulationResult`, worker API responses, and episode artifacts without custom JSON parsing. (AC: 2)
  - [ ] Keep `shared/models/simulation.py` and `shared/workers/schema.py` as the canonical typed contract for stress summaries.
  - [ ] Verify the heavy-worker response path in `worker_heavy/api/routes.py` and `worker_heavy/activities/heavy_tasks.py` continues to expose the summaries.
- [ ] Extend or adjust the existing integration coverage for INT-102/111, INT-104, and INT-107 so the FEM gate, stress-summary population, and stress-objective path are all exercised through HTTP-only calls. (AC: 1-3)
  - [ ] Keep Genesis-dependent assertions behind `skip_unless_genesis`.
  - [ ] Assert against worker/controller responses and persisted artifacts, not imported internals.

## Dev Notes

- Relevant architecture patterns and constraints:
  - `specs/architecture/fluids-and-deformables.md` makes Genesis the required backend for fluid, deformable, and stress-aware runs, while MuJoCo remains valid for rigid-only tasks and the fast static preview path.
  - `specs/architecture/simulation-and-dod.md` keeps `/benchmark/validate` on MuJoCo-backed preview by default. Do not add a Genesis validation parity gate here.
  - `shared/models/simulation.py` already carries `StressSummary`, `SimulationResult`, `SimulationMetrics`, and failure payloads. Keep that result shape stable and typed.
  - `worker_heavy/simulation/objectives.py` is where stress objectives and breakage are evaluated.
  - `worker_heavy/simulation/genesis_backend.py` already extracts von Mises stress fields and computes stress summary stats.
  - `worker_heavy/simulation/builder.py` currently decides when to export deformable meshes based on `physics.fem_enabled` and soft/elastomer materials; keep that logic aligned with the benchmark contract.
  - `worker_heavy/api/routes.py` and `worker_heavy/activities/heavy_tasks.py` serialize `stress_summaries` into the worker API and artifact bundles; any change in the result surface should stay typed all the way through.
- Source tree components to touch:
  - `worker_heavy/simulation/builder.py`
  - `worker_heavy/simulation/objectives.py`
  - `worker_heavy/simulation/loop.py`
  - `worker_heavy/simulation/genesis_backend.py`
  - `worker_heavy/utils/validation.py`
  - `worker_heavy/api/routes.py`
  - `worker_heavy/activities/heavy_tasks.py`
  - `shared/models/simulation.py`
  - `shared/workers/schema.py`
  - `tests/integration/architecture_p0/test_int_102_111.py`
- Testing standards summary:
  - Use integration tests only. Do not add unit-test-only coverage for FEM behavior.
  - The integration tests must be HTTP-only and assert against worker/controller responses and persisted artifacts.
  - Keep the Genesis-only tests conditionally skipped when the backend is unavailable.
  - Preserve the rigid-only path as a fail-closed guard, not a fallback.
  - Keep the FEM material-field negative path from INT-102/111 in regression coverage, but do not broaden this story beyond the explicit deformable/stress-aware behavior scope.

### Project Structure Notes

- FEM behavior already lives in the worker-heavy simulation stack; do not create a parallel FEM subsystem.
- `shared/models/*` and `shared/workers/*` are the typed contracts. If a field is missing, add it there first rather than parsing ad hoc JSON in tests or routes.
- `tests/integration/architecture_p0/test_int_102_111.py` is the natural home for regression coverage; extend it instead of scattering new tests across unrelated suites.

### References

- [Source: \_bmad-output/planning-artifacts/epics.md, Epic 11: FEM: Simulation, Story 11.1]
- [Source: specs/architecture/fluids-and-deformables.md, backend split, material classes, stress summaries, breakage]
- [Source: specs/architecture/simulation-and-dod.md, Genesis/MuJoCo split and validation-preview contract]
- [Source: specs/architecture/CAD-and-other-infra.md, part metadata and rendering split]
- [Source: specs/architecture/distributed-execution.md, worker-heavy responsibility split]
- [Source: specs/architecture/observability.md, stress summary and simulation event contracts]
- [Source: worker_heavy/simulation/builder.py, deformable mesh handling and fem_enabled branch]
- [Source: worker_heavy/simulation/objectives.py, stress objective evaluation and breakage handling]
- [Source: worker_heavy/simulation/genesis_backend.py, stress field extraction and summary computation]
- [Source: worker_heavy/simulation/loop.py, final result assembly]
- [Source: shared/models/simulation.py, stress result models]
- [Source: shared/workers/schema.py, worker artifact response models]
- \[Source: worker_heavy/api/routes.py, `/benchmark/simulate` artifact plumbing\]
- [Source: tests/integration/architecture_p0/test_int_102_111.py, INT-102/104/107 coverage]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
