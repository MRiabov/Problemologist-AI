# Validation on Light Worker

<!-- Completed migration record. Historical checklist retained for traceability. -->

## Purpose

Move benchmark `validate()` execution off `worker-heavy` and onto `worker-light`
to remove an unnecessary Temporal/heavy-worker hop for a fast geometry gate.
The controller remains the agent-facing entrypoint, preview rendering remains
owned by `worker-renderer`, and validation semantics do not change.

This migration is a transport and ownership refactor, not a new user-facing
feature. The target architecture is still defined by:

- [Distributed execution](../../architecture/distributed-execution.md)
- [CAD and other infrastructure](../../architecture/CAD-and-other-infra.md)
- [Simulation and rendering](../../architecture/simulation-and-rendering.md)

## Status

This migration is complete. The validation boundary has been moved to
`worker-light`, the architecture docs and contracts have been aligned, and the
integration suite has been migrated away from legacy assumptions where it
previously expected `/benchmark/simulate`-style query or route behavior.
Checklist items below are retained as a closed record, not an open task list.

## Problem Statement

The current validation path still routes through the heavy-worker stack even
though benchmark validation is usually a cheap geometry check.

1. The controller path for `/validate` currently bundles the workspace and
   hands off to a Temporal-backed heavy workflow.
2. `worker-heavy` still owns the actual validation execution boundary.
3. Preview rendering is already split to `worker-renderer`, so validation is
   the remaining expensive hop in an otherwise split render/geometry path.
4. The current path pays controller, bundle, and Temporal overhead even when
   the validation result is near-instant.
5. The render contract is already decoupled from validation, so the remaining
   work is to move the geometry gate without regressing artifact ownership.

## Current-State Inventory

The inventory below is the pre-cutover snapshot that motivated the migration
and is kept for traceability.

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `specs/architecture/distributed-execution.md` | States that validation is still part of `worker-heavy` and that only preview rendering is split to `worker-renderer`. | The migration target is a light-worker validation boundary, so the architecture and migration docs must eventually agree. |
| `specs/architecture/CAD-and-other-infra.md` | Says `/benchmark/validate` is render-free, while preview uses `worker-renderer`. | This is correct for semantics, but not for the current execution boundary. |
| `controller/api/routes/script_tools.py` | `POST /validate` proxies into controller middleware and returns the validation response. | The controller route must keep the same surface while no longer forcing the heavy-worker path. |
| `controller/middleware/remote_fs.py` | `validate()` bundles the workspace and runs `HeavyValidationWorkflow`. | This is the current orchestration hop that has to be removed or bypassed for the new default path. |
| `controller/workflows/heavy.py` | `HeavyValidationWorkflow` dispatches `worker_validate_design` on `heavy-tasks-queue`. | This is the current Temporal boundary for validation. |
| `controller/clients/worker.py` | Local/non-controller validation still posts to `worker-heavy` at `/benchmark/validate`. | The worker client needs a light-worker validation path so non-controller runs do not keep hitting heavy. |
| `shared/utils/agent/__init__.py` | `validate()` calls the heavy worker when no controller URL is present. | Codex/local execution must follow the same light-worker default path as the controller. |
| `worker_heavy/api/routes.py` | Exposes `/benchmark/validate` and materializes validation artifacts there. | This route is the heavy-worker implementation that should stop being the default validation boundary. |
| `worker_heavy/activities/heavy_tasks.py` | `worker_validate_design` loads the script, validates geometry, and collects validation artifacts. | This activity is the current heavy execution unit for validation. |
| `shared/rendering/renderer_client.py` and `worker_light/api/routes.py` | Preview delegation already uses a dedicated renderer client and light-worker-facing preview route. | These are the reusable pieces that validation should lean on rather than reimplementing preview plumbing. |
| `tests/integration/architecture_p0/test_int_188_validation_preview.py` and `tests/integration/architecture_p0/test_architecture_p0.py` | Existing integration coverage already asserts the render-free validation contract and preview delegation behavior. | These tests need to be updated to reflect the new validation boundary without losing the render-free contract. |

## Proposed Target State

1. The controller keeps the same `validate` tool surface and agent-facing
   contract.
2. `/validate` no longer requires the heavy-worker validation workflow in the
   normal product path.
3. `worker-light` loads benchmark scripts, performs geometry validation, and
   persists `validation_results.json`.
4. When validation needs preview evidence, `worker-light` calls the renderer
   through the shared renderer client and keeps `worker-renderer` as the sole
   owner of static preview rendering.
5. Validation and preview continue to share one canonical render-manifest
   builder path so `renders/<bundle>/render_manifest.json` stays consistent and
   `renders/render_manifest.json` can remain a compatibility alias only.
6. `worker-heavy` remains responsible for simulation, submit/review handoff,
   verification, and other heavy tasks, but it is no longer the default
   validation execution boundary.
7. Agents do not gain any new direct transport to `worker-light` or
   `worker-renderer`; the controller remains the only agent-facing boundary.

## Required Work

### Routing and transport

1. Add or expose a light-worker validation path that executes the same
   `validate()` semantics without going through `HeavyValidationWorkflow`.
2. Update `controller/middleware/remote_fs.py` so controller-side validation
   uses the light-worker path instead of Temporal-heavy dispatch.
3. Update `controller/api/routes/script_tools.py` so the public controller
   route preserves its response shape while calling the new light-worker
   validation path.
4. Update `controller/clients/worker.py` and
   `shared/utils/agent/__init__.py` so local/non-controller validation follows
   the same light-worker boundary.

### Validation and artifact ownership

1. Keep validation semantics unchanged: geometry/objective checks still fail
   closed exactly as before.
2. Keep validation render-free by default; explicit preview generation must
   still happen only through the preview helper path.
3. Make sure `validation_results.json` remains the canonical validation record
   written back into the session workspace.
4. Reuse the existing render-manifest builder path so preview and validation
   continue to produce consistent bundle-local `render_manifest.json` data.

### Worker implementation

1. Implement the light-worker validation route in `worker_light/api/routes.py`
   or the nearest equivalent light-worker surface.
2. Reuse the shared renderer client instead of adding renderer-specific
   networking inside controller code.
3. Keep `worker_heavy/api/routes.py` and
   `worker_heavy/activities/heavy_tasks.py` available only as legacy/compat
   surfaces until the migration cutover is complete, then remove or narrow them
   as needed.

### Docs and contracts

1. Update `specs/architecture/distributed-execution.md` after the code path
   changes so the authoritative architecture docs match the new routing
   contract.
2. Keep `specs/architecture/CAD-and-other-infra.md` aligned with the final
   manifest ownership decision.
3. Update any controller or worker contract comments that still describe
   validation as a heavy-worker default path.

### Tests and evals

1. Update the validation/preview integration suites so they assert the light
   worker route instead of the heavy-worker route for normal validation.
2. Keep the render-free validation tests intact so the migration does not
   accidentally reintroduce preview generation into `/benchmark/validate`.
3. Refresh any mock-response scenarios or handoff fixtures that still encode a
   heavy-worker validation assumption.
4. Add or retune tests so controller-path validation, local Codex validation,
   and preview delegation all agree on the same artifact contract.

## Non-Goals

1. Do not change validation semantics.
2. Do not add a new agent-facing worker transport.
3. Do not move simulation, submit/review handoff, or DFM analysis off
   `worker-heavy`.
4. Do not change the preview contract or turn validation into a render gate.
5. Do not require a new render artifact format; keep the existing bundle-local
   manifest contract.

## Sequencing

1. Add the light-worker validation implementation and its artifact plumbing.
2. Switch the controller and local client paths to the new validation route.
3. Refresh the integration tests and fixture expectations.
4. Remove or narrow the heavy-worker validation compatibility surface once the
   new path is stable.
5. Update the architecture docs to reflect the new steady state.

## Acceptance Criteria

1. Controller-path `validate()` calls no longer require
   `HeavyValidationWorkflow` or `worker_validate_design` in normal operation.
2. `worker-light` produces the same validation result payload and
   `validation_results.json` artifact as the old path.
3. `/benchmark/validate` remains render-free by default and still fails closed
   on invalid geometry, missing handoff artifacts, and reserved label/objective
   violations.
4. Explicit preview rendering still goes through `worker-renderer` and the
   bundle-local manifest contract remains valid.
5. Integration tests covering validation, preview delegation, and controller
   script-tools routing pass with the new boundary.
6. The authoritative architecture docs are updated to describe the new
   steady-state routing after the cutover.

## Migration Checklist

### Routing

- [x] Add a light-worker validation entrypoint that accepts the same
  validation inputs as the current heavy-worker path.
- [x] Route controller `validate()` calls to the light-worker path without
  changing the public `BenchmarkToolResponse` shape.
- [x] Route local/non-controller `validate()` calls through the same
  light-worker boundary.
- [x] Keep the controller as the only agent-facing transport boundary.
- [x] Remove `HeavyValidationWorkflow` from the normal validation path once
  the light-worker path is proven.
- [x] Verify direct `worker-heavy` validation is no longer required for the
  happy path.

### Artifacts

- [x] Preserve `validation_results.json` generation in the session workspace.
- [x] Keep artifact sync back to the light-worker workspace working after each
  validation call.
- [x] Keep bundle-local `renders/<bundle>/render_manifest.json` generation
  consistent with the preview helper path.
- [x] Confirm the root `renders/render_manifest.json` alias still works as
  compatibility plumbing only.
- [x] Verify validation remains render-free by default.
- [x] Verify explicit preview generation still lands in the correct render
  bucket and does not appear as a validation side effect.
- [x] Confirm the manifest builder stays canonical and does not fork between
  controller, light worker, and renderer worker paths.

### Tests

- [x] Update `tests/integration/architecture_p0/test_int_188_validation_preview.py`
  to assert the new validation boundary while preserving the render-free
  contract.
- [x] Update `tests/integration/architecture_p0/test_architecture_p0.py` to
  cover controller-path validation through the light worker.
- [x] Update any related validation/preview/controller-path integration
  fixtures.
- [x] Refresh mock-response scenarios that still assume heavy-worker validation.
- [x] Migrate any tests that previously expected `/benchmark/simulate` query or
  route assumptions to the current route-ownership contract.
- [x] Verify the controller script-tools validation path and the direct
  worker client validation path produce the same response contract.
- [x] Verify preview delegation still reaches `worker-renderer` and produces
  the same bundle-local manifest outputs.
- [x] Verify the negative cases still fail closed on invalid geometry,
  missing handoff artifacts, or reserved label/objective violations.

### Docs

- [x] Update `specs/architecture/distributed-execution.md` after the code
  cutover.
- [x] Update `specs/architecture/CAD-and-other-infra.md` if the manifest
  ownership wording changes.
- [x] Update any inline route or client comments that still describe validation
  as heavy-worker owned by default.
- [x] Mark the migration frozen once implementation is complete so the doc
  stops drifting into a live task list.

### Cutover Verification

- [x] Confirm controller-path validation works on a clean workspace start.
- [x] Confirm Codex/local validation works without the controller proxy.
- [x] Confirm preview-only workflows still use `worker-renderer` and do not
  require validation first.
- [x] Confirm `worker-heavy` still handles simulation, submit/review handoff,
  and other heavy tasks after validation moves.
- [x] Confirm the light-worker path fails closed if the renderer dependency is
  unavailable and the request actually needs preview evidence.
- [x] Confirm no new agent-facing routes or transports were introduced during
  the migration.

## File-Level Change Set

- `controller/api/routes/script_tools.py`
- `controller/middleware/remote_fs.py`
- `controller/clients/worker.py`
- `controller/workflows/heavy.py`
- `shared/utils/agent/__init__.py`
- `shared/rendering/__init__.py`
- `shared/rendering/renderer_client.py`
- `worker_light/api/routes.py`
- `worker_heavy/api/routes.py`
- `worker_heavy/activities/heavy_tasks.py`
- `worker_heavy/utils/validation.py`
- `worker_renderer/utils/rendering.py`
- `shared/workers/schema.py`
- `tests/integration/architecture_p0/test_int_188_validation_preview.py`
- `tests/integration/architecture_p0/test_architecture_p0.py`

## Open Questions

None. The migration is complete and the remaining design questions were
resolved in the implemented code and migrated test coverage.
