---
title: Remove the worker-heavy validation compatibility surface
status: migration
agents_affected: []
added_at: '2026-04-04T15:29:20Z'
---

# Remove the worker-heavy validation compatibility surface

## Purpose

`worker-heavy` used to carry the old validation implementation even though the
normal validation boundary had moved to `worker-light`.

This note is a cleanup record for the removed compatibility surface. It does
not change the validation contract; it records the legacy code that was removed
and the remaining validation transport that now stays on `worker-light`.

Relevant architecture context:

- `specs/architecture/distributed-execution.md`
- `specs/architecture/CAD-and-other-infra.md`
- `specs/architecture/simulation-and-rendering.md`
- `specs/migrations/major/validation-on-light-worker.md`

## Problem Statement

The repo used to contain a few old heavy-worker validation entrypoints and
helper paths:

1. `worker_heavy/api/routes.py::api_validate` exposed `/benchmark/validate`
   and ran `run_validation_in_isolated_process(...)`.
2. `worker_heavy/activities/heavy_tasks.py::validate_design_activity` defined
   `worker_validate_design` and executed the legacy validation flow for the
   heavy worker queue.
3. `controller/workflows/heavy.py::HeavyValidationWorkflow` and
   `controller/temporal_worker.py` registered the Temporal validation path.
4. `shared/utils/agent/__init__.py::validate` had an `IS_HEAVY_WORKER`
   branch that bypassed worker-light and called the heavy validation helper
   directly.

The normal product path now depends on the worker-light validation surface:

- `controller/middleware/remote_fs.py:1016-1037` now calls the worker-light
  validation path.
- `controller/clients/worker.py:1076-1114` posts validation to worker-light
  when the controller is absent.
- `shared/utils/agent/__init__.py:521-556` falls back to worker-light for local
  validation when no controller is present.

That means the remaining heavy-worker code is now compatibility-only, but it is
still easy to keep around by accident.

## Cleanup Checklist

- [x] Remove the legacy `/benchmark/validate` handler from
  `worker_heavy/api/routes.py`.
- [x] Remove `worker_validate_design` and the heavy validation activity from
  `worker_heavy/activities/heavy_tasks.py`.
- [x] Remove `HeavyValidationWorkflow` from `controller/workflows/heavy.py`
  and its registration from `controller/temporal_worker.py`.
- [x] Remove the `IS_HEAVY_WORKER` validation branch from
  `shared/utils/agent/__init__.py::validate`.
- [x] Keep `worker_light/api/routes.py::api_validate` as the canonical
  validation route and preserve its response shape.
- [x] Update integration tests that still posted validation requests to
  `WORKER_HEAVY_URL` so they target `WORKER_LIGHT_URL` instead.

## Target State

1. Normal validation flows use `worker-light` only.
2. No controller, agent, or worker client path routes new validation calls to
   `worker-heavy` by default.
3. The public validation contract stays unchanged.
4. The removed heavy-worker validation surfaces remain documented only as
   historical context.

## Required Work

### 1. Narrow the remaining heavy-worker validation entrypoints

- Keep the heavy `/benchmark/validate` route and the `worker_validate_design`
  activity clearly marked as compatibility-only.
- If a later cleanup removes them, delete the route, the workflow registration,
  and the activity together so the legacy path does not survive in a partial
  state.

### 2. Keep the worker-light path authoritative

- Keep controller-side validation on the worker-light path in
  `controller/middleware/remote_fs.py:1016-1037`.
- Keep direct worker-client validation on worker-light in
  `controller/clients/worker.py:1076-1114`.
- Keep local Codex validation on worker-light in
  `shared/utils/agent/__init__.py:521-556`.

### 3. Prevent accidental reintroduction

- Avoid adding new callers to `worker_heavy/utils/validation.py:1375-1408` or
  `worker_heavy/utils/validation.py:2074-2274`.
- If validation behavior changes, add the change to worker-light first and then
  decide whether the heavy compatibility layer still needs to mirror it.
- Prefer deleting dead legacy branches over adding new fallback logic.

## Non-Goals

- Do not change validation semantics.
- Do not change the public `BenchmarkToolResponse` shape.
- Do not move simulation or preview rendering back to `worker-heavy`.
- Do not add a new transport for validation.

## Acceptance Criteria

1. A repo-wide search shows that the normal validation path no longer depends
   on `worker-heavy`.
2. The only remaining direct runtime reference to the heavy validation
   implementation is the explicit `IS_HEAVY_WORKER` compatibility branch in
   `shared/utils/agent/__init__.py:531-537`, if that branch is still retained.
3. The worker-light validation route continues to serve the public contract
   from `worker_light/api/routes.py:386-429`.
4. Any later removal of the compatibility surface can be done without changing
   controller or agent-facing validation behavior.

## File-Level Change Set

- `worker_heavy/api/routes.py`
- `worker_heavy/activities/heavy_tasks.py`
- `worker_heavy/utils/validation.py`
- `controller/workflows/heavy.py`
- `controller/temporal_worker.py`
- `worker_light/api/routes.py`
- `controller/middleware/remote_fs.py`
- `controller/clients/worker.py`
- `shared/utils/agent/__init__.py`
