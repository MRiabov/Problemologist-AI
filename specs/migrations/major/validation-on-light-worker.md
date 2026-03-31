# Validation on Light Worker

<!-- Investigation doc. No behavior change yet. -->

## Scope Summary

- This document investigates a potential refactor that moves benchmark `validate()` execution out of `worker-heavy` and into `worker-light`.
- The controller remains the agent-facing entrypoint.
- The renderer worker remains the owner of static preview rendering.
- This is a cross-cutting refactor candidate, not an accepted architecture change.

## Why This Is Being Considered

- `validate()` is a frequent LLM touchpoint and is usually near-instant relative to simulation.
- The current route pays controller, bundle, and Temporal overhead even when the geometry check is cheap.
- The static preview path already has a dedicated renderer-worker seam.
- The validation result and render manifest are artifact problems as much as they are geometry problems.

## Current-State Path

1. The controller receives the agent-facing `validate` call through the script-tools surface.
2. `controller/middleware/remote_fs.py` bundles the workspace and routes the operation through Temporal.
3. Temporal dispatches `worker_validate_design` on the heavy-task queue.
4. `worker_heavy` loads the script, validates geometry, and materializes validation artifacts.
5. The controller syncs the returned artifacts back to the session filesystem.

The preview side of that path is already split away from the physics stack:

1. The renderer worker exposes a dedicated static-preview endpoint.
2. `shared/rendering/renderer_client.py` already contains a direct client for that service.
3. The renderer worker already synthesizes or persists `render_manifest.json` in its own preview flow.

## Proposed Target State

1. The controller keeps the same `validate` tool surface.
2. `worker-light` performs script loading, geometry validation, and validation result persistence.
3. `worker-light` queries `worker-renderer` directly for static preview generation.
4. `worker-light` finalizes or updates `renders/render_manifest.json` after preview generation completes.
5. `worker-heavy` is no longer the default validation execution boundary.
6. The controller remains the only agent-facing transport boundary; agents do not gain direct renderer or worker-light transport.

## Why The Renderer Must Be Queried Directly

- Static preview generation is already isolated in the renderer worker.
- The refactor should not pull VTK, OpenGL, or preview post-processing back into the controller or light worker.
- Renderer output remains preview evidence, not simulation parity evidence.

## Manifest Ownership Question

- Today, `render_manifest.json` is already synchronized in shared filesystem plumbing and renderer-side flows.
- If validation moves to `worker-light`, the manifest should likely be finalized there, because that worker will know the complete artifact set before handoff.
- This needs one canonical builder path. Duplicating manifest logic in the controller, light worker, and renderer worker would be brittle.

## Surface Area Likely Affected

- `controller/api/routes/script_tools.py`
- `controller/middleware/remote_fs.py`
- `controller/clients/worker.py`
- `worker_heavy/api/routes.py`
- `worker_heavy/activities/heavy_tasks.py`
- `worker_heavy/utils/validation.py`
- `worker_renderer/api/routes.py`
- `shared/rendering/renderer_client.py`
- `shared/workers/filesystem/router.py`
- `shared/workers/schema.py`
- `tests/integration/**` validation, preview, handover, and end-to-end workflow suites

## Likely Test Impact

- Controller tool-path integration tests will need to assert the new light-worker route.
- Renderer preview tests will need to verify direct renderer calls still produce the same artifacts.
- Handover and review tests will need to confirm `validation_results.json` and `render_manifest.json` still appear where downstream stages expect them.
- Any tests that currently assume validation is a heavy-worker admission event will need to be rewritten, not merely updated.

## Risks

- This refactor is broader than a transport swap because artifact ownership changes with it.
- Validation and preview can drift if the preview response and manifest are built in different places.
- A partial migration could leave the controller with two overlapping validation paths, which would be harder to reason about than the current single path.
- If the controller keeps the same proxy but worker-light takes the work, the system still needs a clear failure model for light-worker busy states.

## Open Questions

- Should the controller still bundle the workspace before calling `worker-light`, or should `worker-light` own the bundling step?
- Should preview generation be a direct renderer call from `worker-light`, or should `worker-light` call the renderer through a shared client?
- Should `render_manifest.json` be built in `worker-light`, in shared filesystem plumbing, or by the renderer worker and then normalized by `worker-light`?
- Should the light-worker path preserve the same `BenchmarkToolResponse` schema, or does the refactor justify a narrower validation response type?
- Which exact validation tests need to move from heavy-worker assumptions to light-worker assumptions?

## Non-Goals

- Do not change validation semantics.
- Do not change the fast geometry + preview contract.
- Do not move simulation, submit, or DFM analysis off `worker-heavy`.
- Do not expose additional agent-facing worker transports beyond the controller proxy.

## Assessment

- This refactor is plausible.
- It is not small.
- The preview, render, and manifest boundary is the real risk, not the geometry checks themselves.
- The safest migration shape is likely: controller proxy -> worker-light validation orchestration -> renderer-worker preview call -> shared manifest materialization.
