---
title: Remove the worker-renderer local render shim
status: migration
agents_affected: []
added_at: '2026-03-31T17:16:00Z'
---

# Remove the worker-renderer local render shim

## Purpose

`WORKER_RENDERER_LOCAL_RENDER` exists only as a migration shim. It keeps the renderer service from self-calling the HTTP client path while the renderer code still reuses the older `worker_heavy` preview helpers.

That shim is not part of the target architecture. The renderer worker is supposed to be a dedicated headless service, and the public render contract is supposed to flow through the split worker plane described in:

- `specs/architecture/distributed-execution.md`
- `specs/architecture/simulation-and-rendering.md`
- `specs/splitting-into-renderer-worker.md`

The local render branch is not a valid fallback path. It should be removed after the current renderer-area work lands the replacement renderer-owned helper path.

## Problem Statement

The current codebase still carries a two-path render implementation:

1. `worker_heavy` delegates static preview rendering to `worker-renderer` over HTTP.
2. `worker-renderer` sets `WORKER_RENDERER_LOCAL_RENDER=1` and then renders locally through the same shared helper code.

That arrangement is fragile for two reasons:

- The renderer service is still coupled to a helper that was written for the heavy worker.
- The branch exists to avoid recursion, not because the local path is a supported architecture choice.

The result is a compatibility layer that can mask real failures. If the local path is broken, the renderer service should fail explicitly or use its own internal renderer helper, not silently switch between local and remote behavior.

## Current-State Inventory

The following runtime pieces still depend on the shim or on the older display-based renderer bootstrap:

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `worker_renderer/app.py` | Sets `WORKER_RENDERER_LOCAL_RENDER=1` on startup | This bakes the shim into the renderer process instead of removing it |
| `worker_heavy/utils/rendering.py` | Reads `WORKER_RENDERER_LOCAL_RENDER` and chooses local build123d/VTK rendering when it is present | The helper must have one behavior, not two |
| `worker_renderer/api/routes.py` | Falls back to `prerender_24_views()` for script-backed static preview | That route is currently what makes the shim necessary |
| `worker_renderer/Dockerfile` | Starts `Xvfb` and exports `DISPLAY` before launching Uvicorn | The renderer should be headless by construction, not by local Xvfb bootstrapping |
| `worker_heavy/utils/vtk_display.py` | Still validates ambient `DISPLAY` and describes private-Xvfb fallback behavior | This is the old display contract in code form |
| docs and tests | Still mention the removed hint or the private-Xvfb/display workaround | Documentation must not keep a dead path alive |

## Target State

The migration is complete only when the renderer path has one clear contract:

1. `worker-heavy` always delegates preview rendering to `worker-renderer` over the shared renderer client.
2. `worker-renderer` executes render work through renderer-owned internal helpers, not through a branch in `prerender_24_views()`.
3. No runtime component reads `WORKER_RENDERER_LOCAL_RENDER`.
4. The renderer container starts without `Xvfb`, without relying on an ambient desktop session, and without requiring `DISPLAY` or `XAUTHORITY`.
5. The public artifact contract stays unchanged: `BenchmarkToolResponse`, `render_manifest.json`, and workspace-visible render files keep the same shapes and paths.

## Required Work

### 1. Remove the startup shim

- Delete `os.environ.setdefault("WORKER_RENDERER_LOCAL_RENDER", "1")` from `worker_renderer/app.py`.
- Remove any equivalent env injection from compose, bootstrap scripts, or test runners.
- Treat the renderer service as the owner of its own execution path instead of as a special-case local mode for the shared helper.

### 2. Remove the branch from the shared preview helper

- Delete the `WORKER_RENDERER_LOCAL_RENDER` condition from `worker_heavy/utils/rendering.py`.
- Make `prerender_24_views()` deterministic in one direction only.
- Keep the remote delegation path as the heavy-worker contract unless that helper is replaced entirely by a cleaner renderer entrypoint.
- Preserve output content and artifact locations while removing the conditional local behavior.

### 3. Stop the renderer route from depending on the shim

- Replace the `worker_renderer/api/routes.py` script-backed static preview fallback with a renderer-owned helper that does not call back through `shared/rendering/renderer_client.py`.
- Keep `api_preview`, `api_static_preview`, and `api_simulation_video` on the renderer side, but make them use renderer-internal execution rather than the compatibility branch.
- Eliminate any self-call recursion risk when the renderer handles its own preview request.

### 4. Remove display bootstrapping from the renderer container

- Delete the `Xvfb` bootstrap from `worker_renderer/Dockerfile`.
- Remove the container-level assumption that `DISPLAY` must exist for the renderer service to function.
- Keep only the graphics libraries actually required by the final headless renderer backend.
- If the replacement renderer implementation still needs a display, the migration is not done yet.

### 5. Clean up the old display contract code

- Rework or delete `worker_heavy/utils/vtk_display.py` if it exists only to support the retired private-Xvfb path.
- Remove prose that claims the renderer container owns a private Xvfb session.
- Remove any requirement that renderer startup depends on ambient `DISPLAY` or `XAUTHORITY`.

### 6. Update infrastructure entrypoints

- Update `docker-compose.yml` so the renderer service matches the new headless startup contract.
- Update `scripts/env_up.sh`, `scripts/env_down.sh`, and `scripts/internal/integration_runner.py` so they no longer encode the retired shim assumptions.
- Keep service health checks, dependency ordering, and port wiring consistent with the renderer service remaining containerized and internal-only.

### 7. Update tests and documentation

- Remove or rewrite tests that assert the old hint, the bad-display workaround, or the private-Xvfb fallback.
- Update `docs/nightly-work-plans/nightly-work-plan.md` and any experiment notes that treat the removed path as valid.
- Add or update integration coverage for the renderer boot contract, static preview handoff, and simulation-video handoff.

## Non-Goals

- Do not change the agent-facing tool names.
- Do not change the public `BenchmarkToolResponse` or render artifact schema.
- Do not move validation or simulation orchestration back onto the controller.
- Do not introduce a second renderer API.
- Do not keep a permissive local fallback just to make the migration easier.

## Sequencing

This migration is intentionally sequenced after the current renderer-area work that replaces the shared-helper local branch with a renderer-owned helper path.

The cleanup should land as a small follow-on change:

1. The renderer replacement path becomes available.
2. The shim is removed from the shared helper and renderer startup.
3. The container and test harness stop asserting the retired display behavior.
4. The repository no longer treats local rendering as a supported runtime mode.

## Acceptance Criteria

1. A repo-wide search across runtime code, compose files, scripts, and tests finds no live reference to `WORKER_RENDERER_LOCAL_RENDER`. This spec may still mention the term for historical context until the cleanup lands.
2. `worker-renderer` starts and serves `/health` and `/ready` without setting `DISPLAY`, `XAUTHORITY`, or launching `Xvfb`.
3. `worker_renderer/api/routes.py` does not depend on `prerender_24_views()` as a local fallback and cannot recurse into the renderer HTTP client path.
4. `worker_heavy/utils/rendering.py` has one render behavior only; there is no conditional local branch behind an environment variable.
5. Static preview and simulation-video requests still produce the same workspace-visible files, manifest data, and `BenchmarkToolResponse` shapes expected by the existing render contract.
6. The renderer-related integration slice passes through `./scripts/run_integration_tests.sh` after the migration, including the renderer boot and preview handoff checks.
7. No remaining doc or test fixture describes the retired local pipeline, private-Xvfb fallback, or bad-display workaround as a valid supported behavior.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce the new contract:

- `worker_renderer/app.py`
- `worker_heavy/utils/rendering.py`
- `worker_renderer/api/routes.py`
- `worker_renderer/Dockerfile`
- `worker_heavy/utils/vtk_display.py` if the helper is only used for the retired path
- `docker-compose.yml`
- `scripts/env_up.sh`
- `scripts/env_down.sh`
- `scripts/internal/integration_runner.py`
- renderer-related integration tests
- the renderer workaround notes in `docs/`

If a later renderer refactor introduces a cleaner internal render helper, this migration spec still applies: the shim must disappear, and the renderer must not depend on the old branch to function.
