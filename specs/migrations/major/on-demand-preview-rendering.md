---
title: On-Demand Preview Rendering
status: completed
agents_affected:
  - benchmark_planner
  - benchmark_coder
  - engineer_coder
added_at: '2026-03-31T20:20:26Z'
---

# On-Demand Preview Rendering

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This document investigates a major refactor that turns preview rendering into an
on-demand helper instead of a validation-time side effect.

The target is a direct `render_cad(...)` surface that can inspect a live
`build123d.Part` or `build123d.Compound` at a chosen angle and modality without
requiring the heavy validation/simulation path first.

The renderer worker remains the owner of VTK/OpenGL and image generation. The
change is about exposure, orchestration, and artifact policy.

This migration sits on top of the existing split architecture in
[Simulation and Rendering](../../architecture/simulation-and-rendering.md),
[Distributed Execution](../../architecture/distributed-execution.md),
[CAD and other infrastructure](../../architecture/CAD-and-other-infra.md), and
[Agent tools](../../architecture/agents/tools.md).

## Resolved Decisions

- The preview helper is async and returns a small structured job ack, not a
  bare `Path` or a final render bundle.
- The helper emits queued/start/view-ready/finish status text while the
  renderer worker materializes files, so long renders remain observable to the
  caller.
- The helper accepts modality booleans (`rgb`, `depth`, `segmentation`) and
  normalizes scalar camera inputs into view lists before dispatch.
- Zip-paired multi-view requests are supported, with the preview job capped at
  64 rendered views per call.
- The current angle-based render naming family stays in use, with the rendered
  component label prefixed into the basename and a request-scoped view index
  added when repeated poses or multi-view bundles need disambiguation.
- `renders/render_manifest.json` remains the assumed companion artifact and is
  runtime-owned, read-only to agents, rewritten atomically, and keyed by view
  index plus pose metadata as preview outputs are added.
- `preview_design(...)` is not kept as a long-term alias; `render_cad(...)`
  replaces it.
- The helper is exposed by the public `utils` package, alongside `render_cad(...)`
  and `validate(...)`.
- Validation does not generate preview artifacts by default; preview evidence
  is produced only through explicit preview requests.
- Benchmark previews stay geometry-only at the helper boundary; `build()`
  supplies the benchmark assembly geometry and `utils.objectives_geometry()`
  supplies the objective overlays reconstructed from the `objectives` section
  of `benchmark_definition.yaml`.
  The caller composes both into a `Compound(children=[...])`, or an equivalent
  composed component, before calling `render_cad(...)`.

## Why This Exists

- The current system already has the rendering primitives, but they are split
  across validation, preview, and renderer-worker code paths.
- Multi-view inspection is useful for engineer coder, benchmark coder, planner,
  and reviewer flows when a full legacy validation bundle is unnecessary.
- The current controller-side preview path still routes through the heavy-worker
  Temporal path, which is too expensive for interactive inspection.
- The existing preview helpers are RGB-first and do not expose explicit
  modality booleans or multi-view normalization.

## Current-State Evidence

The repository already contains several partial preview implementations:

- `worker_heavy.utils.build123d_rendering.render_preview_view(...)` already
  exports a live `Compound` into a render bundle and renders a single view,
  which is still a useful lower-level primitive for the new async helper.
- `worker_heavy.utils.build123d_rendering.render_preview_bundle(...)` already
  renders the full RGB/depth/segmentation render bundle.
- `worker_renderer/api/routes.py` already serves `/benchmark/preview`,
  `/benchmark/static-preview`, and `/benchmark/simulation-video`.
- `shared/rendering/renderer_client.py` already knows how to call the renderer
  worker.
- `worker_heavy/utils/preview.py` already contains a single-view preview helper,
  but it is RGB-only and still lives under the heavy-worker namespace.
- `controller/middleware/remote_fs.py` currently exposes `render_cad(...)`, but it
  routes through Temporal and `worker-heavy`.

What is missing is the worker-light-facing, modality-aware `render_cad(...)`
surface that the agent/runtime can call directly when it wants one live shot of
the current model.

## Problem Statement

Today, preview rendering is still coupled to heavyweight execution semantics:

1. The controller routes preview through heavy-worker orchestration.
2. The heavy worker forwards to the renderer worker.
3. The renderer worker renders the image and returns a path or bytes.

That works, but it is the wrong default for interactive inspection.

The refactor should make the preview path explicit, direct, and cheap:

- render only when requested,
- render only the requested modality,
- render from the requested angle,
- keep render ownership in `worker-renderer`,
- keep orchestration and artifact syncing in `worker-light`.

## Proposed Target State

1. Introduce a canonical `render_cad(...)` helper on the authoring/runtime surface
   that accepts a live `Part | Compound`.
2. Support explicit camera control with `orbit_pitch` and `orbit_yaw`, using
   scalar-or-list inputs that normalize into view bundles.
3. Support explicit preview modality booleans instead of a single modality
   selector.
4. Cap each preview call at 64 rendered views and pair camera inputs by index.
5. Make the worker-light runtime the direct orchestration boundary for preview
   requests and stream queued/view-ready/completed status over the existing
   websocket control path.
6. Keep the renderer worker as the only process that touches VTK/OpenGL and
   writes the actual image artifacts.
7. Retire the legacy 24-view validation bundle entirely; validation evidence is
   geometry/objective validation only.
8. Keep on-demand preview separate from simulation render provenance.

### Proposed signature

```py
async def render_cad(
    component: Part | Compound,
    orbit_pitch: float | list[float] = 45.0,
    orbit_yaw: float | list[float] = 45.0,
    rgb: bool = True,
    depth: bool = True,
    segmentation: bool = False,
) -> PreviewJobAck
```

### Desired semantics

- `rgb`, `depth`, and `segmentation` are preview payload choices, not
  simulation artifact modes.
- At least one modality boolean must be enabled; the all-false request is
  rejected.
- Scalar `orbit_pitch` / `orbit_yaw` inputs are wrapped into single-item view
  lists at the tool boundary.
- When one camera list has length 1 and the other has length N, the singleton
  broadcasts to the longer list; when both lists are longer than one, their
  lengths must match and are paired by index.
- The helper caps each request at 64 rendered views, counting views rather
  than output files.
- The helper returns a structured job ack immediately and streams queued,
  running, and view-ready status while the renderer worker materializes the
  files.
- The helper should persist under the workflow-specific render bundle
  directory, not flatten everything into a single root-level render folder.
- The helper should keep a stable orbit-angle naming convention plus a
  request-scoped view index so repeated runs remain visually and textually easy
  to distinguish in the workspace.
- The helper should orbit around the component centroid.

## API Contract Notes

This refactor should not reuse `shared.models.simulation.RenderMode` for the new
preview selector.

- `RenderMode` already means `static_preview` versus `simulation_video`.
- The new preview tool needs explicit modality booleans because `rgb`, `depth`,
  and `segmentation` are preview payload choices, not simulation artifact modes.

The preview request/response shape should stay small and deterministic:

- input: live component, `orbit_pitch`, `orbit_yaw`, modality booleans, optional
  workspace/output hints,
- output: structured job ack, then manifest-backed modality and view metadata
  once the renderer worker resolves each view.

<!-- Warning: the implementation previously used `pitch` / `yaw` naming instead of `orbit_pitch` / `orbit_yaw`, so updating the names is due. -->

## Current Call Graph

Current preview flow:

1. Agent or script code requests preview.
2. Worker-light is the direct authoring/runtime entrypoint for the preview
   helper.
3. The helper normalizes the request into a preview job and opens the existing
   websocket control path for queue and progress updates.
4. The controller routes through Temporal.
5. Temporal dispatches the renderer job.
6. `worker_renderer/api/routes.py:/benchmark/preview` loads or reconstructs the
   scene and renders each requested view.
7. The renderer worker streams view-ready updates as each requested render
   completes, and the materialized artifacts are written back to the session
   workspace.

Target on-demand flow:

1. Agent or script code requests preview.
2. Worker-light owns the authoring/runtime helper.
3. Worker-light normalizes scalar inputs into view bundles, submits the job
   through the controller orchestration boundary, and returns a structured job
   ack immediately.
4. Controller + Temporal dispatch the render job to `worker-renderer`.
5. The renderer worker emits the selected modality set only, may fan out the
   requested views internally within the admitted job, and streams view-ready
   notifications as it goes.
6. The caller later inspects the materialized artifact paths through the
   existing media path.

## Why This Is a Major Refactor

- It changes the agent-facing utility surface, not just a backend route.
- It introduces a new modality dimension and a new multi-view normalization
  model to preview requests.
- It moves the default preview control path off `worker-heavy`.
- It affects worker schemas, websocket status delivery, controller proxies,
  prompt allowlists, and tests.
- It creates a new separation between preview evidence and simulation evidence.

## Non-Goals

- Do not replace the renderer worker.
- Do not send raw geometry objects over HTTP.
- Do not collapse preview into simulation provenance.
- Do not route preview generation back through `/benchmark/validate`.
- Do not reintroduce a default validation-time render bundle.
- Do not add a generic render queue.
- Do not change simulation semantics.

## Surface Area Likely Affected

- `utils/__init__.py`
- `utils/submission.py`
- `shared/utils/agent/__init__.py`
- `shared/rendering/renderer_client.py`
- `shared/rendering/__init__.py`
- `shared/workers/schema.py`
- `worker_light/api/routes.py`
- `worker_light/runtime/executor.py`
- `controller/api/routes/script_tools.py`
- `controller/clients/worker.py`
- `controller/clients/worker_ws.py`
- `controller/middleware/remote_fs.py`
- `controller/agent/tools.py`
- `controller/workflows/heavy.py`
- `worker_heavy/activities/heavy_tasks.py`
- `worker_heavy/utils/preview.py`
- `worker_heavy/utils/build123d_rendering.py`
- `worker_renderer/api/routes.py`
- `tests/integration/architecture_p1/test_script_tools_proxy.py`
- `config/prompts.yaml`
- `config/agents_config.yaml`
- `tests/integration/architecture_p0/test_architecture_p0.py`
- `tests/integration/architecture_p0/test_int_188_validation_preview.py`
- `tests/integration/architecture_p0/test_codex_runner_mode.py`

## Phased Migration Shape

### Phase 1: Add the canonical helper

- Introduce the `render_cad(...)` helper in the script/runtime utility surface.
- Re-export it from `utils/__init__.py`.
- Replace `preview_design(...)` with `render_cad(...)`; do not keep a compatibility
  alias.
- Reuse the existing preview-scene bundling machinery instead of inventing a new
  renderer format.

### Phase 2: Make modality explicit

- Replace the single preview modality selector with `rgb`, `depth`, and
  `segmentation` booleans.
- Extend the renderer request schema and manifest so modality-specific artifact
  metadata is persisted per view.
- Keep depth and segmentation outputs PNG-based so the data is not lossy.

### Phase 3: Normalize multi-view camera requests

- Accept scalar-or-list `orbit_pitch` and `orbit_yaw` inputs.
- Normalize scalar camera inputs to single-item lists at the tool boundary.
- Zip-pair the camera lists by index after normalization and reject
  incompatible non-singleton length mismatches.
- Enforce the 64-view cap before the request reaches the renderer worker.

### Phase 4: Stream preview status

- Stream queued, running, and view-ready status over the existing websocket
  control path.
- Keep the structured preview job ack small and immediate.
- Preserve the final manifest as the durable ledger for the materialized views.

### Phase 5: Move orchestration to worker-light

- Worker-light should become the direct orchestration boundary for preview.
- The heavy worker should stop being the default preview hop.
- Any controller proxy that still exists during rollout should be a compatibility
  bridge, not the primary path.

### Phase 6: Update role prompts and allowlists

- Expose the new preview helper in the engineer and benchmark tool prompts.
- Add it to any role allowlists that need live inspection of a component or
  assembly.
- Benchmark-facing prompts should point callers at the benchmark assembly plus
  objective-overlay composition rule instead of expecting hidden benchmark
  context in the helper signature.
- Keep reviewer roles on `inspect_media(...)` for evidence review, but allow
  preview when the role needs to materialize a new view first.

## Risks

- The main risk is geometry serialization drift between the live `Part | Compound`
  and the staged renderer bundle.
- The next highest risk is a mismatch between the view-normalization rule and
  the manifest view index, which would make streamed renders hard to correlate.
- Depth and segmentation artifacts need stronger fidelity guarantees than the
  current JPEG-first single-view helper.
- A partial migration could leave both the heavy-worker preview path and the
  worker-light preview path active at the same time.
- Prompt allowlists can lag behind the code and silently hide the new helper
  from the roles that need it.
- If the preview helper returns inconsistent file naming, the render manifest
  and reviewer gates will drift apart.

## Completion Note

Implementation status: complete for the current on-demand preview surface in this branch. No further functional edits should be made to this migration unless a later architecture revision reopens the scope.

<!-- Frozen after implementation completion: avoid further edits unless the architecture changes. -->

## Benchmark Geometry Composition

The preview helper stays geometry-only at the public boundary. Benchmark
context comes from the benchmark geometry source contract defined in
[Benchmark Geometry Source and Read-Only Benchmark Script](../minor/benchmark-geometry-source-and-read-only-script.md).

- `benchmark_script.py` exposes the benchmark assembly through `build()`.
- `utils.objectives_geometry()` is the zero-argument utility that reconstructs
  the objective overlays declared in the `objectives` section of
  `benchmark_definition.yaml` for the current workspace. It is exposed through
  the public `utils` package, alongside `render_cad(...)` and `validate(...)`, so
  callers import it rather than defining an agent-local geometry helper.
- The caller combines `build()` output with `objectives_geometry()` output into
  a `Compound(children=[...])`, or an equivalent composed component, before
  calling `render_cad(...)`.
- This keeps the public helper singular while still rendering the benchmark
  assembly and objectives together in the same view bundle.

## Test Impact

The migration should keep the existing direct renderer contract and extend it
with modality-aware coverage. The integration catalog entries most likely to be
affected are:

### Rerun first

- INT-024 - benchmark validate toolchain; the shared objective-geometry source
  must still validate cleanly after the helper rename.
- INT-031 - benchmark planner -> plan reviewer -> CAD -> reviewer flow;
  benchmark preview artifacts are part of the benchmark handoff surface.
- INT-032 - benchmark-to-engineer handoff package; the bundle depends on the
  benchmark geometry source and preview evidence composition.
- INT-033 - engineering full loop; coder preview of solution geometry uses the
  new helper and should be rechecked end to end.
- INT-039 - render artifact generation policy; on-demand render discovery and
  objective-box visuals share the same artifact pipeline.
- INT-188 - validation render-free contract; the validation-side path should
  remain render-free by default while explicit preview coverage moves to the
  preview helper tests.
- INT-189 - engineer solution evidence default contract; artifact selection
  order must remain stable after the preview path changes.
- INT-190 - seeded render evidence sanity gate; seeded render artifacts must
  still be recognized as visible or blank consistently.

### Expect assertion updates

- INT-034 - reviewer evidence completeness; render-manifest metadata and
  `inspect_media(...)` evidence may shift with the new preview artifact
  contract.
- INT-204 - latest-revision render inspection gate; revision-scoped render
  bundle lookup may shift with the new preview helper.
- INT-207 - engineer workspace preview renderer delegation contract; this
  directly exercises the worker-light preview path that is being introduced.
- INT-208 - renderer worker direct preview contract; this becomes the canonical
  smoke test for the new `/benchmark/preview` helper path.
- INT-214 - multi-view preview normalization contract; this covers zip pairing,
  scalar normalization, and the 64-view cap.
- INT-215 - preview websocket streaming contract; this covers queued and
  view-ready status propagation while files materialize.

## Rollout Order

1. Add the canonical helper and export it from the script/runtime surface.
2. Replace the single modality selector with modality booleans.
3. Normalize multi-view camera requests and enforce the 64-view cap.
4. Add websocket streaming for queued and view-ready preview status.
5. Make the worker-light orchestration path the default for preview.
6. Update prompts, allowlists, and role guidance.
7. Remove or demote the heavy-worker preview bridge only after the new path is
   stable.

## Assessment

This is a real major refactor, but it is not a rewrite.

The rendering engine already exists. The scene-bundling machinery already
exists. The renderer worker already exists. The missing piece is a clean,
worker-light-facing preview contract that makes live inspection explicit,
multi-view capable, and modality-aware instead of piggybacking on heavyweight
validation paths.

## Migration Checklist

### Workflow and routing

- [x] Add the worker-light-facing `render_cad(...)` helper to the public utils
  surface and keep the boundary geometry-only.
- [x] Add or update the controller preview entrypoint so preview requests flow
  through controller orchestration instead of calling the renderer directly.
- [ ] Add the Temporal workflow/activity path for preview and keep the live
  chain `worker-light -> controller -> Temporal -> worker-renderer`.
- [x] Normalize scalar preview inputs to lists, zip multi-view requests by
  index, and enforce the 64-view cap before dispatch.
- [x] Return the structured preview job ack immediately and keep queue/view-
  ready status visible while the renderer worker is working.
- [ ] Remove validation-time preview generation from `/benchmark/validate` and
  retire the 24-view validation bundle contract.
- [ ] Persist preview workflow state, correlation metadata, and timeout/failure
  reasons so stalled renders fail closed instead of hanging or silently
  degrading.
- [ ] Make the preview workflow accept a render request, persist the request
  context, and return only after the renderer has produced the final manifest
  entry for each requested view.
- [x] Keep any heavy-worker preview bridge as a compatibility path during
  rollout only; the default path must become worker-light driven.
- [ ] Define timeout and failure behavior in the workflow so a stalled preview
  fails closed instead of silently hanging or falling back.

### Geometry composition

- [ ] Export `objectives_geometry()` as the zero-argument objective-overlay
  helper from the utils surface, and keep `build()` as the pre-existing
  benchmark assembly constructor.
- [ ] Keep benchmark previews composed at the call site from `build()` plus
  `objectives_geometry()`, not from hidden geometry injected inside the helper.
- [ ] Ensure benchmark input geometry, solution geometry, and whole-assembly
  geometry can all be passed through the same preview path as native
  build123d shapes.
- [ ] Keep the benchmark geometry source read-only for downstream engineering
  roles while still allowing live preview of the composed result.

### Modality and artifacts

- [ ] Replace `rendering_type` with `rgb`, `depth`, and `segmentation` in the
  agent-facing helper and worker schema.
- [ ] Thread `rgb`, `depth`, and `segmentation` through the renderer request
  schema and the manifest record for each render.
- [ ] Update the renderer client, worker schema, and renderer route handlers so
  the modality set and view list are carried end to end without lossy
  conversion.
- [ ] Preserve the existing preview file naming convention, including the angle
  tags already used in render artifacts, and add request-scoped `view_index`
  metadata for repeated poses.
- [ ] Keep the `render_e15_a45`-style angle family, plus a run-unique suffix or
  timestamp where repeated previews need disambiguation.
- [x] Keep `renders/render_manifest.json` synchronized with each preview write
  using atomic update behavior, not ad hoc agent edits.
- [ ] Keep `renders/render_manifest.json` runtime-owned and read-only to agent
  roles, with no direct agent write path.
- [x] Keep benchmark, engineer, and final preview artifacts in their existing
  bucketed directories so workflow provenance stays visible.
- [x] Preserve the current image format policy for depth and segmentation
  previews.
- [x] Stream per-view ready updates over the websocket control path so callers
  can attach images as they arrive.

### Prompts and permissions

- [x] Update `utils/__init__.py` and the `shared/utils/agent` export layer so
  `render_cad(...)` and `utils.objectives_geometry()` are both importable from the
  agent surface.
- [ ] Update `config/prompts.yaml` and `config/agents_config.yaml` so roles that
  need preview access can discover the async, multi-view helper without stale
  `preview_design(...)` guidance.
- [ ] Update engineer and benchmark prompts, allowlists, and tool exports so
  the new helper is discoverable where live preview is needed.
- [x] Remove or demote any stale guidance that refers to `preview_design(...)`
  or implies benchmark context is hidden inside the helper signature.
- [ ] Keep `inspect_media(...)` as the review-time evidence path rather than a
  generation helper.

### Integration tests

- [ ] Refresh the preview-surface INT slice:
  `INT-032`, `INT-033`, `INT-034`, `INT-039`, `INT-040`, `INT-074`, `INT-075`,
  `INT-181`, `INT-182`, `INT-183`, `INT-185`, `INT-186`, `INT-188`,
  `INT-189`, `INT-190`, `INT-203`, `INT-204`, `INT-205`, `INT-207`,
  `INT-208`, `INT-209`, `INT-212`, `INT-213`, `INT-214`, and `INT-215`.
  Key updates: `INT-033`/`INT-034` add explicit `render_cad(...)` before
  `inspect_media(...)`; `INT-188` stays render-free; `INT-212`-`INT-215`
  cover list-normalized multi-view preview, manifest identity, and websocket
  status streaming; `INT-204`/`INT-074`/`INT-075` keep the latest-revision
  review gate aligned with the new render bundle shape.
- [ ] Add a dedicated controller/Temporal preview-path integration test if no
  existing INT already exercises the full worker-light -> controller ->
  Temporal -> renderer chain.
- [ ] Confirm the benchmark validation and handoff tests still pass with the
  new geometry helper name and preview composition contract.
- [ ] Confirm the render-evidence and review-gate tests still observe the
  correct manifest path, artifact buckets, and media-inspection flow.
- [ ] Confirm the direct renderer smoke test exercises the new worker-light
  preview path rather than a heavy-worker fallback.
