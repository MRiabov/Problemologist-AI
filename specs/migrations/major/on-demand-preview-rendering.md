# On-Demand Preview Rendering

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This document investigates a major refactor that turns preview rendering into an
on-demand helper instead of a validation-time side effect.

The target is a direct `preview(...)` surface that can inspect a live
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

- The preview helper returns a small structured response, not a bare `Path`.
- The helper emits start/finish status text so long renders are visible to the
  caller.
- The current angle-based render naming family stays in use, for example
  `render_e15_a45`, with a run-unique suffix or timestamp when repeated runs
  need disambiguation.
- `renders/render_manifest.json` remains the assumed companion artifact and is
  runtime-owned, read-only to agents, and rewritten atomically as preview
  outputs are added.
- `preview_design(...)` is not kept as a long-term alias; `preview(...)`
  replaces it.
- The helper lives in `utils` as an export layer.
- Benchmark previews stay geometry-only at the helper boundary; benchmark
  geometry and objective boxes are composed into a `Compound(children=[...])`
  by the caller before calling `preview(...)`.

## Why This Exists

- The current system already has the rendering primitives, but they are split
  across validation, preview, and renderer-worker code paths.
- Single-view inspection is useful for engineer coder, benchmark coder, planner,
  and reviewer flows when a full 24-view bundle is unnecessary.
- The current controller-side preview path still routes through the heavy-worker
  Temporal path, which is too expensive for interactive inspection.
- The existing preview helpers are RGB-first and do not expose an explicit
  modality selector.

## Current-State Evidence

The repository already contains several partial preview implementations:

- `worker_heavy.utils.build123d_rendering.render_preview_view(...)` already
  exports a live `Compound` into a preview bundle and renders a single view.
- `worker_heavy.utils.build123d_rendering.render_preview_bundle(...)` already
  renders the full RGB/depth/segmentation preview bundle.
- `worker_renderer/api/routes.py` already serves `/benchmark/preview`,
  `/benchmark/static-preview`, and `/benchmark/simulation-video`.
- `shared/rendering/renderer_client.py` already knows how to call the renderer
  worker.
- `worker_heavy/utils/preview.py` already contains a single-view preview helper,
  but it is RGB-only and still lives under the heavy-worker namespace.
- `controller/middleware/remote_fs.py` currently exposes `preview(...)`, but it
  routes through Temporal and `worker-heavy`.

What is missing is the worker-light-facing, modality-aware `preview(...)`
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

1. Introduce a canonical `preview(...)` helper on the authoring/runtime surface
   that accepts a live `Part | Compound`.
2. Support explicit camera control with `orbit_pitch` and `orbit_yaw`.
3. Support an explicit preview modality selector.
4. Make the worker-light runtime the direct orchestration boundary for preview
   requests.
5. Keep the renderer worker as the only process that touches VTK/OpenGL and
   writes the actual image artifacts.
6. Keep the existing 24-view validation bundle only where the benchmark or
   review contract requires it.
7. Keep on-demand preview separate from simulation render provenance.

### Proposed signature

```py
preview(
    component: Part | Compound,
    orbit_pitch: float = 45,
    orbit_yaw: float = 45,
    rendering_type: RenderingType | Literal["rgb", "depth", "segmentation"],
)
```

### Desired semantics

- `rendering_type="rgb"` should preserve the normal inspection preview.
- `rendering_type="depth"` should produce a depth artifact that remains
  modality-aware and non-lossy.
- `rendering_type="segmentation"` should produce a segmentation artifact and
  preserve legend metadata.
- The helper should return a structured result with at least the artifact path
  and user-facing status text.
- The helper should persist under the workflow-specific preview bundle
  directory, not flatten everything into a single root-level render folder.
- The helper should keep a stable orbit-angle naming convention so repeated
  runs remain visually and textually easy to distinguish in the workspace.
- The helper should orbit around the component centroid.

## API Contract Notes

This refactor should not reuse `shared.models.simulation.RenderMode` for the new
preview selector.

- `RenderMode` already means `static_preview` versus `simulation_video`.
- The new preview tool needs its own modality enum, because `rgb`, `depth`, and
  `segmentation` are preview payload choices, not simulation artifact modes.

The preview request/response shape should stay small and deterministic:

- input: live component, `orbit_pitch`, `orbit_yaw`, modality, optional workspace/output
  hints,
- output: persisted artifact path, plus manifest-backed modality metadata.

<!-- Warning: the implementation previously used `pitch` / `yaw` naming instead of `orbit_pitch` / `orbit_yaw`, so updating the names is due. -->

## Current Call Graph

Current preview flow:

1. Agent or script code requests preview.
2. Worker-light is the direct authoring/runtime entrypoint for the preview
   helper.
3. The helper calls the controller boundary.
4. The controller routes through Temporal.
5. Temporal dispatches the renderer job.
6. `worker_renderer/api/routes.py:/benchmark/preview` loads or reconstructs the
   scene and renders it.
7. The preview artifact is written back to the session workspace.

Target on-demand flow:

1. Agent or script code requests preview.
2. Worker-light owns the authoring/runtime helper.
3. Worker-light passes through the controller orchestration boundary.
4. Controller + Temporal dispatch the render job to `worker-renderer`.
5. The renderer worker emits the selected modality only.
6. The caller receives the structured response and inspects the artifact path
   through the existing media path.

## Why This Is a Major Refactor

- It changes the agent-facing utility surface, not just a backend route.
- It introduces a new modality dimension to preview requests.
- It moves the default preview control path off `worker-heavy`.
- It affects worker schemas, controller proxies, prompt allowlists, and tests.
- It creates a new separation between preview evidence and simulation evidence.

## Non-Goals

- Do not replace the renderer worker.
- Do not send raw geometry objects over HTTP.
- Do not collapse preview into simulation provenance.
- Do not remove the current validation bundle until the benchmark and review
  contracts are updated.
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
- `config/prompts.yaml`
- `config/agents_config.yaml`
- `tests/integration/architecture_p0/test_architecture_p0.py`
- `tests/integration/architecture_p0/test_int_188_validation_preview.py`
- `tests/integration/architecture_p0/test_codex_runner_mode.py`

## Phased Migration Shape

### Phase 1: Add the canonical helper

- Introduce the `preview(...)` helper in the script/runtime utility surface.
- Re-export it from `utils/__init__.py`.
- Replace `preview_design(...)` with `preview(...)`; do not keep a compatibility
  alias.
- Reuse the existing preview-scene bundling machinery instead of inventing a new
  renderer format.

### Phase 2: Make modality explicit

- Add a preview-specific modality enum with `rgb`, `depth`, and
  `segmentation`.
- Extend the renderer request schema so the caller asks for one modality
  explicitly.
- Persist modality-specific artifact metadata in `renders/render_manifest.json`.
- Keep depth and segmentation outputs PNG-based so the data is not lossy.

### Phase 3: Move orchestration to worker-light

- Worker-light should become the direct orchestration boundary for preview.
- The heavy worker should stop being the default preview hop.
- Any controller proxy that still exists during rollout should be a compatibility
  bridge, not the primary path.

### Phase 4: Update role prompts and allowlists

- Expose the new preview helper in the engineer and benchmark tool prompts.
- Add it to any role allowlists that need live inspection of a component or
  assembly.
- Benchmark-facing prompts should point callers at the benchmark geometry
  composition rule instead of expecting hidden benchmark context in the helper
  signature.
- Keep reviewer roles on `inspect_media(...)` for evidence review, but allow
  preview when the role needs to materialize a new view first.

## Risks

- The main risk is geometry serialization drift between the live `Part | Compound`
  and the staged renderer bundle.
- Depth and segmentation artifacts need stronger fidelity guarantees than the
  current JPEG-first single-view helper.
- A partial migration could leave both the heavy-worker preview path and the
  worker-light preview path active at the same time.
- Prompt allowlists can lag behind the code and silently hide the new helper
  from the roles that need it.
- If the preview helper returns inconsistent file naming, the render manifest
  and reviewer gates will drift apart.

## Benchmark Geometry Composition

The preview helper stays geometry-only at the public boundary. Benchmark
context comes from the benchmark geometry source contract defined in
[Benchmark Geometry Source and Read-Only Benchmark Script](../minor/benchmark-geometry-source-and-read-only-script.md).

- `benchmark_script.py` exposes the benchmark assembly through `build()`.
- `objectives_geometry()` is the zero-argument utility that materializes the
  objective boxes from the canonical benchmark definition path for the current
  workspace.
- The caller combines `build()` output with `objectives_geometry()` into a
  `Compound(children=[...])`, or an equivalent composed component, before
  calling `preview(...)`.
- This keeps the public helper singular while still rendering the benchmark
  assembly and objectives together in the same frame.

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
- INT-188 - validation preview backend split contract; the validation-side
  preview bundle still rides the shared renderer and manifest path.
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
- INT-207 - engineer workspace bad-display preview fallback contract; this
  directly exercises the worker-light preview path that is being introduced.
- INT-208 - renderer worker direct preview contract; this becomes the canonical
  smoke test for the new `/benchmark/preview` helper path.

## Rollout Order

1. Add the canonical helper and export it from the script/runtime surface.
2. Thread the modality selector through the renderer request schema.
3. Make the worker-light orchestration path the default for preview.
4. Update prompts, allowlists, and role guidance.
5. Remove or demote the heavy-worker preview bridge only after the new path is
   stable.

## Assessment

This is a real major refactor, but it is not a rewrite.

The rendering engine already exists. The scene-bundling machinery already
exists. The renderer worker already exists. The missing piece is a clean,
worker-light-facing preview contract that makes live inspection explicit and
modality-aware instead of piggybacking on heavyweight validation paths.

## Migration Checklist

### Workflow and routing

- [x] Add the worker-light-facing `preview(...)` helper to the public utils
  surface and keep the boundary geometry-only.
- [x] Add or update the controller preview entrypoint so preview requests flow
  through controller orchestration instead of calling the renderer directly.
- [ ] Add the Temporal workflow/activity path for preview and keep the live
  chain `worker-light -> controller -> Temporal -> worker-renderer`.
- [ ] Persist preview workflow state, correlation metadata, and timeout/failure
  reasons so stalled renders fail closed instead of hanging or silently
  degrading.
- [ ] Return the structured preview response only after the workflow has a
  resolved artifact path, manifest path, modality, and angle metadata.
- [ ] Make the preview workflow accept a render request, persist the request
  context, and return only after the renderer has produced the artifact path and
  manifest entry.
- [x] Keep any heavy-worker preview bridge as a compatibility path during
  rollout only; the default path must become worker-light driven.
- [ ] Define timeout and failure behavior in the workflow so a stalled preview
  fails closed instead of silently hanging or falling back.

### Geometry composition

- [x] Export `objectives_geometry()` as the zero-argument benchmark geometry
  helper from the utils surface.
- [ ] Keep benchmark previews composed at the call site from `build()` plus
  `objectives_geometry()`, not from hidden geometry injected inside the helper.
- [ ] Ensure benchmark input geometry, solution geometry, and whole-assembly
  geometry can all be passed through the same preview path as native
  build123d shapes.
- [ ] Keep the benchmark geometry source read-only for downstream engineering
  roles while still allowing live preview of the composed result.

### Modality and artifacts

- [x] Thread `rgb`, `depth`, and `segmentation` through the renderer request
  schema and the manifest record for each render.
- [x] Update the renderer client, worker schema, and renderer route handlers so
  the modality enum is carried end to end without lossy conversion.
- [x] Preserve the existing preview file naming convention, including the angle
  tags already used in render artifacts.
- [x] Keep the `render_e15_a45`-style angle family, plus a run-unique suffix or
  timestamp where repeated previews need disambiguation.
- [x] Keep `renders/render_manifest.json` synchronized with each preview write
  using atomic update behavior, not ad hoc agent edits.
- [ ] Keep `renders/render_manifest.json` runtime-owned and read-only to agent
  roles, with no direct agent write path.
- [x] Keep benchmark, engineer, and final preview artifacts in their existing
  bucketed directories so workflow provenance stays visible.
- [x] Preserve the current image format policy for depth and segmentation
  previews.

### Prompts and permissions

- [x] Update `utils/__init__.py` and the `shared/utils/agent` export layer so
  `preview(...)` and `objectives_geometry()` are both importable from the agent
  surface.
- [ ] Update `config/prompts.yaml` and `config/agents_config.yaml` so roles that
  need preview access can discover the new helper without stale
  `preview_design(...)` guidance.
- [ ] Update engineer and benchmark prompts, allowlists, and tool exports so
  the new helper is discoverable where live preview is needed.
- [x] Remove or demote any stale guidance that refers to `preview_design(...)`
  or implies benchmark context is hidden inside the helper signature.
- [ ] Keep `inspect_media(...)` as the review-time evidence path rather than a
  generation helper.

### Integration tests

- [ ] Rerun or update `INT-024`, `INT-031`, `INT-032`, `INT-033`, `INT-034`,
  `INT-039`, `INT-188`, `INT-189`, `INT-190`, `INT-204`, `INT-207`, and
  `INT-208`.
- [ ] Add a dedicated controller/Temporal preview-path integration test if no
  existing INT already exercises the full worker-light -> controller ->
  Temporal -> renderer chain.
- [ ] Confirm the benchmark validation and handoff tests still pass with the
  new geometry helper name and preview composition contract.
- [ ] Confirm the render-evidence and review-gate tests still observe the
  correct manifest path, artifact buckets, and media-inspection flow.
- [ ] Confirm the direct renderer smoke test exercises the new worker-light
  preview path rather than a heavy-worker fallback.
