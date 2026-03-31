# Splitting Into Renderer Worker

## Purpose

This document is the implementation plan for moving all headless rendering into a dedicated `worker-renderer` service.

The goal is not to redesign the whole platform. The goal is to stop rendering flakiness, remove the current X11/Xvfb coupling, and keep the agent-facing contract stable while the render backend becomes a separate containerized worker in dev, integration, and production.

This plan operationalizes the architecture already described in:

- [Simulation and Rendering](./architecture/simulation-and-rendering.md)
- [Distributed Execution](./architecture/distributed-execution.md)
- [CAD and other infrastructure](./architecture/CAD-and-other-infra.md)

## Non-goals

- Do not change the agent-facing tool names or response shapes unless a test explicitly requires a new field.
- Do not introduce a new public render API surface for agents.
- Do not build a generic render queue or a new orchestration tier just for rendering.
- Do not keep a host-side or in-process renderer as a fallback path.
- Do not send raw Python geometry objects across the network.

## Decision

Rendering becomes a separate containerized worker:

- `controller` remains orchestration only.
- `worker-light` remains workspace, git, and light execution.
- `worker-heavy` remains validation/simulation orchestration and physics execution.
- `worker-renderer` owns all VTK, EGL, OpenGL, and render post-processing work.

The agent-facing boundary does not change. The internal boundary does:

- public tools still call `validate`, `simulate`, `preview`, and `submit` through the existing controller/Temporal/worker path;
- `worker-heavy` becomes the only service that talks to `worker-renderer`;
- `worker-renderer` is internal-only and containerized in every environment.

## Render Contract

Use one renderer service and one endpoint family. Static preview and heavy simulation rendering should hit the same service contract and differ only by job kind.

Recommended shape:

- `POST /render`
- body includes `job_kind`, `session_id`, `episode_id`, `input_ref`, scene reference or bundle reference, camera policy, backend hint, and output policy
- response includes `success`, `message`, `artifact_paths`, `manifest_path`, and object-store keys when applicable

Example job kinds:

- `static_preview`
- `selection_snapshot`
- `simulation_video`
- `depth`
- `segmentation`

The renderer should remain single-flight per instance. If busy, it should fail closed with the same deterministic busy behavior used elsewhere in the worker plane.

## Data Flow

The source of truth stays in the session workspace, not in controller memory and not in the renderer service.

Recommended flow:

1. Authoring and session files live in the normal worker filesystem.
2. `worker-heavy` stages the minimal render input bundle for the render job.
3. `worker-renderer` consumes that bundle or bundle reference.
4. `worker-renderer` writes render outputs and manifest data.
5. `worker-heavy` mirrors only the required artifacts back into the existing handoff path.

The renderer must not receive raw `build123d.Compound` objects over HTTP. It should receive a staged scene bundle or a reference to staged files.

For static CAD preview, that bundle can contain the authored script and the workspace files needed to reconstruct the geometry, or a synthesized `preview_scene.json` bundle when the geometry already exists in memory and no persistent `script.py` is available.
For MuJoCo, FEM, and fluid jobs, the bundle can contain the backend-produced scene state, meshes, fields, or particle snapshots required for rendering.
For simulation video, the bundle can contain captured frame files plus a small manifest of frame paths and encoding parameters; the renderer worker encodes the MP4 from those files rather than from in-memory frame arrays.
The bundle transport itself is a gzipped tarball encoded as base64 so it can cross the existing JSON HTTP boundary without introducing a new binary upload protocol.
When a preview bundle is scene-backed, the renderer must preserve modality metadata in the manifest, including depth/segmentation groupings and segmentation legends, rather than flattening every PNG to RGB.

This is the key rule:

- raw compounds stay in the authoring layer;
- meshes and scene bundles are filesystem artifacts;
- render requests carry references, not in-memory geometry objects.

## Storage Model

| Data class | Canonical storage | Transfer policy |
| -- | -- | -- |
| Authoring sources | Session workspace on the worker filesystem | Referenced by path or staged bundle |
| Render intermediates | Renderer temp workspace | Ephemeral, not agent-facing |
| Preview images and manifest | `renders/` in the session workspace | Synced back for inspection |
| Large videos / bulky outputs | S3 or MinIO | Upload directly, return object keys |
| Validation / simulation records | Session workspace files | Keep existing `validation_results.json` and `simulation_result.json` contract |

This means the renderer should not push large artifacts through the controller if it can avoid it.

If an artifact is small and needed immediately for inspection, it can still be mirrored into the workspace path.
If an artifact is large, especially MP4 output, the renderer should upload it directly to object storage and return a pointer.
If the current prototype keeps MP4 materialization local, the renderer should still be the component that creates the video file and returns a workspace-relative artifact path.
When a large artifact is object-store-backed, the workspace should keep the pointer and manifest metadata, not a redundant byte-for-byte copy unless a specific test or review path needs the local file.

## Network Boundary

The network boundary should be:

`agent/controller -> worker-light -> Temporal -> worker-heavy -> worker-renderer`

Important rules:

- no browser or agent should call `worker-renderer` directly;
- no controller route should depend on renderer internals;
- the controller should only see the existing worker contract and the same artifact paths it already understands;
- the renderer should never become the public entrypoint for agent tool calls.

This keeps the public API stable and confines render-specific complexity to the worker plane.

## Implementation Steps

1. Extract the render-specific code into a new `worker_renderer/` package.

   - Move the build123d/VTK rendering code, preview helpers, and render manifest generation there first.
   - Keep the validation and simulation logic in `worker-heavy`.

2. Add a container image and compose service for `worker-renderer`.

   - The renderer image should be headless by construction.
   - It should not depend on host display state or `xvfb-run`.
   - It should have its own `/health` and `/ready` endpoints.
   - Update `docker-compose.yml`, `scripts/env_up.sh`, `scripts/internal/integration_runner.py`, and any test compose overlay so the renderer starts and is health-checked in dev and integration.

3. Add a renderer client inside the worker plane.

   - Prefer a `worker-heavy -> worker-renderer` client or adapter.
   - Keep the controller-side tool surface unchanged.
   - Do not add a new agent-facing renderer client.

4. Delegate render jobs from `worker-heavy` to `worker-renderer`.

   - Validation still validates.
   - Simulation still simulates.
   - Only the render step moves out.
   - Preserve the current `BenchmarkToolResponse` / `SimulationArtifacts` contract so the controller keeps seeing the same artifact fields.
   - Simulation video generation moves with the same rule: `worker-heavy` captures the frames or render inputs, `worker-renderer` encodes the video.

5. Remove graphics dependencies from `worker-heavy` after the new service is stable.

   - Drop the ambient display assumption.
   - Remove `xvfb-run` and X11-specific bootstrap from the heavy worker image.
   - Keep render dependency ownership only in the renderer image.
   - Remove any remaining X11-specific bootstrap assumptions from the heavy-worker startup path once tests pass.
   - Move simulation video encoding out of `worker-heavy` once the renderer worker can encode captured MuJoCo and Genesis frame bundles.

6. Keep render artifact sync behavior stable.

   - The renderer may write files to its own workspace or upload directly to object storage.
   - The final handoff into the agent-visible workspace should still produce the same `renders/` paths and `render_manifest.json`.

7. Make rendering on-demand.

   - Static validation preview remains generated when validation requires it.
   - Simulation videos and extra render variants should be generated only when the workflow requests them.
   - Do not pre-render all views or all media types just because the renderer exists.

## Latency and Scale

At the current prototype scale, latency concerns are secondary to reliability.

Expected overhead:

- one extra local service hop for render jobs;
- one bundle/reference transfer for the render input;
- optional object-store upload for large media.

That is acceptable because the current failure mode is not network latency, it is render instability.

For the prototype, prioritize determinism and service isolation.
If the system later grows enough that render throughput matters, add caching or queueing later, not now.

## Test Plan

The test policy stays the same:

- integration tests must exercise real compose services and HTTP boundaries;
- render tests must not be reduced to in-process unit tests;
- do not mock the renderer service in integration coverage except for unavoidable third-party instability.

### Existing tests to edit

- `INT-001`: include `worker-renderer` in the compose health contract.
- `INT-188`: keep the validation preview contract, but update the assertions so they reflect renderer-worker delegation and the lack of any ambient display requirement.
- `INT-039`: update render/video assertions so they check renderer-worker ownership and storage behavior for large outputs.
- `INT-062` or equivalent worker OpenAPI coverage: include the renderer service OpenAPI artifact if the renderer exposes its own API schema.
- `INT-207`: retire the old private-Xvfb fallback contract. That behavior is no longer part of the design.

### New tests to add

- `INT-208`: renderer worker boots headless with no `DISPLAY` and passes `/health` and `/ready`.
- `INT-209`: `/benchmark/validate` and `/benchmark/preview` still produce the expected preview artifacts, but the render work is executed by `worker-renderer`.
- `INT-210`: MuJoCo simulation video rendering emits a real MP4 through `worker-renderer` and persists the artifact path or object key without routing raw frame payloads through the controller.
- `INT-211`: Genesis-backed render jobs use the same renderer service contract and persist bulky render outputs by pointer or object key, not by copying raw bytes through the controller.

### Tests to remove or rewrite

- Any in-process test that only proves the old bad-display/Xvfb fallback should be deleted or rewritten.
- The old fallback-specific regression in `tests/integration/architecture_p0/test_codex_runner_mode.py` should be replaced with a headless renderer boot smoke test.
- Any fixture or mock response that assumes a private-Xvfb recovery log should be updated or retired.

### Test effects by area

- Most mock-response scenarios should remain valid because the agent-visible artifact paths do not change.
- Tests that assert service topology, health checks, or render bootstrap logs will need updates.
- Tests that only inspect `renders/` contents should usually not need path changes.

## Rollout Order

1. Add the renderer service and make it pass a no-display smoke test.
2. Switch the static preview path first.
3. Switch simulation video and other heavy render jobs next.
4. Remove the render stack from `worker-heavy` only after the new service has proven stable.
5. Delete the old fallback logic once no tests or code paths depend on it.

## Explicit Answers To The Main Questions

- Agent boundary: unchanged.
- Latency: the extra hop exists, but it is not the main concern at prototype scale.
- Same endpoints: yes, one renderer service contract should handle quick preview, MuJoCo video, Genesis video, and heavy render jobs.
- Model storage: session workspace remains the source of truth.
- Raw compounds vs meshes: do not send raw compounds. Send staged bundles or file references, with meshes and backend scene data serialized as files when needed. For preview handoff, prefer a `preview_scene.json` plus mesh bundle over persisting or depending on a `script.py` entrypoint.
- On-demand rendering: yes, render when requested or required by workflow policy, not proactively for everything.
