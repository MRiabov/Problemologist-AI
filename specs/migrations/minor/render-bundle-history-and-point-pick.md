---
title: Render Bundle History and Point-Pick Queries
status: investigation
agents_affected:
  - benchmark_reviewer
  - engineer_coder
  - engineer_execution_reviewer
added_at: '2026-04-02T08:33:51Z'
---

# Render Bundle History and Point-Pick Queries

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration adds a render-bundle history contract and a worker-light
render-query helper family. The target is to let the runtime resolve a
screen-space click into a world-space hit against an immutable render bundle
without changing preview ownership, simulation ownership, or the existing
visual-inspection policy.

The target contract is described in:

- [CAD and other infrastructure](../../architecture/CAD-and-other-infra.md)
- [Simulation and Rendering](../../architecture/simulation-and-rendering.md)
- [Auxiliary Agent Tools](../../architecture/agents/auxiliary-agent-tools.md)

## Problem Statement

The current render path is useful for image review, but it is not yet a full
history/query system.

1. A render response can still be treated as "the latest render" instead of as
   one immutable bundle in a larger history.
2. The current manifest path is convenient for current-revision tooling, but it
   is not enough for historical lookup or replayable point queries.
3. Video review already splits `mp4` artifacts into images for multimodal
   inspection, but that attachment path does not preserve object pose history.
4. A pixel is only meaningful relative to one exact bundle snapshot. Without
   that snapshot, point queries can drift when a later render overwrites the
   visible surface.
5. The model needs a helper that can say "what world point is under this pixel
   in this bundle?" instead of forcing it to infer coordinates from image bytes
   or stale filenames.

## Current-State Inventory

| Area | Current behavior | Why it matters |
| -- | -- | -- |
| `worker_renderer/api/routes.py` | Synthesizes a preview manifest from the current render paths and returns the current preview payload. | The preview output is still current-bundle oriented, not explicitly history indexed. |
| `shared/rendering/renderer_client.py` | Materializes preview responses into the workspace and writes one manifest companion for preview bundles. | The preview materializer has no durable append-only history surface yet. |
| `worker_heavy/simulation/media.py` and `worker_heavy/simulation/renderer.py` | Capture MuJoCo simulation frames, provenance, and final simulation-video artifacts from the physics backend. | Simulation metadata is heavy-owned, so the bundle contract must account for a second producer. |
| `shared/workers/schema.py` | Defines per-artifact render metadata and the current render manifest shape. | The schema captures artifact metadata, but not a typed bundle index entry or the bundle-local lookup contract. |
| `shared/rendering/preview_scene.py` and `worker_renderer/utils/build123d_rendering.py` | Already build and serialize preview-scene state plus the camera math used by preview renders. | These are the right base for bundle-local scene snapshots, but they are still preview-oriented. |
| `controller/agent/nodes/base.py` | Resolves current-revision render paths from the manifest and ignores older bundles when a newer revision exists. | Historical lookup can fail or appear empty if the query path depends only on the latest published bundle entry. |
| `controller/middleware/remote_fs.py`, `controller/clients/worker.py`, `controller/api/routes/datasets.py`, `controller/api/routes/script_tools.py`, `controller/agent/review_handover.py`, `worker_heavy/utils/handover.py` | Read or forward the current render manifest and review evidence paths through current-bundle plumbing. | These readers and collectors need to stay compatible with the bundle-local manifest while the append-only index lands. |
| `worker_light/tools/topology.py` | Inspects topology by part/face/edge index only. | It is useful for local geometry inspection, but it does not answer screen-space point queries. |
| `worker_light.utils` | Exposes runtime helper plumbing without a render-query family. | The helper surface exists, but the query contract is not yet implemented. |

## Proposed Target State

01. Every render-producing request publishes an immutable bundle directory under
    `renders/**`, with preview bundles authored by `worker-renderer` and MuJoCo
    simulation bundles authored by `worker-heavy`.
02. Each bundle carries a bundle-local manifest and any lookup sidecars required
    by agent tooling.
03. The append-only discovery surface is `renders/render_index.jsonl`. Each row
    records `bundle_id`, `created_at`, `revision`, `scene_hash`, bundle path,
    and primary media paths for one published bundle.
04. `renders/render_manifest.json` is removed from the active contract; the
    bundle-local manifest is the source of truth for each published bundle.
05. `preview_scene.json` remains the exact bundle-local scene snapshot used for
    preview rendering.
06. `frames.jsonl` stores sparse frame metadata for MuJoCo simulation evidence.
07. `objects.parquet` stores dense object pose tables for query helpers.
    The table is produced by the active `PhysicsBackend` implementation for
    the bundle, so both MuJoCo and Genesis can emit it. Preview bundles may
    not need the table.
08. `worker_light.utils.render_query` exposes the planned helper family:
    - `pick_preview_pixel(...)`
    - `list_render_bundles(...)`
    - `query_render_bundle(...)`
09. `pick_preview_pixel(...)` resolves a click against the bundle-local scene
    snapshot and returns a structured world-space hit record.
10. `query_render_bundle(...)` returns compact frame/object slices instead of
    full media blobs.
11. `inspect_media(...)` continues to split videos into review frames when the
    config allows it, but that review path does not replace the bundle-local
    query contract.

## Required Work

### 1. Publish render bundles explicitly

- Keep render outputs in immutable bundle directories instead of treating the
  latest manifest as the only durable artifact.
- Write a bundle-local manifest beside each bundle.
- Treat the producer as part of the contract: preview bundles are written by
  `worker-renderer`; MuJoCo simulation bundles are written by `worker-heavy`.
- Append a history row for each published bundle to `renders/render_index.jsonl`.
- Carry the bundle-local manifest through current-bundle compatibility paths
  only after the bundle-specific files are written.

### 2. Persist lookup sidecars

- Persist `preview_scene.json` with the exact scene snapshot used for preview
  rendering.
- Persist `frames.jsonl` for frame-level MuJoCo video metadata when a bundle
  includes `mp4` evidence.
- Persist `objects.parquet` from the `PhysicsBackend` object-pose export
  path when the bundle needs query-time inspection.
- Keep the sidecar format small and query-oriented. Do not promote the bundle
  manifest into a full data lake.

### 3. Add the point-pick helper family

- Implement `worker_light.utils.render_query` as the worker-light helper
  namespace for render lookup and ray-pick.
- Make `pick_preview_pixel(...)` resolve a pixel through the bundle-local scene
  snapshot, then raycast against the persisted build123d geometry.
- Make the helper return a typed hit record with bundle identity, view index,
  pixel coordinates, ray origin/direction, world point, hit state, and object
  identity fields.
- Make batch queries use one request object per pick. Do not rely on
  parallel scalar/list broadcasting as the stable contract.

### 4. Keep inspection and history lookup aligned

- Update `inspect_media(...)` to resolve metadata from the bundle-local
  manifest when the inspected file belongs to a published bundle.
- Remove the global `renders/render_manifest.json` plumbing and route readers
  directly through bundle-local manifests and the append-only index.
- Update controller-side current-revision discovery so historical bundles stay
  discoverable through the append-only index instead of only through the latest
  published bundle.

### 5. Update docs and tests

- Keep the owning architecture docs as the canonical description of the render
  output contract.
- Update render-related integration coverage to exercise:
  - current-bundle lookup
  - historical-bundle lookup
  - bundle-local manifest resolution
  - point-pick query responses
  - `mp4` review frame attachment

## Non-Goals

- Do not move preview rendering out of `worker-renderer`.
- Do not change simulation ownership or simulation-video provenance.
- Do not replace `inspect_media(...)` with the point-pick helper.
- Do not infer world coordinates from video bytes alone.
- Do not introduce `objects.npz`; dense object tables use `objects.parquet`.
- Do not collapse all renders back into a single unscoped root directory.
- Do not change the visual-inspection policy or the modality flags.

## Sequencing

The migration should land in this order:

1. Publish bundle-local manifests and the append-only render index.
2. Persist the preview scene snapshot and bundle sidecars.
3. Add the worker-light render-query helper family.
4. Update `inspect_media(...)` and controller discovery to use the bundle
   history contract.
5. Update integration tests and any narrative docs that still assume a single
   mutable render companion file.

## Execution Checklist

This checklist expands the migration into implementation-sized steps. The
bundle contract is only considered landed once the append-only history path,
the bundle-local lookup path, and the worker-light query helpers all agree on
the same bundle identity.

### 1. Freeze the bundle contract

- [x] Define the bundle identity fields that must exist for every published
  render artifact: `bundle_id`, `created_at`, `revision`, `scene_hash`, and
  bundle path.
- [x] Remove the root-level `renders/render_manifest.json` path from the
  active reader contract and route readers directly to bundle-local manifests
  or the append-only index.
- [x] Make the bundle-local manifest the authoritative source for one published
  bundle.
- [ ] Keep the manifest compact enough that a model can inspect it without
  reading the full media payload.
- [x] Preserve the existing preview contract while the new index lands, so
  current-bundle lookups continue to work during the transition.

### 2. Separate the render producers

- [ ] Keep preview rendering on `worker-renderer`.
- [ ] Keep simulation-video rendering switchable between `worker-renderer` and
  `worker-heavy`.
- [ ] Default simulation-video rendering to `worker-heavy`/MuJoCo while the
  renderer-side video backend remains unimplemented.
- [ ] Add the switch in a way that makes the intended target explicit rather
  than relying on hidden routing behavior.
- [ ] Leave a clean path for a future renderer-side video backend such as VTK
  or an equivalent stack.

### 3. Publish immutable bundles

- [x] Write each preview bundle into its own immutable bundle directory.
- [x] Write each MuJoCo simulation-video bundle into its own immutable bundle
  directory.
- [x] Emit the bundle-local manifest only after the bundle contents are fully
  materialized.
- [x] Append one row per published bundle to `renders/render_index.jsonl`.
- [x] Include bundle-local paths to the preview scene snapshot and any
  per-bundle sidecars.

### 4. Persist query sidecars

- [x] Persist `preview_scene.json` for preview bundles as the exact scene
  snapshot used to render the image.
- [x] Persist `frames.jsonl` for simulation bundles that contain video evidence
  or frame-level metadata.
- [x] Persist `objects.parquet` from the `PhysicsBackend` object-pose export
  path when a bundle needs dense object pose tables for query-time inspection.
- [x] Keep the sidecars bundle-local so history lookup does not depend on
  mutable root-level files.
- [ ] Avoid expanding the manifest into a full data store.

### 5. Implement worker-light point picking

- [x] Add `worker_light.utils.render_query` as the render-query helper
  namespace.
- [x] Implement `pick_preview_pixel(...)` so it resolves one pixel against one
  bundle-local scene snapshot.
- [x] Convert the screen-space click into a world-space ray using the same
  camera math as preview rendering.
- [x] Raycast against the persisted build123d geometry instead of inferring
  coordinates from image bytes.
- [x] Return a typed hit record with bundle identity, view identity, pixel
  coordinates, ray data, world-space coordinates, and object identity fields.
- [x] Make batch queries use one request object per pick.
- [x] Keep the helper fail-closed when the bundle snapshot or scene hash does
  not match the requested artifact.

### 6. Keep historical lookup stable

- [x] Resolve historical bundles through `renders/render_index.jsonl`.
- [x] Resolve the bundle-local manifest directly; do not route through the
  legacy root-level manifest path.
- [x] Keep current-revision discovery working by targeting the bundle-local
  manifest or the append-only index directly.
- [x] Prevent older bundles from being hidden or overwritten when a newer
  render is published.
- [x] Make `inspect_media(...)` resolve the correct bundle-local manifest when
  the inspected file belongs to a published bundle.

### 7. Verify the migration boundary

- [x] Add integration coverage for current-bundle lookup through the live
  worker boundary.
- [x] Add integration coverage for historical-bundle lookup through the append
  only index.
- [x] Add integration coverage for bundle-local manifest resolution.
- [x] Add integration coverage for point-pick query responses.
- [x] Add integration coverage for simulation-video frame attachment and review
  slicing.
- [ ] Confirm that the docs in the owning architecture sections describe the
  same bundle contract and the same simulation-video backend switch.

## Acceptance Criteria

1. Every render-producing job writes a bundle-local manifest and appends a row
   to `renders/render_index.jsonl`.
2. Historical bundles remain discoverable after later renders are created.
3. `inspect_media(...)` resolves the correct manifest for a historical bundle.
4. `pick_preview_pixel(...)` returns a structured hit record rather than a bare
   tuple.
5. The hit record includes bundle identity, view identity, pixel coordinates,
   ray data, world-space coordinates, and object identity fields.
6. `objects.parquet` is the dense object-state format for bundle queries, and
   it is emitted by the active `PhysicsBackend` implementation rather than a
   MuJoCo-only branch.
7. There is no root-level manifest fallback path in the active contract; tools
   resolve bundle-local manifests or `renders/render_index.jsonl` directly.
8. The integration suite can prove current-bundle lookup, historical lookup,
   and point-pick behavior through the live worker boundary.

## File-Level Change Set

The implementation should touch the smallest set of files that enforce the new
contract:

- `worker_renderer/api/routes.py`
- `worker_renderer/utils/rendering.py`
- `shared/rendering/renderer_client.py`
- `shared/rendering/preview_scene.py`
- `shared/workers/schema.py`
- `worker_light/utils/render_query.py`
- `worker_light/api/routes.py`
- `shared/utils/agent/__init__.py`
- `controller/agent/nodes/base.py`
- `controller/middleware/remote_fs.py`
- `controller/clients/worker.py`
- `controller/api/routes/datasets.py`
- `controller/api/routes/script_tools.py`
- `controller/agent/review_handover.py`
- `worker_heavy/utils/handover.py`
- `specs/architecture/CAD-and-other-infra.md`
- `specs/architecture/simulation-and-rendering.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/tools.md`
- `specs/architecture/agents/auxiliary-agent-tools.md`
- `specs/integration-test-rules.md`
- `specs/integration-test-list.md`
- `tests/integration/**`

If a later implementation chooses to expose the helper through a different
runtime facade, this migration still applies: the bundle index must stay
append-only, the bundle-local manifest must stay authoritative for the bundle,
and point queries must stay tied to a persisted scene snapshot.
