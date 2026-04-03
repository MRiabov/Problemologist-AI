# Server-to-Server Networking

## Purpose

Move the platform from ad hoc JSON/base64 relay patterns to a small set of
predictable server-to-server transport rules.

This is a transport cleanup, not a protocol rewrite. The goal is to keep the
control plane simple, keep bytes out of JSON when they do not belong there, and
stop controller-mediated byte relays where the owning service can write data
directly.

The target architecture is still defined by:

- [Distributed execution](../../architecture/distributed-execution.md)
- [Distributed execution networking](../../architecture/distributed-execution.md#networking)
- [CAD and other infrastructure](../../architecture/CAD-and-other-infra.md)

## Problem Statement

The current server-to-server path mixes control traffic, workspace snapshots,
and artifact bytes in ways that are easy to implement but expensive to run.

1. Workspace snapshots are frequently moved as gzipped tarballs encoded as
   base64 and then embedded in JSON.
2. Some paths relay the same payload through the controller even when the
   destination service could write the bytes directly.
3. Multi-file transfers are often implemented as sequential single-file
   requests, which adds unnecessary round-trips.
4. Artifact handoff often repeats the same bytes through several layers before
   the final workspace copy is written.
5. Validation and preview paths still blur the line between control metadata
   and bulk file transport.
6. The system has enough internal complexity that a clear transport policy is
   now more valuable than more ad hoc compatibility helpers.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `shared/workers/bundling.py` | Packs the whole workspace into a gzipped tarball and base64-encodes it for transport. | This is acceptable for explicit snapshot semantics, but not as the default data plane. |
| `controller/clients/worker.py` | Bundles the session, base64-encodes it again for downstream calls, and syncs artifacts back one file at a time. | The controller should not act as a byte relay between workers. |
| `controller/api/routes/script_tools.py` | Fetches render blobs and manifests through sequential `exists`/`read_file_binary` loops. | Bulk artifact retrieval should be batched or pointer-based. |
| `worker_light/api/routes.py` | Accepts batch uploads as JSON base64 payloads and still exposes workspace snapshot bundling. | Batch is better than N requests, but base64-in-JSON should not be the preferred byte transport. |
| `worker_heavy/api/routes.py` | Reassembles validation and simulation artifacts as base64 blobs for return transfer. | Artifact ownership should be direct and minimal, not repeatedly reserialized. |
| `worker_heavy/activities/heavy_tasks.py` | Mirrors the same artifact-assembly pattern for durable activity outputs. | Durable activities should return paths, manifests, or object keys where possible. |
| `worker_heavy/utils/validation.py` | Produces preview evidence during validation and then materializes the render payload. | Validation should not become a hidden render-and-transfer pipeline. |
| `shared/rendering/renderer_client.py` | Materializes preview/simulation artifacts from response blobs into the workspace. | This is fine for final materialization, but the payload shape should stay small and explicit. |

## Route Ownership Map

Apply the transport policy by route family, not by isolated call site.

| Route family | Target owner | Transport shape | Notes |
| -- | -- | -- | -- |
| `worker_light/api/routes.py`: `/fs/ls`, `/fs/read`, `/fs/write`, `/fs/edit`, `/fs/upload_file`, `/fs/upload_files`, `/fs/upload_files_object_store`, `/fs/delete`, `/fs/grep` | `worker-light` | JSON for control, multipart or binary for file bytes, batch endpoints for multiple files, object-store pull for staged files | Workspace CRUD and agent tooling. |
| `worker_light/api/routes.py`: `/fs/exists`, `/fs/read_blob`, `/fs/read_files` | `worker-light` | JSON control and raw bytes | Low-level probes and inspection helpers. Use them sparingly in tests and orchestration. |
| `worker_light/api/routes.py`: `/fs/bundle` | `worker-light` | snapshot bundle / compatibility path | Explicit workspace snapshot transport only. Do not treat as the default file API. |
| `worker_light/api/routes.py`: `/git/*`, `/runtime/execute`, `/lint`, `/topology/inspect` | `worker-light` | JSON for control, multipart or binary for file bytes where needed | Remaining light-worker control and execution paths. |
| `worker_light/api/routes.py`: `/benchmark/validate` | `worker-light` | JSON control plus workspace-local outputs | Fast geometry gate. Do not attach preview render artifacts by default. |
| `worker_light/api/routes.py`: `/benchmark/preview`, `/render/*` | `worker-light` + `worker-renderer` | staged bundle, binary bytes, or object-store pointer | Light worker may orchestrate the renderer, but it should not relay large bytes through the controller. |
| `worker_heavy/api/routes.py`: `/benchmark/simulate`, `/benchmark/submit`, `/benchmark/analyze`, `/benchmark/build` | `worker-heavy` | bundle only when snapshot semantics are required; otherwise binary or pointer-based transfer | Heavy compute and handoff paths. |
| `worker_heavy/api/routes.py`: `/benchmark/verify`, `/benchmark/validate_circuit` | `worker-heavy` | small JSON control; return manifest, path, or key where possible | Specialized compute that is still heavy-worker owned. |
| `worker_heavy/api/routes.py`: `/benchmark/validate`, `/benchmark/preview` | legacy compatibility only | JSON/bundle fallback | Keep only until controller and agent-facing paths are fully on the light-worker route family. |
| `controller/api/routes/script_tools.py`: `/validate`, `/simulate`, `/verify`, `/preview`, `/submit` | controller orchestration | no bulk byte relay; envelopes and coordination only | The controller selects routes and collates results; it should not reserialize payload bytes unless unavoidable. |
| `worker_heavy/activities/heavy_tasks.py` and `shared/rendering/renderer_client.py` | internal worker-plane helpers | same as the owning route family | Helpers inherit the same transport contract as the route they serve. |

Integration tests should prefer batch uploads or fixture seeding over repeated
`exists`/`read`/`read_blob` loops unless the transport contract itself is the
subject of the test.

## Proposed Target State

1. HTTP JSON is the control plane, not the bulk-data plane.
2. WebSockets are for long-lived control streams, status updates, and
   interactive progress, not for bulk artifact transport.
3. Binary file payloads move as binary bytes or multipart uploads, not as
   base64-in-JSON when a binary transport is available.
4. Multiple files that belong to one logical change move together in one batch
   request, not as a loop of single-file requests.
5. Full workspace snapshots remain explicit and rare. They are reserved for
   operations that truly need the whole tree as the unit of work.
6. Large immutable artifacts are stored once and referenced by object key or
   manifest path.
7. The controller remains the orchestration boundary, not a general-purpose
   relay for file bytes between workers.
8. When a service can write to its own workspace or storage directly, it should
   do so and return metadata, not bounce bytes through another service.
9. Compatibility wrappers may remain during migration, but they should be
   treated as fallback plumbing, not the preferred path.

## Required Work

### Transport policy

1. Keep `specs/architecture/distributed-execution.md#networking` as the source
   of truth for transport selection.
2. Standardize one clear transport choice per payload shape:
   - JSON for small metadata and commands
   - WebSockets for live progress and session control
   - binary HTTP or multipart for raw file bytes
   - batch file endpoints for multiple files
   - object-store pointers for large immutable outputs
3. Make the control plane explicit about session, episode, and request
   correlation.

### File transfer

1. Prefer batch file upload/download helpers over repeated single-file loops.
2. Keep workspace-relative paths as the unit of file identity across services.
3. Use a full archive bundle only when the next stage truly requires the whole
   workspace snapshot.
4. Avoid base64-in-JSON for large or repeated file bytes unless a compatibility
   surface cannot be avoided yet.
5. Let the owning service write final artifacts directly to the session
   workspace or object storage instead of routing them through the controller.

### Controller and worker paths

1. Remove controller-side byte relay behavior where a worker can talk to the
   worker filesystem or object store directly.
2. Replace sequential artifact collection loops with batched retrieval or
   manifest-driven reads.
3. Keep WebSocket usage limited to live control/status traffic.
4. Keep busy handling deterministic and explicit. Do not hide a transport
   failure behind a success-shaped response.

### Migration hygiene

1. Leave compatibility wrappers in place until the new transport is stable.
2. Mark the legacy bundle/base64 paths clearly so they are not copied into new
   code.
3. Update integration tests to assert the new transport shape where the hot
   path changes.
4. Keep response shapes stable for the agent-facing tools unless a specific
   transport change requires a documented contract update.

## Non-Goals

1. Do not introduce gRPC, protobuf, or a service-mesh dependency.
2. Do not add a new agent-facing worker transport.
3. Do not rewrite every payload schema in one pass.
4. Do not remove all JSON from the system. JSON remains the right choice for
   small commands, metadata, and status.
5. Do not change service ownership boundaries.
6. Do not optimize for zero-copy microbenchmarks; optimize for clarity and a
   low number of network round-trips.

## Sequencing

1. Publish the networking policy in `distributed-execution.md`.
2. Replace the highest-traffic bundle and artifact relay paths with batch or
   binary transfers.
3. Move large immutable outputs to pointer-based handoff where possible.
4. Reduce controller-side relay logic to compatibility-only surfaces.
5. Retire legacy base64/bundle helpers from hot paths after integration
   coverage proves the new flow.

## Acceptance Criteria

1. The architecture docs clearly state when to use JSON, WebSockets, batch file
   endpoints, binary file transfer, and object-store pointers.
2. Multi-file transfers no longer rely on per-file sequential round-trips on
   the hot path.
3. The controller is no longer used as a mandatory byte relay between workers.
4. Full workspace bundles are reserved for explicit snapshot semantics, not
   routine control-plane traffic.
5. Large outputs are referenced by path, manifest, or object key instead of
   being repeatedly base64-embedded in JSON.
6. Existing compatibility paths continue to work until the migration cutover
   is complete.

## Implementation Status

The transport cleanup is partially implemented and validated on the hot paths
that matter most for this migration:

1. `worker-light` now supports batch upload/download helpers and the websocket
   RPC transport for normal workspace CRUD.
2. `controller/clients/worker.py` now uses multipart file uploads and batch
   file reads instead of per-file binary relay loops on the normal path.
3. Handover synchronization now prefers object-store-backed uploads when the
   destination can pull artifacts directly.
4. Controller script-tool preview and simulation paths now batch artifact reads
   instead of re-fetching one file at a time.
5. Compatibility bundle/snapshot paths remain in place for explicit snapshot
   semantics and legacy handoff surfaces.
6. Legacy agent-side helpers in `shared/utils/agent/__init__.py` still collect
   a workspace bundle before calling heavy-worker compatibility endpoints when
   the newer orchestration path is unavailable.
7. `controller/api/tasks.py` still performs controller-mediated benchmark bundle
   copying for approved handoffs, because that path is a session-to-session
   transfer rather than a direct worker-owned write.
8. Renderer-emitted files are persisted to object storage before they leave
   `worker-renderer`; downstream proxy hops can re-materialize them locally,
   but the renderer boundary itself is not the long-lived byte sink.
9. `worker_renderer/utils/technical_drawing.py` now uploads PNG, SVG, DXF, and
   manifest outputs to object storage when available and only falls back to
   inline base64 payloads when storage is unavailable.

## Remaining Watchpoints

These are the places most likely to be missed in a later transport cleanup pass:

1. `shared/utils/agent/__init__.py` bundle collection before heavy-worker calls.
2. `controller/api/tasks.py` benchmark artifact copy between sessions.

## Migration Checklist

### Filesystem surface split

- [ ] Keep `/fs/ls`, `/fs/read`, `/fs/write`, `/fs/edit`, `/fs/upload_file`,
  `/fs/upload_files`, `/fs/delete`, and `/fs/grep` as the normal workspace
  CRUD surface.
- [ ] Keep `/fs/exists`, `/fs/read_blob`, and `/fs/read_files` as low-level
  probe helpers, not generic scaffolding for ordinary integration setup.
- [ ] Keep `/fs/bundle` as explicit snapshot/compatibility transport only.
- [ ] Prefer `/fs/upload_files` for multi-file setup instead of repeated single
  writes when the test is not validating per-file transport behavior.
- [ ] Prefer fixture seeding or batch helpers over repeated `exists` or
  `read_blob` loops unless the probe transport itself is under test.
- [ ] Keep privileged filesystem bypass requests internal-only and avoid using
  them in agent-facing flows unless the bypass policy is itself under test.

### Documented transport rules

- [ ] Keep the transport matrix in `distributed-execution.md#networking`.
- [ ] Keep WebSockets limited to live status/control traffic.
- [ ] Keep JSON limited to small metadata and command payloads.
- [ ] Keep batch file endpoints as the default for multi-file transfers.
- [ ] Keep full workspace bundles as explicit snapshot semantics only.

### Hot paths to simplify

- [x] Replace sequential artifact relay loops with batched retrieval or
  manifest-driven reads.
- [x] Replace base64-in-JSON byte transport with binary or multipart transport
  where the boundary allows it.
- [ ] Remove controller-side relay behavior that only exists to pass bytes
  through to another worker.
- [ ] Make large immutable outputs pointer-based instead of copy-based.

### Cutover verification

- [ ] Confirm the hot path still succeeds when the controller is not used as a
  byte relay.
- [ ] Confirm multi-file batch transfer stays atomic at the logical operation
  level.
- [ ] Confirm WebSocket progress messages remain small and ordered.
- [ ] Confirm compatibility bundle paths still work for the places that truly
  need them.
