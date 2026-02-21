# Code Review Report - 2026-02-21 Round 3

## 1. [Architecture] In-memory singletons in Controller
**Issue:** `ConnectionManager` and `TaskTracker` in `controller/api/manager.py` are implemented as in-memory singletons.
**Impact:** This prevents the Controller from scaling horizontally. If multiple instances of the Controller are running, they won't share WebSocket connections or task states. Also, restarting the service loses all active task tracking and connection states.
**Recommendation:** Use a distributed state management system like Redis for connection routing (Pub/Sub) and task tracking.
**User Review:**

## 2. [Implementation] `AnalyzeRequest` parameters ignored in `api_analyze`
**Issue:** In `worker_heavy/api/routes.py`, the `api_analyze` endpoint receives an `AnalyzeRequest` containing `method` and `quantity`, but it doesn't pass these to `analyze_component`.
**Impact:** The analysis always falls back to heuristics or defaults (CNC, quantity 1), even if the user explicitly requested a different manufacturing method or quantity.
**Recommendation:** Update `api_analyze` to pass `request.method` and `request.quantity` to `analyze_component`.
**User Review:**

## 3. [Performance] Inefficient `grep_raw` implementation
**Issue:** `FilesystemRouter.grep_raw` in `shared/workers/filesystem/router.py` uses `f.read_text()` to load entire files into memory.
**Impact:** For large codebases or large individual files, this can lead to high memory consumption and potential OOM errors in the worker.
**Recommendation:** Refactor `grep_raw` to read files line-by-line or use a streaming approach.
**User Review:**

## 4. [Performance] Inefficient GLB topology export
**Issue:** `MeshProcessor.export_topology_glb` in `worker_heavy/simulation/builder.py` exports each face to a temporary STL and then re-imports it via `trimesh`.
**Impact:** High I/O overhead and redundant processing for components with many faces.
**Recommendation:** Use `trimesh` or `build123d` APIs to directly extract face meshes without intermediate disk I/O.
**User Review:**

## 5. [Performance] Fixed `check_interval` in `SimulationLoop`
**Issue:** `SimulationLoop.step` in `worker_heavy/simulation/loop.py` uses a hardcoded `check_interval = 1` for collision and motor overload checks.
**Impact:** Checking these conditions every single step (every 2ms) is computationally expensive and often unnecessary for stability.
**Recommendation:** Make `check_interval` configurable or increase it to a more reasonable value (e.g., every 5-10 steps) while ensuring critical events aren't missed.
**User Review:**

## 6. [Implementation] Lack of FEM entity caching in `GenesisBackend`
**Issue:** `GenesisBackend.step` in `worker_heavy/simulation/genesis_backend.py` iterates over all entities to check stress, regardless of whether they are FEM-enabled.
**Impact:** Rigid bodies are unnecessarily checked for stress fields every step, adding overhead.
**Recommendation:** Cache FEM-enabled entities during `load_scene` and only iterate over those in the `step` method.
**User Review:**

## 7. [Architecture] Workspace coupling via shared filesystem
**Issue:** Services (Controller, Worker Light, Worker Heavy) share state via `WORKER_SESSIONS_DIR`.
**Impact:** This tightly couples the services to the same host or requires complex network filesystem setups (NFS/EFS), complicating cloud-native deployments and horizontal scaling.
**Recommendation:** Transition to an object-storage based approach (e.g., S3) or a more robust data handover mechanism.
**User Review:**

## 8. [Data Modeling] Storing file content in DB `Asset` model
**Issue:** The `Asset` model in `controller/persistence/models.py` has a `content` column that stores file data as strings.
**Impact:** Bloats the database and slows down queries. The model already has an `s3_path` field, suggesting it's intended for external storage.
**Recommendation:** Ensure all large assets are stored in object storage and only keep metadata/paths in the database.
**User Review:**

## 9. [Performance] In-memory tarball bundling
**Issue:** `bundle_session` in `worker_light/api/routes.py` and `bundle_context` (in some parts) build/process tarballs in memory using `io.BytesIO`.
**Impact:** High memory pressure for large workspaces.
**Recommendation:** Stream the tarball directly to the response or use temporary files on disk.
**User Review:**

## 10. [Implementation] `BenchmarkToolResponse.artifacts` nullability risk
**Issue:** `artifacts` in `BenchmarkToolResponse` is nullable.
**Impact:** Parsers or UI code might crash if they assume `artifacts` is always present.
**Recommendation:** Use robust access patterns or default to an empty object/model in the response.
**User Review:**
