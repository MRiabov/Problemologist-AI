# Code Smells & Architectural Review - 2026-02-16 - Round 3

## 1. Frontend API Orchestration Bypassing

The frontend (Vite configuration) explicitly proxies `/api/benchmark/build` to the Worker service (`:18001`) while other `/api` requests go to the Controller (`:18000`). This bypasses the Controller's role as the primary orchestrator and forces the frontend to know about internal service routing.

- **Location:** `frontend/vite.config.ts`, `frontend/src/api/client.ts`
- **User Review:**

## 2. Global Side-Effects in Dynamic Component Loading

The `_load_component` utility (used for loading build123d scripts) modifies `sys.path` globally using `sys.path.insert(0, ...)`. In a multi-session or concurrent environment, this can lead to unpredictable import collisions and memory leaks of path strings.

- **Location:** `worker/tools/topology.py`, `worker/api/routes.py`
- **User Review:**

## 3. Duplicated and Brittle S3 Client Instantiation

`S3Client` and `S3Config` are instantiated directly within Temporal activities and Worker tools with hardcoded default credentials (`minioadmin`) and redundant environment variable lookups. This makes mocking difficult and risks configuration drift between services.

- **Location:** `controller/activities/simulation.py`, `worker/activities/rendering.py`
- **User Review:**
We have a pydantic settings model somewhere.

## 4. Business Logic Leakage into Worker Router

The Worker's API router contains significant business logic, including FEM validation requirements, handover gate result persistence (writing `validation_results.json`), and event collection. This should be encapsulated in a dedicated Service layer or within specific Workbench implementations.

- **Location:** `worker/api/routes.py`
- **User Review:**

## 5. Temporal Workflow Type Safety and Observability

The `SimulationWorkflow` uses string-based activity execution and untyped dictionaries for parameters. Furthermore, long-running activities (like `run_simulation_activity`) lack Temporal Heartbeats, which could lead to "zombie" activities if a worker crashes during a heavy simulation.

- **Location:** `controller/workflows/simulation.py`, `controller/activities/simulation.py`
- **User Review:**

## 6. Duplicate Implementation of `_load_component`

The logic for loading build123d components from source files is duplicated almost identically between the topology tools and the main API routes. This increases the risk of inconsistent behavior when the loading logic needs to be updated (e.g., for security sandboxing).

- **Location:** `worker/tools/topology.py` vs `worker/api/routes.py`
- **User Review:**

## 7. Context-Agnostic Error Swallowing

Many try-except blocks in the Worker and Controller activities catch generic `Exception` and return `str(e)` or a simplified error message. This strips the structured logging context and stack traces that are essential for debugging complex failures in the simulation pipeline.

- **Location:** `worker/filesystem/backend.py`, `worker/api/routes.py`
- **User Review:**
Confirm

## 8. Hardcoded Physics Engine Assumptions

`MuJoCoSimulationBuilder` and `GenesisSimulationBuilder` share a `CommonAssemblyTraverser` that relies on dynamic `getattr` calls for `label` and `constraint`. This assumes the user's CAD scripts follow specific naming conventions that aren't enforced by the build123d library itself, leading to brittle simulation exports.

- **Location:** `worker/simulation/builder.py`
- **User Review:**
I suggest creating a `PartMetadata` and `CompoundMetadata` Pydantic models. This will serve as doc for what fields are there. So we'll do `part.metadata=PartMetadata(material_id="alu_6061)` or similar.
Fail validation if the part doesn't contain it; or if any fields are missing. Fail with a string/error.

it will be imported just other (function) utils.
