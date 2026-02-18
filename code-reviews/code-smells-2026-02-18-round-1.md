# Code Smells & Architectural Review - 2026-02-18 - Round 1

## 1. Broken and Inefficient `WorkerClient.exists`

The `WorkerClient.exists` method is both broken and highly inefficient. It attempts to check for a file's existence by listing the entire parent directory and searching the results. However, it expects a `"files"` key in the response which doesn't exist (the API returns a list directly), and the O(N) complexity is unnecessary when a simple `HEAD` or dedicated endpoint could be used.

- **Location:** `controller/clients/worker.py`
- **User Review:**

## 2. Business Logic Leakage into Middleware and API Routes

The `RemoteFilesystemMiddleware` has become a "God Middleware," handling observability events, frontend broadcasting, and complex failure reason mapping. Similarly, `worker/api/routes.py` contains deep business logic like material validation and handover gate results persistence. This violates the Single Responsibility Principle and makes these components hard to test and maintain.

- **Location:** `controller/middleware/remote_fs.py`, `worker/api/routes.py`
- **User Review:**

## 3. Structural Coupling via Cross-Service Imports

The Controller directly imports utility code from the Worker module (e.g., `validate_node_output`). If these were deployed as truly independent services, this would break. It also forces the Controller to have all Worker dependencies installed just to perform basic validation that could be handled via an API call or a shared library.

- **Location:** `controller/agent/nodes/coder.py`, `controller/agent/nodes/planner.py`
- **User Review:**

## 4. Concurrency Bottlenecks in Worker Operations

Heavy operations in the Worker (simulations, renders, builds) are serialized via a global `HEAVY_OPERATION_LOCK` and a `ProcessPoolExecutor` with `max_workers=1`. While likely intended to prevent GPU/resource contention, this creates a significant bottleneck that prevents the system from scaling to multiple concurrent users even if resources are available.

- **Location:** `worker/api/routes.py`
- **User Review:**

## 5. Conflicting Naming for `validate_and_price`

There is a major naming conflict between the `validate_and_price` function in `worker/utils/dfm.py` (which analyzes CAD geometry) and the `validate_and_price.py` skill script (which updates `assembly_definition.yaml`). This is highly confusing for both agents and human developers and risks incorrect tool selection or implementation errors.

- **Location:** `worker/utils/dfm.py`, `skills/manufacturing-knowledge/scripts/validate_and_price.py`
- **User Review:**

## 6. Disk Space Leak in Session Management

The `_SESSION_DIR_REGISTRY` in `worker/filesystem/backend.py` stores `TemporaryDirectory` objects to keep them alive, but there is no mechanism to clean them up when a session is finished or evicted from the `BACKEND_CACHE`. This leads to a persistent disk space leak in the worker's temp directory over time.

- **Location:** `worker/filesystem/backend.py`, `worker/simulation/factory.py`
- **User Review:**

## 7. Inconsistent and Suspicious Logic in `AssemblyDefinition`

The `moving_parts` property in `AssemblyDefinition` contains a suspicious `hasattr(p_config.config, "config")` check. Given the schema definitions, `p_config.config` is an `AssemblyPartConfig` which does not have a `config` attribute. This suggests a workaround for malformed data or a misunderstanding of the schema hierarchy.

- **Location:** `shared/models/schemas.py`
- **User Review:**

## 8. Missing Abstraction in Agent Nodes

`CoderNode`, `PlannerNode`, and `ElectronicsEngineerNode` all implement nearly identical retry loops, DSPy program invocations, and validation gates. This logic should be abstracted into `BaseNode` or a specialized `AgentExecutor` to reduce duplication and ensure consistent behavior across all agent types.

- **Location:** `controller/agent/nodes/`
- **User Review:**

## 9. Incomplete Event Collection in Worker

The `_collect_events` utility in the Worker API reads from `events.jsonl` but the code to delete the file after reading is missing or commented out. This leads to "event pollution" where events from previous runs are repeatedly collected and reported in subsequent API calls.

- **Location:** `worker/api/routes.py`
- **User Review:**

## 10. Hardcoded Physics Engine Assumptions and Heuristics

The `CommonAssemblyTraverser` and `SceneCompiler` rely on brittle naming conventions (e.g., `zone_` prefix) and magic strings (e.g., `weld:` constraints) that aren't enforced by the CAD library. Furthermore, `SceneCompiler` has many hardcoded simulation parameters (lighting, integrator, timestep) that should be configurable via `PhysicsConfig`.

- **Location:** `worker/simulation/builder.py`
- **User Review:**

## 11. Regressions in Node Validation Tests

The test `tests/test_node_validation.py` is currently failing because it doesn't provide the `assembly_definition.yaml` file, which is now mandatory for the `planner` node according to `worker/utils/file_validation.py`. This indicates that either the requirement was added without updating the tests, or the requirement itself is over-aggressive for early planning stages.

- **Location:** `tests/test_node_validation.py`, `worker/utils/file_validation.py`
- **User Review:**
