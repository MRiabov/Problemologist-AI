# Code Review: Architectural and Implementation Decisions (Round 1)
**Date:** 2026-03-03
**Status:** Open for Review

## 1. Tight Coupling between Controller and Worker
**File:** `controller/agent/nodes/base.py`

### Description
The `BaseNode._run_program` method explicitly imports and calls `validate_node_output` from `worker_heavy.utils.file_validation`.

```python
from worker_heavy.utils.file_validation import validate_node_output
# ...
is_valid, validation_errors = validate_node_output(node_type, artifacts)
```

### Risk
This creates a hard dependency from the Controller service onto the Heavy Worker's utility code. In a microservices architecture, the Controller should ideally be agnostic of the internal implementation details of the worker. This makes the services harder to scale independently and introduces risks of circular dependencies or versioning conflicts if the shared libraries aren't perfectly managed.

### Suggestion
The validation logic should either be moved to a `shared/` library if it's purely schema-based, or the Controller should rely on a "Validation" API endpoint provided by the worker to check the agent's output.

**User Review:**

---

## 2. Overloaded `SimulationLoop` Constructor
**File:** `worker_heavy/simulation/loop.py`

### Description
The `SimulationLoop.__init__` method has become an architectural bottleneck. It performs backend initialization, scene loading, custom configuration parsing, full DFM validation/pricing (calling `validate_and_price`), material lookup building, site/actuator caching, and sub-manager (Electronics, Evaluators) initialization.

### Risk
High complexity in a constructor makes the class difficult to unit test and extremely fragile. A failure in a secondary task (like DFM pricing) can prevent the entire simulation loop from initializing, even if the user only wanted to run a simple physics check. It also leads to long initialization times that block the main simulation thread.

### Suggestion
Refactor the initialization logic into a Factory or Builder pattern. Move the heavy lifting (like DFM validation and material lookup construction) out of the constructor and pass in pre-configured objects.

**User Review:**

---

## 3. The `simulate` "God Function"
**File:** `worker_heavy/utils/validation.py`

### Description
The `simulate` function acts as a central orchestrator for the entire heavy worker pipeline. It manages directory state, loads multiple YAML configurations, performs proactive electronics validation, handles physics scene building, executes the simulation loop, manages GPU OOM retries, triggers 24-view rendering, calculates total assembly costs/weights, and generates stress heatmaps.

### Risk
This "God Function" is highly resistant to change. Any modification to how cost is calculated or how renders are captured requires touching this massive function. It violates the Single Responsibility Principle and makes error handling inconsistent (some errors are caught and returned as `SimulationResult`, others might bubble up).

### Suggestion
Break `simulate` into a series of discrete, pipeline-style steps or use a Task/Command pattern. The orchestration logic should be separated from the individual execution steps (rendering, pricing, physics).

**User Review:**

---

## 4. Throughput Bottlenecks in `SIMULATION_EXECUTOR`
**File:** `worker_heavy/api/routes.py`

### Description
The `SIMULATION_EXECUTOR` is hardcoded with `max_workers=1`.

```python
SIMULATION_EXECUTOR = ProcessPoolExecutor(
    max_workers=1,
    ...
)
```

### Risk
This creates a global bottleneck where only one simulation or heavy operation can run at a time across all user sessions. While physics simulations are resource-intensive, a modern multi-core server (or a cluster) should be able to handle concurrent tasks. This will lead to high latency and "wait position" queuing for users.

### Suggestion
Make `max_workers` configurable via environment variables or `settings`. Implement a more sophisticated resource-aware scheduling if Genesis/MuJoCo resource usage is the concern.

**User Review:**

---

## 5. Fragile Global State and Initialization in `GenesisBackend`
**File:** `worker_heavy/simulation/genesis_backend.py`

### Description
The `GenesisBackend` relies on a global `gs` object and has complex, nested retry logic for EGL/Display issues and GPU OOM. The `load_scene` method also includes a manual "skip rebuild" check that depends on comparing file paths and dictionaries.

### Risk
Global state management for a complex library like Genesis is prone to race conditions and "already initialized" errors, especially in a multi-threaded or multi-process environment. The scene-reuse logic is fragile and might fail to detect changes in nested MJCF or STL files if only the top-level path is compared.

### Suggestion
Encapsulate the Genesis state more strictly. Use a checksum or hash-based approach for scene reuse detection instead of comparing raw config dictionaries and paths.

**User Review:**

---

## 6. Simplistic Fallback in `ElectronicsManager`
**File:** `worker_heavy/simulation/electronics.py`

### Description
The `_fallback_update` method uses a "Simplified BFS for power propagation" that ignores component switch states and electrical properties.

### Risk
If the primary SPICE simulation fails, the fallback might report a component as "powered" simply because it is wired to a source, even if there is an open switch or a blown fuse in the path. This can lead to misleading simulation results where actuators move when they shouldn't.

### Suggestion
The fallback should at least respect the `switch_states` cached in the manager. Better yet, the "fallback" should be a more robust graph-based connectivity check that accounts for series/parallel paths and basic switch logic.

**User Review:**

---

## 7. Violation of Test/Production Separation
**File:** `controller/agent/nodes/base.py`

### Description
`SharedNodeContext.create` contains explicit logic to switch to `MockDSPyLM` based on `settings.is_integration_test`.

### Risk
Production code should not have "knowledge" of the test environment if possible. This makes the code harder to reason about and can lead to accidental "mocking" in production if the environment variable is misconfigured.

### Suggestion
Use Dependency Injection. The `SharedNodeContext` should accept a `dspy_lm` instance (or a factory) as an argument, and the caller (e.g., the FastAPI route or a test runner) should be responsible for providing the correct implementation (real or mock).

**User Review:**

---
