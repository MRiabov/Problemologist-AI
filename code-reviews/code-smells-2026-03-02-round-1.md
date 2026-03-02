# Code Review - March 2, 2026 - Round 1

## Overview
This review focuses on architectural and implementation "smells" in the `worker_heavy`, `controller`, and `shared` modules. Several issues related to violation of the Single Responsibility Principle, brittle heuristics, and serialization issues in the agent state were identified.

## Identified Issues & Recommendations

### 1. Bloated `SimulationLoop.__init__` (Violation of SRP)

**Component:** `worker_heavy/simulation/loop.py`

**Issue:** The `SimulationLoop.__init__` method is excessively long (over 100 lines) and handles multiple distinct responsibilities: environment setup, physics backend initialization, material lookup building, material property mapping, forbidden zone caching, and electronics initialization. This makes the class difficult to test and maintain.

**Recommendation:** Decompose the initialization logic into smaller, dedicated helper methods or specialized factory/builder classes. Specifically, the material and zone discovery logic could be moved to a `SceneAnalyzer` class.

**User Review:**

---

### 2. Brittle "Fell off world" Detection

**Component:** `worker_heavy/simulation/evaluator.py`

**Issue:** In `SuccessEvaluator.check_failure`, the default "out of bounds" check is hardcoded to `qpos[2] < -5.0`. This is brittle and may not be appropriate for all scenarios or scales. While it allows for `simulation_bounds` from `objectives.yaml`, the default fallback is too simplistic.

**Recommendation:** Instead of a hardcoded constant, make the safety floor a configurable parameter or derive it from the scene's bounding box.

**User Review:**

---

### 3. Inefficient Recreation of Infrastructure in `BaseNode._run_program`

**Component:** `controller/agent/nodes/base.py`

**Issue:** `WorkerInterpreter` and `DatabaseCallbackHandler` are recreated inside `_run_program` on every call. This is inefficient and prevents potential reuse of connections or state within a node's lifecycle across retries.

**Recommendation:** Move the initialization of these infrastructure components to the `__init__` or provide them via the `SharedNodeContext` to ensure they are managed at a higher level of the lifecycle.

**User Review:**

---

### 4. Non-Serializable Objects in `AgentState`

**Component:** `controller/agent/state.py`

**Issue:** `AgentState` includes `worker_client` and `fs` fields typed as `Any`. These are often populated with `WorkerClient` and `RemoteFilesystemMiddleware` instances. Since LangGraph uses msgpack for checkpointing, these non-serializable objects will cause `TypeError` when the graph attempts to save state.

**Recommendation:** Remove `worker_client` and `fs` from `AgentState`. These dependencies should be managed exclusively via `SharedNodeContext` or passed as `config` to nodes, ensuring the state remains pure and serializable.

**User Review:**

---

### 5. Brittle Target Body Identification Heuristics

**Component:** `worker_heavy/simulation/loop.py`

**Issue:** `_identify_target_body` uses string-based heuristics (`"target" in name.lower() or "bucket" in name.lower()`) to find the primary object for tracking. This is prone to failure if models use different naming conventions and should be more explicitly defined.

**Recommendation:** Enforce that the target body must be explicitly defined in `objectives.yaml` or `assembly_definition.yaml`. The fallback heuristic should be a last resort and perhaps logged as a warning.

**User Review:**

---

### 6. Redundant File Reading in `_get_steer_context`

**Component:** `controller/agent/nodes/base.py`

**Issue:** `_get_steer_context` reads the entire file from the worker for every code reference provided in the steering data. If multiple references point to the same file, the file is fetched multiple times.

**Recommendation:** Implement a simple cache or use `asyncio.gather` to fetch unique files once and then extract the required snippets.

**User Review:**

---

### 7. Implicit Dependency on `Compound` for Clearance Checks

**Component:** `worker_heavy/simulation/loop.py`

**Issue:** `_validate_wire_clearance` contains logic to convert `Part` to `Compound` because the underlying `check_wire_clearance` utility requires a `Compound`. This leak of implementation details into the simulation loop suggests the utility should handle single parts more gracefully.

**Recommendation:** Refactor `shared.wire_utils.check_wire_clearance` to accept both `Part` and `Compound`, moving the normalization logic into the utility itself.

**User Review:**

## Conclusion
While the system is functional, these architectural "smells" indicate a need for better separation of concerns and more robust handling of state and configuration. Addressing the serialization issues in `AgentState` is particularly critical for the reliability of long-running agent sessions.
