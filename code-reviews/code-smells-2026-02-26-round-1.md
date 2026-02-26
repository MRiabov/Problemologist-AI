# Code Review - Feb 26, 2026 - Round 1

## Overview
This review focuses on architectural and implementation "smells" in the `controller`, `worker_light`, and `shared` modules. Several critical issues related to resource management, code duplication, and security boundaries were identified.

## Identified Issues & Recommendations

### 1. Resource Leak in `WorkerClient`

**Component:** `controller/clients/worker.py`

**Issue:** `WorkerClient` maintains a dictionary `_loop_clients` to cache `httpx.AsyncClient` instances per event loop. However, these clients are never automatically closed during the `WorkerClient` lifecycle. Since the `BaseNode` currently creates a new event loop for **every tool call** from DSPy, this results in a new `httpx.AsyncClient` being leaked for every single tool invocation.

**Recommendation:** Implement an `aclose()` call in the `BaseNode._run_program`'s `finally` block to ensure all per-loop clients are cleaned up. Alternatively, refactor tool execution to reuse a single event loop where possible.

**User Review:** [Pending]

---

### 2. Inefficient Event Loop Management in Tool Execution

**Component:** `controller/agent/nodes/base.py`

**Issue:** In `BaseNode._get_tool_functions`, a synchronous wrapper is used to call asynchronous tools from the synchronous DSPy `ReAct` program. This wrapper creates a **new event loop** for every tool call (`asyncio.new_event_loop()`), which is extremely inefficient and exacerbates the resource leak in `WorkerClient`.

**Recommendation:** Use a persistent worker thread with its own event loop or use `nest_asyncio` more effectively to allow calling async tools from the main event loop without recreating it on every call.

**User Review:** [Pending]

---

### 3. Redundant Context and Client Recreation

**Component:** `controller/agent/nodes/*.py` (factory functions)

**Issue:** Factory functions like `coder_node`, `electronics_engineer_node`, etc., recreate a `SharedNodeContext` (and thus a new `WorkerClient` and `RemoteFilesystemMiddleware`) on **every single node execution**. This prevents effective connection pooling and adds significant overhead to agent turns.

**Recommendation:** Inject or cache the `SharedNodeContext` at the graph level (e.g., in `AgentState` or a global registry) to allow for persistent clients and better resource management.

**User Review:** [Pending]

---

### 4. Significant Code Duplication in Python Runtime Executor

**Component:** `worker_light/runtime/executor.py`

**Issue:** `run_python_code` and `run_python_code_async` share almost 90% of their logic, including environment setup (`PYTHONPATH` calculation), temporary file creation, and cleanup. This duplication makes maintenance difficult and increases the risk of bugs (e.g., if the environment setup needs to change).

**Recommendation:** Refactor the common logic into a private helper function or a context manager that handles environment preparation and temporary file lifecycle.

**User Review:** [Pending]

---

### 5. Missing Path Traversal Validation in Controller Proxy

**Component:** `controller/api/routes/episodes.py`

**Issue:** The `get_episode_asset` endpoint proxies requests to the worker but does not perform any validation on the `path` parameter. While the worker's filesystem backend has its own path traversal protection, the controller should also validate inputs at its own boundary to prevent potentially malicious requests from reaching internal services.

**Recommendation:** Add a validation step in `get_episode_asset` to ensure the `path` does not contain segments like `..` or point to sensitive directories.

**User Review:** [Pending]

---

### 6. Linear and Rigid Agent Graph Structure

**Component:** `controller/agent/graph.py`

**Issue:** The current `StateGraph` follows a very linear sequence (`planner` -> `electronics_planner` -> `plan_reviewer` -> `coder` ...). This rigid flow may be inefficient for tasks that don't require every step (e.g., a simple code fix that doesn't affect electronics).

**Recommendation:** Consider a more dynamic or hierarchical graph structure where nodes can route more intelligently based on the task type or the nature of the changes made.

**User Review:** [Pending]

---

### 7. Brittle Keyword-Based Task Filtering

**Component:** `controller/agent/nodes/coder.py`

**Issue:** `CoderNode._get_next_step` uses a hardcoded list of keywords (`["circuit", "wire", "electronics", "routing", "psu", "power"]`) to decide which tasks to skip. This is brittle and could lead to the coder incorrectly skipping relevant tasks or attempting to handle electronics tasks if they don't contain these keywords.

**Recommendation:** Use a more robust classification system for tasks, perhaps by adding metadata to the `todo.md` items during the planning phase.

**User Review:** [Pending]

---

### 8. Simplified and Hacky Electronics Modeling

**Component:** `shared/pyspice_utils.py` and `shared/circuit_builder.py`

**Issue:** The electronics simulation relies on brittle monkeypatching of `NgSpiceShared._send_char` to ignore certain output strings. Furthermore, motors are modeled as simple resistors, which fails to account for back-EMF, inductance, and other dynamic effects critical for accurate stall and start-up simulations.

**Recommendation:** Replace monkeypatching with proper error handling in `PySpice` (if possible) and implement more sophisticated component models (e.g., behavioral models for motors) to improve simulation fidelity.

**User Review:** [Pending]

## Conclusion
The current architecture exhibits several performance and resource management issues, particularly around the agent-worker interaction layer. Addressing the resource leaks and redundant object creation should be a priority to ensure the system scales effectively.
