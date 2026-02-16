# Problemologist-AI Codebase Architecture Review #10 (Feb 16)

This document identifies **NEW** architectural smells and questionable decisions discovered during the tenth deep-dive review of the codebase on **February 16, 2026**.

---

## 🔴 Critical Architectural Issues

### 1. Blocking Subprocess Calls in FastAPI Routes

**Problem:** The `/lint` endpoint executes `subprocess.run` directly within the async route handler.

**Evidence:**

- [routes.py:L692](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/api/routes.py#L692): `subprocess.run(["ruff", ...])`

**The Smell:**

- **Performance Bottleneck**: `subprocess.run` is synchronous and blocking. It will freeze the entire FastAPI event loop for that worker process while waiting for the linter.
- **Scale Issues**: Under load, this could lead to timeouts for other requests.
- **Better Alternatives**: Use `asyncio.create_subprocess_exec` or move the linting logic to a background task/isolated worker.

**User Review:**

---

### 2. Violation of Controller/Worker Isolation (Cross-Component Import)

**Problem:** The `controller` depends on `worker` internal packages, breaking the intended separation of concerns and making independent deployment difficult.

**Evidence:**

- [nodes.py:L231](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/nodes.py#L231): `from worker.utils.file_validation import validate_node_output`

**The Smell:**

- **Tight Coupling**: The controller cannot be updated or deployed without the worker code present.
- **Dependency Leak**: The controller's environment must satisfy the worker's dependencies (like `build123d`).
- **Architectural Regression**: Validation that depends on worker logic should be requested via an API call to the worker, not imported directly.

**User Review:**

---

## 🟠 Moderate Design Issues

### 3. Brittle Heuristic-Based Asset Serving

**Problem:** The `/assets` endpoint uses a guessing game to find the source code associated with a CAD file.

**Evidence:**

- [routes.py:L583-588](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/api/routes.py#L583-588): `candidate_paths = [...]` (main.py, component.py, etc.)

**The Smell:**

- **Fragility**: If a developer names their file `bridge_v1.py` and it's not in the candidate list, the asset serving logic (including syntax error checks) will fail or be bypassed.
- **Inconsistency**: The security check (syntax check) relies on finding the "correct" file, which isn't guaranteed.
- **Solution**: The asset request should include the script path or a session-relative mapping.

**User Review:**

---

### 4. Persistent `dict` Usage in Simulation Schemas

**Problem:** Despite the Pydantic-only rule, several core simulation schemas still use freeform dictionaries for critical data.

**Evidence:**

- [schemas.py:L45](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/simulation/schemas.py#L45): `metrics: dict[str, Any]`
- [schemas.py:L61](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/simulation/schemas.py#L61): `target_object_properties: dict[str, Any]`

**The Smell:**

- **Data Loss**: Non-validated dictionaries are black boxes for the observability database.
- **Type Safety**: Consumers of these schemas have no IDE support or runtime guarantees for what's inside the metrics.

**User Review:**

---

## 🟡 Minor Issues & Smells

### 5. Local Persistence in Worker (Handover Gate)

**Problem:** The worker writes `validation_results.json` directly to the local filesystem to satisfy the "handover gate".

**Evidence:**

- [routes.py:L465-472](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/api/routes.py#L465-472)
- [routes.py:L528-540](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/api/routes.py#L528-540)

**The Smell:**

- **State Management**: If the worker is containerized or distributed (e.g., K8s), writing to a local disk without a shared volume/bucket makes the validation status inaccessible to other components or future requests.
- **Redundancy**: This result should likely be returned as part of the response or stored in the central episode database.

**User Review:**

---

### 6. Non-Distributed Concurrency Control

**Problem:** Concurrency for simulations is managed via an in-memory `asyncio.Semaphore`.

**Evidence:**

- [routes.py:L366](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/api/routes.py#L366): `SIMULATION_SEMAPHORE = asyncio.Semaphore(1)`

**The Smell:**

- **Process Isolation**: This only works if Uvicorn runs with a single worker process. If multiple workers are used, concurrency is not actually limited.
- **Distributed Incompatibility**: Doesn't work across multiple nodes. Use Redis or a central status lock if this limit is critical.

**User Review:**

---

## Summary of Refactor Progress

| Previous Issue (Feb 15) | Status | Note |
|-------|----------|--------|
| Ubiquitous `dict` usage | 🟠 Partially Improved | Some improvements in objectives, but simulation schemas still lagging. |
| Manual Tool Loops | ✅ Resolved | Nodes refactored to use `create_react_agent`. |
| SRP Violation in graph runner | ⚠️ Pending | Logic still tightly coupled in `graph.py`. |
| Brittle JSON Parsing | ✅ Resolved | Switched to `with_structured_output` in `planner_node`. |
| Hardcoded Eval Verification | ⚠️ Pending | No changes seen in `run_evals.py`. |
| Empty Simulation Schemas | ✅ Resolved | `shared/simulation/schemas.py` populated. |
| sys.path hacks | ⚠️ Pending | Still present in evaluation scripts. |

| New Issue (Feb 16) | Severity | Effort |
|-------|----------|--------|
| 1. Blocking Subprocess Calls | 🔴 Critical | Low |
| 2. Cross-Component Dependency | 🔴 Critical | Medium |
| 3. Brittle Asset Serving | 🟠 Moderate | Medium |
| 4. Persistent `dict` usage | 🟠 Moderate | High |
| 5. Local Persistence in Worker | 🟡 Minor | Low |
| 6. In-process Concurrency Lock | 🟡 Minor | Low |

---

*Review conducted on 2026-02-16 by Gemini (Antigravity)*
