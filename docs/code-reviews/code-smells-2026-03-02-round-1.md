# Codebase Review - March 02, 2026 - Round 1

## Overview

This review identifies new architectural and implementation issues, focusing on test/production code separation, concurrency bottlenecks in the heavy worker, and logic flaws in electronics power propagation.

---

## 🚨 Critical Issues

### 1. Architectural Smell: Leakage of Test Mocks into Production Factories

**File**: `controller/agent/nodes/base.py`
**Issue**: `SharedNodeContext.create` explicitly checks for `settings.is_integration_test` to instantiate `MockDSPyLM`.
**Evidence**:
```python
# controller/agent/nodes/base.py:53
if settings.is_integration_test:
    from controller.agent.mock_llm import MockDSPyLM
    logger.info("using_mock_llms_for_integration_test", session_id=session_id)
    dspy_lm = MockDSPyLM(session_id=session_id)
```
**Impact**: High. This violates the separation of concerns. Production code should not be aware of test-specific mocks. It makes the code harder to maintain and increases the risk of test-only logic accidentally running in production if environment variables are misconfigured.
**Recommendation**: Use dependency injection. The `SharedNodeContext.create` method should accept an optional `dspy_lm` instance, and the mock injection should happen at the entry point of the integration tests or via a separate test-only factory.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Implementation Smell: Blocking Subprocess Execution in API Lifecycle

**File**: `worker_heavy/api/routes.py`
**Issue**: The `bundle_context` context manager uses `subprocess.run` to extract bundles, which is a blocking operation.
**Evidence**:
```python
# worker_heavy/api/routes.py:82
subprocess.run(
    ["tar", "-zxf", tf_path, "-C", str(tmp_root), "--no-same-owner"],
    check=True,
    capture_output=True,
    text=True,
)
```
**Impact**: High. Since this is called within FastAPI routes (e.g., `api_simulate`, `api_validate`), it blocks the event loop thread. In a "heavy" worker where bundle extraction might involve many files, this will cause the entire service to become unresponsive to other requests (including health checks) during the extraction.
**Recommendation**: Use `asyncio.create_subprocess_exec` to perform the extraction asynchronously, and refactor `bundle_context` into an `@contextlib.asynccontextmanager`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Logic Bug: Incomplete Power Propagation Logic in `ElectronicsManager`

**File**: `worker_heavy/simulation/electronics.py`
**Issue**: The `_fallback_update` method (connectivity-based BFS) does not actually check switch/relay states.
**Evidence**:
```python
# worker_heavy/simulation/electronics.py:81
if v and v not in visited:
    # Check if connection is closed (if it involves a switch)
    # This is simplified logic
    visited.add(v)
    powered.add(v)
    queue.append(v)
```
The comment "This is simplified logic" acknowledges the omission, but the current implementation effectively treats all switches as closed, meaning power always propagates regardless of the agent's control inputs.
**Impact**: Medium/High. Simulations using the fallback (which occurs if SPICE fails) will produce incorrect results where unpowered actuators appear powered, leading to false successes or confusing failures.
**Recommendation**: Implement the switch state check in the BFS. Only traverse a wire if it's connected to a closed switch or a non-switch component.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 4. Resource Management: Hardcoded Concurrency Limit in `SIMULATION_EXECUTOR`

**File**: `worker_heavy/api/routes.py`
**Issue**: `SIMULATION_EXECUTOR` is hardcoded to `max_workers=1`.
**Evidence**:
```python
# worker_heavy/api/routes.py:126
SIMULATION_EXECUTOR = ProcessPoolExecutor(
    max_workers=1,
    ...
)
```
**Impact**: Medium. While physics simulations are CPU-intensive, limiting the worker to a single process effectively serializes all simulations across all sessions. On a multi-core machine, this is a significant bottleneck and results in high `SIMULATION_QUEUE_DEPTH` as noted in the logs.
**Recommendation**: Make the number of workers configurable via `settings`, defaulting to a value based on `os.cpu_count()`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. Implementation Smell: Unsafe Use of `eval()` for Reward Formulas

**File**: `controller/agent/dspy_utils.py`
**Issue**: `evaluate_formula` uses the built-in `eval()` function to calculate scores.
**Evidence**:
```python
# controller/agent/dspy_utils.py:100
return float(eval(formula, {"__builtins__": {}}, allowed_names))
```
**Impact**: Medium. Although `__builtins__` is cleared, `eval` is notoriously difficult to sandbox in Python. If a malicious or poorly-formatted reward configuration is loaded, it could potentially be exploited.
**Recommendation**: Use a safer alternative like `numexpr` or a dedicated expression parser like `simpleeval` that restricts the AST to mathematical operations only.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 6. Logic/Performance: Redundant Client and Middleware Instantiation

**Files**: `controller/agent/nodes/planner.py`, `controller/agent/nodes/coder.py`
**Issue**: Node factory functions (e.g., `planner_node`, `coder_node`) instantiate a new `SharedNodeContext` (and thus a new `WorkerClient` and `RemoteFilesystemMiddleware`) on every invocation.
**Evidence**:
```python
# controller/agent/nodes/planner.py:106 (and coder.py:133)
ctx = SharedNodeContext.create(...)
```
**Impact**: Low/Medium. This leads to redundant HTTP client creation and prevents effective connection pooling. More importantly, it ignores existing `worker_client` and `fs` instances that might already be in the `AgentState`.
**Recommendation**: The factory functions should first check if `state.worker_client` and `state.fs` are present and reuse them if possible, or the `SharedNodeContext` should be created once at the graph entry point.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
