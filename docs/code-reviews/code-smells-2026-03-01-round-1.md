# Codebase Review - March 01, 2026 - Round 1

## Overview

This review identifies several new architectural and implementation issues, focusing on simulation side-effects, code duplication in the runtime executor, and inefficiencies in the agent node initialization.

---

## 🚨 Critical Issues

### 1. Architectural Smell: Mutation of Input Configuration in `SimulationLoop`

**File**: `worker_heavy/simulation/loop.py`
**Issue**: `SimulationLoop._handle_wire_failure` modifies the `self.electronics.wiring` list in-place.
**Evidence**:

```python
# worker_heavy/simulation/loop.py:657 (approx)
self.electronics.wiring.remove(wire)
```

The `electronics` object (an `ElectronicsSection` Pydantic model) is passed into the `SimulationLoop` constructor and should be treated as an immutable input configuration. Modifying it during simulation means that the original configuration is lost. If the same `electronics` object is reused for a subsequent simulation run (e.g., for a retry or comparison), it will be missing the "failed" wires from the previous run.
**Impact**: High. Non-idempotent simulations and potential for extremely hard-to-debug state leakage between simulation attempts.
**Recommendation**: Create a deep copy of the `electronics` configuration (e.g., `self.electronics = electronics.model_copy(deep=True)`) in the constructor before any potential mutation.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Logic Bug: API and Schema Case Sensitivity Mismatch

**Files**: `shared/enums.py`, `tests/worker_light/test_api.py`, `tests/models/test_schemas.py`
**Issue**: Several enums (e.g., `ResponseStatus`, `MovingPartType`) use uppercase values, but tests and some API clients expect or provide lowercase strings, leading to assertion failures and validation errors.
**Evidence**:

- `ResponseStatus.SUCCESS` is defined as `"SUCCESS"`. `test_fs_write` in `test_api.py` fails because it asserts `== "success"`.
- `MovingPartType` uses `"MOTOR"` and `"PASSIVE"`. `TestMovingPart` passes lowercase strings to the constructor, which fails Pydantic validation because `MovingPartType` does not yet use `UppercaseStrEnum` for normalization.
**Impact**: Medium. Broken test suite and inconsistent API behavior.
**Recommendation**: Systematically adopt `UppercaseStrEnum` for all API-facing enums and update tests to consistently use either the enum members or the correct case-sensitive strings.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Implementation Smell: Significant Logic Duplication in `worker_light/runtime/executor.py`

**File**: `worker_light/runtime/executor.py`
**Issue**: `run_python_code` and `run_python_code_async` contain almost identical logic for environment preparation (`PYTHONPATH`), temporary file management, and logging.
**Evidence**:
Both functions manually construct the `actual_env`, resolve `project_root`, manage a `NamedTemporaryFile`, and handle exceptions/cleanup in virtually identical ways.
**Impact**: Medium. High maintenance burden. Any change to how the environment is set up (e.g., adding a new path to `PYTHONPATH`) must be applied in two places, increasing the risk of desynchronization.
**Recommendation**: Refactor the common setup and cleanup logic into a private context manager or helper function.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 3. Resource Inefficiency: Redundant Client Instantiation in `SharedNodeContext`

**File**: `controller/agent/nodes/base.py`
**Issue**: `SharedNodeContext.create` is called by factory functions for every node execution, which in turn instantiates new `WorkerClient` and `RemoteFilesystemMiddleware` objects.
**Evidence**:

```python
# controller/agent/nodes/base.py:46
worker_client = WorkerClient(...)
# controller/agent/nodes/base.py:79
fs=RemoteFilesystemMiddleware(worker_client),
```

These objects are not being reused across different nodes or even across multiple turns of the same node within a single graph execution. This prevents the `httpx.AsyncClient` inside `WorkerClient` from effectively using connection pooling.
**Impact**: Medium. Increased latency and resource consumption due to repeated TCP/TLS handshakes for every interaction with the worker.
**Recommendation**: Pass a pre-instantiated `WorkerClient` and `RemoteFilesystemMiddleware` to the node factory functions, or store them in the `AgentState`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Maintainability: Brittle and Duplicated Task Filtering Keywords

**Files**: `controller/agent/nodes/coder.py`, `controller/agent/nodes/electronics_engineer.py`
**Issue**: Both `CoderNode` and `ElectronicsEngineerNode` use hardcoded lists of keywords to determine whether to skip a task.
**Evidence**:

- `coder.py` has a list like `["circuit", "wire", "electronics", "routing", "psu", "power"]`.
- `electronics_engineer.py` has a similar list used for the inverse logic.
**Impact**: Low/Medium. Brittle task routing. If a new electronics-related keyword is introduced in a plan but not added to these lists, the nodes will behave incorrectly.
**Recommendation**: Move these keywords to a shared constant or, preferably, use the `ManufacturingMethod` or other metadata in the task definition to route tasks more reliably.

> **User Review**:
> [ ] Agree - [x] Disagree
> *Comments:*
Skip for now... but this is clearly a bad functionality tbh. The goal would be to route via YAML frontmatter or via other [YAML] configs.

---

## 🧹 Implementation Smells

### 5. Code Quality: High Complexity and Hardcoded Constants in `SimulationLoop`

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The `SimulationLoop.step` method is quite long and handles many disparate concerns (electronics updates, dynamic controllers, backend stepping, failure checks, video recording). Additionally, several physical constants are hardcoded.
**Evidence**:

- `MOTOR_OVERLOAD_THRESHOLD_SECONDS = 2.0`
- `SIMULATION_STEP_S = 0.002`
- `MAX_SIMULATION_TIME_SECONDS = 30.0`
**Impact**: Low/Medium. Difficulty in tuning simulation parameters and testing different scenarios without modifying core code.
**Recommendation**: Refactor `SimulationLoop.step` into smaller, well-defined private methods and move hardcoded constants to a configuration object or `objectives.yaml`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
Move constants into (some) file in `config/` folder.
