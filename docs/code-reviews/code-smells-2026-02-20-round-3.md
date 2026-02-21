# Codebase Review - Feb 20, 2026 - Round 3

## Overview

This third round of review focuses on scalability bottlenecks in the controller, fragility in the agent's state management, and pervasive implementation patterns that hinder performance and reliability.

---

## 🚨 Critical Issues

### 1. Non-Persistent Controller State (Scaling Bottleneck)

**File**: `controller/api/routes/episodes.py`, `controller/api/manager.py`
**Issue**: Real-time communication (`manager`) and background task tracking (`task_tracker`) are implemented as in-memory Python objects.
**Impact**: High. This prevents the controller from being scaled horizontally (multiple instances). Furthermore, any restart of the controller will lose tracking of all running agent tasks and disconnect all active WebSockets without a way to resume them.
**Recommendation**: Use a persistent/distributed store like Redis or a database for session management, and rely on a task queue or workflow engine (like Temporal, which is already present) for task tracking.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Bypassing Temporal Workflows for Agent Loops

**File**: `controller/api/routes/episodes.py`
**Issue**: The `continue_episode` endpoint uses `asyncio.create_task` to run agent logic instead of utilizing the existing Temporal workflow infrastructure.
**Impact**: High. Long-running agent loops are susceptible to failure if the controller process dies. Temporal's primary benefit is stateful persistence and recovery for exactly these kinds of multi-step processes.
**Recommendation**: Refactor agent execution to be fully driven by Temporal workflows/signals, ensuring that state is persisted between turns and can survive service restarts.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Brittle StateGraph Routing Logic

**File**: `controller/agent/graph.py`
**Issue**: The `should_continue` function determines the next state by checking for a specific markdown substring in the agent's TODO list.
**Evidence**:
```python
if state.status == AgentStatus.APPROVED:
    if "- [ ]" in state.todo:
        return "coder"
```
**Impact**: Medium. If the agent changes the format of its TODO list slightly (e.g., using `* [ ]` or just `[ ]`), the control flow will break, causing the agent to finish prematurely.
**Recommendation**: Use a structured state field (e.g., a boolean `all_tasks_completed` or a list of task objects) to drive graph transitions rather than parsing unstructured markdown strings.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Potential Runtime Crash in `SimulationLoop.step`

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The variable `state` is used in a conditional expression but is only defined if `target_body_name` is truthy.
**Evidence**:
```python
if target_body_name:
    state = self.backend.get_body_state(target_body_name)
    ...
fail_reason = self.success_evaluator.check_failure(
    current_time,
    target_pos,
    state.vel if target_pos is not None else np.zeros(3), # Crash if target_body_name was None
)
```
**Impact**: Medium. If a benchmark is run without a designated target body, the simulation will crash with an `UnboundLocalError`.
**Recommendation**: Ensure `state` is properly initialized or that `check_failure` is called with safe defaults when no target body is present.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 5. Redundant Node Context Initialization

**File**: `controller/agent/nodes/planner.py`, `coder.py`, etc.
**Issue**: Every agent turn recreates a `SharedNodeContext`, which in turn instantiates new LLM clients (LangChain/DSPy) and Worker clients.
**Impact**: Medium. This is inefficient and prevents the reuse of connection pools. It also adds overhead to every single step of the agent's execution.
**Recommendation**: Pass a singleton context or use a dependency injection pattern to provide the context to nodes without recreating it on every call.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Inefficient Workspace Asset Synchronization

**File**: `controller/api/tasks.py`
**Issue**: `execute_agent_task` synchronizes assets from the worker by recursively listing and reading every file individually.
**Impact**: Medium. For workspaces with many small files (e.g., multiple CAD iterations, logs, renders), this creates hundreds of small HTTP requests, significantly slowing down the end of an episode.
**Recommendation**: Use the existing `bundle_session` (tarball) endpoint to download all assets in a single request and extract them locally.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Invasive Monkeypatching of PySpice

**File**: `shared/pyspice_utils.py`
**Issue**: The module performs a global monkeypatch on `NgSpiceShared._send_char` to handle NGSpice 42 stderr output.
**Impact**: Low/Medium. Monkeypatching third-party libraries at the module level can lead to unexpected side effects in other parts of the system or failures when the library is updated. It also relies on PySpice internal FFI handles.
**Recommendation**: Wrap the PySpice interaction in a more robust way, or contribute the fix upstream to PySpice if it's a general compatibility issue.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## Sweep of Implementation Smells

### 8. Naive Grep Implementation in `FilesystemRouter`

**File**: `shared/workers/filesystem/router.py`
**Issue**: `grep_raw` reads the entire content of every file in mounted directories into memory for regex searching.
**Impact**: Medium. If `utils/` or `skills/` contains large files or many files, this will cause significant memory pressure and slow down search tools used by the agent.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Duplicate Session/Episode Data Models

**File**: `controller/persistence/models.py`
**Issue**: `GenerationSession` and `Episode` coexist and overlap in functionality, but use different status enums and schemas.
**Impact**: Medium. This creates confusion about which model to use and leads to duplicate logic for status tracking and asset management.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Repeated `strip_null_bytes` Pydantic Validators

**File**: `controller/api/routes/episodes.py`, `controller/api/tasks.py`, etc.
**Issue**: The same logic to strip null bytes is copied across multiple Pydantic models.
**Impact**: Low. Violates DRY principle and makes it harder to update the validation logic consistently.
**Recommendation**: Use a custom `Annotated` type with a `BeforeValidator` to reuse the logic.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
