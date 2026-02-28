# Codebase Review - Feb 28, 2026 - Round 1

## Overview

This review identifies several architectural "smells" and implementation risks across the agent orchestration, simulation, and resource management layers. The most significant concerns involve complex event loop handling, lack of resource cleanup, and non-idempotent simulation states.

---

## 🏗️ Architectural Observations

### 1. Complexity: Multi-layered Event Loop Management

**Files**: `controller/agent/nodes/base.py`, `controller/agent/dspy_utils.py`
**Issue**: The system currently uses three distinct strategies to bridge synchronous and asynchronous code during agent execution.
**Evidence**:
1. `BaseNode._run_program` uses `asyncio.to_thread` to run the synchronous `dspy.ReAct` program.
2. `BaseNode._get_tool_functions` creates a `sync_wrapper` for tools that uses `asyncio.run_coroutine_threadsafe` to jump back to the main loop.
3. `WorkerInterpreter.__call__` (used within the same `dspy.ReAct` program) uses `loop.run_until_complete` and `nest_asyncio.apply` to execute remote code.
**Impact**: High. This "event loop inception" is extremely difficult to debug, prone to deadlocks if not handled perfectly, and carries significant performance overhead. It indicates that the orchestration layer is fighting against its chosen framework (DSPy) rather than integrating with it.
**Recommendation**: Standardize on a single bridging mechanism. Ideally, move towards a fully async agent framework or use a dedicated persistent worker thread with a single, clear interface for async tool execution.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Resource Management: New Connections on Every Node Turn

**File**: `controller/agent/nodes/*.py` (factory functions)
**Issue**: Every node execution in the LangGraph (e.g., `planner_node`, `coder_node`) recreates the entire `SharedNodeContext`, including a new `WorkerClient` and `RemoteFilesystemMiddleware`.
**Evidence**:
```python
# controller/agent/nodes/planner.py
async def planner_node(state: AgentState) -> AgentState:
    ctx = SharedNodeContext.create(...)
    node = PlannerNode(context=ctx)
    return await node(state)
```
**Impact**: Medium. This prevents any effective connection pooling between agent turns. It also leads to resource leaks (see Issue 7) because these clients are never explicitly closed.
**Recommendation**: Persist the `SharedNodeContext` (or at least the `WorkerClient`) within the `AgentState` or a session-scoped dependency injection container.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Graph Logic: Unreachable `cots_search` Node

**File**: `controller/agent/graph.py`
**Issue**: The `cots_search` node is defined and has an outgoing edge, but no other node or condition in the graph ever routes to it.
**Evidence**:
```python
builder.add_node("cots_search", cots_search_node)
builder.add_edge("cots_search", "planner")
# No builder.add_edge(..., "cots_search") or conditional routing entry exists.
```
**Impact**: Low. Dead code in the orchestration graph. It might indicate a missing feature (autonomous search phase) or an abandoned architectural direction.
**Recommendation**: Either integrate the node into the conditional routing logic (e.g., from `plan_reviewer`) or remove it to keep the graph clean.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 4. Simulation Smell: Mutating Input Configuration

**File**: `worker_heavy/simulation/loop.py`
**Issue**: `SimulationLoop._handle_wire_failure` modifies the `self.electronics.wiring` list in-place by removing "broken" wires.
**Evidence**:
```python
# worker_heavy/simulation/loop.py:657
self.electronics.wiring.remove(wire)
```
**Impact**: Medium. The `electronics` object is the input specification for the simulation. Mutating it means that if the simulation is re-run (e.g., for a different trial or with slight tweaks), it starts with a corrupted configuration. This violates the expectation of idempotency in simulation runs.
**Recommendation**: The simulation should track internal "live" state separately from the input configuration, or deep-copy the configuration at initialization.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. Logic Bug: Silent Validation Gap in Power Budgeting

**File**: `shared/pyspice_utils.py`
**Issue**: `calculate_power_budget` calls `validate_circuit` without providing the `section` argument, effectively disabling proactive connectivity checks for that path.
**Evidence**:
`validate_circuit` only performs the "Proactive open-circuit check" if `section` is provided. `calculate_power_budget` (line 307) omits it.
**Impact**: Low/Medium. An agent using the power budget tool might receive a "Safe" result for a completely disconnected or incomplete circuit, leading to confusing failures later in full simulation.
**Recommendation**: Ensure the `section` is passed through to `validate_circuit` in all utility functions.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Resource Management: `WorkerClient` Connection Leak

**File**: `controller/clients/worker.py`, `controller/agent/nodes/base.py`
**Issue**: `WorkerClient` creates `httpx.AsyncClient` instances per event loop and stores them in `_loop_clients`. While it has an `aclose()` method, this method is never called in the standard agent execution path.
**Evidence**:
Combined with Issue 2 (recreating clients every turn) and Issue 1 (using `new_event_loop` in some contexts), this leads to a continuous leak of HTTP clients and underlying sockets.
**Impact**: Medium/High. Over time, the controller will exhaust available file descriptors or ports, leading to "Too many open files" errors and system instability.
**Recommendation**: Implement a `finally` block in `BaseNode._run_program` or the node factory functions to ensure `ctx.worker_client.aclose()` is called.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Maintainability: Brittle Keyword-Based Filtering

**File**: `controller/agent/nodes/coder.py`
**Issue**: The decision to skip electronics-related tasks is based on a hardcoded list of strings.
**Evidence**:
```python
# controller/agent/nodes/coder.py
elec_keywords = ["circuit", "wire", "electronics", "routing", "psu", "power"]
# ...
if any(k in item.lower() for k in elec_keywords):
    continue
```
**Impact**: Low/Medium. If a task is described as "Connect the battery" or "Fix the motor lines", it might bypass this filter and cause the `Coder` (who lacks electronics tools) to fail or produce garbage.
**Recommendation**: Use explicit task categorization (metadata) during the planning phase instead of fuzzy keyword matching.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
