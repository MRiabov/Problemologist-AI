# Codebase Review - Feb 27, 2026 - Round 2

## Overview

This second round of review highlights persistent issues from Round 1 that remain unaddressed, alongside new observations regarding resource management efficiency and simulation logic redundancy.

---

## 🏗️ Architectural Observations

### 1. Resource Inefficiency: SQLite Engine Overhead in COTS Search
**File**: `shared/cots/runtime.py`
**Issue**: The `search_parts` function creates a new SQLAlchemy engine and session for every single search query.
**Evidence**:
```python
# shared/cots/runtime.py:9
def search_parts(query: SearchQuery, db_path: str) -> list[COTSItem]:
    engine = create_engine(f"sqlite:///{db_path}")
    # ...
    with Session(engine) as session:
```
**Impact**: Medium. While SQLite is light, repeatedly creating engines and opening/closing file handles for every tool invocation by the agent is unnecessary overhead. In a high-concurrency environment, this could lead to performance bottlenecks or file lock contention.
**Recommendation**: Initialize the engine once at the module level or within a singleton service and reuse it.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Dead Code: Unreachable `cots_search` Node (Persisting)
**File**: `controller/agent/graph.py`
**Issue**: The `cots_search` node is added to the StateGraph but has no incoming edges, making it impossible to reach during agent execution.
**Evidence**:
```python
builder.add_node("cots_search", cots_search_node)
# ...
builder.add_edge("cots_search", "planner") # Outgoing edge exists, but no incoming edge
```
**Impact**: Low. Dead code that increases maintenance complexity and provides a false sense of architectural capability.
**Recommendation**: Either integrate the node into the conditional routing logic or remove it if the `search_cots_catalog` tool is the preferred interface.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Architecture: Divergent COTS Data Sources (Persisting)
**File**: `worker_heavy/utils/validation.py`, `tests/test_cots_search.py`
**Issue**: `calculate_assembly_totals` continues to rely on hardcoded imports of `COTSPart` subclasses instead of querying the centralized COTS database. Additionally, unit tests for the centralized COTS search are broken as they attempt to use `.invoke()` on a standard function.
**Evidence**:
- In `worker_heavy/utils/validation.py:273`: Hardcoded `PowerSupply`, `ElectronicRelay`, etc. are imported and instantiated directly.
- In `tests/test_cots_search.py`: `search_cots_catalog.invoke(...)` results in `AttributeError` because it is a standard function, not a LangChain/DSPy tool.
**Impact**: High. Adding new parts to the catalog requires code changes in `shared/cots/parts/` and manual imports in `validation.py`, bypassing the flexibility offered by the SQL database. The lack of working tests for the database search indicates it is likely bit-rotting.
**Recommendation**: Refactor assembly total calculations to use `shared.cots.runtime.search_parts`. Fix unit tests by calling the search function directly.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🚨 Resource & Stability Issues

### 4. Database Connection Leaks (Persisting)
**File**: `shared/observability/persistence.py`
**Issue**: `ControllerCheckpointSaver.create_saver` still creates a new `AsyncConnectionPool` on every invocation.
**Impact**: Medium/High. Exhausts Postgres connection limits under load. This was flagged in Round 1 and remains a significant risk.
**Recommendation**: Implement a singleton connection pool for the `PostgresSaver`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. Simulation Smell: Input Mutation in `SimulationLoop` (Persisting)
**File**: `worker_heavy/simulation/loop.py`
**Issue**: `_handle_wire_failure` modifies the `electronics.wiring` list in-place.
**Evidence**:
```python
# worker_heavy/simulation/loop.py:657
self.electronics.wiring.remove(wire)
```
**Impact**: Medium. Prevents the simulation from being idempotent. If a simulation is rerun on the same session (e.g., for optimization), the "input" configuration will have been permanently altered by previous failures.
**Recommendation**: Deep copy the electronics configuration during `SimulationLoop.__init__`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 6. Redundant Logic: Double Electronics Updates
**File**: `worker_heavy/simulation/loop.py`
**Issue**: The `step` method contains redundant flag resets and update calls.
**Evidence**:
```python
# worker_heavy/simulation/loop.py:350
if self.electronics and self._electronics_dirty:
    self._update_electronics()
    self._electronics_dirty = False # This is already handled inside _update_electronics? (No, it's not)
    self._apply_gated_controls(control_inputs)
    # ...
```
Actually, `_update_electronics` does NOT clear the flag, but the code calls it and then clears it. However, the simulation loop often sets and clears this in ways that are hard to trace. More importantly, `_update_electronics` is called in `__init__`, but `_electronics_dirty` is initialized to `False` even though the manager might need an update if initialized with data.
**Recommendation**: Centralize electronics state transitions to ensure updates only happen when necessary and the "dirty" state is managed predictably.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Logic Bug: Incomplete Circuit Validation (Persisting)
**File**: `shared/pyspice_utils.py`
**Issue**: `calculate_power_budget` still calls `validate_circuit` without the `section` argument, bypassing open-circuit detection.
**Impact**: Low. Missing validation coverage in the power budget tool path.
**Recommendation**: Pass the `ElectronicsSection` to `validate_circuit`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. API Drift: Manual Frontend Requests (Persisting)
**File**: `frontend/src/api/client.ts`
**Issue**: Several critical endpoints (benchmark generation, steering) are still implemented using manual `__request` calls.
**Impact**: Low/Medium. Reduced type safety and increased maintenance burden for the frontend.
**Recommendation**: Synchronize the OpenAPI generator or move these to a typed service layer.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
