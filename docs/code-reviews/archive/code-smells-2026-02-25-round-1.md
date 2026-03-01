# Codebase Review - Feb 25, 2026 - Round 1

## Overview

This review identifies a critical path traversal vulnerability, significant architectural drift in COTS management, and several resource management issues that could impact system stability and performance.

---

## 🚨 Critical Issues

### 1. Vulnerability: Path Traversal in `FilesystemRouter`

**File**: `shared/workers/filesystem/router.py`
**Issue**: The `_resolve_local_path` method resolves virtual paths to local paths by simple concatenation without validating that the result remains within the intended mount point's directory.
**Evidence**:
```python
# shared/workers/filesystem/router.py:108
def _resolve_local_path(self, path: str, mount: MountPoint) -> Path:
    normalized = path if path.startswith("/") else f"/{path}"
    relative = normalized[len(mount.virtual_prefix) :].lstrip("/")
    return mount.local_path / relative
```
An attacker (or a compromised agent) can provide a path like `/utils/../../etc/passwd` which resolves to `/app/worker/utils/../../etc/passwd`, effectively allowing read access to any file on the worker container. This bypasses the isolation intended for mount points.
**Impact**: High. Information disclosure and potential credential theft from the worker environment.
**Recommendation**: Use `.resolve()` on the resulting path and verify it starts with `mount.local_path.resolve()`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Architecture: Divergent COTS Data Sources

**File**: `worker_heavy/utils/validation.py`, `shared/cots/parts/electronics.py`
**Issue**: There is a massive disconnect between hardcoded `COTSPart` subclasses and the `COTSItemORM` SQL database.
**Evidence**:
`calculate_assembly_totals` in `worker_heavy/utils/validation.py` uses a large `if-elif` chain to manually import and instantiate classes like `PowerSupply` or `ServoMotor`, which themselves have hardcoded data dictionaries. Meanwhile, `shared/cots/database/models.py` defines a full database schema for parts that is being ignored in these critical calculation paths.
**Impact**: Medium/High. Violates "fetch prices from database" requirement. Adding new COTS parts requires code changes in multiple places instead of just a database update.
**Recommendation**: Refactor `calculate_assembly_totals` to query the `parts.db` using the `cots_part_id`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 3. Graph Logic: Dead `cots_search` Node

**File**: `controller/agent/graph.py`, `controller/agent/tools.py`
**Issue**: The `cots_search` node is unreachable in the current LangGraph topology, yet `search_cots_catalog` is already provided as a tool to all nodes.
**Evidence**:
In `graph.py`, there is an edge *from* `cots_search` to `planner`, but no edge leading *to* `cots_search`. Every node already has access to `search_cots_catalog` via `get_common_tools`.
**Impact**: Low/Medium. Redundant code and confusing orchestration. It suggests an unfinished transition between a tool-based and a node-based approach.
**Recommendation**: Either remove the `cots_search` node if the tool is sufficient, or properly integrate it into the conditional routing logic if a dedicated agentic search phase is desired.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Resource Management: Database Connection Leaks

**File**: `shared/observability/persistence.py`
**Issue**: `ControllerCheckpointSaver.create_saver` creates a new `AsyncConnectionPool` on every invocation.
**Evidence**:
```python
# shared/observability/persistence.py:34
@staticmethod
@asynccontextmanager
async def create_saver() -> AsyncIterator[PostgresSaver]:
    conn_string = get_db_url()
    async with AsyncConnectionPool(conn_string, max_size=20) as pool:
        saver = PostgresSaver(pool)
        yield saver
```
This context manager is likely used frequently (e.g., in `setup_persistence` or when interacting with the graph). Creating a new pool per-use is extremely expensive and will quickly exhaust Postgres connection limits if multiple operations occur in parallel.
**Impact**: Medium. Potential for "Too many connections" errors and performance degradation.
**Recommendation**: Initialize a singleton `AsyncConnectionPool` at the module or application level and reuse it.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. Inefficient Execution: Async-over-Sync-over-Async

**File**: `controller/agent/nodes/base.py`
**Issue**: The pattern used to execute agent tools involves nested event loops and unnecessary thread offloading.
**Evidence**:
`_run_program` uses `asyncio.to_thread` to run `dspy.ReAct` (which is sync), and `_get_tool_functions` wraps async tools in a `sync_wrapper` that creates a `new_event_loop()` and calls `run_until_complete`.
**Impact**: Medium. High overhead for every tool call. Creating and destroying event loops hundreds of times per agent session is a significant anti-pattern.
**Recommendation**: Use a native async ReAct implementation if available, or maintain a single worker thread with a persistent event loop for all tool executions.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 6. Simulation Smell: Mutating Input Configuration

**File**: `worker_heavy/simulation/loop.py`
**Issue**: `SimulationLoop._handle_wire_failure` modifies `self.electronics.wiring` in-place.
**Evidence**:
```python
# worker_heavy/simulation/loop.py:657
self.electronics.wiring.remove(wire)
```
The `electronics` object is passed into the constructor and should be treated as immutable input configuration. Modifying it during simulation means that subsequent attempts to simulate the *same* configuration (e.g., if the user wants to retry with different parameters or for optimization) will start with a corrupted (partially deleted) circuit.
**Impact**: Medium. Non-idempotent simulations and hard-to-debug state issues.
**Recommendation**: Create a deep copy of the electronics configuration at the start of the simulation loop if mutation is required for internal state tracking.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Maintainability: Magic Numbers in Simulation

**Files**: `worker_heavy/simulation/loop.py`, `worker_heavy/simulation/genesis_backend.py`
**Issue**: Several critical physical and safety thresholds are hardcoded as constants.
**Evidence**:
- `MOTOR_OVERLOAD_THRESHOLD_SECONDS = 2.0` (Hardcoded in `loop.py`)
- `SIMULATION_STEP_S = 0.002` (Hardcoded in `loop.py`)
- `ELECTRONICS_FLUID_DAMAGE` distance `0.05` (5cm) (Hardcoded in `genesis_backend.py`)
**Impact**: Low/Medium. Difficult to tune behavior for different benchmarks or environments without modifying core simulation code.
**Recommendation**: Move these to a central `SimulationSettings` Pydantic model or allow overriding them via `objectives.yaml`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. Logic Bug: Incomplete Circuit Validation

**File**: `shared/pyspice_utils.py`
**Issue**: `calculate_power_budget` skips proactive open-circuit checks.
**Evidence**:
`calculate_power_budget` calls `validate_circuit(circuit, psu_config)` but fails to pass the `section` argument. `validate_circuit` only performs the "Proactive open-circuit check" if `section` is provided.
**Impact**: Low. Missing validation coverage in certain utility paths.
**Recommendation**: Pass the `ElectronicsSection` to `validate_circuit` in all call sites.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. API Drift: Manual Frontend Requests

**File**: `frontend/src/api/client.ts`
**Issue**: Newer API endpoints are being implemented via manual `__request` calls instead of using the generated service client.
**Evidence**:
Endpoints like `generateBenchmark`, `updateBenchmarkObjectives`, and `steerAgent` are implemented manually at the bottom of `client.ts` with comments like "Manual API calls avoiding generator dependency".
**Impact**: Low/Medium. Loss of type safety for these endpoints and increased maintenance burden to keep them in sync with the backend.
**Recommendation**: Update the OpenAPI generation pipeline to correctly handle these endpoints or investigate why the generator is being avoided.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
