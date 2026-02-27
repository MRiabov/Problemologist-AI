# Codebase Review - Feb 27, 2026 - Round 2

## Overview

This round focuses on implementation bugs in observability, resource management inefficiencies, and architectural disconnects in the simulation and COTS handling.

---

## 🚨 Critical Issues

### 1. Bug: Nested Method Definitions in `DatabaseCallbackHandler`

**File**: `controller/observability/database.py`
**Issue**: `record_tool_start_sync` and `record_tool_end_sync` are incorrectly defined *inside* the `record_tool_end` method.
**Evidence**:
```python
# controller/observability/database.py:112
    async def record_tool_end(
        self, trace_id: int, output_data: str, is_error: bool = False
    ) -> None:
        ...
        try:
            async with self.session_factory() as db:
                ...
        except Exception as e:
            logger.warning("database_tool_end_failed", error=str(e))

        def record_tool_start_sync(self, tool_name: str, input_data: str) -> int:
            ...
```
**Impact**: High. These methods are intended to be class methods but are currently local functions within another method. Any caller attempting to use `db_callback.record_tool_start_sync` will fail with an `AttributeError`. This likely breaks synchronous tool recording which is used in the agent nodes.
**Recommendation**: Move these methods to the class level and ensure they are properly decorated if necessary, or called as standard instance methods.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Resource Management: Database Connection Leaks

**File**: `shared/observability/persistence.py`
**Issue**: `ControllerCheckpointSaver.create_saver` initializes a new `AsyncConnectionPool` on every call.
**Evidence**:
```python
# shared/observability/persistence.py:38
@staticmethod
@asynccontextmanager
async def create_saver() -> AsyncIterator[PostgresSaver]:
    conn_string = get_db_url()
    async with AsyncConnectionPool(conn_string, max_size=20) as pool:
        saver = PostgresSaver(pool)
        yield saver
```
**Impact**: High. Every time a node starts or state is persisted, a new pool of 20 connections is created. In a busy system, this will immediately exhaust the Postgres `max_connections` limit.
**Recommendation**: Use a singleton pool or a long-lived pool managed at the application lifecycle level.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 3. Graph Logic: Unreachable `cots_search` Node

**File**: `controller/agent/graph.py`
**Issue**: The `cots_search` node exists in the graph and has an outgoing edge to `planner`, but no node or entry point leads *to* it.
**Evidence**:
```python
builder.add_node("cots_search", cots_search_node)
...
builder.add_edge("cots_search", "planner")
# No builder.add_edge(...) or conditional edge pointing TO "cots_search"
```
**Impact**: Medium. Dead code in the orchestration layer. It indicates an incomplete feature where COTS searching was meant to be a dedicated phase but was instead implemented as a tool.
**Recommendation**: Either remove the node or implement the routing logic to utilize it when the agent expresses intent to search the catalog.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Implementation: Non-Idempotent Simulation Mutation

**File**: `worker_heavy/simulation/loop.py`
**Issue**: `SimulationLoop._handle_wire_failure` modifies the input `electronics` object in-place by removing failed wires.
**Evidence**:
```python
# worker_heavy/simulation/loop.py:657
self.electronics.wiring.remove(wire)
```
**Impact**: Medium. If a simulation is run multiple times on the same `ElectronicsSection` instance (e.g., for sensitivity analysis or optimization), subsequent runs will start with a degraded circuit.
**Recommendation**: Deep copy the `electronics` object in the `SimulationLoop` constructor.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 5. Logic: Missing Validation Coverage in `calculate_power_budget`

**File**: `shared/pyspice_utils.py`
**Issue**: `calculate_power_budget` calls `validate_circuit` without passing the `section` argument.
**Evidence**:
```python
# shared/pyspice_utils.py:273
def calculate_power_budget(circuit: Circuit, psu_config: PowerSupplyConfig) -> dict:
    res = validate_circuit(circuit, psu_config) # Missing 'section'
```
The `validate_circuit` function only performs the "Proactive open-circuit check" if the `section` is provided.
**Impact**: Low/Medium. Reduced validation rigor in the power budget calculation path compared to the main simulation path.
**Recommendation**: Ensure `section` is passed to all `validate_circuit` calls where available.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Simulation: Hardcoded Physics Thresholds

**File**: `worker_heavy/simulation/loop.py`, `worker_heavy/simulation/genesis_backend.py`
**Issue**: Critical constants for failure detection are hardcoded.
**Evidence**:
- `MOTOR_OVERLOAD_THRESHOLD_SECONDS = 2.0`
- `SIMULATION_STEP_S = 0.002`
- `dist < 0.05` (5cm) for electronics fluid damage in `genesis_backend.py`.
**Impact**: Low. Makes it difficult to adapt the simulation for different scales (e.g., very small precision robots vs large machinery) without changing core code.
**Recommendation**: Move these to `ObjectivesYaml` or a simulation configuration file.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. API Drift: Manual Frontend Client Implementation

**File**: `frontend/src/api/client.ts`
**Issue**: Multiple endpoints are being implemented manually using `__request` instead of the generated `EpisodesService` or `DefaultService`.
**Evidence**:
```typescript
// Manual API calls avoiding generator dependency
export async function generateBenchmark(...) { ... }
export async function updateBenchmarkObjectives(...) { ... }
```
**Impact**: Low. Maintenance burden to keep types and paths in sync. It suggests the OpenAPI generator is failing or is too cumbersome for these specific routes.
**Recommendation**: Fix the generator configuration to include these routes or document why manual implementation is required.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. COTS: Hardcoded Part Calculations

**File**: `worker_heavy/utils/validation.py`
**Issue**: `calculate_assembly_totals` manually imports and instantiates classes like `PowerSupply` and `ServoMotor` which contain hardcoded data, ignoring the `COTSItemORM` database.
**Evidence**:
A long `if-elif` chain in `calculate_assembly_totals` manually maps `ElectronicComponentType` to specific Python classes.
**Impact**: Medium. Adding a new part requires code changes in `shared/cots/parts/` and potentially `worker_heavy/utils/validation.py`, rather than just a database entry.
**Recommendation**: Refactor to use a unified part lookup service that queries the SQL database.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
