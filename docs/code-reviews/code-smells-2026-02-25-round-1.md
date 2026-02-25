# Codebase Review - Feb 25, 2026 - Round 1

## Overview

This review identifies critical architectural risks in database management and simulation logic, alongside implementation inefficiencies in the API and agent orchestration layers. Key focus areas include resource management (connection pools) and the fidelity of the electronics-physics coupling.

---

## 🚨 Critical Issues

### 1. Database Connection Exhaustion due to Multi-Engine Caching

**File**: `controller/persistence/db.py`
**Issue**: The `get_engine` function caches SQLAlchemy engines per `asyncio` event loop to avoid cross-loop errors. However, `BaseNode._get_tool_functions` (in `controller/agent/nodes/base.py`) creates a **new event loop** for every asynchronous tool call to wrap it in a synchronous interface for DSPy.
**Evidence**:
```python
# controller/persistence/db.py
_engine_cache = {}
def get_engine():
    loop = asyncio.get_running_loop()
    if loop in _engine_cache:
        return _engine_cache[loop]
    # ... creates new engine ...
    _engine_cache[loop] = engine
```
```python
# controller/agent/nodes/base.py
def sync_wrapper(*args, _t=t, **kwargs):
    new_loop = asyncio.new_event_loop() # New loop per tool call!
    try:
        return new_loop.run_until_complete(_t(*args, **kwargs))
```
**Impact**: High. Each tool call that triggers a database interaction (directly or via middleware) spawns a new SQLAlchemy engine and connection pool. A single agent turn can easily exhaust the database connection limit, leading to `Internal Server Error` responses across the entire system.
**Recommendation**: Implement a stable event loop for tool execution or use a thread-safe singleton engine that isn't tied to the current loop. Reusing a single `asyncio` loop with `nest_asyncio` (as done in `WorkerInterpreter`) is a better alternative.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 2. "Stateless" Switches in Physics Simulation

**Files**: `worker_heavy/simulation/electronics.py`, `worker_heavy/simulation/loop.py`
**Issue**: `ElectronicsManager` initializes `switch_states` but there is no mechanism to update these states during the physics simulation steps.
**Impact**: Medium. The system cannot simulate interactive components where physics should trigger electronics changes (e.g., a robotic arm pressing a physical button to power a circuit). Switches remain at their initial value (defaulting to `True`) for the entire 30s simulation regardless of physical collisions.
**Recommendation**: Implement a collision-to-event mapping where specific bodies (buttons/switches) can toggle their corresponding electronic state when hit with sufficient force.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Inconsistent Fallback Power Propagation

**File**: `worker_heavy/simulation/electronics.py`
**Issue**: When SPICE simulation fails, the system falls back to a connectivity-based BFS (`_fallback_update`). This fallback logic completely ignores `self.switch_states`.
**Evidence**:
```python
# worker_heavy/simulation/electronics.py:88
for wire in self.electronics.wiring:
    # ...
    if v and v not in visited:
        # Check if connection is closed (if it involves a switch)
        # This is simplified logic <--- IT DOES NOT CHECK switch_states
        visited.add(v)
```
**Impact**: Medium. In fallback mode, all switches behave as closed circuits (always on). This leads to non-deterministic simulation results depending on whether the SPICE engine succeeded or failed.
**Recommendation**: Ensure the BFS traversal respects the current state of switches/relays in `self.switch_states`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Scalability Issue: Individual State Retrieval in Loop

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The main simulation loop performs individual calls to `get_body_state` and `get_actuator_state` for every monitored component in every step check.
**Evidence**:
```python
# worker_heavy/simulation/loop.py:465
for bname in self.body_names:
    bstate = self.backend.get_body_state(bname) # Multiple calls
```
**Impact**: Medium. For complex assemblies (e.g., a hexapod with 18+ servos), the overhead of these individual calls and the subsequent Pydantic model instantiations significantly degrades simulation throughput.
**Recommendation**: Extend the `PhysicsBackend` protocol to support bulk state retrieval (e.g., `get_all_body_states()`) which can return a single vectorized result from the underlying engine (MuJoCo/Genesis).

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 5. Fragile Terminal Naming Heuristics

**File**: `worker_heavy/simulation/electronics.py`
**Issue**: Power detection in the electronics manager relies on hardcoded terminal name suffixes (`_+` or `_in`).
**Evidence**:
```python
node_name = f"{comp.component_id}_+"
if comp.type in ["switch", "relay"]:
    node_name = f"{comp.component_id}_in"
```
**Impact**: Low/Medium. Components with alternative naming conventions (e.g., `VDD`, `VCC`, `ENABLE`) will be incorrectly reported as unpowered, causing simulation failures even for valid designs.
**Recommendation**: Use the component's metadata or pins definition from the COTS catalog to identify the primary power-delivery terminal.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Suboptimal `httpx.AsyncClient` Lifecycle

**File**: `controller/api/routes/episodes.py`
**Issue**: Proxy endpoints (like `get_episode_asset`) instantiate a new `httpx.AsyncClient` context for every request.
**Impact**: Medium. Each request incurs the overhead of establishing a new connection (TCP/TLS handshake) instead of reusing existing ones. This increases latency and resource pressure on both the controller and worker services.
**Recommendation**: Utilize a shared `httpx.AsyncClient` instance stored in the FastAPI `app.state`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Redundant Circuit Validation Gate

**File**: `worker_heavy/utils/validation.py`
**Issue**: The `simulate` function performs a full circuit validation *before* initializing the `SimulationLoop`. The `SimulationLoop` then creates its own `ElectronicsManager` which performs the exact same validation logic.
**Impact**: Low. Redundant computation and maintenance burden.
**Recommendation**: Perform validation once and pass the result (or the validated circuit object) into the `SimulationLoop`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. Hardcoded Model Provider Prefix

**File**: `controller/agent/nodes/base.py`
**Issue**: `SharedNodeContext` hardcodes the `openai/` prefix for all model names passed to DSPy.
**Evidence**:
```python
dspy_lm = dspy.LM(f"openai/{settings.llm_model}", ...)
```
**Impact**: Low. Prevents the use of other providers (e.g., `anthropic/`, `google/`, `ollama/`) via configuration alone, requiring code changes to switch providers.
**Recommendation**: Allow the provider to be specified in `settings.llm_model` (e.g., `anthropic/claude-3-opus`) and only prepend `openai/` if no provider is present.
