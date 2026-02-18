# Codebase Review - Feb 18 - Round 1

## Overview

Evaluated core agent logic (`controller/agent`), simulation backend (`worker/simulation`), and integration endpoints (`worker/api`).
Focus areas: Concurrency, Architecture, Hardcoded values, and recent refactors.

## 🚨 Critical Issues

### 1. Concurrency vs. Serialization in Worker

**File**: `worker/api/routes.py`
**Issue**: The `SIMULATION_EXECUTOR` uses `ProcessPoolExecutor(max_workers=1)`. This effectively serializes all simulations **per worker instance**.
However, `worker/simulation/genesis_backend.py` implements a global `threading.Lock` (`_lock`).
**Impact**:

- If `max_workers=1` in the process pool, the global lock in `GenesisBackend` is redundant for preventing concurrent *access* (since only one process runs at a time per pool).
- If the goal is **parallel** simulation, `max_workers` should be > 1. But `Genesis` has global state issues (referenced in code comments).
- **Confusion**: The integration test `test_int_004_simulation_serialization` asserts that two simulations can "run". With `max_workers=1`, they run *sequentially*. If the specific requirement is *concurrency* (parallel execution), the current implementation fails that requirement, masquerading as success via queueing.
**Recommendation**: Clarify the concurrency requirement. If parallel simulation is needed, we need distinct worker *processes* (not just a pool in one worker) or a way to isolate Genesis state better. If serialization is intended, remove the complex global locking in `GenesisBackend` as the process pool constraints handle it.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Global State in API Routes

**File**: `worker/api/routes.py`
**Issue**: `SIMULATION_QUEUE_DEPTH` and `HEAVY_OPERATION_LOCK` are module-level global variables.
**Impact**:

- In a production setting with multiple Uvicorn workers (processes), these globals are **local to each process**.
- `SIMULATION_QUEUE_DEPTH` will not reflect the *total* system load, only the load on that specific worker process.
- `HEAVY_OPERATION_LOCK` cannot synchronize heavy operations across different Uvicorn worker processes.
**Recommendation**: Use an external coordinator (like Redis) for queue depth tracking and distributed locking if strict global limits are required.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

## 🏗️ Architectural Observations

### 3. God Classes in Simulation

**Files**: `worker/simulation/genesis_backend.py`, `worker/simulation/loop.py`
**Issue**:

- `GenesisBackend` handles: initialization, scene loading (JSON parsing), stepping, stress analysis, fluid damage, and rendering.
- `SimulationLoop` handles: backend selection, manufacturing validation, video rendering, success evaluation, and metric collection.
**Impact**: Hard to maintain and test. `GenesisBackend._load_scene_internal` is ~200 lines of mixed logic.
**Recommendation**:
- Extract `GenesisSceneLoader` to handle JSON/MJCF parsing and entity creation.
- Extract `SceneRenderer` to handle camera and rendering logic specific to Genesis.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Hard Dependency on DSPy in Agent Nodes

**File**: `controller/agent/nodes/base.py`
**Issue**: `SharedNodeContext` initializes `dspy.LM` directly using `settings.openai_api_key`.
**Impact**: Makes unit testing `BaseNode` or its subclasses difficult without mocking the network or providing real API keys.
**Recommendation**: Inject the `dspy_lm` provider or factory into the context, allowing tests to provide a dummy/mock LM more easily.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

## 🧹 Implementation Smells

### 5. Hardcoded "Dummy" Values

**File**: `controller/agent/nodes/base.py`
**Code**: `api_key = "dummy"` (Line 43)
**Issue**: Silently falls back to "dummy" if `settings.openai_api_key` is missing.
**Recommendation**: Explicitly raise an error if the API key is missing in production/dev modes, or use a more descriptive placeholder that clearly indicates "MOCK_MODE".

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Hardcoded Task Keywords

**File**: `controller/agent/nodes/coder.py`
**Code**: `elec_keywords = ["circuit", "wire", ...]`
**Issue**: Domain logic (what constitutes an "electronics task") is buried in string matching within the Coder node.
**Recommendation**: Move this logic to a centralized task classifier or property on the Task object.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Approximate Physics Logic

**File**: `worker/simulation/genesis_backend.py` -> `_check_electronics_fluid_damage`
**Code**: `if np.any(dist < 0.05): # 5cm threshold` and "Very rough approximation".
**Issue**: 5cm is a huge margin for electronics. This could trigger false positives easily.
**Recommendation**: Use actual collision detection or a much tighter bounding box check.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. Hardcoded Logic in `GenesisBackend`

**File**: `worker/simulation/genesis_backend.py`
**Issue**: Hardcoded material defaults (`aluminum_6061`, `310e6` Pa limit) scattered throughout `step()`.
**Recommendation**: These should come from the `mfg_config` or the scene definition exclusively.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

## ✅ Positive Notes

- **Pydantic Usage**: `worker/workbenches/models.py` uses strict types (`StrictStr`, `StrictFloat`). Excellent adherence to data safety rules.
- **Process Isolation**: using `'spawn'` context for `SIMULATION_EXECUTOR` is the correct move for dealing with sensitive C-extension libraries like Genesis/Torch.
- **Integration Tests**: `test_architecture_p0.py` covers the critical paths (health, isolation, simulation flow) well.
