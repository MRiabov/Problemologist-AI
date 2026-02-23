# Codebase Review - Feb 23, 2026 - Round 1

## Overview

This round of review focuses on deeper architectural bottlenecks, event loop safety in the agentic orchestration, and persistent logic errors in the simulation heavy-lifting components. We also address discrepancies between system documentation (memory) and the actual implementation state.

---

## 🚨 Critical Issues

### 1. Architectural Bottleneck: Strictly Sequential Heavy Operations

**File**: `worker_heavy/api/routes.py`
**Issue**: The heavy worker uses a global `HEAVY_OPERATION_LOCK` and a `ProcessPoolExecutor` with `max_workers=1`.
**Evidence**:
```python
# worker_heavy/api/routes.py:114
SIMULATION_EXECUTOR = ProcessPoolExecutor(
    max_workers=1,
    ...
)

# worker_heavy/api/routes.py:126
async with HEAVY_OPERATION_LOCK:
    # ...
```
**Impact**: High. Even with multiple worker instances, if they share a GPU or are limited by these settings, the system cannot handle concurrent requests from multiple agents. Scaling will be impossible without revisiting this serialization.
**Recommendation**: Increase `max_workers` if hardware allows, or implement a more sophisticated multi-tenant queue that manages GPU memory instead of a binary lock.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Event Loop Risk: Unsafe `asyncio.to_thread` with `nest_asyncio`

**File**: `controller/agent/nodes/base.py`, `controller/agent/dspy_utils.py`
**Issue**: DSPy programs are run in a background thread via `asyncio.to_thread`, which then attempts to run async worker calls using `loop.run_until_complete` and `nest_asyncio`.
**Evidence**:
```python
# controller/agent/dspy_utils.py:100 (WorkerInterpreter.__call__)
if loop.is_running():
    import nest_asyncio
    nest_asyncio.apply(loop)
result = loop.run_until_complete(self._execute_remote(code))
```
**Impact**: High. This pattern is notorious for causing deadlocks, especially when combined with thread-local storage or complex library interactions (like DSPy's global settings). It effectively bypasses the benefits of async/await in the controller.
**Recommendation**: Refactor `WorkerInterpreter` to be natively async if DSPy supports it, or use a dedicated, managed event loop in a single implementation thread.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Logic Bug: `SimulationLoop` ignores Objective-defined Target Body

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The simulation loop uses hardcoded strings ("target_box", "bucket") to identify the object to track for success, ignoring the `label` provided in `objectives.yaml`.
**Evidence**:
```python
# worker_heavy/simulation/loop.py:503
def _identify_target_body(self) -> str | None:
    target_body_name = "target_box"
    # ... checks for "target" or "bucket"
    return None
```
**Impact**: Medium. Benchmarks that define a custom moved object (e.g., "cylinder_red") will fail to track success even if the object reaches the goal, as the loop is looking for a "target_box".
**Recommendation**: Prioritize `self.objectives.moved_object.label` if available.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 4. Redundant Data Models: `GenerationSession` vs `Episode`

**File**: `controller/persistence/models.py`
**Issue**: The database contains both `GenerationSession` and `Episode` tables which track nearly identical information (status, logs, prompt).
**Evidence**:
```python
class GenerationSession(Base):
    __tablename__ = "generation_sessions"
    # ... session_id, prompt, status, validation_logs

class Episode(Base):
    __tablename__ = "episodes"
    # ... id, task, status, metadata_vars (containing validation_logs)
```
**Impact**: Medium. Synchronization between these two tables is manual and error-prone. It increases database complexity without clear separation of concerns (Benchmark Generation vs. Engineering Problem Solving).
**Recommendation**: Consolidate into a single polymorphic table or a clear Parent-Child relationship where `Episode` is the primary execution record.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. False Optimization: Genesis `render_only` still builds physics

**File**: `worker_heavy/simulation/genesis_backend.py`
**Issue**: The `load_scene` method accepts a `render_only` flag, but the Genesis backend still executes a full `self.scene.build(n_envs=1)`, which triggers kernel compilation and physics setup.
**Impact**: Medium. Static renders (like the 24-view validation renders) take significantly longer than necessary because they are initializing a full physics world they never use.
**Recommendation**: Implement a true "visual-only" build path in `GenesisBackend` or use a lighter renderer (like MuJoCo) for purely geometric validation previews.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Event Loop Blocking in COTS Wrapper

**File**: `shared/cots/agent.py`
**Issue**: `DSPyLangGraphWrapper.ainvoke` is an async method that performs a synchronous, blocking call to `self.program(...)`.
**Evidence**:
```python
# shared/cots/agent.py:73
async def ainvoke(self, input_data: dict, _config: dict | None = None) -> dict:
    # ...
    result = self.program(requirement=task) # Sync call!
```
**Impact**: Medium. This blocks the main event loop of the controller during COTS search, preventing other concurrent operations (like UI updates or other agent turns) from progressing.
**Recommendation**: Wrap the call in `asyncio.to_thread` or use an async version of the DSPy program.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 7. Missing Infrastructure: Registry of Backups

**File**: `controller/temporal_worker.py`
**Issue**: Documentation (Memory) suggests that `BackupWorkflow` and `run_backup_activity` are registered for automated backups, but they are absent from the code.
**Impact**: Low/Medium. False sense of security regarding data safety.
**Recommendation**: Implement and register the backup workflows or update documentation to reflect current capabilities.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. State Inconsistency: Missing `journal` in Benchmark State

**File**: `controller/agent/benchmark/state.py`
**Issue**: `BenchmarkGeneratorState` lacks the `journal` field present in `AgentState`.
**Impact**: Medium. Prevents unified reasoning tracking and summarization (e.g., by the `SummarizerNode`) when running benchmark generation episodes.
**Recommendation**: Add `journal: str = ""` to `BenchmarkGeneratorState` to maintain parity with the engineering agent state.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Fragile PSU Identification in SPICE Validation

**File**: `shared/pyspice_utils.py`
**Issue**: The `validate_circuit` function assumes the main power source is named "vsupply" when calculating total current draw.
**Impact**: Low/Medium. If an agent names their PSU differently or uses multiple sources, the overcurrent protection and power budget calculations will return 0.0 or fail.
**Recommendation**: Use the `component_id` from `psu_config` (e.g., `f"v{psu_config.component_id}"`) to reliably identify the source branch.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. (Repeated) Persistent Electronics Dirty Flag

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The `_electronics_dirty` flag is never reset after `_update_electronics()`. (Carried over from Feb 22 as it remains unfixed).
**Impact**: High. Massive performance degradation after any wire breakage.
**Recommendation**: Set `self._electronics_dirty = False` after successful update.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
