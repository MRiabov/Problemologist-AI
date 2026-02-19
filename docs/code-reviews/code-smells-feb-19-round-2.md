# Codebase Review - Feb 19 - Round 2

## Overview

This round of review identifies deeper architectural and implementation smells, focusing on simulation accuracy, resource management safety, and consistency across physics backends.

---

## 🚨 Critical Issues

### 1. Angular Velocity Mismatch in Genesis Backend

**File**: `worker/simulation/genesis_backend.py`
**Issue**: The `get_body_state` method populates the `angvel` field of `BodyState` using `entity.get_ang()`, which returns Euler angles (orientation), instead of `entity.get_angvel()`.
**Evidence**:
```python
# L540
return BodyState(
    pos=entity.get_pos().tolist(),
    quat=entity.get_quat().tolist(),
    vel=entity.get_vel().tolist(),
    angvel=entity.get_ang().tolist(), # Should be get_angvel()
)
```
**Impact**: High. Any logic relying on angular velocity (e.g., centrifugal force calculations, stability metrics, or agent rewards based on rotational speed) will receive orientation data instead of velocity. This can lead to incorrect simulation success/failure determination.
**Recommendation**: Use `entity.get_angvel()` to populate the `angvel` field.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Potential Thread/Resource Leak in `BaseNode._run_program`

**File**: `controller/agent/nodes/base.py`
**Issue**: The execution loop uses `asyncio.wait_for` on a thread created by `asyncio.to_thread`. When a timeout occurs, the thread is NOT cancelled.
**Evidence**:
```python
# L161
try:
    prediction = await asyncio.wait_for(
        asyncio.to_thread(program, **inputs),
        timeout=300.0,  # 5 minutes
    )
except asyncio.TimeoutError:
    # ... logging ...
    raise RuntimeError(f"DSPy program {node_type} timed out after 300s")
```
**Impact**: Medium-High. In Python, `asyncio.to_thread` runs the function in a `ThreadPoolExecutor`. `wait_for` only stops waiting for the future; it cannot kill the underlying thread. If an LLM agent hangs or enters an infinite loop, the thread will leak and continue to consume resources (CPU, memory, network) indefinitely.
**Recommendation**: Consider using a separate process for the DSPy program execution if timeout reliability is critical, or ensure the `program` itself is responsive to an interruption signal.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 3. God Function SRP Violation in `simulate` and `api_validate`

**File**: `worker/utils/validation.py` and `worker/api/routes.py`
**Issue**: The `simulate` function handles too many responsibilities: loading objectives, assembly definitions, building scenes, orchestrating loops, handling GPU OOM retries, rendering, and persistence.
**Impact**: High complexity makes testing difficult and increases the risk of side effects when modifying one part of the flow.
**Recommendation**: Extract these into a pipeline or orchestrator class. For example, separate "Scene Preparation", "Execution", "Post-Processing", and "Reporting" into distinct, testable units.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Fragile Global `BACKEND_CACHE` in Multi-Process Environment

**File**: `worker/simulation/factory.py`
**Issue**: `BACKEND_CACHE` is a module-level global dictionary.
**Impact**: In a multi-worker production environment (e.g., Uvicorn with multiple workers), each worker process will have its own cache. While this provides isolation, it also means GPU memory usage is multiplied by the number of workers. Additionally, `MAX_ACTIVE_SESSIONS = 4` might be too low or too high depending on the actual GPU memory available.
**Recommendation**: Implement a more robust resource-aware manager for backends, possibly using a shared state (like Redis) or a dedicated "Simulation Coordinator" service to manage GPU allocation.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 5. Redundant Actuator State Retrieval

**File**: `worker/simulation/loop.py`
**Issue**: The `step` method fetches all actuator states to calculate energy, and then `_check_motor_overload` (called in the same loop) fetches them again for monitored actuators.
**Evidence**:
```python
# L256 (in step loop)
actuator_states = {
    n: self.backend.get_actuator_state(n) for n in self.actuator_names
}

# L290 (in same loop)
if self._check_motor_overload():
    # ...
```
And in `_check_motor_overload`:
```python
# L418
forces = [
    abs(self.backend.get_actuator_state(n).force) for n in self._monitor_names
]
```
**Impact**: Unnecessary overhead in the hot loop of the simulation.
**Recommendation**: Pass the already-fetched `actuator_states` to `_check_motor_overload`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Hardcoded Financial/Physical Heuristics

**File**: `worker/utils/validation.py`
**Issue**: `calculate_assembly_totals` uses hardcoded values for wire cost and weight.
**Evidence**:
```python
# L201
length_m = wire.length_mm / 1000.0
total_cost += length_m * 0.5
total_weight += length_m * 20.0
```
**Impact**: Inaccurate cost and weight estimates. Different wire gauges (AWG) and materials have significantly different costs and weights.
**Recommendation**: Move these constants to `manufacturing_config.yaml` and look them up based on the wire properties.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Physically Incorrect Flow Rate Heuristic

**File**: `worker/simulation/loop.py`
**Issue**: The flow rate calculation uses a hardcoded heuristic relating particle count to volume.
**Evidence**:
```python
# L376
# Heuristic: 1 particle ~= 0.001L (1ml) for MVP
measured_volume_l = passed_count * 0.001
```
**Impact**: The result is entirely dependent on the `particle_budget`. If a user increases the particle budget for higher fidelity, the reported flow rate will artificially increase, potentially causing a false "success".
**Recommendation**: Calculate the volume per particle as `total_initial_volume / n_particles` where `total_initial_volume` comes from the `FluidDefinition`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. Redundant File Operations in `ReviewerNode`

**File**: `controller/agent/nodes/reviewer.py`
**Issue**: The node manually reads `assembly_definition.yaml` and then passes it again to `_run_program` for validation.
**Evidence**:
```python
# L66
if await self.ctx.worker_client.exists("assembly_definition.yaml"):
    assembly_definition = await self.ctx.worker_client.read_file("assembly_definition.yaml")

# L81
validate_files = ["simulation_result.json", "assembly_definition.yaml"]
```
**Impact**: Minor efficiency loss due to double-reading the same file from the remote worker.
**Recommendation**: Consolidate the reading and validation logic.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Geometric Shift Risk in `MeshProcessor`

**File**: `worker/simulation/builder.py`
**Issue**: `_recenter_mesh` shifts the mesh based on its `centroid`.
**Impact**: If the `build123d` part's local origin (0,0,0) is not at its centroid, this recentering will cause a mismatch between the simulation body position and the expected visual position. Most simulation engines expect the mesh to be relative to the body's local frame.
**Recommendation**: Instead of recentering to centroid, the mesh should be exported relative to the `build123d` local coordinate system, or the transformation should be explicitly tracked and compensated in the MJCF/JSON `pos` field.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Inefficient Recursive Asset Synchronization

**File**: `controller/api/tasks.py`
**Issue**: `sync_dir` performs a recursive walk with individual network calls for every file and directory.
**Impact**: Extremely slow for sessions with many files or complex directory structures.
**Recommendation**: Use the `/fs/bundle` endpoint to download the entire workspace as a single tarball and extract it locally.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 11. Backend Feature Inconsistency: Welds

**File**: `worker/simulation/builder.py`
**Issue**: `MuJoCoSimulationBuilder` supports weld constraints extracted from part metadata, but `GenesisSimulationBuilder` completely ignores them.
**Impact**: Assemblies that rely on welds for structural integrity will fall apart in Genesis but work in MuJoCo.
**Recommendation**: Unified weld resolution in `CommonAssemblyTraverser` and implementation in both builders.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 12. Stale Scene Cache in `GenesisBackend`

**File**: `worker/simulation/genesis_backend.py`
**Issue**: `load_scene` reuses the built scene if the `scene_path` matches.
**Evidence**:
```python
# L102
if (
    self._is_built
    and self.scene_meta
    and self.scene_meta.scene_path == scene.scene_path
    and self.scene_meta.config == scene.config
):
    return
```
**Impact**: If a user modifies the `scene.json` or `scene.xml` file on disk but keeps the same filename, the backend will reuse the old, stale scene.
**Recommendation**: Include a hash of the file content or a timestamp in the cache key.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
