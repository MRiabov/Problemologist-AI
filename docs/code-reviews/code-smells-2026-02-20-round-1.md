# Codebase Review - Feb 20, 2026 - Round 1

## Overview

This review identifies architectural and implementation smells, focusing on performance bottlenecks, thread safety, and data integrity across the controller and worker nodes.

---

## 🚨 Critical Issues

### 1. Thread Leak Risk in `BaseNode._run_program`

**File**: `controller/agent/nodes/base.py`
**Issue**: The use of `asyncio.to_thread` wrapped in `asyncio.wait_for` creates a risk of thread leaks.
**Evidence**:
```python
prediction = await asyncio.wait_for(
    asyncio.to_thread(program, **inputs),
    timeout=300.0,  # 5 minutes
)
```
**Impact**: In Python, `asyncio.to_thread` runs the function in a separate thread from a thread pool. If `asyncio.wait_for` times out, it cancels the *awaitable*, but it cannot cancel the underlying thread. If the DSPy `program` hangs or takes a very long time, the thread will persist until it finishes, potentially exhausting the thread pool over time.
**Recommendation**: Consider using a subprocess-based execution for long-running, potentially hanging LLM programs, or ensure the underlying library (DSPy/OpenAI) has its own robust timeout mechanisms that propagate to the thread.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:*
Doesn't DSPy have this stuff? Why not at least async, why give a whole thread to it? afaik, it's very light on CPU, why threading?
### 2. Inconsistent Units and Broken Formulas in Reward Metrics

**File**: `controller/agent/dspy_utils.py`
**Issue**: A combination of unit mismatches and field naming discrepancies breaks reward calculation.
**Evidence**:
1. **Unit Mismatch**: `map_events_to_prediction` (L300) divides `weight_g` by 1000, converting it to kilograms, while the rest of the system (and tests) expects grams.
2. **Naming Mismatch**: `cad_simulation_metric` (L158) looks for `max_weight`, but the schema and test fixtures use `max_weight_g`.
3. **Broken Eval**: This causes `evaluate_formula` to fail with `NameError: name 'max_weight' is not defined`, resulting in a score of 0.0 for those components.
**Impact**: High. Distorts agent training and evaluation. Reward signals for weight optimization are either zeroed out or 1000x incorrect.
**Recommendation**: Standardize on grams across the entire pipeline. Align field names between the reward configuration, `PredictionMetrics`, and `ObjectivesYaml`.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:*

### 3. Performance Bottleneck in `MuJoCoBackend.check_collision`

**File**: `worker_heavy/simulation/mujoco_backend.py`
**Issue**: Extremely inefficient collision check implementation for meshes.
**Evidence**:
Lines 232-243: The code iterates over all vertices of a mesh and performs a manual vertex-in-box/sphere check.
```python
            if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                mesh_id = self.model.geom_dataid[geom_id]
                vert_adr = self.model.mesh_vertadr[mesh_id]
                vert_num = self.model.mesh_vertnum[mesh_id]
                raw_verts = self.model.mesh_vert[vert_adr : vert_adr + vert_num]
                vertices = geom_pos + raw_verts @ geom_mat.T
```
**Impact**: High. This is called for every body against every forbidden zone site, every step of the simulation. For assemblies with many parts and complex meshes, this will drastically slow down simulation speed.
**Recommendation**: Use MuJoCo's built-in collision detection (contacts) or at least perform a broad-phase AABB check before iterating over vertices.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:*
Agree, but what about genesis? I know it has a native collision check too
### 4. `SimulationLoop` Ignores `simulation_bounds` from Objectives

**File**: `worker_heavy/simulation/loop.py` and `worker_heavy/simulation/evaluator.py`
**Issue**: Hardcoded failure criteria for "out of bounds" ignoring the declarative specification.
**Evidence**:
- `evaluator.py:L37`: `if qpos is not None and len(qpos) >= 3 and qpos[2] < -2.0: return SimulationFailureMode.OUT_OF_BOUNDS`
- `loop.py` does not pass `objectives.simulation_bounds` to the evaluator.
**Impact**: Medium. Benchmarks that require different world bounds (e.g., flying drones or underwater ROVs) will fail incorrectly or miss genuine out-of-bounds events.
**Recommendation**: Pass `simulation_bounds` from `ObjectivesYaml` to `SuccessEvaluator` and use it to validate the object's position.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:* Looks critical. However, however, maybe we rescale it? I doubt that we do. Anyway, it should come from objectives yaml.

### 5. Logic Bug in `AssemblyDefinition.moving_parts`

**File**: `shared/models/schemas.py`
**Issue**: Redundant or incorrect attribute access in the `moving_parts` property.
**Evidence**:
Lines 473-475:
```python
control=p_config.config.config
if hasattr(p_config.config, "config")
else p_config.config.control,
```
**Impact**: Low but confusing. `p_config.config` is an `AssemblyPartConfig`, which does not have a `config` attribute. This logic will always fall back to `.control`, making the `hasattr` check useless and suggesting a misunderstood schema structure.
**Recommendation**: Simplify to `control=p_config.config.control`.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:* `hasattr` and `getattr` is always a code smell. refactor ofc

---

## 🏗️ Architectural Observations

### 6. Inefficient `SharedNodeContext` Recreation

**File**: `controller/agent/nodes/base.py` and various node factory functions (e.g., `reviewer.py`, `planner.py`)
**Issue**: Shared context is recreated on every node execution.
**Evidence**:
```python
@type_check
async def reviewer_node(state: AgentState) -> AgentState:
    ctx = SharedNodeContext.create(...)
    node = ReviewerNode(context=ctx)
    return await node(state)
```
**Impact**: Medium. Re-initializes `PromptManager` (which loads YAML files), `ChatOpenAI`, and `dspy.LM` instances every time a node is visited in the LangGraph. In a cyclic graph, this adds unnecessary overhead.
**Recommendation**: Pass the context as part of the LangGraph's `config` or use a singleton/cached pattern for shared resources.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:* Probably a singleton. But to be clear... we will soon optimize to dspy prompt optimization/compilation, so ???

### 7. Mixed Responsibilities in `SimulationLoop` Initialization

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The constructor performs heavy tasks like pricing and material lookup.
**Evidence**:
Lines 98-124: Calls `validate_and_price` and builds `material_lookup` inside `__init__`.
**Impact**: Makes the class harder to test and violates the Single Responsibility Principle. If pricing fails or takes too long, it blocks scene loading.
**Recommendation**: Move pricing and pre-simulation validation to a separate orchestrator or factory.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 8. Redundant Self-Import

**File**: `worker_heavy/simulation/loop.py`
**Issue**: Importing a constant from its own module inside a method.
**Evidence**:
Line 414: `from worker_heavy.simulation.loop import SIMULATION_STEP_S`
**Impact**: Very low, but indicates poor code organization and is confusing to read.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:* huh?

### 9. Hardcoded Camera in Video Rendering

**File**: `worker_heavy/simulation/loop.py`
**Issue**: Video rendering assumes a camera named "main" exists.
**Evidence**:
Line 322: `frame = self.backend.render_camera("main", 640, 480)`
**Impact**: Low. If the scene (MJCF/Genesis) doesn't define a "main" camera, rendering might fail or default to an undesirable view.
**Recommendation**: Allow the camera name to be specified in `objectives.yaml` or fallback to the first available camera.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Incorrect `dt` used in Motor Overload Check

**File**: `worker_heavy/simulation/loop.py`
**Issue**: `_check_motor_overload` uses the constant `SIMULATION_STEP_S` even when the loop's actual `dt` is different.
**Evidence**:
Lines 221-223:
```python
        dt = SIMULATION_STEP_S
        if self.smoke_test_mode and self.backend_type == SimulatorBackendType.GENESIS:
            dt = 0.05
...
        return self.success_evaluator.check_motor_overload(
            self._monitor_names, forces, self._monitor_limits, SIMULATION_STEP_S
        )
```
**Impact**: Medium. In smoke test mode (where `dt=0.05`), motor overload will be calculated 25x slower than it should be, potentially missing stalls.
**Recommendation**: Pass the local `dt` variable to `_check_motor_overload`.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:* It was done for simulaton speed, but it appears this is a bug that runs over the entire codebase now. double-check.

### 11. Hardcoded Wire Properties in Totals Calculation

**File**: `worker_heavy/utils/validation.py`
**Issue**: `calculate_assembly_totals` uses hardcoded magic numbers for wire cost and weight.
**Evidence**:
Lines 240-241:
```python
total_cost += length_m * 0.5
total_weight += length_m * 20.0
```
**Impact**: Low. Inconsistent with the rest of the system which uses AWG-based lookups.
**Recommendation**: Use `shared.wire_utils.get_awg_properties` to get accurate weight and cost per meter.

> **User Review**:
> [x] Agree - [ ] Disagree
> *Comments:* Wire is a COTS?

### 12. `ElectronicsManager` uses Stale Types in Fallback

**File**: `worker_heavy/simulation/electronics.py`
**Issue**: Check for non-existent component types.
**Evidence**:
Line 75: `if c.type in [ElectronicComponentType.POWER_SUPPLY, "battery", "v_source"]`
**Impact**: Low. `"battery"` and `"v_source"` are not in the `ElectronicComponentType` enum and will never be assigned to `c.type` if the data is Pydantic-validated.
**Recommendation**: Remove the stale string literals or add them to the enum if they are intended to be supported.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
