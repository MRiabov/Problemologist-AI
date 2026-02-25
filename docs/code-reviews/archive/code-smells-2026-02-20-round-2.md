# Codebase Review - Feb 20, 2026 - Round 2

## Overview

This second round of review for today focuses on deeper architectural bottlenecks, inefficient IO patterns, and misleading implementation details in the worker and simulation layers.

---

## 🚨 Critical Issues

### 1. Misleading `to_mjcf` Implementation

**File**: `worker_heavy/utils/validation.py`
**Issue**: The `to_mjcf` utility is hardcoded to use the Genesis backend but returns the result as "MJCF" (MuJoCo XML).
**Evidence**:
```python
def to_mjcf(component: Compound, renders_dir: Path | None = None, ...) -> str:
    ...
    builder = get_simulation_builder(
        output_dir=renders_dir, backend_type=SimulatorBackendType.GENESIS
    )
    scene_path = builder.build_from_assembly(component, ...)
    return scene_path.read_text()
```
**Impact**: High. If Genesis is used, `scene_path` points to a `scene.json` file. A function named `to_mjcf` returning JSON content is highly misleading and will cause failures in any consumer expecting valid XML.
**Recommendation**: Rename the function to `to_scene_description` or ensure it correctly selects the MuJoCo builder when MJCF is explicitly required.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Excessive Serialization Bottleneck in Worker API

**File**: `worker_heavy/api/routes.py`
**Issue**: A combination of a global `HEAVY_OPERATION_LOCK` and a single-worker `ProcessPoolExecutor` serializes all heavy operations across all sessions.
**Evidence**:
- `HEAVY_OPERATION_LOCK = asyncio.Lock()` used in almost every POST endpoint.
- `SIMULATION_EXECUTOR = ProcessPoolExecutor(max_workers=1, ...)`
**Impact**: High. While likely intended to manage GPU memory, this prevents concurrent execution of non-GPU heavy tasks (like `analyze`, `validate`, or `build`) and makes the worker a massive bottleneck in multi-session environments.
**Recommendation**: Fine-grain the locking strategy (e.g., only lock GPU-bound simulations) and consider increasing `max_workers` for CPU-bound tasks like geometric validation or topology analysis.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Inefficient Mesh Face Export Round-trip

**File**: `worker_heavy/simulation/builder.py`
**Issue**: `export_topology_glb` performs a slow filesystem round-trip for every face of a part.
**Evidence**:
```python
for i, face in enumerate(faces):
    fname = tmp_path / f"face_{i}.stl"
    export_stl(face, fname) # Filesystem write
    try:
        face_mesh = trimesh.load(fname) # Filesystem read
```
**Impact**: Medium. For complex parts with hundreds of faces, this creates significant IO overhead and slows down scene building/preview generation.
**Recommendation**: Use `trimesh` directly on the vertices/faces data or find a way to pass the data in-memory without multiple STL exports/imports.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. $O(N^2)$ Intersection Check in `validate`

**File**: `worker_heavy/utils/validation.py`
**Issue**: Geometric validation uses a nested loop to check every solid against every other solid for intersections.
**Evidence**:
```python
    solids = component.solids()
    if len(solids) > 1:
        for i in range(len(solids)):
            for j in range(i + 1, len(solids)):
                intersection = solids[i].intersect(solids[j])
```
**Impact**: Medium. As assembly complexity grows, this $O(N^2)$ check becomes prohibitively slow. `build123d` or `cadquery` often have more efficient ways to check for self-intersections or use spatial indexing.
**Recommendation**: Implement a broad-phase check (e.g., bounding box overlap) before performing the expensive `intersect()` operation.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 5. Redundant Actuator State Fetching

**File**: `worker_heavy/simulation/loop.py`
**Issue**: `_check_motor_overload` re-fetches actuator states that were just retrieved in the main step loop.
**Evidence**:
Inside `step()`:
```python
actuator_states = {n: self.backend.get_actuator_state(n) for n in self.actuator_names}
...
if self._check_motor_overload(dt * check_interval):
```
Inside `_check_motor_overload()`:
```python
forces = [abs(self.backend.get_actuator_state(n).force) for n in self._monitor_names]
```
**Impact**: Medium. Re-querying the physics backend (especially MuJoCo or Genesis) for states is expensive. Doing it twice per check interval is wasteful.
**Recommendation**: Pass the already-fetched `actuator_states` to `_check_motor_overload`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. IO-Heavy `objectives.yaml` Updates

**File**: `worker_heavy/utils/validation.py`
**Issue**: Helper functions like `define_fluid` and `set_soft_mesh` perform a full load-modify-save cycle on `objectives.yaml`.
**Impact**: Low/Medium. If multiple properties are set sequentially, the file is rewritten multiple times. More importantly, concurrent updates from different tools could lead to race conditions or data loss if not carefully synchronized.
**Recommendation**: Use an in-memory representation of objectives that is flushed once, or use a more robust configuration management system.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Over-reliance on Internal Imports

**File**: `worker_heavy/utils/validation.py` and `worker_heavy/simulation/builder.py`
**Issue**: Pervasive use of imports inside functions to avoid circular dependencies.
**Evidence**: `import numpy`, `from .rendering import ...`, `from shared.models.schemas import ...` are found inside many functions.
**Impact**: Medium. Makes the code harder to read, hides dependencies, and slightly degrades performance on the first call. It suggests that the module boundaries between `models`, `utils`, and `simulation` are blurred.
**Recommendation**: Refactor module structure to resolve circularities at the top level, or use `TYPE_CHECKING` blocks where appropriate.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## Sweep of Implementation Smells

### 8. Unnecessary `subprocess` for Tar Extraction

**File**: `worker_heavy/api/routes.py`
**Issue**: `bundle_context` uses `subprocess.run(["tar", ...])` instead of the built-in `tarfile` module.
**Impact**: Low. Adds a dependency on the system `tar` binary and is less portable. The comment suggests "robustness against metadata bugs," but `tarfile` is generally preferred in Python.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Hardcoded and Inconsistent Particle Budgets

**File**: Multiple (`loop.py`, `factory.py`, `validation.py`)
**Issue**: Particle budgets for Genesis are hardcoded in multiple places with different defaults (5000 vs 100000).
**Evidence**: `SimulationLoop` defaults to 5000/100000, `simulate()` has another check, and `GpuOomRetryEvent` hardcodes 5000.
**Impact**: Low. Makes it difficult to tune performance or memory usage globally.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Missing Error Handling in Event Collection

**File**: `worker_heavy/api/routes.py`
**Issue**: `_collect_events` is called only in the success path or explicitly in some catch blocks, but not consistently.
**Impact**: Low. Events generated during a failed operation might be lost if the endpoint returns early with an error.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
