# Codebase Review - Feb 22, 2026 - Round 2

## Overview

This review focuses on data persistence, orchestration dead ends, and simulation loop fragilities. Several issues identified in previous rounds remain unaddressed or have surfaced in new forms as the WP2 and WP3 features matured.

---

## 🚨 Critical Issues

### 1. Bug: Simulation Loop Resets Metrics on every `step()` call

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The `step` method calls `self.reset_metrics()` at its beginning.
**Evidence**:
```python
def step(self, ...):
    self.reset_metrics()
    # ...
```
**Impact**: High. If a simulation is executed in multiple chunks (e.g., for progressive rendering or mid-simulation analysis), all previously collected metrics (energy, max velocity, events) are wiped. This prevents accurate cumulative data collection.
**Recommendation**: Move `reset_metrics()` to a separate `start()` or `initialize()` method, or only call it if an explicit `reset` flag is passed.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. High Lint Debt in Worker Services

**Component**: `worker_light/` and others
**Issue**: Running `ruff check` reveals over 900 linting violations across the codebase.
**Impact**: High. Extreme linting noise makes it impossible for developers (or agents) to identify genuine code quality regressions. It also suggests that pre-commit hooks are either missing, bypassed, or poorly configured for the repository's current scale.
**Recommendation**: Perform a codebase-wide linting fix (`ruff check --fix`) and enforce linting in the CI/CD pipeline. Standardize on a single configuration.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Bug: Hardcoded COTS Data in Python Source

**File**: `shared/cots/parts/electronics.py`
**Issue**: Component data (voltage, current, price, weight) is hardcoded in Python dictionaries within class definitions.
**Evidence**:
```python
class PowerSupply(COTSPart):
    psu_data = {
        "LRS-350-24": { ... "price": 35.00 },
        # ...
    }
```
**Impact**: High. This contradicts the "database-backed catalog" requirement. Updates to pricing or specifications require code changes and redeployment instead of simple database updates. It also makes it difficult for the `cots_search` subagent to return consistent results if it queries the DB while implementers use hardcoded classes.
**Recommendation**: Refactor COTS classes to fetch their `data` from `parts.db` during initialization.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Logic Error: `calculate_assembly_totals` ignores generic COTS weight

**File**: `worker_heavy/utils/validation.py`
**Issue**: The calculation loop for `cots_parts` (generic estimates) updates `total_cost` but completely ignores weight.
**Evidence**:
```python
if cots_parts:
    for p in cots_parts:
        total_cost += p.unit_cost_usd * p.quantity
        # ... (weight lookup logic commented out/suppressed)
```
**Impact**: Medium/High. Final weight calculations in `SimulationResult` will be underestimated if the agent uses generic COTS parts, potentially bypassing weight constraints unfairly.
**Recommendation**: Implement weight lookup or ensure `CotsPartEstimate` includes a `weight_g` field that is populated during planning.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 5. Dead Node: `cots_search` still unreachable

**File**: `controller/agent/graph.py`
**Issue**: While the user noted this might be integrated, the code shows no entry edges to `cots_search`.
**Evidence**:
```python
builder.add_node("cots_search", cots_search_node)
builder.add_edge("cots_search", "planner")
```
No node in the graph currently routes *to* `cots_search`.
**Impact**: Medium. The specialized COTS search capability is currently inaccessible via the orchestrator.
**Recommendation**: Add conditional edges from `planner` or `coder` to `cots_search` when a search is requested, or integrate it as a tool that the nodes can call directly (which they currently do via `get_engineer_tools`, making the separate *graph node* redundant).

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Fragile Body Identification in `SimulationLoop`

**File**: `worker_heavy/simulation/loop.py`
**Issue**: `_identify_target_body` ignores the `moved_object.label` provided in `objectives.yaml`.
**Evidence**:
```python
def _identify_target_body(self) -> str | None:
    target_body_name = "target_box"
    # ... falls back to 'target' or 'bucket'
```
**Impact**: Medium. If a benchmark names the moved object anything other than the hardcoded defaults (e.g., "payload_sphere"), the simulation will fail to track the target correctly, leading to false negatives.
**Recommendation**: Prioritize `self.objectives.moved_object.label` if available.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Missing Workflow Registration: `BackupWorkflow`

**File**: `controller/temporal_worker.py`
**Issue**: `BackupWorkflow` is defined in the codebase but not registered in the Temporal worker.
**Impact**: Medium. Automated backups or observability tasks using this workflow will fail or hang indefinitely.
**Recommendation**: Add `BackupWorkflow` to the `workflows` list in `temporal_worker.py`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 8. Non-Package Worker Structure

**Directory**: `worker_light/`
**Issue**: Missing `__init__.py` in the root of `worker_light`.
**Impact**: Low/Medium. Prevents standard package-based imports and makes testing/linting harder (requires `sys.path` hacks).
**Recommendation**: Add `__init__.py` to `worker_light/`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Unused Logic in `validate_circuit`

**File**: `shared/pyspice_utils.py`
**Issue**: A check for PSU voltage drop is implemented but results in a `pass` statement.
**Evidence**:
```python
if v_vcc < (psu_config.voltage_dc * 0.9 if psu_config else 0.1):
    # ...
    pass
```
**Impact**: Low. Dead code that doesn't actually detect shorts via voltage drop.
**Recommendation**: Implement a warning or error if the voltage is significantly lower than expected, or remove the block.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Primitive Collision Detection for Primitive Shapes

**File**: `worker_heavy/simulation/mujoco_backend.py`
**Issue**: `check_collision` for non-mesh geoms only checks the center point of the geometry.
**Evidence**:
```python
else:
    # For non-mesh geoms, just use center for now
    vertices = np.array([geom_pos])
```
**Impact**: Medium. Collision detection for Boxes, Spheres, and Cylinders (common in agent designs) is inaccurate, only triggering if the *center* enters a zone.
**Recommendation**: Sample points on the surface/volume of primitive shapes or use MuJoCo's native collision detection queries.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 11. Persistent Electronics Dirty Flag (Unresolved)

**File**: `worker_heavy/simulation/loop.py`
**Issue**: As identified in Round 1, `_electronics_dirty` is never reset to `False`.
**Evidence**: Re-verified in current source. `_update_electronics` still lacks `self._electronics_dirty = False`.
**Impact**: High. Re-running SPICE 500 times per second after a wire break is a major performance bottleneck.
**Recommendation**: Reset the flag.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 12. Overly Strict Pydantic Validation for Agents

**File**: `shared/models/schemas.py`
**Issue**: `AssemblyDefinition.validate_caps` uses `model_validator(mode="after")` to raise errors if estimated costs exceed caps.
**Impact**: Medium. If an agent makes a tiny calculation error in a YAML file, the entire Pydantic load fails with a hard error.
**Recommendation**: Move this logic to a specialized validation tool (like `validate_and_price.py`) that returns descriptive markdown errors, allowing the agent to "recover" instead of crashing the Pydantic parser.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
