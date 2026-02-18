# Codebase Review - Feb 18 - Round 2

## Overview

This review identifies a second round of architectural and implementation smells, focusing on physics accuracy, cross-component reliability, and resource management.

---

## 🚨 Critical Issues

### 1. Massive Simulation Time Drift (Genesis Smoke Mode)

**File**: `worker/simulation/loop.py` & `worker/simulation/genesis_backend.py`
**Issue**: When Genesis is in "smoke test mode", it uses a hardcoded `dt=0.05` for its internal simulation step. However, the `SimulationLoop` continues to iterate using a fixed `dt=0.002` and increments the total simulation time by only `0.002` per Genesis step.
**Evidence**:
- `genesis_backend.py:L142`: `dt=0.05 if is_smoke else 0.002`
- `loop.py:L268`: `dt = 0.002 # Default step for loop logic`
- `loop.py:L283-284`: `res = self.backend.step(dt)` -> `current_time = res.time`
**Impact**: The simulation clock drifts by 25x. A requested "10 second" simulation will actually execute 250 seconds of physics. This renders all time-based objectives (like flow rates or stability durations) completely incorrect in smoke mode.
**Recommendation**: The `SimulationLoop` must use the actual `dt` reported by the backend, or the backend must perform the correct number of substeps to match the requested `dt`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Broken and Inefficient `WorkerClient.exists`

**File**: `controller/clients/worker.py`
**Issue**:
1. **Bug**: The implementation expects a dictionary with a `"files"` key (`response.json()["files"]`), but the worker API `/fs/ls` returns a raw list of objects. This will cause a `TypeError` at runtime.
2. **Efficiency**: It checks for file existence by performing a full `ls` of the parent directory.
**Evidence**:
- `worker.py:L106`: `files = response.json()["files"]`
- `routes.py:L76`: `@router.post("/fs/ls", response_model=list[FileInfo])`
**Impact**: The controller cannot reliably check if files exist on the worker. In large directories, this operation is also O(N) instead of O(1).
**Recommendation**: Fix the JSON access bug. Implement a dedicated `/fs/exists` endpoint in the worker API that uses `os.path.exists` for O(1) performance.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Broken Energy Metric in Genesis Backend

**File**: `worker/simulation/genesis_backend.py` & `worker/simulation/loop.py`
**Issue**: The `energy` calculation in the loop relies on the `ctrl` value from `ActuatorState`. However, `GenesisBackend` always returns `ctrl=0.0` because it does not track the last applied control signal.
**Evidence**:
- `genesis_backend.py:L430`: `return ActuatorState(..., ctrl=0.0, ...)`
- `loop.py:L291`: `energy = sum(abs(state.ctrl * state.velocity) ...)`
**Impact**: The `total_energy` metric is always 0.0 for all Genesis simulations. This prevents the agent from optimizing for power efficiency.
**Recommendation**: The `GenesisBackend` should store the values passed to `apply_control` in a local dictionary and return them in `get_actuator_state`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Memory Leak in Dynamic Component Loading

**File**: `worker/utils/loader.py`
**Issue**: `load_component_from_script` generates a unique `module_name` (using UUID) for every call and executes it using `importlib`. These modules are added to `sys.modules` and are never removed.
**Evidence**:
- `loader.py:L70`: `module_name = f"dynamic_build_{uuid.uuid4().hex}"`
- `loader.py:L77`: `spec.loader.exec_module(module)`
**Impact**: In a long-running worker process, `sys.modules` will grow indefinitely, eventually leading to Out-of-Memory (OOM) failures.
**Recommendation**: Use `sys.modules.pop(module_name, None)` in a `finally` block after the component has been successfully loaded and used.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. Brittle CI Workflow Configuration

**File**: `.github/workflows/gemini-review.yml` (and others)
**Issue**: The CI workflows for Gemini integration do not check for the presence of required secrets/variables before execution. Additionally, a bug in the `google-github-actions/run-gemini-cli@v0` action causes a confusing "Matching delimiter not found 'EOF'" error when the CLI fails and outputs JSON without a trailing newline.
**Evidence**:
- `gemini-review.yml:L44`: Runs the action regardless of whether auth is configured.
- CI Logs: `::error::Invalid value. Matching delimiter not found 'EOF'` and `Please set an Auth method...`
**Impact**: CI fails on all Pull Requests if the repository owner hasn't specifically configured Gemini secrets, blocking the development flow for non-configured environments (like automated bot PRs or forks). The EOF error masks the true root cause (missing auth).
**Recommendation**: Add `if` conditions to skip Gemini steps if no authentication method is provided. (Note: This has been addressed in the current PR to unblock CI).

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 6. Brittle Electronics Power Propagation

**File**: `worker/simulation/electronics.py`
**Issue**:
1. The fallback BFS power propagation checks for component types `"battery"` and `"v_source"`, but these are not valid values in the `ElectronicComponentType` enum.
2. The logic ignores `switch_states`, meaning power flows through switches regardless of whether they are open or closed.
**Evidence**:
- `electronics.py:L35`: `if c.type in ["battery", "v_source"]`
- `enums.py:L94`: `POWER_SUPPLY = "power_supply"`
- `electronics.py:L52-54`: Switch check is a placeholder comment.
**Impact**: Electronics simulation is disconnected from actual circuit logic. All components will appear unpowered (due to the type mismatch) or incorrectly powered (ignoring switches).
**Recommendation**: Align the check with `ElectronicComponentType.POWER_SUPPLY` and implement actual state checks for `switch` and `relay` components during the BFS traversal.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Inconsistent Units in Assembly Definition

**File**: `shared/models/schemas.py`
**Issue**: The `AssemblyDefinition` model mixes grams and kilograms for weight. `totals.estimated_weight_g` is in grams, but `constraints.planner_target_max_weight_kg` is in kilograms.
**Evidence**:
- `schemas.py:L513`: `estimated_weight_g: float`
- `schemas.py:L529`: `planner_target_max_weight_kg: float`
**Impact**: Extreme risk of 1000x scaling errors. The validator at `L563` correctly divides by 1000, but having both units in the same object is a known recipe for "Mars Climate Orbiter" style failures.
**Recommendation**: Standardize on a single unit (grams preferred for small assemblies) or use a custom `Weight` type that encapsulates the unit.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 8. Crude Schematic Visualization

**File**: `controller/api/routes/episodes.py`
**Issue**: The `get_episode_schematic` endpoint generates a "Soup JSON" for schematics using a simplistic linear layout with hardcoded `10 + i * 40` spacing.
**Evidence**:
- `episodes.py:L166`: `"center": {"x": 10 + i * 40, "y": 10}`
**Impact**: For any circuit with more than 3-4 components, the schematic becomes a mess or an unreadable straight line. It also assumes every component has exactly 2 pins, which is false for relays and complex ICs.
**Recommendation**: Use a basic force-directed graph algorithm or at least separate components into "Source", "Logic/Switch", and "Load" rows.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Blind Retry Loop in Agent Nodes

**File**: `controller/agent/nodes/base.py`
**Issue**: The `_run_program` method implements a retry loop for validation failures. However, it does not feed the `validation_errors` back into the next iteration of the DSPy program.
**Evidence**:
- `base.py:L142-166`: The loop increments `retry_count` but doesn't update the `inputs` or the `journal` in a way the model can see.
**Impact**: The agent is likely to repeat the same error three times, as it has no context on *why* its previous output was invalid.
**Recommendation**: Append the validation errors to the `feedback` or a dedicated `errors` field in the `inputs` for subsequent retries.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Blatant Code Duplication in `wire_utils.py`

**File**: `shared/wire_utils.py`
**Issue**: The `AWG_PROPERTIES` dictionary and the `get_awg_properties` function are defined twice in the same file.
**Evidence**:
- `wire_utils.py:L16` and `L64` (AWG_PROPERTIES)
- `wire_utils.py:L114` and `L141` (get_awg_properties)
**Impact**: Maintenance risk; changes to one definition might not be reflected in the other.
**Recommendation**: Remove the duplicated blocks.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 11. Inefficient Actuator Monitoring

**File**: `worker/simulation/loop.py`
**Issue**: The simulation step loop calls `self.backend.get_actuator_state(n)` twice for every actuator in every step (once for energy, once for overload checks).
**Evidence**:
- `loop.py:L293-294` and `L324` (via `_check_motor_overload`)
**Impact**: Unnecessary overhead. For high-frequency simulations with many motors, this can measurably slow down the loop.
**Recommendation**: Fetch the state once per step, cache it in a local variable, and reuse it for both energy and overload calculations.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 12. Mean Position for FEM/MPM Bodies

**File**: `worker/simulation/genesis_backend.py`
**Issue**: For deformable bodies (FEM) or fluids (MPM), `get_body_state` returns the mean position of all nodes/particles.
**Evidence**:
- `genesis_backend.py:L413`: `pos = state.pos[0].mean(axis=0).tolist()`
**Impact**: This can be highly misleading for asymmetric deformations or spread-out fluids, where the "mean" might be a point in empty space.
**Recommendation**: Consider returning the bounding box center or the position of the "root" node if applicable.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
