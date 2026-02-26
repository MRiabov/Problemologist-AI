# Codebase Review - Feb 19 - Round 1

## Overview

This review identifies architectural and implementation smells, focusing on logic duplication, state management safety, and consistency between components.

---

## 🚨 Critical Issues

### 1. Blatant Logic Duplication in `SimulationLoop.step`

**File**: `worker/simulation/loop.py`
**Issue**: The logic to determine simulation success or failure is duplicated exactly at the end of the `step` method.
**Evidence**:

- Lines 511-526 and 528-536 contain nearly identical blocks for setting `is_success`.
- The second block uses `fail_reason` (local variable) while the first uses `self.fail_reason`.
**Impact**: High maintenance risk. Changes to success criteria must be made in two places. Potential for bugs if the local `fail_reason` and `self.fail_reason` diverge.
**Recommendation**: Consolidate the success determination logic into a single block or a dedicated helper method.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Non-Thread-Safe Global State in Validation Utils

**File**: `worker/utils/validation.py`
**Issue**: The module uses a global variable `LAST_SIMULATION_RESULT` to cache the result of the last simulation.
**Evidence**:

- `LAST_SIMULATION_RESULT: SimulationResult | None = None` (Line 31)
- Used in `get_stress_report` and `preview_stress`.
**Impact**: Since simulations are run in a `ProcessPoolExecutor`, the child process updates its own local copy of this global variable, but the parent process (the API worker) remains unaware. Subsequent calls to `get_stress_report` in the parent process will fall back to disk I/O, making the global variable useless at best and misleading at worst in a multi-request environment.
**Recommendation**: Remove the global state. Pass the `SimulationResult` explicitly to functions that need it, or rely solely on persistent storage (disk) with proper session isolation.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Broken Power Propagation in Electronics Fallback

**File**: `worker/simulation/electronics.py`
**Issue**: The `_fallback_update` method attempts to find power sources using component types that do not exist in the defined `ElectronicComponentType` enum.
**Evidence**:

- `electronics.py:L48`: `if c.type in ["battery", "v_source"]`
- `shared/enums.py:L114`: `POWER_SUPPLY = "power_supply"`
**Impact**: The electronics simulation will never find a power source, resulting in all components being marked as unpowered. This renders the power-gating logic in `SimulationLoop` ineffective.
**Recommendation**: Align the check with `ElectronicComponentType.POWER_SUPPLY`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Broken and Inefficient `WorkerClient.exists`

**File**: `controller/clients/worker.py`
**Issue**:

1. **Bug**: The implementation expects a dictionary with a `"files"` key, but the worker API `/fs/ls` returns a raw list.
2. **Efficiency**: It checks for file existence by performing a full `ls` of the parent directory.
**Evidence**:

- `worker.py:L157`: `files = response.json()["files"]`
**Impact**: The controller cannot reliably check if files exist on the worker, causing silent failures in `ReviewerNode` when it tries to read optional files.
**Recommendation**: Fix the JSON access. Implement a proper `/fs/exists` endpoint on the worker that uses `os.path.exists`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 5. Mixed Units in Assembly Schema

**File**: `shared/models/schemas.py`
**Issue**: `AssemblyDefinition` mixes grams (`estimated_weight_g`) and kilograms (`benchmark_max_weight_kg`) in the same object.
**Impact**: Extreme risk of 1000x scaling errors during validation or reporting.
**Recommendation**: Standardize on a single unit (preferably grams for the scale of parts handled).

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. SRP Violation in Validation Utilities

**File**: `worker/utils/validation.py`
**Issue**: The `simulate` and `validate` methods are massive and handle too many responsibilities, including file loading, scene building, loop orchestration, rendering, and result persistence.
**Impact**: Hard to test in isolation and difficult to maintain.
**Recommendation**: Refactor these into smaller, single-purpose functions or classes (e.g., `SimulationOrchestrator`).

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 7. Missing `MOTOR_OVERLOAD` Failure Mode

**File**: `shared/enums.py` and `worker/simulation/loop.py`
**Issue**: The memory mentions a `MOTOR_OVERLOAD` failure mode, but it is missing from `SimulationFailureMode`. Instead, the loop uses `OVERCURRENT` for motor stalls.
**Evidence**:

- `loop.py:L415`: `self.fail_reason = SimulationFailureMode.OVERCURRENT`
**Impact**: Lack of granularity in failure reporting. `OVERCURRENT` should ideally be reserved for electronics/wiring issues, while `MOTOR_OVERLOAD` describes a physical stall.
**Recommendation**: Add `MOTOR_OVERLOAD` to `SimulationFailureMode` and update the loop to use it.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. Inefficient Actuator Monitoring

**File**: `worker/simulation/loop.py`
**Issue**: The simulation loop calls `self.backend.get_actuator_state(n)` twice per step for every actuator (once for energy calculation and once for overload checks).
**Impact**: Unnecessary overhead in high-frequency simulation loops.
**Recommendation**: Fetch the state once per step and pass it to the monitoring functions.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Redundant Wire Length Logic

**File**: `shared/wire_utils.py`
**Issue**: Both `calculate_path_length` and `calculate_length` exist with slightly different implementations.
**Recommendation**: Consolidate into a single robust implementation.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Silent Exception Swallowing

**File**: `worker/utils/validation.py`
**Issue**: `calculate_assembly_totals` uses bare `except Exception: pass` blocks.
**Impact**: Masked bugs during cost and weight estimation, potentially leading to incorrect "success" reports for invalid assemblies.
**Recommendation**: Log exceptions and consider them validation failures.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 11. Import in Hot Loop

**File**: `worker/simulation/loop.py`
**Issue**: `from shared.wire_utils import get_awg_properties` is called inside the `step` loop.
**Recommendation**: Move the import to the top of the file.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
