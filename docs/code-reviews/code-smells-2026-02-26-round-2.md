# Codebase Review - Feb 26, 2026 - Round 2

## Overview

This second round of review focuses on security vulnerabilities in the virtual filesystem, implementation debt in simulation utilities, and performance inefficiencies in the frontend state management.

---

## 🚨 Critical Issues

### 1. Vulnerability: Unprotected Path Traversal in Mount Resolution

**File**: `shared/workers/filesystem/router.py`
**Issue**: While `LocalFilesystemBackend` has path traversal protection, the `FilesystemRouter`'s `_resolve_local_path` method (which handles virtual mounts like `/utils`, `/skills`, etc.) does not.
**Evidence**:
```python
# shared/workers/filesystem/router.py:108
def _resolve_local_path(self, path: str, mount: MountPoint) -> Path:
    normalized = path if path.startswith("/") else f"/{path}"
    relative = normalized[len(mount.virtual_prefix) :].lstrip("/")
    return mount.local_path / relative
```
An agent could provide a path like `/utils/../../../../etc/passwd`. The `relative` part would become `../../../../etc/passwd`, and when joined with `mount.local_path`, it allows escaping the intended directory.
**Impact**: High. Allows reading sensitive files on the worker host if the agent is compromised or malicious.
**Recommendation**: Apply the same `.resolve()` and `.startswith()` check used in `LocalFilesystemBackend._resolve` to `_resolve_local_path`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 2. Boilerplate Smell: Repeated Objective/Config Loading

**File**: `worker_heavy/utils/validation.py`
**Issue**: `ObjectivesYaml` and manufacturing `config` are manually loaded using `yaml.safe_load` and `Path.read_text` in almost every function (`define_fluid`, `set_soft_mesh`, `simulate`, `validate`, `validate_fem_manufacturability`).
**Evidence**:
Multiple blocks like:
```python
if obj_path.exists():
    data = yaml.safe_load(obj_path.read_text())
    objs = ObjectivesYaml(**data)
```
**Impact**: Medium. High maintenance burden and inconsistent handling of template placeholders (some functions check for `[TEMPLATE]`, others don't).
**Recommendation**: Create a centralized `load_session_objectives(path: Path)` utility that handles template detection and Pydantic validation.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Refactoring Debt: Python Runtime Executor Duplication

**File**: `worker_light/runtime/executor.py`
**Issue**: `run_python_code` and `run_python_code_async` share nearly identical setup logic for environment variables, `PYTHONPATH` resolution, and temporary file management.
**Evidence**:
Lines 55-82 and 132-159 are near-perfect duplicates.
**Impact**: Low/Medium. Increases the risk of bugs (e.g., fixing a `PYTHONPATH` issue in one but forgetting the other).
**Recommendation**: Extract the common setup and cleanup logic into a context manager or a shared private helper.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. API Design: Inconsistent Validation vs Simulation Outputs

**File**: `worker_heavy/utils/validation.py`
**Issue**: The `validate()` function returns a simple `tuple[bool, str | None]`, whereas `simulate()` returns a rich `SimulationResult` Pydantic model.
**Evidence**:
- `validate` (line 697): `return True, None`
- `simulate` (line 467): `return SimulationResult(...)`
**Impact**: Medium. Forces the caller (and the API layer) to handle two completely different response patterns for geometric vs. physical validation.
**Recommendation**: Refactor `validate` to return a `ValidationResult` model that inherits from or shares structure with `SimulationResult`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 5. Engineering Inaccuracy: Simplified Motor Load Model

**File**: `shared/circuit_builder.py`
**Issue**: Motors are modeled as simple resistors based on `stall_current_a`.
**Evidence**:
```python
# shared/circuit_builder.py:86
resistance = v_rated / i_stall
circuit.R(f"m_{comp.component_id}", ..., resistance @ u_Ohm)
```
**Impact**: Medium. This represents the absolute worst-case (stalled) draw. During normal "operating point" analysis, this leads to extremely pessimistic power budget calculations and frequent "Overcurrent" warnings that might not reflect real-world running conditions.
**Recommendation**: Implement a more sophisticated motor model (e.g., using a behavioral source to account for back EMF) or provide both "Stall" and "Nominal" DC operating point checks.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Technical Debt: Divergent Wire Property Logic

**File**: `shared/circuit_builder.py`, `shared/wire_utils.py`
**Issue**: `circuit_builder.py` uses a rough heuristic formula for wire resistance, while `wire_utils.py` contains a standardized industrial table.
**Evidence**:
- `circuit_builder.py:120`: `r_per_m = 0.01 * (1.26 ** (wire.gauge_awg - 15))`
- `wire_utils.py:13`: `AWG_PROPERTIES = { 10: { "resistance_ohm_m": 0.00327, ... } }`
**Impact**: Low. Inconsistent simulation results. A wire might pass a "clearance" check based on one diameter and fail a "voltage drop" check based on a different resistance.
**Recommendation**: `circuit_builder.py` should import and use `get_awg_properties` from `shared.wire_utils`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Brittle Implementation: NGSpice Error Detection

**File**: `shared/pyspice_utils.py`
**Issue**: Detection of singular matrices (floating nodes) relies on parsing strings from the captured `stdout` of the `NgSpiceShared` library.
**Evidence**:
```python
# shared/pyspice_utils.py:112
for line in simulator._ngspice_shared._stdout:
    if "singular matrix" in line.lower():
```
**Impact**: Medium. This is fragile and may fail if the underlying `ngspice` version changes its output formatting or language.
**Recommendation**: Investigate if PySpice provides a more programmatic way to catch simulation convergence failures or singular matrix errors.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. Performance Smell: Inefficient Episode Polling

**File**: `frontend/src/context/EpisodeContext.tsx`
**Issue**: The fallback polling mechanism uses `JSON.stringify` to detect changes in the `selectedEpisode` object.
**Evidence**:
```typescript
// frontend/src/context/EpisodeContext.tsx:354
if (JSON.stringify(prev) === JSON.stringify(currentEp)) return prev;
```
**Impact**: Medium. For long-running episodes with hundreds of traces and assets, this stringification happens every 10 seconds on the main thread, causing UI jank.
**Recommendation**: Implement a shallow comparison or use a `version` / `updated_at` field to detect changes.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Security/UX: No Resource Constraints on User Code

**File**: `worker_light/runtime/executor.py`
**Issue**: Subprocesses for user-provided Python scripts are launched with a timeout but no memory or CPU limits.
**Evidence**:
`asyncio.create_subprocess_exec` is called without any `cgroups` or `ulimit` configuration.
**Impact**: Medium. A malicious or accidental script (e.g., `[0]*10**10`) can easily crash the worker by consuming all available RAM.
**Recommendation**: Use `ulimit` (via `preexec_fn`) or run scripts inside more restricted sub-containers.

### 10. Orchestration Smell: Mixed Import Patterns in Frontend

**File**: `frontend/src/context/EpisodeContext.tsx`
**Issue**: Some API services are imported statically at the top, while others use dynamic `import().then()` inside function bodies.
**Evidence**:
- Line 2: `import { fetchEpisodes, ... } from '../api/client';`
- Line 174: `await import('../api/client').then(m => m.confirmBenchmark(id, comment));`
**Impact**: Low. Inconsistent code style and slightly unpredictable bundle behavior. It suggests a "quick fix" for a circular dependency or build issue that wasn't fully resolved.
**Recommendation**: Standardize on static imports or move the core API logic to a dedicated service layer that handles the async resolution.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
