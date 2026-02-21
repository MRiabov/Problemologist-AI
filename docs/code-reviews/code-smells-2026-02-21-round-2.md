# Codebase Review - Feb 21, 2026 - Round 2

## Overview

This second round of review for today focuses on critical lifecycle bugs in heavy operations, inconsistencies in physical modeling, and architectural debt in agent orchestration and persistence.

---

## 🚨 Critical Issues

### 1. Artifact Persistence Race Condition in Heavy Worker

**File**: `worker_heavy/api/routes.py`
**Issue**: When `bundle_base64` is used, the heavy worker extracts it into a `TemporaryDirectory`, generates artifacts (previews, renders, MJCFs) into that directory, and then immediately deletes the directory when the request finishes.
**Evidence**:
```python
# worker_heavy/api/routes.py
with bundle_context(request.bundle_base64, fs_router.local_backend.root) as root:
    # ... generates artifacts into root / "renders" ...
    return BenchmarkToolResponse(
        # returns paths relative to root
        artifacts={"render_paths": result.render_paths, ...}
    )
# Block ends here, tmpdir is deleted!
```
**Impact**: High. Any artifact path returned to the controller points to a file that no longer exists by the time the controller (or frontend) attempts to fetch it. This breaks simulation previews, stress heatmaps, and asset rebuilding for any session that isn't already sharing a physical disk with the worker.
**Recommendation**: Artifacts generated in a temporary context must be moved to a persistent location or returned as base64/binary in the response before the cleanup happens.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Dead Node in LangGraph Agent Orchestration

**File**: `controller/agent/graph.py`
**Issue**: The `cots_search` node is added to the StateGraph but is unreachable.
**Evidence**:
```python
builder.add_node("cots_search", cots_search_node)
# ...
builder.add_edge("cots_search", "planner")  # Exit path exists
# BUT: No START -> cots_search or node -> cots_search edge exists.
```
**Impact**: Medium. The agent cannot autonomously decide to enter a dedicated "COTS search" mode, even though the node and logic exist. It likely relies on the COTS search tool being called from within other nodes, making the dedicated graph node redundant and confusing.
**Recommendation**: Either add a conditional edge from `planner` or `coder` to `cots_search` when parts are missing, or remove the dead node from the graph definition.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Inconsistent AWG Resistance Logic

**Files**: `shared/wire_utils.py`, `shared/circuit_builder.py`, `worker_heavy/utils/validation.py`
**Issue**: There are at least three different ways the system calculates or looks up wire resistance and weight.
**Evidence**:
- `wire_utils.py`: Uses `AWG_PROPERTIES` table + heuristic fallback.
- `circuit_builder.py`: Uses approximation formula `0.01 * (1.26 ** (wire.gauge_awg - 15))`.
- `validation.py`: Hardcodes copper density `8.96` and cost `0.5` if config is missing.
**Impact**: Medium. Discrepancies between the CAD price estimation and the SPICE simulation results. A wire might pass validation but fail cost constraints due to different rounding or formulas.
**Recommendation**: Centralize all AWG-related math into `shared/wire_utils.py` and ensure `circuit_builder.py` and `validation.py` use the same source of truth.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 4. Fragile Global Engine Caching in Persistence Layer

**File**: `controller/persistence/db.py`
**Issue**: The SQLAlchemy engine is cached in a global dictionary keyed by the current `asyncio` event loop.
**Evidence**:
```python
_engine_cache = {}
def get_engine():
    loop = asyncio.get_running_loop()
    if loop in _engine_cache:
        return _engine_cache[loop]
    # ...
```
**Impact**: Medium. While solving "Future attached to a different loop" errors, this is a non-standard pattern that can lead to memory leaks (loop objects never cleared from dict) and hidden side effects in testing environments where loops are created frequently.
**Recommendation**: Use a more standard singleton pattern or a lifecycle-managed dependency injection if possible. At minimum, use a `WeakKeyDictionary` for the loop cache.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. Violation of "Enum Mandate" in `GenerationSession`

**File**: `controller/persistence/models.py`
**Issue**: The `GenerationSession` model uses a raw `sa.Enum` with string literals instead of the `SessionStatus` enum defined in `shared/enums.py`.
**Evidence**:
```python
# models.py
status: Mapped[str | None] = mapped_column(
    sa.Enum("planning", "planned", ..., name="sessionstatus")
)
```
**Impact**: Low/Medium. Bypasses the type safety and centralized naming intended by `shared/enums.py`. Changes to `SessionStatus` won't be reflected in the database schema without manual string updates.
**Recommendation**: Update the model to use `SQLEnum(SessionStatus)`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Lying in Observability Events (Placeholder Logic)

**File**: `shared/wire_utils.py`
**Issue**: `route_wire` emits a `WireRoutingEvent` with `clearance_passed=True` hardcoded as a "placeholder".
**Evidence**:
```python
emit_event(
    WireRoutingEvent(
        ...,
        clearance_passed=True,  # Placeholder until check_wire_clearance is integrated
    )
)
```
**Impact**: Medium. Observability traces show successful clearance checks even if the wire is intersecting geometry. This makes debugging "wire torn" or "short circuit" failures harder as the trace history is fundamentally dishonest.
**Recommendation**: Use `None` (as noted in memory) or a dedicated `PENDING` status instead of `True` for checks that haven't run.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 7. Inefficient Monkeypatching in `pyspice_utils.py`

**File**: `shared/pyspice_utils.py`
**Issue**: The monkeypatch for `NgSpiceShared._send_char` performs local imports of `ffi` and `ffi_string_utf8` *inside* the static method, which is called for every single character of SPICE output.
**Impact**: Low/Medium. Significant overhead during long simulations with lots of output.
**Recommendation**: Move the imports to the top level (guarded by `with contextlib.suppress(ImportError)`) or capture them during the initial patch.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. Fragile Client-side Topology Parsing

**File**: `frontend/src/components/visualization/ModelViewer.tsx`
**Issue**: The frontend guesses the hierarchy (faces vs parts vs assemblies) by parsing mesh names (`face_`, `edge_`, etc.).
**Impact**: Low. If the CAD exporter changes its naming convention, the Model Browser and selection logic will break. This logic belongs in the backend during asset generation.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Incomplete Open Circuit Validation

**File**: `shared/pyspice_utils.py`
**Issue**: `validate_circuit` proactive check only identifies required terminals for `motor`, `switch`, and `relay`.
**Impact**: Low. `connector` components and other generic types are not checked for floating pins, which can lead to silent simulation failures (singular matrix) that are harder to diagnose than an explicit "Open Circuit" error.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Broken Schematic Generation for Multipolar Components

**File**: `controller/api/routes/episodes.py`
**Issue**: `get_episode_schematic` hardcodes `p1` and `p2` as the only pins for every component.
**Evidence**:
```python
source = f"comp_{wire.from_terminal.component}_p1" # Always p1
target = f"comp_{wire.to_terminal.component}_p2"   # Always p2
```
**Impact**: Low (currently). The tscircuit visualization will show incorrect wiring for any component that isn't a 2-terminal resistor-like object. It ignores the actual terminal name provided in the wiring schema.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
