# Codebase Review - Feb 22, 2026 - Round 1

## Overview

This review identifies significant architectural inefficiencies, logic bugs in the simulation loop, and project-level maintenance issues. Key focus areas include performance bottlenecks in electronics simulation and the agentic workflow.

---

## 🚨 Critical Issues

### 1. Bug: Persistent Electronics Dirty Flag causing Extreme Inefficiency

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The `_electronics_dirty` flag is set to `True` when a wire breakage is detected, but it is never reset to `False` after the circuit is re-evaluated.
**Evidence**:
```python
# worker_heavy/simulation/loop.py:539 (inside step loop)
if self.electronics and self._electronics_dirty:
    self._update_electronics() # Re-evaluates SPICE
    # ...
```
The `_update_electronics` method does not clear the flag.
**Impact**: High. Once a wire breaks, the system performs a full SPICE circuit validation on *every* physics step (500Hz). For a 30s simulation, this results in 15,000 redundant SPICE simulations, causing massive performance degradation.
**Recommendation**: Reset `self._electronics_dirty = False` at the end of `_update_electronics`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Broken Entry Point: Outdated `main.py`

**File**: `main.py`
**Issue**: The root `main.py` references a non-existent `src.simulation_engine.main` package.
**Evidence**:
```python
uvicorn.run("src.simulation_engine.main:app", ...)
```
The project structure uses `controller/`, `worker_heavy/`, etc., and no `src/` directory exists in the root.
**Impact**: High. New developers or automated systems following standard entry points will encounter immediate failures.
**Recommendation**: Update `main.py` to point to the correct entry points (e.g., `controller.api.app:app`) or remove it if service-specific entry points are preferred.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🏗️ Architectural Observations

### 3. Inefficient Orchestration: Heavy Simulation per Task

**File**: `controller/agent/graph.py`, `controller/agent/nodes/coder.py`
**Issue**: The LangGraph workflow triggers a full `electronics_engineer` -> `execution_reviewer` cycle (including heavy physics simulation) for every single atomic task in `todo.md`.
**Impact**: Medium/High. Agents typically have 5-10 tasks in a plan. Running 10 physics simulations (30s each) for a single episode is extremely slow and expensive.
**Recommendation**: Batch implementation tasks or allow the `coder` to complete all mechanical tasks before a single "Integration & Simulation" phase.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Unsafe "Async-over-Sync-over-Async" Pattern

**File**: `controller/agent/nodes/base.py`, `controller/agent/dspy_utils.py`
**Issue**: `BaseNode` runs DSPy programs in a thread (`asyncio.to_thread`), which then uses `loop.run_until_complete` inside `WorkerInterpreter` to call async worker clients.
**Impact**: Medium. This pattern is prone to deadlocks and thread-safety issues, especially when sharing `httpx.AsyncClient` across different loops (even if cached by loop ID, the lifecycle is complex).
**Recommendation**: Transition to a fully async DSPy integration if possible, or use a dedicated background thread with a stable event loop for all worker interactions.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. Excessive Reviewer Permissions

**Files**: `controller/agent/nodes/plan_reviewer.py`, `controller/agent/nodes/execution_reviewer.py`
**Issue**: Reviewer nodes are initialized with `get_engineer_tools`, giving them full write access and execution power.
**Impact**: Medium. Reviewers should be independent auditors. Giving them the ability to modify the code they are reviewing risks "shadow implementation" by the reviewer and degrades the quality of feedback.
**Recommendation**: Create a `get_reviewer_tools` factory that only provides read-only filesystem access and reporting tools.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🧹 Implementation Smells

### 6. "God Method" in `SimulationLoop.step`

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The `step` method is ~300 lines long and handles physics, SPICE, video rendering, objective evaluation, and failure detection.
**Impact**: Medium. Hard to test and maintain.
**Recommendation**: Extract concerns into sub-managers (e.g., `ObjectiveEvaluator`, `MediaRecorder`).

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 7. Fragile Clearance Logic for Short Wires

**File**: `shared/wire_utils.py`
**Issue**: `check_wire_clearance` ignores collisions within 5mm of endpoints.
**Impact**: Low/Medium. For short wires (< 10mm), clearance checking is effectively disabled, allowing wires to pass through solid obstacles without penalty.
**Recommendation**: Make `ignore_endpoints_dist` proportional to wire length or use a more precise "terminal exclusion zone" based on component geometry.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. Expensive Constructor Side-Effects in `SkillsNode`

**File**: `controller/agent/nodes/skills.py`
**Issue**: `SkillsNode.__init__` performs git repository synchronization.
**Impact**: Low/Medium. Since node classes are often instantiated per-turn or per-graph-execution, this adds significant latency to the agent loop.
**Recommendation**: Move git synchronization to an explicit startup task or a background worker.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Dead Node: `cots_search`

**File**: `controller/agent/graph.py`
**Issue**: Node exists and has an exit, but no entry edge from `START`, `planner`, or `coder`.
**Impact**: Low. Dead code in the orchestration layer.
**Recommendation**: Clean up or properly integrate.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Inconsistent Skill Directories

**File**: `controller/agent/nodes/base.py` vs `controller/agent/nodes/skills.py`
**Issue**: `base.py` reads skills from `.agent/skills`, but `SkillsNode` saves them to `suggested_skills/`.
**Impact**: Medium. New skills learned by the agent are never actually read by the planner or coder in subsequent turns.
**Recommendation**: Unify the directory path for skills storage and retrieval.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 11. Competing Skill Sync Logic

**File**: `worker_light/app.py` vs `controller/agent/nodes/skills.py`
**Issue**: Both services attempt to pull/push the same git repository for skills.
**Impact**: Low. Potential for race conditions or dirty working trees if both services share a volume.
**Recommendation**: Designate one service as the "Skill Master" for sync.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 12. Duplicated Node Resolution Logic

**Files**: `shared/circuit_builder.py`, `shared/pyspice_utils.py`
**Issue**: Logic for normalizing terminal names (e.g., `a` -> `+`) is duplicated.
**Impact**: Low. Maintenance burden.
**Recommendation**: Centralize in a helper function.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 13. Non-Polymorphic COTS Pricing

**File**: `worker_heavy/utils/validation.py`
**Issue**: `calculate_assembly_totals` uses a large `if-elif` chain to handle different component types.
**Impact**: Low. Harder to add new COTS types.
**Recommendation**: Use a registry of COTS handlers.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
