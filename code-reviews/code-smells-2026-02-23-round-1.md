# Codebase Review - Feb 23, 2026 - Round 1

## Overview

This review focuses on architectural inefficiencies in the agentic workflow, brittle coordination between specialized nodes, and technical debt in the electronics and simulation subsystems.

---

## 🏗️ Architectural & Workflow Issues

### 1. Inefficient Serial Planning Workflow

**Files**: `controller/agent/graph.py`
**Issue**: The graph currently enforces a strict `planner` -> `electronics_planner` -> `plan_reviewer` sequence.
**Impact**: Medium. The `electronics_planner` is invoked even for tasks that have zero electronics requirements (e.g., "Build a simple wooden box"). This wastes LLM tokens and increases latency.
**Recommendation**: Implement conditional routing after the initial `planner` node to skip `electronics_planner` if no electrical components are identified in the initial mechanical plan.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 2. Destructive Plan Updates in specialized Planners

**File**: `controller/agent/nodes/electronics_planner.py`
**Issue**: The `ElectronicsPlannerNode` replaces `plan.md` and `todo.md` entirely with its output.
**Impact**: Medium/High. If the specialized planner is not perfectly context-aware, it may accidentally delete or overwrite critical mechanical implementation details from the primary planner.
**Recommendation**: Transition to an "append" or "merge" strategy where specialized planners only add tasks to the existing `plan.md` and `todo.md`, or use structured sections that are individually managed.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 3. Brittle Node Coordination via String Parsing

**File**: `controller/agent/nodes/coder.py`
**Issue**: The `CoderNode` identifies "electronics tasks" to skip by searching for keywords like `["circuit", "wire", "electronics", "routing", "psu", "power"]`.
**Impact**: Medium. This is highly brittle. A task like "Implement power-saving structural support" might be skipped by the coder but ignored by the electronics engineer, leading to implementation gaps.
**Recommendation**: Use structured task metadata (e.g., a `type` field in `todo.md` or a structured state) rather than fuzzy keyword matching to coordinate work between specialized agent nodes.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 4. Overlapping Session and Episode Models

**File**: `controller/persistence/models.py`
**Issue**: `GenerationSession` and `Episode` models both track the lifecycle of a generation task with redundant state and metadata.
**Impact**: Low/Medium. Increases maintenance burden and risk of state desynchronization.
**Recommendation**: Consolidate into a single `Session` model or clearly separate concerns (e.g., `Session` for user-facing status, `Episode` for internal execution traces).

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 5. Disruptive Summarizer Routing

**File**: `controller/agent/graph.py`
**Issue**: The `summarizer` node always routes back to the `planner` node after completion.
**Impact**: Medium. If an agent is in the middle of a complex coding iteration (Iteration 4/5) and hits the journal limit, it is forcibly reset to the planning phase. This loses the immediate momentum of the implementation.
**Recommendation**: Update `summarizer` to return to the node that triggered it (the "caller"), allowing the agent to continue its current phase with a compressed context.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 6. Failure to Short-Circuit on Reviewer Rejection

**File**: `controller/agent/graph.py`
**Issue**: The conditional edge for `electronics_reviewer` only checks for `steer` or `next`.
**Impact**: Medium. If the `electronics_reviewer` rejects a design, the graph still proceeds to the `execution_reviewer` instead of immediately returning to the `coder` or `electronics_engineer`. This results in redundant evaluation of a known-bad design.
**Recommendation**: Add a conditional routing check after *every* reviewer node to handle `REJECTED` statuses immediately.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## ⚡ Electronics & Simulation Smells

### 7. Inaccurate Connectivity Fallback in `ElectronicsManager`

**File**: `worker_heavy/simulation/electronics.py`
**Issue**: The `_fallback_update` method (used when SPICE fails) implements a BFS that ignores `switch_states`.
**Impact**: Medium. If SPICE fails (e.g., due to a singular matrix), the fallback will report all components as "powered" even if a critical safety switch is open.
**Recommendation**: Update the BFS to only traverse edges through switches/relays if their state in `switch_states` is `True`.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 8. God Object Anti-pattern in `SimulationLoop`

**File**: `worker_heavy/simulation/loop.py`
**Issue**: The `SimulationLoop` has expanded to handle DFM pricing, video recording, electronics logic, and objective evaluation.
**Impact**: Medium. Extremely high coupling. Changing the pricing logic shouldn't require touching the physics loop.
**Recommendation**: Remove `validate_and_price` from the `SimulationLoop` constructor. Pricing and manufacturability should be handled by the agent via dedicated tools *before* or *after* simulation, not as a side-effect of physics initialization.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 9. Hardcoded Node Expectations in Electronics Logic

**File**: `worker_heavy/simulation/electronics.py`
**Issue**: Logic for determining if a component is powered depends on hardcoded terminal suffixes like `_+` or `_in`.
**Impact**: Medium. Prevents the system from supporting components with non-standard terminal names (e.g., `VCC`, `Line`, `Phase`).
**Recommendation**: Use the `ElectronicComponent` model to explicitly define which terminal is the "power input" for simulation purposes.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 10. Genesis Backend Functional Gaps

**File**: `worker_heavy/simulation/genesis_backend.py`
**Issue**: `get_site_state` and `set_site_pos` are currently no-ops (returning zeros or doing nothing).
**Impact**: Low/Medium. Breaks objective evaluation for benchmarks that rely on dynamic "sites" (e.g., following a moving target or checking distance to a changing goal).
**Recommendation**: Implement site state tracking in Genesis by mapping sites to rigid bodies or ghost entities.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

---

## 🛠️ Implementation & Performance

### 11. Inefficient Async Tool Execution

**File**: `controller/agent/nodes/base.py`
**Issue**: `_get_tool_functions` creates a `new_event_loop` for *every* call to an async tool within a DSPy ReAct loop.
**Impact**: Medium. Significant overhead for tasks with many tool calls. It also risks resource leaks if loops are not closed properly in all error paths (though there is a `finally` block).
**Recommendation**: Use `asyncio.run()` (which is optimized) or, better, use `nest_asyncio` to allow the sync-wrapped tools to share the existing thread's loop if it exists.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*

### 12. Arbitrary Fluid Damage Thresholds

**File**: `worker_heavy/simulation/genesis_backend.py`
**Issue**: Electronics fluid damage is detected using a hardcoded `0.05` (5cm) distance threshold from the entity center.
**Impact**: Low. For very large or very small components, this threshold is either too conservative or too loose.
**Recommendation**: Use the entity's bounding box or collision geometry to detect fluid contact more precisely.

> **User Review**:
> [ ] Agree - [ ] Disagree
> *Comments:*
