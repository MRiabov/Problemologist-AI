# Code Smell Review - 2026-02-16 - Round 2

## Summary of Findings

This round of review focused on the interaction between the `worker`, `controller`, and `shared` modules. While the transition to Pydantic models is well underway (as per memories), there are still significant pockets of technical debt, inconsistent typing, and architectural violations where concerns are mixed (e.g., agent nodes handling git sync).

---

## 1. Typing & Data Integrity

### [Smell] Inconsistent usage of TypedDict/Dataclass vs Pydantic

**File**: [shared/backend/protocol.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/backend/protocol.py)
**Description**: `FileInfo` and `GrepMatch` are `TypedDict`, and several result classes (e.g., `WriteResult`, `EditResult`) are `dataclass`.
**Impact**: Violates the core project rule: *"ALWAYS prefer strict, typed pydantic.BaseModel classes over freeform dict objects"*. It also reduces the effectiveness of observability tools that expect Pydantic's `model_dump()`.
**Suggested Fix**: Convert all `TypedDict` and `dataclass` in the protocol layer to `BaseModel`.

**User Review**:
<!-- Fill here -->

---

### [Smell] Coercion overkill in schemas

**File**: [shared/models/schemas.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/models/schemas.py)
**Description**: The `coerce_list_to_tuple` validator is repeated ~10 times across different models.
**Impact**: Code duplication and maintenance burden. Any change to how we handle coordinates requires updating many locations.
**Suggested Fix**: Use a custom `Annotated` type or a shared base class with the validator.

**User Review**:
<!-- Fill here -->

---

### [Smell] Loose `Any` in Assembly Data

**File**: [worker/simulation/builder.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/simulation/builder.py#L31)
**Description**: `AssemblyPartData.part` is typed as `Any`.
**Impact**: Loss of type safety in the simulation builder logic, where many operations depend on the part being a `build123d.Solid` or `Compound`.
**Suggested Fix**: Use a proper Union type or a Protocol.

**User Review**:
<!-- Fill here -->

---

## 2. Architecture & Design

### [Smell] Fat Traverser and Mixed Concerns

**File**: [worker/simulation/builder.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/simulation/builder.py#L45)
**Description**: `CommonAssemblyTraverser.traverse` handles geometry extraction, metadata resolution, joint parsing, zone detection, and electronics mapping.
**Impact**: Violation of the Single Responsibility Principle. This method is the "God Object" of assembly processing.
**Suggested Fix**: Break down the traversal into a pipeline of specialized processors (e.g., `GeometryExtractor`, `MetadataResolver`, `JointParser`).

**User Review**:
<!-- Fill here -->

---

### [Smell] Mixing Agent Logic with Infrastructure (Git/DB)

**File**: [controller/agent/nodes/skills.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/nodes/skills.py)
**Description**: The `SkillsNode` directly handles git cloning, repo configuration, conflict resolution, and pushing.
**Impact**: Makes the agent node very heavy and hard to test. Infrastructure concerns like "keeping a directory in sync with Git" should be abstracted into a service or a worker middleware.
**Suggested Fix**: Extract Git management into a `SkillStore` or `GitManager` utility.

**User Review**:
<!-- Fill here -->

---

### [Smell] Magical Heuristics in Builders

**File**: [worker/simulation/builder.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/simulation/builder.py#L501)
**Description**: `add_actuator` contains hardcoded heuristics to guess KP/KV and torque from COTS names.
**Impact**: Very fragile. If a user names a part "Servo_SG90_new", it might trigger different physics than "Servo_SG90".
**Suggested Fix**: Centralize the mapping of COTS components to simulation parameters in `shared/cots`.

**User Review**:
We should have a cots ID in cots parts. It should resolve from cots ID, not from the name of it. We can set it in part metadata.

---

## 3. Observability & Hygiene

### [Smell] Inlined Dictionaries for Broadcasting

**File**: [controller/observability/database.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/observability/database.py#L38)
**Description**: `_broadcast_trace` constructs a raw `dict` for broadcasting.
**Impact**: Violates Pydantic-first rule and makes it hard to track changes to the broadcast schema.
**Suggested Fix**: Create a `TraceBroadcast` Pydantic model.

**User Review**:
<!-- Fill here -->

---

### [Smell] Inlined Imports Chaos

**File**: [controller/observability/database.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/observability/database.py)
**Description**: `import structlog` and `import datetime` are repeated inside almost every method.
**Impact**: Code smells of circular dependency hacks. It's distracting and non-standard.
**Suggested Fix**: Move imports to the top level. If circular dependencies exist, refactor the module structure.

**User Review**:
<!-- Fill here -->

---

### [Smell] Legacy Duplication

**File**: [worker/filesystem/backend.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/filesystem/backend.py)
**Description**: The `ls` method is defined twice (Line 132 and Line 367) with slightly different logic or just as a duplicate override.
**Impact**: Confusion about which one is active.
**Suggested Fix**: Consolidate into a single legacy wrapper or remove if `ls_info` is the standard.

**User Review**:
<!-- Fill here -->

---

### [Smell] Hardcoded Filenames in Workers

**File**: [controller/agent/benchmark/nodes.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/nodes.py)
**Description**: "script.py", "objectives.yaml", and "review.md" are hardcoded throughout the nodes.
**Impact**: Prevents running multiple scripts or custom review filenames without refactoring.
**Suggested Fix**: Pass filenames via `SharedNodeContext` or an `AgentConfig`.

**User Review**:
<!-- Fill here -->
