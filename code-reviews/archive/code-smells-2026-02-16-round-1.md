# Problemologist-AI Codebase Architecture Review (Feb 16 - Round 1)

This document identifies **NEW** architectural smells and questionable decisions discovered during a deep-dive review of the codebase on **February 16, 2026**.

---

## 🔴 Critical Architectural Issues

### 1. Mixed Concerns in `DatabaseCallbackHandler`

**Problem:** The `DatabaseCallbackHandler` is responsible for both persisting traces to the database AND broadcasting them via a websocket/broadcaster.

**Evidence:**

- [database.py:L31-48](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/observability/database.py#L31-48) (`_broadcast_trace`)
- [database.py:L90, L136, L181, L215](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/observability/database.py) (Calls to `_broadcast_trace` after every DB commit)

**The Smell:**

- **Violation of SRP**: A callback handler for observability should either store data or notify other systems, but doing both in one class makes it harder to swap or disable one without affecting the other.
- **Tightly Coupled UI Logic**: The "new_trace" event format is hardcoded to what the UI expects, leaking frontend concerns into the controller's persistence layer.

**User Review:**

---

### 2. Manual Retry and Validation Loops in `PlannerNode`

**Problem:** Similar to `CoderNode` (identified yesterday), `PlannerNode` implements its own manual retry loop and validation logic instead of using LangGraph patterns or standardized node abstractions.

**Evidence:**

- [planner.py:L115-186](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/nodes/planner.py#L115-186)

**The Smell:**

- **Logic Duplication**: The pattern of "Invoke -> Validate -> Retry with Feedback" is being manually implemented in every node.
- **Brittle Retries**: Manual `retry_count` logic is harder to manage than declarative graph edges (e.g., a "validate" node that routes back to "planner" on failure).

**User Review:**

---

## 🟠 Moderate Design Issues

### 3. Redundant Pydantic Coercion Logic

**Problem:** Over 10 repetitive `field_validator` blocks exist in `schemas.py` solely to coerce lists to tuples for 3D coordinates.

**Evidence:**

- [schemas.py:L27-33](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/models/schemas.py#L27-33)
- [schemas.py:L56-61](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/models/schemas.py#L56-61)
- [schemas.py:L70-75](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/models/schemas.py#L70-75)
- (and many more...)

**The Smell:**

- **Code Bloat**: `schemas.py` is 550 lines long, with a significant portion being redundant validation boilerplate.
- **Maintenance**: Adding a new coordinate field requires copy-pasting the validator. Should use a custom `Coordinate` type or a shared base model.

**User Review:**

---

### 4. "Magical" Input Cleaning with `ast.literal_eval`

**Problem:** `DatabaseCallbackHandler` tries to "fix" tool inputs by parsing them as Python literals if they aren't valid JSON.

**Evidence:**

- [database.py:L106-112](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/observability/database.py#L106-112)

**The Smell:**

- **Brittleness**: Relying on `ast.literal_eval` to fix malformed inputs is a band-aid for upstream issues (why is the agent sending malformed JSON?).
- **Security/Stability**: While `literal_eval` is safer than `eval`, it still adds unnecessary complexity to the tracing layer.

**User Review:**

---

## 🟡 Minor Issues & Smells

### 5. Redundant Imports in Hot Paths

**Problem:** `DatabaseCallbackHandler` and `PlannerNode` perform imports inside methods (hot paths).

**Evidence:**

- [database.py:L33, L53, L71, L114, L140, L165, L183, L199, L251](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/observability/database.py)
- [planner.py:L128](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/nodes/planner.py#L128)

**The Smell:**

- **Noise**: It litters the code with `import structlog` and `import datetime` calls, making it harder to read.
- **Micro-optimization**: Unless these imports cause circular dependencies (which should be solved by refactoring), they should be at the top of the file.

**User Review:**

---

### 6. Inconsistent Configuration Defaults

**Problem:** `PlannerNode` uses a hardcoded default for `worker_url` that differs from what the factory function uses.

**Evidence:**

- [planner.py:L34](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/nodes/planner.py#L34): `worker_url: str = "http://worker:8001"`
- [planner.py:L201](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/nodes/planner.py#L201): `node = PlannerNode(worker_url=settings.spec_001_api_url, ...)`

**The Smell:**

- **Confusion**: Developers looking at the class definition might assume the default is `worker:8001`, but the app actually uses `settings.spec_001_api_url`. Both should point to the same source of truth.

**User Review:**

---

## Summary

| Issue | Severity | Effort |
|-------|----------|--------|
| 1. Mixed Concerns in Tracing | 🔴 Critical | Medium |
| 2. Manual Retry Loops (Planner) | 🔴 Critical | Medium |
| 3. Redundant Pydantic Coercion | 🟠 Moderate | Low |
| 4. Magical Input Cleaning | 🟠 Moderate | Low |
| 5. Redundant Imports | 🟡 Minor | Trivial |
| 6. Inconsistent Config | 🟡 Minor | Trivial |

---

*Review conducted on 2026-02-16 by Gemini (Antigravity)*
