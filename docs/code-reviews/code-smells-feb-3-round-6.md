# Problemologist-AI Codebase Architecture Review #6

This document identifies **NEW** architectural smells and questionable decisions discovered during the sixth deep-dive review of the codebase.

---

## 🔴 Critical Architectural Issues

### 1. Fragile Code-in-String Injection (Evaluator)

**Problem:** `Evaluator.preview_design` and `Evaluator.validate_and_export` construct complex Python runner scripts by concatenating large strings. This mirrors the issue found in `MujocoBridge` (Round 5) but is pervasive in the `Evaluator` as well.

**Evidence:**

- [evaluator.py:L76-118](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/evaluator.py#L76-118)
- [evaluator.py:L160-233](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/evaluator.py#L160-233)

**The Smell:**

- **Unmaintainable Embedded Code**: Core validation logic exists as a string literal. It gets no linting, type checking, or syntax highlighting in the IDE.
- **Injection Risks**: While run in a sandbox, the construction `exec(code, globals(), locs)` inside the generated script involves multiple layers of dynamic execution that are hard to debug.
- **Duplication**: The `_get_runner_script_prefix` method duplicates import logic and helper functions as strings.

**Planned Resolution:**

- **Externalize Runner Scripts**: Move the runner logic to a proper Python module (e.g., `src/environment/runners/validation.py`). Read this file at runtime to inject into the sandbox.
- **Structured Inputs**: Pass arguments to the runner via JSON file or environment variables, not by f-string interpolation into the script code.

**User Review:**

<!-- User comments here -->

---

### 2. High-Risk Geometry Hashing

**Problem:** `CNCWorkbench.calculate_cost` uses a naive string concatenation of center and volume to hash parts for caching checks.

**Evidence:**

- [cnc.py:L87-89](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/workbenches/cnc.py#L87-89)

```python
part_hash = hashlib.md5(
    str(part.center()).encode() + str(part.volume).encode()
).hexdigest()
```

**The Smell:**

- **Collision Guarantee**: Distinct shapes with the same volume and center (e.g., a sphere vs. a cube at origin) will produce the exact same hash.
- **Invalid Caching**: This will lead to false positives in the `context` cache, potentially applying "reuse discounts" to completely different parts.
- **Float Instability**: `str(part.center())` relies on float formatting. Micro-deviations in float precision could cause cache misses for identical parts.

**Planned Resolution:**

- **Topology Hashing**: Use a more robust geometric signature. If `build123d` supports it, use topological hashing. Otherwise, hash a combination of vertex counts, face counts, and bounding box dimensions + volume.
- **Mesh Hashing**: Convert to trimesh and hash the vertices/faces array (if feasible for performance).

**User Review:**

Yep, so, we can use `part.label` to start with. then concat with edge count, face count, AABB and volume - it's a good idea.
By the way. I've dealt with a similar issue in the past - the parts wouldn't match if there is a numeric instability. The part with volume=4.0000001 would be different from volume 3.9999999 - and it actually happened. let's round to 4-5 digits to ensure it doesn't happen.
not just to a 001 or 0001, but for digits - to handle different-volume parts.

---

## 🟠 Moderate Design Issues

### 3. Brittle Graph Routing Logic

**Problem:** The `graph.py` module determines control flow by parsing tool names (`preview_design`, `submit_design`) from message history strings.

**Evidence:**

- [graph.py:L144](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/graph.py#L144)

```python
if "preview_design" in tool_names or validation_tool_name in tool_names:
    return "critic"
```

**The Smell:**

- **Implicit Coupling**: The graph structure depends on the exact string names of tools. If a tool is renamed (e.g., to `visualize_design`), the agent will silently stop routing to the critic.
- **State Leaking**: The routing logic has to "peek" into the message history to reconstruct what just happened.

**Planned Resolution:**

- **Explicit State Signals**: Have the `tools_node` return a structured signal in `AgentState` (e.g., `state["next_node"] = "critic"`) specific to the tool executed.
- **Tool Metadata**: Decorate tools with `triggers_review=True` metadata, and check this metadata instead of name strings.

**User Review:**

We should have a CI test for it; and also fail hard if the string doesn't match any tools.
Perhaps we could even use an enum. Actually, we probably should use an enum. Would be better for the strictness and for the DB
---

## 🟡 Minor Issues & Smells

### 4. Magic Numbers in Workbench Logic

**Problem:** `CNCWorkbench` contains hardcoded constants like `mrr = 1000.0` that override or ignore configuration.

**Evidence:**

- [cnc.py:L63](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/workbenches/cnc.py#L63)

**The Smell:**

- **Configuration Bypass**: Even though `self.config` is loaded, key physics parameters are hardcoded in the method body.

**Planned Resolution:**

- Move `mrr` and other constants to `config/workbenches.yaml` and load them.

**User Review:**

Confirm.

### 5. Fragile Test Assertions

**Problem:** Integration tests rely on string inclusion checks against JSON output, which is brittle.

**Evidence:**

- [test_tool_integration.py:L34-36](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/test_tool_integration.py#L34-36)

```python
assert '"status": "fail"' in _obs["last_output"]
```

**The Smell:**

- **Assertion Roulette**: If this fails, it's hard to know why.
- **Formatting Dependency**: A change in JSON spacing or ordering could break the test.

**Planned Resolution:**

- Parse the JSON output in the test and assert against structured data.

**User Review:**

This is new for me, but we have structlog which we can test against? I've heard this is a thing. But to make it less brittle - well yes, add the data.
Why bother with json at all if we could use pydantic models or similar? we have @file:models.py.
