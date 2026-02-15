# Problemologist-AI Codebase Architecture Review #7

This document identifies **NEW** architectural smells and questionable decisions discovered during the seventh deep-dive review of the codebase.

---

## 🔴 Critical Architectural Issues

### 1. Dead Import: `set_current_role` Function Does Not Exist

**Problem:** Two graph nodes import `set_current_role` from `env_adapter`, but this function is **never defined** anywhere in the codebase. This will cause `ImportError` at runtime when these nodes are used.

**Evidence:**

- [planner.py:L4](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/nodes/planner.py#L4)
- [skill_populator.py:L5](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/nodes/skill_populator.py#L5)
- [skill_populator.py:L15](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/nodes/skill_populator.py#L15)

```python
from src.agent.tools.env_adapter import set_current_role
# ...
set_current_role("SkillPopulator")  # Called but function doesn't exist
```

**The Smell:**

- **Dead Code**: The function was likely removed during a refactor but the imports and usages were not cleaned up.
- **Runtime Failure**: This will crash when `skill_populator_node` is reached.
- **CI Gap**: Tests apparently don't cover these exact code paths, or they're mocked.

**Planned Resolution:**

- **Option A**: Remove the dead import and the call in `skill_populator.py` if the functionality is not needed.
- **Option B**: Implement `set_current_role` in `env_adapter.py` if role tracking is actually required for logging/observability.

**User Review:**

---

### 2. InjectionMoldingWorkbench Uses Unsafe Part Hash (Same Issue as CNC)

**Problem:** `InjectionMoldingWorkbench.calculate_cost` still uses the naive hashing pattern that was identified and fixed for `CNCWorkbench` in Round 6.

**Evidence:**

- [injection_molding.py:L107-109](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/workbenches/injection_molding.py#L107-109)

```python
part_hash = hashlib.md5(
    str(part.center()).encode() + str(part.volume).encode()
).hexdigest()
```

**The Smell:**

- **Inconsistency**: `CNCWorkbench` was fixed to use topology + rounded geometry hashing, but `InjectionMoldingWorkbench` was not updated.
- **Same Collision Risk**: Sphere vs cube at origin with same volume will produce identical hash.
- **Float Instability**: `str(part.center())` relies on unpredictable float formatting.

**Planned Resolution:**

- Apply the same robust hashing strategy used in `CNCWorkbench` (label + edge count + face count + AABB + rounded volume/area) to `InjectionMoldingWorkbench`.
- Consider extracting the hash logic into `analysis_utils.py` as a reusable `compute_part_hash(part)` function.

**User Review:**

---

## 🟠 Moderate Design Issues

### 3. Actor Node Has Duplicate/Conflicting `trim_messages` Calls

**Problem:** The `actor_node` function calls `trim_messages` twice with different parameters. The first call's result is immediately overwritten.

**Evidence:**

- [actor.py:L54-65](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/nodes/actor.py#L54-65)

```python
# First call (result is discarded!)
trimmed_messages = trim_messages(
    messages,
    strategy="last",
    token_limit=15,  # This is a message count, not token count?
    include_system=True,
    allow_partial=False,
)
# Note: If token_limit is small, it acts as message count...
# Second call (overwrites the first)
trimmed_messages = trim_messages(
    messages,
    strategy="last",
    token_limit=4000,
    include_system=True,
)
```

**The Smell:**

- **Dead Code**: The first `trim_messages` call is useless and its result is overwritten.
- **API Confusion**: The inline comment suggests confusion about whether `token_limit=15` is a message count or token count. Indeed, `trim_messages` uses `token_counter` for tokens, and the `max_tokens` arg for token limit. The parameter `token_limit` is likely wrong.
- **Wasted Computation**: Two calls to `trim_messages` when only one is needed.

**Planned Resolution:**

- Remove the first (useless) `trim_messages` call.
- Verify the correct API usage for `langchain_core.messages.trim_messages` and use appropriate parameters.

**User Review:**

---

### 4. Global Dashboard Data Layer Instance Creates Hidden State

**Problem:** `DashboardDataLayer` is instantiated at module import time as `_dal = DashboardDataLayer()`. This creates a global singleton with hidden state and no explicit lifecycle management.

**Evidence:**

- [data.py:L106-107](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/dashboard/data.py#L106-107)

```python
# Global instance (lazily initialized if needed, but here eager for simplicity)
_dal = DashboardDataLayer()
```

**The Smell:**

- **Eager Loading**: The `DatabaseManager` is created at import time, which may not be desirable for testing or when not using the dashboard.
- **Hidden State**: The global `_dal` makes the module hard to test - you can't easily inject a different data layer.
- **Connection Leaks**: No clear lifecycle (startup/shutdown) for the database connection.

**Planned Resolution:**

- Use lazy initialization pattern (create `_dal` only when first accessed).
- Consider implementing a proper factory or context-based approach for better testability.

**User Review:**

---

### 5. Hardcoded Container Paths in Multiple Places

**Problem:** The container path `/workspace` is hardcoded in multiple locations across the codebase, making it fragile and hard to configure.

**Evidence:**

- [sandbox.py:L38](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/sandbox.py#L38)
- [mujoco_bridge.py:L147-148](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/compiler/mujoco_bridge.py#L147-148)
- [simulation.py:L98](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/simulation_engine/simulation.py#L98)
- [preview_runner.py:L11](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/assets/scripts/preview_runner.py#L11)

```python
container_workspace = "/workspace"
sys.path.append("/workspace")
```

**The Smell:**

- **Magic String Duplication**: Container path is repeated in 10+ places with no single source of truth.
- **Fragile Coupling**: If the container workspace path ever needs to change, many files would need updating.

**Planned Resolution:**

- Create a constant `CONTAINER_WORKSPACE = "/workspace"` in a central location (e.g., `src/environment/constants.py`).
- Update all usages to reference this constant.
- For scripts that run inside the sandbox (like `preview_runner.py`), this constant can be passed via config or kept as is since they always run in `/workspace`.

**User Review:**

---

### 6. Fragile CLI-Based Container Communication

**Problem:** The current container communication relies on parsing raw stdout/stderr from `podman exec`. This is brittle and error-prone compared to structural communication.

**Evidence:**

- [sandbox.py:L131](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/sandbox.py#L131)
- [env_adapter.py:L168-172](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/tools/env_adapter.py#L168-172)

```python
# src/agent/tools/env_adapter.py
if isinstance(output, str):
    try:
        return json.loads(output)
    except Exception:
        return {"error": output}
```

**The Smell:**

- **Unstructured Data**: We rely on the called script *only* printing JSON to stdout. Any other print (deprecation warnings, logs) breaks the JSON parsing.
- **No Typed Contracts**: Input/output schemas are implicit dicts, not Pydantic models.
- **High Latency**: Spawning a new `podman exec` process for every tool call is slow compared to a persistent connection.

**Planned Resolution:**

- **Implement HTTP Service**: Spin up a FastAPI service inside the container listening on a port.
- **Use `httpx`**: Communicate with the container service using `httpx` and Pydantic models for request/response bodies.
- **Strong Typing**: Define shared Pydantic models for all tool inputs/outputs.

**User Review:**

---

### EXTRA:  Naming drift after refactors

I have identified several classes and functions that appear to be misnamed or logically inconsistent following the recent refactors:

   1. `ToolRuntime.check_manufacturability`: This is explicitly marked as a "Legacy compatibility" wrapper for validate_and_export. Its name is now too narrow as the underlying method provides a full ValidationReport including cost analysis, not just manufacturability.
   2. `ToolRuntime.read_script`: This is a legacy wrapper that simply calls view_file. The view_file tool is now the standard for reading all files, making read_script redundant and misnamed as it implies it only works for scripts.
   3. `Evaluator`: While it handles "evaluation," it also handles preview_design (visualization). This suggests it has evolved into more general DesignRunner or TaskExecutor.
   4. `Actor` node (`actor_node`): Following the transition to the "Claude Code" archetype, this node now functions as a Coder or Engineer, yet it retains the generic name Actor.
   5. `Critic` node (`critic_node`): The prompts and reviews refer to this as the Reviewer or Design Reviewer. Its role has evolved from a simple critic into a quality assurance and validation gate.
   6. `skill_populator`: This name implies it populates skills, but after the removal of specialized skill tools, it has evolved into a node that performs "Knowledge Distillation" or "Lesson Learning" by editing files directly.
   7. `log_to_env` (in `env_log.py`): This function name is a leftover from the CADEnv era. Since CADEnv was replaced by ToolRuntime, it should logically be named log_to_runtime.
   8. `submit_design`: This function now encompasses validation, budget checks, and simulation. The name "submit" implies a final action, but it is often used iteratively for evaluation.

---

## 🟡 Minor Issues & Smells

### 7. RAG Search Uses Hardcoded `lru_cache` Without Size Limit Strategy

**Problem:** The `load_docs` function caches directory contents with `@lru_cache(maxsize=32)`. There's no invalidation strategy for when new docs are added.

**Evidence:**

- [search.py:L38](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/rag/search.py#L38)

```python
@lru_cache(maxsize=32)
def load_docs(directory: str) -> list[dict[str, str]]:
```

**The Smell:**

- **Stale Cache**: If documentation is updated, the cache won't reflect changes until process restart.
- **Unpredictable Behavior**: Users may add docs and not see them in search results.

**Planned Resolution:**

- Add an explicit cache invalidation function or use file modification time checking.
- For development, consider disabling the cache.

**User Review:**

---

### 8. Unused Import in Planner Node

**Problem:** `planner.py` imports `set_current_role` (which doesn't exist) but never calls it. This is both a dead import AND the function doesn't exist.

**Evidence:**

- [planner.py:L4](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/nodes/planner.py#L4)

```python
from src.agent.tools.env_adapter import set_current_role
# ... but never used in the file
```

**The Smell:**

- **Dead Import**: Import is never used in the function body.
- **ImportError**: Even if it were used, it would fail because the function doesn't exist.

**Planned Resolution:**

- Remove the unused import line.

**User Review:**
How is this not tested? I know the CI passes green. Add tests.

---

### 9. Evaluator Still Uses String Concatenation for Config Injection

**Problem:** While the main runner scripts were moved to assets (addressing Round 6 feedback), the `Evaluator` still builds a preamble by concatenating strings to inject config file writing.

**Evidence:**

- [evaluator.py:L76-91](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/evaluator.py#L76-91)

```python
bootstrap_script = f'''
import json
import sys

config = {json.dumps(config)}
with open("{config_filename}", "w") as f:
    json.dump(config, f)
...
'''
```

**The Smell:**

- **Partial Fix**: The heavy runner logic was externalized, but a bootstrapper still uses string interpolation.
- **Complexity**: The bootstrapper prepends imports, writes config, then appends the runner content.

**Planned Resolution:**

- Consider making the runner scripts accept config via environment variables or a predictable fixed path, eliminating the need for a string-built preamble entirely.
- Alternatively, use `run_sandboxed_script` with a pre-written config file approach.

**User Review:**
not only there . @manager.py also has a similar string
---

### 10. Mock Fallback Data in Dashboard Data Layer

**Problem:** The `get_all_episodes` and `get_episode_by_id` functions fall back to hardcoded mock data when database queries fail or return empty. This can mask real issues.

**Evidence:**

- [data.py:L139-151](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/dashboard/data.py#L139-151)

```python
# Mock fallback
return [
    EpisodeSummary(
        id="ep_001",
        timestamp=datetime(2026, 2, 1, 10, 0, 0),
        name="Designing a Cube (Mock)",
    ),
    ...
]
```

**The Smell:**

- **Silent Failures**: Database errors are logged as warnings but then masked with fake data.
- **Confusing UX**: Users may see mock data without realizing their database connection is broken.
- **Debugging Difficulty**: Real issues are hidden behind mock responses.

**Planned Resolution:**

- Consider making this opt-in via an environment variable (e.g., `ENABLE_MOCK_DATA=true`).
- In production, return empty results or error states rather than mock data.

**User Review:**
>
>- Consider making this opt-in via an environment variable (e.g., `ENABLE_MOCK_DATA=true`).

Confirm.

---

### 11. `AgentState` TypedDict Has No Default Values

**Problem:** `AgentState` is a `TypedDict` that requires all fields to be present. Some node results don't provide all fields, which could cause KeyError or incomplete state.

**Evidence:**

- [state.py:L7-37](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/state.py#L7-37)

```python
class AgentState(TypedDict):
    messages: Annotated[list[AnyMessage], add_messages]
    plan: str
    step_count: int
    coder_reasoning: str
    full_history: list[dict[str, Any]]
    # ... all required fields
```

**The Smell:**

- **Missing Initialization**: Nodes like `critic_node` only return `{"messages": [response]}`, but the state expects `plan`, `step_count`, etc.
- **Potential Runtime Errors**: Accessing `state["step_count"]` without checking existence can fail.

**Planned Resolution:**

- Use `typing_extensions.NotRequired` (Python 3.11+) to mark optional fields.
- Or use `state.get("step_count", 0)` consistently (which is already done in some places but not all).

**User Review:**

---

## Summary

| Issue | Severity | Effort |
|-------|----------|--------|
| 1. Dead `set_current_role` import | 🔴 Critical | Low |
| 2. IM Workbench unsafe hash | 🔴 Critical | Low |
| 3. Duplicate `trim_messages` | 🟠 Moderate | Low |
| 4. Global DAL instance | 🟠 Moderate | Medium |
| 5. Hardcoded container paths | 🟠 Moderate | Medium |
| 6. Fragile container communication | 🟠 Moderate | Medium |
| 7. RAG cache stale | 🟡 Minor | Low |
| 8. Unused planner import | 🟡 Minor | Trivial |
| 9. Evaluator string concat | 🟡 Minor | Medium |
| 10. Mock fallback in dashboard | 🟡 Minor | Low |
| 11. AgentState required fields | 🟡 Minor | Medium |
