# Problemologist-AI Codebase Architecture Review #3

Following the previous rounds (Volume #1 & #2), this document identifies **NEW** architectural smells and questionable decisions that were not addressed in prior reviews.

---

## Executive Summary

The codebase has evolved with significant fixes from earlier reviews. **Gymnasium has been removed** and `CADEnv` now uses a clean `dispatch()` method. However, deeper inspection reveals additional structural issues:

**Critical Issues:**

- Duplicate tool list definitions across 4+ locations
- Complex runtime fallback chain with unclear ownership
- Session management embedded directly in graph nodes
- MujocoBridge pulls global state from legacy tools module

**Moderate Issues:**

- Unused `session_scope` context manager in persistence
- Workbench imports inside method bodies
- DEBUG prints in `builder.py`
- RAG search is a simplistic substring match

---

## 🔴 Critical Architectural Issues

### 1. Tool List Quadruplication

**Problem:** The list of available tools is defined in **four separate places** with subtle differences:

| Location | Tools Defined |
|----------|---------------|
| [graph.py:44-56](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/graph.py#L44-56) | 12 tools in `build_graph()` |
| [actor.py:35-47](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/nodes/actor.py#L35-47) | 11 tools in fallback `if tools is None` |
| [actor.py:6-17](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/nodes/actor.py#L6-17) | Import list (different order) |
| [runtime.py:184-217](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/runtime.py#L184-217) | Tool dispatch switch (separate enum) |

**Evidence:**

```python
# graph.py - master list
tools = [write_file, edit_file, view_file, run_command, preview_design,
         submit_design, search_docs, check_manufacturability, read_journal,
         search_parts, preview_part, lint_script]

# actor.py - fallback (different order, different count)
tools = [write_file, edit_file, view_file, run_command, preview_design,
         submit_design, search_docs, check_manufacturability, read_journal,
         search_parts, preview_part]  # Missing lint_script!
```

**The Smell:**

- Adding a new tool requires edits in 4+ places
- `lint_script` is missing from actor.py fallback → agent won't have linting in some code paths
- No single source of truth for "what tools does the agent have?"

**Recommendation:** Create a `TOOL_REGISTRY` in one place that all consumers import.

---

### 2. Complex Runtime Fallback Chain

**Problem:** [env_adapter.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/tools/env_adapter.py) has a 3-tier fallback for obtaining a runtime:

```python
def get_runtime(runtime_id: str | None = None) -> ToolRuntime:
    # 1. Check explicit registry
    if runtime_id and runtime_id in _RUNTIMES:
        return _RUNTIMES[runtime_id]
    
    # 2. Check _ACTIVE_ENV.runtime (legacy global)
    if _ACTIVE_ENV and hasattr(_ACTIVE_ENV, "runtime"):
        return _ACTIVE_ENV.runtime
    
    # 3. Check default registry
    if _DEFAULT_RUNTIME_ID in _RUNTIMES:
        return _RUNTIMES[_DEFAULT_RUNTIME_ID]
    
    # 4. Create fallback singleton
    return _get_fallback_runtime()
```

**Why This Is Problematic:**

- **4** different sources of truth for "current runtime"
- The fallback runtime is created lazily with hardcoded `"workspace"` path
- `_run_env_step` at line 111 has special logic to detect "is this the fallback?" and behave differently
- Difficult to reason about which runtime is actually active

**Recommendation:** Eliminate fallbacks. Require explicit runtime injection at graph construction time.

---

### 3. Session Management in Graph Nodes

**Problem:** The planner node directly starts sandbox sessions:

```python
# planner.py:21
async def planner_node(state: AgentState):
    ...
    # Start persistent sandbox session if not already active
    await start_session_async("vlm-cad-session")
```

**Why This Is Problematic:**

- **Side effect in a "planning" node** – planner should just plan, not manage infrastructure
- Hardcoded session ID `"vlm-cad-session"` prevents multiple agents
- No corresponding `stop_session` anywhere visible
- If planner is called multiple times (re-planning loop), it tries to start session repeatedly

**Recommendation:** Session lifecycle should be managed by the graph runner or the runtime, not embedded in nodes.

---

### 4. MujocoBridge Imports Legacy Global State

**Problem:** [mujoco_bridge.py:101-106](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/compiler/mujoco_bridge.py#L101-106) imports and uses the legacy `tools` module:

```python
def run_simulation(...):
    from src.environment import tools  # 🚨 Legacy import!
    
    _ = tools.WORKSPACE_DIR  # Accessing global
    workspace_dir = Path(tools.WORKSPACE_DIR)
    ...
    tools._SANDBOX.run_script(...)  # Depends on global _SANDBOX!
```

**The Smell:**

- Volume #1 identified removing globals from `tools.py`, but `mujoco_bridge` still depends on them
- If `tools._SANDBOX` is `None` (not initialized), this crashes
- Creates implicit coupling between simulation and tool state

**Recommendation:** Pass workspace_dir and sandbox as constructor arguments to `MujocoBridge`.

---

## 🟠 Moderate Design Issues

### 5. Unused `session_scope` Context Manager

**Problem:** [persistence.py:118-129](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/persistence.py#L118-129) defines a proper context manager:

```python
@contextmanager
def session_scope(self) -> Generator[Session, None, None]:
    """Provide a transactional scope around a series of operations."""
    session = self.SessionLocal()
    try:
        yield session
        session.commit()
    except Exception:
        session.rollback()
        raise
    finally:
        session.close()
```

But **none of the methods use it**:

```python
# Every method does this instead (lines 131-137, 153-169, etc.)
def create_episode(self, problem_id: str) -> Episode:
    session = self.get_session()
    ...
    session.commit()
    session.close()  # Manual management!
    return episode
```

**The Smell:**

- Violation of DRY – every method reimplements session lifecycle
- Error handling inconsistency – some methods might not properly rollback on error
- The context manager was added but never integrated

**Recommendation:** Refactor all methods to use `session_scope`.

---

### 6. Imports Inside Method Bodies

**Problem:** Several workbenches import `hashlib` inside their `calculate_cost` methods:

```python
# cnc.py:63-64
def calculate_cost(self, part, quantity=1, context=None):
    ...
    if context is not None:
        import hashlib  # 🚨 Import inside method!
        part_hash = hashlib.md5(...)
```

Same pattern in:

- [injection_molding.py:95](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/workbenches/injection_molding.py#L95)

**Why This Is Problematic:**

- Adds import overhead on every call
- Indicates the code was added hastily without refactoring
- Makes dependencies non-obvious at module level

---

### 7. DEBUG Prints in builder.py

**Problem:** Production code contains debugging prints:

| File | Debug Statement |
|------|-----------------|
| [builder.py:380-389](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/simulation_engine/builder.py#L380-389) | Multiple `print(f"DEBUG: ...")` |
| [builder.py:422-426](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/simulation_engine/builder.py#L422-426) | More DEBUG prints |

**The Smell:**

- Clutters stdout during normal operation
- Not using proper logging levels
- Makes it hard to filter important output

**Recommendation:** Replace with `logger.debug()` using Python's logging module.

---

### 8. InjectionMoldingWorkbench Redundant Method

**Problem:** [injection_molding.py:26-31](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/workbenches/injection_molding.py#L26-31):

```python
def validate(self, part: Part) -> List[Union[Exception, str]]:
    """Validates the part for injection molding."""
    return self.validate_geometry(part)  # Just calls another method!

def validate_geometry(self, part: Part) -> List[Union[Exception, str]]:
    """Actual implementation..."""
```

**The Smell:**

- `validate()` is an abstract method from base, but it just delegates
- Creates confusion about which method to call
- Base class already has `validate_geometry` as alias for `validate` (opposite direction!)

---

### 9. RAG Search Is Primitive

**Problem:** [rag/search.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/rag/search.py) implements search as simple substring matching:

```python
def search(query: str, directory: str = "docs") -> str:
    query = query.lower()
    ...
    for doc in all_docs:
        title_hits = doc["title"].lower().count(query)
        content_hits = doc["content"].lower().count(query)
        score = title_hits * 10 + content_hits
```

Also includes hardcoded stubs (lines 4-30) that provide minimal documentation.

**The Smell:**

- No semantic search, embeddings, or proper ranking
- Hardcoded documentation snippets instead of actual docs
- Will fail for complex queries or synonyms
- The "RAG" namespace is misleading – this isn't Retrieval Augmented Generation

---

### 10. Graph Routing Logic Fragility

**Problem:** [graph.py:145](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/graph.py#L145) uses string matching for routing:

```python
def route_critic(state) -> Literal["planner", "actor", "skill_populator", "__end__"]:
    ...
    if any(x in content for x in ["Validation Passed!", "Task complete", "TASK_COMPLETE"]):
        return END
```

Similarly, `route_tools` (line 165) checks for tool names:

```python
if "preview_design" in tool_names or validation_tool_name in tool_names:
    return "critic"
```

**The Smell:**

- Magic strings for flow control
- If LLM outputs slightly different text ("Validation passed" lowercase), routing breaks
- No structured way to signal completion

**Recommendation:** Use structured output or explicit completion signals in state.

---

## 🟡 Minor Issues & Smells

### 11. `sys.path` Manipulation in Dashboard

```python
# dashboard/main.py:12-14
root_path = Path(__file__).resolve().parent.parent.parent
if str(root_path) not in sys.path:
    sys.path.insert(0, str(root_path))
```

This is a symptom of improper packaging. Should be installable as a proper package.

---

### 12. Hardcoded Paths (Consolidation Incomplete)

Despite Volume #1 recommendations, paths are still hardcoded in multiple places:

| File | Hardcoded Path |
|------|----------------|
| [manager.py:68](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/generators/benchmark/manager.py#L68) | `Path("workspaces/gen")` |
| [manager.py:229](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/generators/benchmark/manager.py#L229) | `".agent_storage/temp_assets"` |
| [env_adapter.py:23](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/tools/env_adapter.py#L23) | `ToolRuntime("workspace")` |
| [rag/search.py:68](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/rag/search.py#L68) | `".agent/skills"` |

---

### 13. Config Module-Level Instantiation

[config.py:50](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/utils/config.py#L50):

```python
# Create a global instance
Config = AppConfig()
```

This is executed at import time. While Pydantic Settings handles `.env` loading well, this pattern:

- Makes testing harder (can't easily mock Config)
- Locks in environment at import time

---

## 📊 Summary Table

| Issue | Severity | Effort | Risk |
|-------|----------|--------|------|
| Tool list quadruplication | 🔴 High | Low | Missing tools in code paths |
| Runtime fallback chain | 🔴 High | Medium | Confusion, hard to debug |
| Session in planner node | 🔴 High | Low | Side effects, multi-agent issues |
| MujocoBridge global deps | 🔴 High | Medium | Crashes if globals unset |
| Unused session_scope | 🟠 Medium | Low | Inconsistent error handling |
| Imports inside methods | 🟠 Medium | Low | Performance, unclear deps |
| DEBUG prints | 🟠 Medium | Low | Noisy output |
| Redundant validate methods | 🟠 Medium | Low | API confusion |
| Primitive RAG search | 🟠 Medium | High | Poor retrieval quality |
| Magic strings in routing | 🟠 Medium | Medium | Fragile flow control |
| sys.path manipulation | 🟡 Low | Medium | Packaging issue |
| Hardcoded paths | 🟡 Low | Low | Portability |
| Config instantiation | 🟡 Low | Low | Testing friction |

---

## Priority Recommendations

### Immediate (Low-Hanging Fruit)

1. Create single `TOOL_REGISTRY` and use everywhere
2. Move session management out of planner node
3. Replace DEBUG prints with proper logging
4. Use `session_scope` in persistence methods

### Short-term (Architecture)

1. Refactor `MujocoBridge` to not depend on legacy globals
2. Simplify runtime fallback to explicit injection only
3. Replace magic strings with structured completion signals

### Medium-term (Foundations)

1. Implement proper RAG with embeddings
2. Package codebase properly to avoid `sys.path` hacks
3. Centralize path configuration in `Config`
