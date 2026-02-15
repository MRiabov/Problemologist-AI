# Problemologist-AI Codebase Architecture Review

This document provides a comprehensive critique of the architectural and implementation decisions in the Problemologist-AI codebase, identifying patterns that could be challenged or improved.

---

## Executive Summary

The codebase implements a **LangGraph-based agentic CAD system** with a Gymnasium-compatible environment, Podman sandboxed code execution, and multi-workbench manufacturing validation. While the overall goal is clear, there are several architectural decisions that create friction, complexity, and potential issues.

**Critical Issues:**

- Pervasive global mutable state across tool layers
- Three-layer tool indirection creating complexity without clear benefit
- Mixed sync/async model causing maintenance burden
- Exec-based code execution with security implications
- Tight coupling between components that should be independent

---

## 🔴 Critical Architectural Issues

### 1. Global Mutable State Epidemic

**Problem:** The codebase relies heavily on module-level global state, making testing, concurrency, and reasoning about behavior difficult.

**Evidence:**

| File | Globals |
|------|---------|
| [env_adapter.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/tools/env_adapter.py) | `_ACTIVE_ENV`, `_CURRENT_ROLE` |
| [tools.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/tools.py#L20-27) | `WORKSPACE_DIR`, `_SANDBOX`, `_ACTIVE_SESSION_ID`, `_PART_INDEX`, `_WORKBENCHES`, `_PROMPTS_CACHE` |
| [prompts.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/utils/prompts.py#L7) | `_PROMPTS_CACHE` |

```python
# From env_adapter.py - globals mutated by setter functions
_ACTIVE_ENV: Any | None = None
_CURRENT_ROLE: str | None = None

def set_active_env(env: Any):
    global _ACTIVE_ENV
    _ACTIVE_ENV = env
```

**Why This Is Problematic:**

- **Cannot run multiple environments concurrently** - the global `_ACTIVE_ENV` is a shared singleton
- **Testing is fragile** - tests must carefully reset globals or suffer leakage
- **Implicit dependencies** - functions like `_run_env_step` depend on globals being set correctly elsewhere
- **Thread safety nightmare** - concurrent async calls could clobber state

**Recommendation:** Use dependency injection or context objects. Pass the environment instance explicitly to tool functions instead of relying on globals.

---

### 2. Three-Layer Tool Indirection

**Problem:** Tools are defined in three places with unclear boundaries:

```
src/environment/tools.py       ← Raw implementations (sync)
         ↑
src/agent/tools/env_adapter.py ← Async wrappers + env.step routing
         ↑
src/agent/tools/env.py         ← LangChain @tool decorators
```

**The Flow:**

1. **`env.py`** defines `@tool` decorated functions that just call `*_async` functions
2. **`env_adapter.py`** wraps sync functions in `asyncio.to_thread()` or routes through `_run_env_step`
3. **`tools.py`** contains the actual implementations

**Why This Is Problematic:**

- **Duplication everywhere** - every tool is defined 3 times with subtle differences
- **Inconsistent routing** - some async wrappers go through `_run_env_step`, others call `env_tools` directly:

```python
# From env_adapter.py - inconsistent patterns
async def search_docs_async(query: str) -> str:
    return await _run_env_step("search_docs", query=query)  # Uses env routing

async def list_skills_async() -> str:
    return await asyncio.to_thread(env_tools.list_skills)   # Direct call!
```

- **Signature mismatches** - `submit_design` in `env.py` takes `control_path`, but `_submit_design` in `core.py` takes `arguments` (dict or string)
- **Maintenance burden** - adding a new tool requires edits in 3 files

**Recommendation:** Consolidate to a single source of truth. Consider a registry pattern where tools are defined once with metadata, and wrappers are generated.

---

### 3. CADEnv Action Dispatch Duplicated in Core

**Problem:** The `CADEnv.step()` method has a 50-line switch statement that duplicates the tool registry:

```python
# From core.py:133-184 - giant switch statement
if tool_name == "write_file":
    tool_output = tools.write_file(**arguments)
elif tool_name == "edit_file":
    tool_output = tools.edit_file(**arguments)
elif tool_name == "preview_design":
    ...
elif tool_name == "submit_design":
    ...
```

**Why This Is Problematic:**

- **Not DRY** - tool names are hardcoded strings matching the adapter
- **Easy to forget** - adding a tool to `tools.py` doesn't automatically add it to `CADEnv`
- **Double handling of `submit_design`** - lines 167-184 handle it twice (once in the if/elif, once after)

```python
# Buggy pattern in core.py
if tool_name == "submit_design":
    reward, tool_output, terminated = self._submit_design(raw_arguments)
    pass  # <-- does nothing, then falls through to...

if tool_name == "submit_design":  # line 183 - DUPLICATE!
    reward, tool_output, terminated = self._submit_design(arguments)
```

**Recommendation:** Use a tool registry dictionary with closures, or route all tool calls through a single dispatcher.

---

### 4. `exec()` with User-Provided Code

**Problem:** Multiple places execute arbitrary Python strings using `exec()`:

| Location | Issue |
|----------|-------|
| [tools.py:539](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/tools.py#L539) | `exec(script_content, globals(), locs)` in `_analyze_cached` |
| [core.py:310](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/core.py#L310) | `exec(code, ctx, locs)` in runner script |
| [tools.py:219](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/tools.py#L219) | Same pattern in preview_design runner |

**In `_analyze_cached`:**

```python
@functools.lru_cache(maxsize=128)
def _analyze_cached(file_hash: str, ...):
    locs = {}
    exec(script_content, globals(), locs)  # 🚨 SECURITY RISK
```

**Why This Is Problematic:**

- **`_analyze_cached` runs OUTSIDE the sandbox** - this is on the host!
- The `file_hash` can't protect against malicious code - same hash = same cached result, but the code still ran once
- **`globals()` is SHARED** - executed code has access to all imports in `tools.py`
- An LLM-generated script could `import os; os.system("rm -rf /")` and it would run on the host

**The Sandbox Usage Is Inconsistent:**

- `preview_design` uses `_SANDBOX.run_script()` ✅
- `check_manufacturability` uses `exec()` directly on host ❌
- `_submit_design` uses sandbox ✅

**Recommendation:** ALL code execution must go through the sandbox. Never exec user-provided code on the host.

---

## 🟠 Moderate Design Issues

### 5. Mixed Sync/Async Everywhere

**Problem:** The codebase can't decide if it's sync or async:

- **Agent graph nodes** are async (`actor_node`, `planner_node`)
- **Tools in `tools.py`** are sync
- **Adapters** wrap sync in `asyncio.to_thread()`
- **LangGraph's `ToolNode`** is async but calls sync tools

```python
# This pattern is everywhere in env_adapter.py
async def view_file_async(path: str) -> str:
    return await asyncio.to_thread(env_tools.view_file, path)
```

**Why This Is Problematic:**

- **Thread pool exhaustion** - `asyncio.to_thread` uses the default ThreadPoolExecutor
- **No real concurrency benefit** - you're just wrapping blocking I/O
- **Debugging is harder** - stack traces cross async/sync boundaries

**Recommendation:** Either make the core tools truly async (use `aiofiles`, etc.) or keep everything sync and use a synchronous executor.

---

### 6. `@functools.lru_cache` on Mutable-Friendly Code

**Problem:** `_analyze_cached` uses `lru_cache` but:

```python
@functools.lru_cache(maxsize=128)
def _analyze_cached(
    file_hash: str, default_process: str, default_quantity: int, script_content: str
):
    ...
    exec(script_content, globals(), locs)  # Side effect: executes code!
    ...
    return report
```

**Issues:**

1. **`script_content` is a string argument but also passed through** - if you call with the same hash but different content (collision?), you'd get wrong results
2. **The exec is a side effect** - caching a function that `exec`s is semantically weird
3. **The report contains mutable dicts** - mutating the returned report corrupts the cache

**Recommendation:** Cache only pure computational results. The `exec` should happen outside the cached function.

---

### 7. Gymnasium Interface Vestigial

**Problem:** The `CADEnv` extends `gym.Env` but doesn't really use it properly:

```python
self.action_space = spaces.Dict({
    "tool": spaces.Text(min_length=0, max_length=50),
    "arguments": spaces.Text(min_length=0, max_length=100000),
})
```

**Issues:**

- **Text action spaces aren't standard RL** - you can't sample from them meaningfully
- **The observation space is unused** - no current RL algorithm would work with this
- **`step()` signature is non-standard** - adds `agent_role` parameter

**Is This Even RL?**
Looking at the codebase, this isn't training an RL agent - it's providing a structured execution interface for an LLM. The Gymnasium interface adds complexity without benefit.

**Recommendation:** Either:

1. Remove Gymnasium entirely and use a simpler interface
2. Properly implement a discrete action space if RL training is actually planned

---

### 8. DatabaseManager Session Handling Anti-Pattern

**Problem:** Every method in `DatabaseManager` creates and immediately closes a session:

```python
def create_episode(self, problem_id: str) -> Episode:
    session = self.get_session()   # Creates new session
    episode = Episode(problem_id=problem_id)
    session.add(episode)
    session.commit()
    session.refresh(episode)
    session.close()                # Immediately closes
    return episode
```

**Issues:**

- **Detached objects** - the returned `Episode` is detached from the session
- **Lazy loading will fail** - accessing `episode.steps` after close will crash
- **Thread-local sessions not used** - SQLAlchemy's scoped_session pattern is missing
- **No transaction context** - multi-step operations can't be atomic

**Recommendation:** Use SQLAlchemy's async session patterns or scoped sessions with proper lifecycle management.

---

### 9. Config Class Is Just Environment Variables

**Problem:** `Config` is a class with class-level attributes from `os.getenv`:

```python
class Config:
    LLM_MODEL = os.getenv("LLM_MODEL", "z-ai/glm-4.7-flash")
    TEMPERATURE = float(os.getenv("LLM_TEMPERATURE", "0.0"))
    ...
```

**Issues:**

- **Evaluated at import time** - changing env vars after import has no effect
- **No validation** - `float("invalid")` would crash on import
- **Class attributes, not instance** - can't have different configs for different runs
- **`validate()` does nothing** - it's literally `pass`

**Recommendation:** Use Pydantic Settings or similar for validated, typed configuration.

---

## 🟡 Minor Issues & Smell

### 10. Hardcoded Paths Everywhere

```python
# From various files
prompts_path = project_root / "config" / "prompts.yaml"
skills_dir = Path(".agent/skills") / skill_name
init_script = Path(".agent/skills/skill-creator/scripts/init_skill.py")
```

Paths are hardcoded relative to working directory or computed from `__file__`. This breaks if:

- Running from a different directory
- Installed as a package
- Running in Docker with different mount points
<!-- dev note - we can fix this by using a config file that specifies the root directory; in fact we already have /config/ dir. -->

---

### 11. Bare `except:` Clauses

```python
# From env_adapter.py:45
try:
    kwargs = json.loads(kwargs["arguments"])
except:  # 🚨 Catches EVERYTHING including KeyboardInterrupt
    pass

# From tools.py:583
try:
    all_solids_raw.extend(list(obj.solids()))
except:
    pass
```

These silently swallow all errors including `SystemExit`, `KeyboardInterrupt`, and `MemoryError`.

---

### 12. Debug Prints Left in Production Code

```python
# From skill_populator.py:20
print("DEBUG: Entering CAD skill_populator_node")
```

---

### 13. Legacy Code with Deprecation Comments

```python
# From tools.py:134-145
def write_script(content: str, filename: str = "design.py") -> str:
    """[DEPRECATED] Writes content to a file..."""
    return write_file(content, filename, mode="overwrite")
```

If they're deprecated, remove them or add `warnings.warn()`.

---

## 📊 Summary Table

| Issue | Severity | Effort to Fix | Risk |
|-------|----------|---------------|------|
| Global mutable state | 🔴 High | Medium | Concurrency bugs |
| Three-layer tool indirection | 🔴 High | High | Maintenance debt |
| Duplicate dispatch in CADEnv | 🔴 High | Low | Bugs (already present) |
| `exec()` outside sandbox | 🔴 High | Medium | Security vulnerability |
| Mixed sync/async | 🟠 Medium | High | Performance |
| LRU cache on impure function | 🟠 Medium | Low | Correctness |
| Gymnasium interface misuse | 🟠 Medium | Medium | Complexity |
| Session handling | 🟠 Medium | Medium | Data integrity |
| Config class design | 🟡 Low | Low | Fragility |
| Hardcoded paths | 🟡 Low | Low | Portability |

---

## Recommendations Priority

### Immediate (Security/Correctness)

1. Move `_analyze_cached` exec into sandbox
2. Fix duplicate `submit_design` handling in `CADEnv.step()`
3. Replace bare `except:` with specific exceptions

### Short-term (Architecture)

1. Introduce dependency injection for environment/role context
2. Create tool registry to eliminate three-layer duplication
3. Properly implement session scoping for SQLAlchemy

### Medium-term (Tech Debt)

1. Remove Gymnasium interface if not needed for RL
2. Consolidate sync/async model
3. Replace Config class with Pydantic BaseSettings
4. Remove deprecated functions
