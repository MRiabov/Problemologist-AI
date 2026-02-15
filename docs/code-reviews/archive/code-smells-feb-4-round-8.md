# Problemologist-AI Codebase Architecture Review #8

This document identifies **NEW** architectural smells and questionable decisions discovered during the eighth deep-dive review of the codebase on **February 4, 2026**.

**Note:** Several issues from Round 7 have been resolved:

- ✅ `set_current_role` dead import — nodes renamed and refactored
- ✅ InjectionMoldingWorkbench hash — now uses shared `compute_part_hash`
- ✅ Duplicate `trim_messages` call — fixed in `coder.py`
- ✅ Global DAL instance — now uses lazy initialization via `get_dal()`
- ✅ RAG lru_cache staleness — now uses file modification time cache
- ✅ AgentState required fields — now uses `NotRequired` properly

---

## 🔴 Critical Architectural Issues

### 1. `MujocoBridge` Constructor Has Broken Signature in Runner Script

**Problem:** The dynamically generated runner script in `bridge.py` instantiates `MujocoBridge()` with no arguments, but the class requires `workspace_dir` and `sandbox` as constructor parameters.

**Evidence:**

- [bridge.py:L150](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/simulation_engine/bridge.py#L150)

```python
# In the dynamically generated runner_script:
bridge = MujocoBridge()  # ← Missing required args!

# But the actual class signature:
def __init__(
    self,
    workspace_dir: str | Path,
    sandbox: Any,  # Required!
):
```

**The Smell:**

- **Runtime Failure**: This script will crash with `TypeError` when executed inside the sandbox.
- **Untested Path**: The sandbox execution path may not be covered by tests if they mock too aggressively.
- **String Interpolation Risk**: The script is built as an f-string, making errors hard to catch statically.

**Planned Resolution:**

- Either:
  - Pass appropriate default values to the constructor when running inside the sandbox, or
  - Factor out `_run_simulation_internal` into a standalone function that doesn't require a full `MujocoBridge` instance.

**User Review:**
Okay. Just move ALL "injected" strings out to `/assets/`, and remove the "injection" logic completely.

---

### 2. Global `workspace` Instance in `memory.py` is Hardcoded

**Problem:** The `memory.py` module creates a global `Workspace` instance with a hardcoded path `workspace`, which doesn't match the actual workspace management pattern used elsewhere.

**Evidence:**

- [memory.py:L11](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/tools/memory.py#L11)

```python
workspace = Workspace(root_dir="workspace")
```

**The Smell:**

- **Path Mismatch**: Other code uses `ToolRuntime.workspace_dir` (which can be `./workspaces/main` or a UUID-based path). This global variable ignores the runtime's actual workspace.
- **State Leak**: The global instance persists across requests/tests, potentially causing cross-contamination.
- **Testability**: Hard to mock or inject a different workspace for testing.

**Planned Resolution:**

- Refactor `read_journal` to accept a `tool_runtime` parameter (like other tools) and use `tool_runtime.workspace_dir` instead.

**User Review:**
We have a requirement to have multiple workers working in paralel - see @file:docs/desired_architecture.md. I don't get it - how can a single "workspace" folder, without temp subdirs handle multiple paralel workers?
Propose a resolution.

Same comments as in issue 3.

---

## 🟠 Moderate Design Issues

### 3. `CONTAINER_WORKSPACE` Constant Not Consistently Used

**Problem:** A `CONTAINER_WORKSPACE = "/workspace"` constant was added to `src/environment/config.py`, but it's only imported in one file (`design_executor.py`). Many other files still hardcode `"/workspace"`.

**Evidence:**

- [config.py:L6](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/config.py#L6) — defines the constant
- [sandbox.py:L119, L280, L349](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/sandbox.py#L119) — uses literal `"/workspace"`
- [bridge.py:L148, L153-154, L167](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/simulation_engine/bridge.py#L148) — uses literal `"/workspace"`
- [manager.py:L55, L89, L114](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/generators/benchmark/manager.py#L55) — uses literal `"/workspace"`
- [preview_runner.py:L11, L45, L62, L78, L99](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/assets/scripts/preview_runner.py#L11) — uses literal `"/workspace"`

**The Smell:**

- **Incomplete Refactor**: The constant exists but wasn't propagated to all usages.
- **Magic String Duplication**: `"/workspace"` appears in 20+ locations.
- **Configuration Drift Risk**: If the container workspace path ever changes, many files would break.

**Planned Resolution:**

- Replace all hardcoded `"/workspace"` strings in host-side code with `CONTAINER_WORKSPACE` constant.
- For sandbox scripts (like `preview_runner.py`), these scripts run *inside* the container, so the path is fixed. However, consider injecting it via config for consistency.

**User Review:**

Okay, but do we not need random temp subfolders? How would a single /workspace/ folder hold multiple paralel workers? We have UUID requirements for these folders, so where are they?

As a matter of fact, the /workspace/ pattern is exclusively for a little simpler debug. In prod it would be running in own containers, and it won't be mounted to the local path.
In fact, it sounds like a good idea to make a `Podman.local.sandbox` and `Podman.prod.sandbox` - split the two; where the other would not be mounted and would send the files back and forth with some other method.

---

### 4. `run_simulation` is Async but Called Synchronously in Runtime

**Problem:** `MujocoBridge.run_simulation` is defined as `async def` method, but `ToolRuntime.verify_solution` appears to call it without awaiting in a sync context.

**Evidence:**

- [bridge.py:L101](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/simulation_engine/bridge.py#L101)

```python
async def run_simulation(
    self,
    xml_string: str,
    ...
) -> SimResult:
```

- [runtime.py:L282](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/runtime.py#L282)

```python
sim_result = self.sim_bridge.run_simulation(injected_xml)  # Not awaited!
```

**The Smell:**

- **Coroutine Leak**: Calling an async function without `await` returns a coroutine object, not the result.
- **Silent Failure**: The coroutine is never awaited, so the simulation never actually runs.
- **Type Confusion**: The code treats `sim_result` as a `SimResult`, but it's actually a coroutine.

**Planned Resolution:**

- Either:
  - Make `verify_solution` async and await the call, or
  - Make `run_simulation` synchronous (since it already wraps `run_sandboxed_script` synchronously inside).

**User Review:**
How is this not tested? Add tests too.
Also, it could be already edited away - I fixed 20 or so test failures.

---

### 5. `log_to_runtime` Still Uses Legacy Naming

**Problem:** Round 7 identified that `log_to_env` (now `log_to_runtime`) is a naming leftover from the `CADEnv` era. The function was renamed but still references "env" in comments and the module is still called `env_log.py`.

**Evidence:**

- [env_log.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/utils/env_log.py)

```python
def log_to_runtime(  # Function renamed ✓
    ...
):
    """Logs a message to the active ToolRuntime if it exists."""
    env = get_runtime()  # Variable still called 'env' ← Legacy naming
```

**The Smell:**

- **Partial Rename**: Function was renamed but internal variables and module name were not updated.
- **Confusion**: The file is `env_log.py` but exports `log_to_runtime`.

**Planned Resolution:**

- Rename `env_log.py` → `runtime_log.py`.
- Rename internal variable `env` → `runtime`.

**User Review:**
Confirm rename.

---

### 6. `execute_tool` Endpoint Returns Placeholder Response

**Problem:** The `/exec` endpoint in `container_agent.py` returns a static placeholder message and never actually executes tools.

**Evidence:**

- [container_agent.py:L197-202](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/assets/scripts/container_agent.py#L197-202)

```python
@app.post("/exec", ...)
def execute_tool(request: ToolRequest):
    return ToolResponse(
        output=f"Tool {request.tool_name} not implemented in agent yet."
    )
```

**The Smell:**

- **Dead Endpoint**: The endpoint exists but does nothing useful.
- **False Advertising**: The OpenAPI schema suggests this works.
- **Confusing**: Is this a planned feature or abandoned code?

**Planned Resolution:**

- Either implement the tool execution logic, or remove the endpoint to avoid confusion.
- If kept for future use, add a clear docstring explaining it's not yet implemented.

**User Review:**
Why do we need it at all? Why would tool execution be in the endpoint? it's called by the agent... I don't think we need it.

---

### 7. `env_adapter` Uses Module-Level Global Runtime State

**Problem:** `env_adapter.py` maintains global state via `_RUNTIMES` dict and `_FALLBACK_RUNTIME`. This pattern, while better than `_ACTIVE_ENV`, still creates implicit coupling.

**Evidence:**

- [env_adapter.py:L10-12](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/tools/env_adapter.py#L10-12)

```python
_DEFAULT_RUNTIME_ID: str = "default"
_RUNTIMES: dict[str, ToolRuntime] = {}
_FALLBACK_RUNTIME: ToolRuntime | None = None
```

**The Smell:**

- **Global State**: Multiple concurrent agent runs could interfere if they share runtimes.
- **Lazy Fallback**: The `_get_fallback_runtime()` function creates a runtime with `"workspace"` as the path—a relative path that may not exist.
- **Architecture Violation**: The `architecture.md` states "No Global State: `_ACTIVE_ENV` patterns are forbidden."

**Planned Resolution:**

- Consider threading/contextvar-based isolation if multiple agents run concurrently.
- Make the fallback more explicit or remove it entirely (require explicit registration).

**User Review:**
Am I sick of this! enough with such fallbacks. This is an ETL pipeline, no fallbacks here except explicitly allowed
---

## 🟡 Minor Issues & Smells

### 8. `learner_node` Uses Hardcoded Skill Path

**Problem:** The `learner_node` hardcodes a specific skill path in its prompt, which may not align with the actual skill directory structure.

**Evidence:**

- [learner.py:L39-40](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/nodes/learner.py#L39-40)

```python
"Use the `write_file` tool to add a new reference file (e.g., 'docs/skills/build123d_cad_drafting_skill/lessons_learned_cad.md') "
```

**The Smell:**

- **Hardcoded Path**: Skills are actually in `.agent/skills/`, not `docs/skills/`.
- **Brittle**: If skill paths change, this prompt becomes misleading.

**Planned Resolution:**

- Update the prompt to use the correct path (`".agent/skills/build123d_cad_drafting_skill/..."`) or make it configurable.

**User Review:**
I think this is a misunderstanding. skills in the agent and skills should be different. I've added a line in the end of @file:desired_architecture.md about it, read it.

---

### 9. Tool Registry vs env_adapter `view_file` Mismatch

**Problem:** The tool registry imports `view_file` from `env_adapter`, but there's also a method `view_file` on `ToolRuntime`. The tool doesn't appear in `tool_map` in `_run_tool`.

**Evidence:**

- [registry.py:L18](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/tools/registry.py#L18) — imports `view_file`
- [runtime.py:L317-330](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/runtime.py#L317-330) — `tool_map` missing `view_file`

```python
tool_map = {
    "write_file": self.write_file,
    "edit_file": self.edit_file,
    # ... view_file is MISSING from this map!
}
```

**The Smell:**

- **Dispatch Failure**: Calling `view_file` via `dispatch()` or `_run_tool()` will raise `ValueError: Unknown tool`.
- **Silent Bug**: The tool works when called directly from the adapter but fails through the runtime dispatch path.

**Planned Resolution:**

- Add `"view_file": self.view_file` to the `tool_map` in `_run_tool`.

**User Review:**
Crazy. this is top priority! Fix it!

---

### 10. Commented-Out Network Flag in `sandbox.py`

**Problem:** There are multiple commented-out network-related flags in `sandbox.py`, suggesting uncertainty about the correct configuration.

**Evidence:**

- [sandbox.py:L146-148](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/sandbox.py#L146-148)

```python
# \"--network\", network, # Removing network restriction to allow port mapping working reliably?
# Or keep it if it works with -p.
# \"-p\", f\"{host_port}:8000\",
```

- [sandbox.py:L354](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/environment/sandbox.py#L354)

```python
# \"--network\", \"none\",
```

**The Smell:**

- **Code Archaeology**: Commented code left in production suggests incomplete decisions.
- **Security Confusion**: Network isolation is a security boundary—it should be clear and documented.

**Planned Resolution:**

- Remove commented-out code and document the network strategy in a comment or docstring.
- If network isolation is important, test and enable it properly.

**User Review:**

---

### 11. `coder_node` Uses Legacy Prompt Key Name

**Problem:** The `coder_node` references a prompt key `"cad_agent.actor.system"` despite being renamed from `actor_node`.

**Evidence:**

- [coder.py:L27](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/graph/nodes/coder.py#L27)

```python
system_prompt_key = "cad_agent.actor.system"  # Still says 'actor'!
```

**The Smell:**

- **Inconsistent Naming**: The node is `coder_node` but references `actor` prompts.
- **Confusion**: Developers looking for `coder` prompts won't find them.

**Planned Resolution:**

- Rename prompt key to `"cad_agent.coder.system"` and update the prompts YAML accordingly.
- Or add a `coder` alias in the prompts file.

**User Review:**

Grep around the codebase for "actor" and "critic" and replace them accordingly. They are mostly deprecated.

---

### 12. `generate_image` Tool Not Available to Agent

**Problem:** The benchmark generator uses an AI agent (`generator_agent`) but there's no `generate_image` tool in the tool registry, limiting the agent's ability to create visual assets.

**Evidence:**

- [registry.py:L24-37](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/src/agent/tools/registry.py) — no image generation tool

**The Smell:**

- **Limited Capability**: Per the architecture, agents should be able to create/update skills and generate assets.
- **Placeholder Gap**: The system has rendering capabilities (`render_scenario`) but no tool for the agent to invoke it.

**Planned Resolution:**

- Consider adding a `render_preview` or similar tool to allow agents to request visual previews.

**User Review:**
...
OMG.
We did have a "preview" or similar tools, but I refactored it away for some reason. Yes, we definitely need a preview tool, for both reviewer and coder at least.
Let's add a "visualize_part" script/tool that the agent can call. I suppose. What I'm hesitant is - should we make it as another tool or a python script that the agent can simply call? e.g. `uv run ...`.

---

## Summary

| Issue | Severity | Effort |
|-------|----------|--------|
| 1. MujocoBridge broken constructor in runner | 🔴 Critical | Low |
| 2. Global workspace in memory.py | 🔴 Critical | Medium |
| 3. CONTAINER_WORKSPACE not used | 🟠 Moderate | Low |
| 4. Async/sync mismatch in run_simulation | 🟠 Moderate | Medium |
| 5. env_log.py legacy naming | 🟠 Moderate | Low |
| 6. /exec endpoint placeholder | 🟠 Moderate | Low |
| 7. env_adapter global state | 🟠 Moderate | High |
| 8. learner_node hardcoded skill path | 🟡 Minor | Trivial |
| 9. view_file missing from tool_map | 🟡 Minor | Trivial |
| 10. Commented network flags | 🟡 Minor | Low |
| 11. Legacy prompt key name | 🟡 Minor | Trivial |
| 12. No generate_image tool | 🟡 Minor | Medium |

---

*Review conducted on 2026-02-04 by Gemini (Antigravity)*
