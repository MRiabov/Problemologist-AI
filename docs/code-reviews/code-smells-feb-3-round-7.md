# Problemologist-AI Codebase Architecture Review #7

This document identifies **NEW** architectural smells and questionable decisions discovered during the seventh deep-dive review of the codebase.

---

## 🔴 Critical Architectural Issues

### 1. Inconsistent Sandbox Mounting Strategy

**Problem:** `PodmanSandbox` exhibits inconsistent mounting behavior between persistent sessions (`start_session`) and transient execution (`run_script`).

**Evidence:**
- [sandbox.py:L51](src/environment/sandbox.py) (In `start_session`): Mounts `.agent/skills` to `/workspace/docs/skills` as `:rw`.
- [sandbox.py:L147-157](src/environment/sandbox.py) (In `run_script`): Only mounts `src` and `config` if `mount_src=True`, but *omits* the skills mount entirely.

**The Smell:**
- **"Works on My Session" Bug Risk**: A script that relies on accessing skills (e.g., searching documentation) will work perfectly when run in a persistent session but will fail mysteriously if run transiently (e.g., by the Evaluator or during a one-off check), as the directory will be missing.
- **Incomplete Abstraction**: The transient runner should mirror the environment of the persistent session as closely as possible to ensure reproducibility.

**Planned Resolution:**
- **Unified Mount Logic**: Refactor the mounting logic into a private helper method `_get_mount_args()` that is used by both `start_session` and `run_script` to ensure consistency.
- **Default Inclusion**: Ensure the skills directory is mounted in transient mode if `mount_src` (or a similar flag) is enabled.

**User Review:**

<!-- User comments here -->

---

## 🟠 Moderate Design Issues

### 2. Tool Adapter Argument Narrowing

**Problem:** The `env_adapter.py` wrapper for `submit_design` exposes a significantly restricted interface compared to the underlying `ToolRuntime` implementation.

**Evidence:**
- [runtime.py:L142](src/environment/runtime.py): `submit_design` accepts `process`, `target_quantity`, `max_unit_cost`, `force_submit`.
- [env_adapter.py:L116](src/agent/tools/env_adapter.py): `submit_design` only accepts `control_path` and implicit `tool_runtime`.

**The Smell:**
- **Artificial Capability Limiting**: The agent is physically unable to specify "I want to submit this for injection molding" or "I want to produce 1000 units". It is forced to use the defaults (3D printing, 1 unit) defined in the runtime method signature.
- **Hidden Constraints**: The agent cannot be budget-conscious if it can't pass `max_unit_cost` or adjust quantity to amortize costs.

**Planned Resolution:**
- **Expose Arguments**: Update the `env_adapter.py` definition of `submit_design` to accept optional `process`, `quantity`, and `max_unit_cost` arguments and pass them to `_execute_tool`.
- **Validation**: Ensure `ToolRuntime` handles these arguments correctly (which it seems to do).

**User Review:**

<!-- User comments here -->

### 3. Duplicated File Access Logic (Workspace vs ToolRuntime)

**Problem:** The `read_journal` tool utilizes a separate `Workspace` utility class instead of the unified `ToolRuntime`, leading to duplicated file I/O logic and potential security bypasses.

**Evidence:**
- [memory.py:L10](src/agent/tools/memory.py): `workspace = Workspace(root_dir="workspace")`
- [runtime.py:L26](src/environment/runtime.py): `self.workspace_dir = ...`

**The Smell:**
- **Bypassing the Sandbox**: `Workspace` reads files directly from the host filesystem. While `ToolRuntime` also does this for now (mostly), `ToolRuntime` is the designated gatekeeper for the environment. Bypassing it makes it harder to enforce policies or migrate to a fully containerized storage model later.
- **Inconsistent Paths**: `Workspace` defaults to `.workspace` (if instantiated without args, though `memory.py` passes `workspace`) while `ToolRuntime` logic is centered around `workspace`.

**Planned Resolution:**
- **Deprecate Workspace**: Remove `src/agent/utils/workspace.py` if it's only used by `read_journal`.
- **Migrate Tool**: Update `read_journal` to use `ToolRuntime.view_file` (or a specialized `read_journal` method on Runtime) to access the journal, leveraging the existing `tool_runtime` injection pattern.

**User Review:**

<!-- User comments here -->

---

## 🟡 Minor Issues & Smells

### 4. Implicit Runtime Fallback & Global State

**Problem:** `runner.py` relies on an implicit fallback mechanism to initialize the environment, rather than explicit configuration.

**Evidence:**
- [runner.py](src/agent/runner.py): Does not instantiate `ToolRuntime`.
- [env_adapter.py:L28](src/agent/tools/env_adapter.py): `_get_fallback_runtime()` lazy-loads a default runtime if none is found.

**The Smell:**
- **Hidden Configuration**: It is not obvious from reading `runner.py` where the workspace directory is defined or how the environment is set up.
- **Testing Difficulty**: It is harder to test `runner.py` with a mock runtime because the runtime creation is buried in a global fallback within a module that `runner.py` doesn't even directly import (it imports `build_graph` which uses `tools` which uses `env_adapter`).

**Planned Resolution:**
- **Explicit Setup**: `runner.py` should instantiate `ToolRuntime` (configured via CLI args or config) and register it using `register_runtime`, or pass the runtime ID in the `initial_state` to the graph.

**User Review:**

<!-- User comments here -->
