# Problemologist-AI Codebase Architecture Review #4

This document identifies **NEW** architectural smells and questionable decisions discovered during the fourth deep-dive review of the codebase.

---

## 🔴 Critical Architectural Issues

### 1. Asymmetric Tool Dispatch & "The Great Split"

**Problem:** The tool execution logic is split across three layers with inconsistent responsibilities, specifically regarding the `submit_design` tool.

**Evidence:**

- `env_adapter.py`: If `_ACTIVE_ENV` (global) is set, it calls `CADEnv.dispatch`. Otherwise, it uses `ToolRuntime.dispatch`.
- `CADEnv.dispatch`: Explicitly handles `submit_design` (150+ lines) but delegates everything else to `ToolRuntime`.
- `ToolRuntime.dispatch`: **Does not contain `submit_design`**.

**The Smell:**

- **Hidden Dependency**: The agent cannot "submit" anything unless a legacy global `CADEnv` is instantiated and registered. This breaks the "Stateless Tools" goal where a `ToolRuntime` should be self-contained.
- **Inconsistent Arguments**: Some tools take `filename`, others `path`, others `control_path`, often referring to the same thing (`design.py`), but with different keys in the `dispatch` maps.

**Planned Resolution:**

- **Atomic ToolRuntime**: Remove `_ACTIVE_ENV` and `set_active_env` entirely. Tools must be async-ready and support multiple concurrent environments.
- **Consolidated Dispatch**: Move the `submit_design` logic into `ToolRuntime` (likely delegating to a shared `Evaluator` component).
- **Uniform API**: Align argument names across `dispatch` and `@tool` definitions.

---

### 2. Host-Side Execution & Security Escalation

**Problem:** `ToolRuntime` and `benchmark/manager.py` perform high-risk operations on the host instead of the sandbox.

**Evidence:**

- `ToolRuntime.run_skill_script`: Calls `subprocess.run([sys.executable, str(path)])` on the host.
- `ToolRuntime.update_skill`: Allows an agent to write arbitrary content to these skill scripts.
- `ToolRuntime.init_skill` / `package_skill`: Run host-side scripts.

**The Smell:**

- **Remote Code Execution (RCE)**: An agent (or a compromised prompt) can use `update_skill` to modify a script and then `run_skill_script` to execute it on the host with the same permissions as the application. **Everything must run in the sandbox.**

**Planned Resolution:**

- **Sandboxed Skills**: All skill-related script execution must move into the Podman sandbox.
- **Skill Persistence & Sync**: Ensure skills updated via `update_skill` are correctly persisted and synced back to the host's `SKILLS_DIR` for future runs, but only after passing automated validation/linting within the sandbox.

---

### 3. Shared Test Workspace Collision

**Problem:** Test suites use hardcoded, non-unique directory names for their workbenches and environments.

**Evidence:**

- `tests/test_core.py`: Uses `Path("test_workspace")` and performs `shutil.rmtree` in the fixture.
- `tests/test_simulation.py`: Likely uses similar fixed paths.

**The Smell:**

- **Parallel Execution Failure**: Running tests with `pytest -n auto` will lead to race conditions where one test's teardown deletes another test's active workspace. This is a blocker for CI/CD speed.

**Planned Resolution:**

- **Unique Test Workspaces**: Use `pytest`'s built-in `tmp_path` fixture or generate unique workspace names using `uuid.uuid4()` to ensure isolation during parallel test runs.

---

## 🟠 Moderate Design Issues

### 4. Brittle Re-planning & Request Extraction

**Problem:** The `planner_node` uses simplistic string matching for critical flow control.

**Evidence:**

- **Re-plan trigger**: `if any(x in msg.content for x in ["Replan", "budget", "cost", "HARD_LIMIT"]): needs_replan = True`
- **Request extraction**: It stops at the *first* `HumanMessage` found in the history, ignoring subsequent refinements or conversation context.

**The Smell:**

- **Fragile Interaction**: If a user provides context in a second message, the planner ignores it. If the critic uses the word "budget" in a positive context, it might trigger an unnecessary re-plan.

**Planned Resolution:**

- **Review Protocol**: Persist critic feedback into a structured markdown artifact (with YAML frontmatter specifying relevant tools/metrics).
- **Context Injection**: Concat this feedback directly to the agent's next prompt. This simplifies the agent's reasoning by making previous "review notes" an explicit part of the current instruction set.

### 5. Benchmark Generator "Marker Protocol" & Isolation

**Problem:** `src/generators/benchmark/manager.py` relies on string parsing and shared temp directories.

**Evidence:**

- `execute_build` uses `.agent_storage/temp_assets` as a fixed path.
- It parses MJCF XML from stdout using `BUILD_RESULT:` markers.
- It performs `shutil.move` from a shared temp directory to final scenario folders.

**The Smell:**

- **Race Conditions**: Generating multiple variations in parallel (or even in a tight loop if cleanup fails) risks cross-contamination between variations if they share the same temp directory.

**Planned Resolution:**

- **Isolated Generations**: Implement strictly unique temporary directories for every generator variation run (e.g., `tmp/gen_<uuid>`).
- **Structured IO**: Move away from stdout marker parsing towards file-based JSON output from the sandbox.

---

## 🟡 Minor Issues & Smells

### 6. Implicit "design.py" Hardcoding

- Several tools (`preview_design`, `submit_design`, `check_manufacturability`) default to `design.py`. While convenient, this is baked deep into the logic, making it difficult to support multi-part designs or complex project structures without significant refactoring.

### 7. Message Windowing Absence

- The agent sends the entire message history to the LLM on every turn. As a designer agent generates code and receives long error logs/linting reports, the context window will quickly bloat, increasing cost and reducing reasoning quality.

### 8. Shadow Global Persistence

- `set_current_role` is still called in every node but is now a `no-op` in `env_adapter.py`. This is vestigial code that should be replaced by proper context-aware logging or removed.

### 9. Lack of Standardized Logging

- The codebase uses a mix of `print()`, `log_to_env()`, and sporadic `logging.getLogger(__name__)`.
- There is no central configuration for log formats, levels, or sinks (e.g., file vs. console).

**The Smell:**

- **Observability Gap**: It's difficult to filter logs by severity or component. Debugging production issues is harder without a consistent log stream.

**Planned Resolution:**

- **Structured Logging with `structlog`**: Implement structured logging using `structlog` for rich, machine-readable logs that include context (e.g., agent role, step count, episode ID) by default.
- **Unified Configuration**: Centralize configuration in `src/utils/logging.py` to handle both console (pretty-printed) and file (JSON) output.
- **Component Loggers**: Replace all `print()` and `log_to_env()` calls with calls to appropriately bound `structlog` loggers.

---

## 📊 Summary Table

| Issue | Severity | Effort | Risk |
|-------|----------|--------|------|
| Asymmetric Dispatch | 🔴 High | Medium | Inconsistency, Bugs |
| Host-side Escalation | 🔴 High | High | **Security Vulnerability** |
| Test Collisions | 🔴 High | Low | Flaky CI, Slow Tests |
| Brittle Planner Logic| 🟠 Medium | Low | Bad Reasoning |
| Generator Isolation | 🟠 Medium | Medium | Flaky Generation |
| Context Windowing | 🟡 Low | Medium | High Latency/Cost |
| design.py Hardcoding | 🟡 Low | Low | Low Flexibility |
| Standardized Logging | 🟡 Low | Low | Observability Debt |

---

## Recommendations

1. **Unify Dispatch**: Move `submit_design` logic into `ToolRuntime` (or a dedicated `Evaluator` it consumes) so it can function without `CADEnv`.
2. **Sanitize Skills**: Role-based access or strictly sandboxed execution for ANY script-related tool.
3. **Unique Workspaces**: Use `uuid.uuid4()` or `pytest` `tmp_path` for ALL environment and generator workspaces.
4. **Structured Signals**: Replace string matching in nodes with structured state flags or specific message types.
5. **Context Management**: Implement a message trimmer/summarizer for the graph history.
