# Problemologist-AI Codebase Architecture Review #2

Following the initial triage (Volume #1), this document focuses on the remaining critical architectural smells and outlines a roadmap toward the **Functional Paradigm** preferred by the project.

---

## 🔴 Remaining Critical Issues

### 1. The "Shadow Global" Pattern in `env_adapter.py`

Despite moving core tools into `ToolRuntime`, the bridge between the Agent and the Environment still relies on implicit global state.

**Evidence:**

- `src/agent/tools/env_adapter.py`: `_ACTIVE_ENV` and `_CURRENT_ROLE`.

**Why this is "Questionable":**

- It makes the "Active Environment" a side effect of initialization rather than a clear data dependency.
- It prevents running multiple agents (e.g., in a multi-agent debate or parallel benchmark generation) because they all share the same "Active" pointer.

**Functional Alternative:**
Pass the `env` or `runtime` explicitly through the LangGraph state. The `ToolNode` should be configured with a specific runtime instance, not pull a global one.

---

## 🟠 Logic Schism: `CADEnv` vs `ToolRuntime`

The system has a split personality regarding its most complex task: **Design Submission**.

**Evidence:**

- `CADEnv._submit_design`: 150+ lines of runner script generation, costing, and simulation orchestration.
- `ToolRuntime.check_manufacturability`: 150+ lines of (nearly identical) runner script generation and costing.

**The Smell:**

- **Duplicated Complexity**: The logic for "how to analyze a build123d script" is defined in two disparate locations.
- **Maintenance Trap**: A change in the costing model or a fix in the runner template must be synchronized manually between these files.

**Recommendation:**
Consolidate the "Analysis Engine" into a single functional component that `CADEnv` uses for evaluation and `ToolRuntime` uses for agent-facing checks.

---

## 🟡 Brittle "Marker" Protocol

Communication between the host and the sandboxed runner scripts is currently based on string prefix matching.

**Evidence:**

- `if "SUBMIT_RESULT:" in stdout:`
- `if "ANALYSIS_RESULT:" in stdout:`
- `if "Validation Passed!" in content:`

**The Smell:**

- Very fragile. If a library being executed (like `build123d` or `mujoco`) prints a warning that happens to contain these strings or breaks the stdout flow, the parser fails.

**Recommendation:**
Use a structured approach. Have the runner scripts write their JSON results to a specific "result pipe" or a known file in the workspace, rather than relying on standard output parsing with magic strings.

---

## 🗺️ Roadmap to Functional Model

To achieve a cleaner, functional state:

1. **Refactor Node State**: Update `AgentState` to include an `env_ref` or `runtime_id`.
2. **Stateless Tools**: Refactor `@tool` functions to accept a runtime instance as an argument (or bind them at creation time in the node).
3. **Unify Evaluation**: Create an `Evaluator` class/module that handles the sandbox orchestration once, taking a script and returning a structured result.
4. **Prompt Deserialization**: Move hardcoded "MANDATORY" strings from `actor.py` and `planner.py` into the `prompts.yaml` templates.

---

## 📈 Current Status Summary

| Item | Phase | Priority |
| :--- | :--- | :--- |
| Global mutable state | Volume 1 (In Progress) | 🔴 High |
| Three-layer indirection | Volume 2 (Planned) | 🔴 High |
| Logic Duplication | Volume 2 (Planned) | 🟠 Medium |
| Marker Protocol | Volume 2 (Planned) | 🟡 Low |
