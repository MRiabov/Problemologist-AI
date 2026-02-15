# Problemologist-AI Codebase Architecture Review #9 (Feb 15)

This document identifies **NEW** architectural smells and questionable decisions discovered during the ninth deep-dive review of the codebase on **February 15, 2026**.

---

## 🔴 Critical Architectural Issues

### 1. Ubiquitous `dict` Usage Violates Pydantic Rule

**Problem:** Despite a strict rule to use Pydantic models for data exchange, several core structures still rely on freeform `dict[str, Any]`.

**Evidence:**

- [state.py:L30](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/state.py#L30): `plan: dict[str, Any] | None`
- [models.py:L34](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/models.py#L34): `custom_objectives: dict[str, Any]`
- [models.py:L26](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/models.py#L26): `metadata: dict[str, Any]`
- [schemas.py:L455](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/models/schemas.py#L455): `final_assembly: list[SubassemblyEstimate | dict[str, AssemblyPartConfig]]`

**The Smell:**

- **Lack of Validation**: Errors in plan generation or custom objective application will only be caught at runtime, often deep in simulation logic.
- **Observability Gap**: Structured schemas are required for logging to our local observability database.
- **Inconsistency**: While some parts (like `ObjectivesYaml`) are well-modeled, others nearby remain completely untyped.

**User Review:**

---

### 2. Manual Tool Loops in Agent Nodes

**Problem:** `coder_node` and `reviewer_node` implement manual ReAct-style loops for tool calling instead of leveraging standard abstractions like `create_react_agent` or LangGraph's built-in tool executor.

**Evidence:**

- [nodes.py:L263-294](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/nodes.py#L263-294) (`coder_node`)
- [nodes.py:L560-590](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/nodes.py#L560-590) (`reviewer_node`)

**The Smell:**

- **Duplication**: The loop logic (invoking tools, appending messages, handling unknown tools) is duplicated across nodes.
- **Inflexibility**: Manual loops are harder to instrument or modify (e.g., adding parallel tool execution).
- **Maintenance Burden**: Changes to tool calling conventions must be updated in multiple places.

**User Review:**

---

## 🟠 Moderate Design Issues

### 3. SRP Violation in `_execute_graph_streaming`

**Problem:** This function handles too many responsibilities: LangGraph iteration, internal state mapping, session status tracking, and direct DB (Episode) persistence.

**Evidence:**

- [graph.py:L78-173](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/graph.py#L78-173)

**The Smell:**

- **Implicit Coupling**: The "business logic" of the graph is tightly coupled to the persistence layer (SQLAlchemy).
- **Hard to Test**: Testing the graph execution flow requires a database connection or heavy mocking of the persistence logic.
- **Complexity**: The nested loops and branching logic for status updates make it difficult to follow.

**User Review:**

---

### 4. Brittle JSON Parsing for LLM Responses

**Problem:** `planner_node` uses a broad regex to extract the first JSON-like structure from LLM output.

**Evidence:**

- [nodes.py:L160](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/benchmark/nodes.py#L160): `re.search(r"\{.*\}", content, re.DOTALL)`

**The Smell:**

- **Fragility**: If the LLM includes multiple JSON blocks (e.g., in a thought process) or preamble text with braces, the regex might fail or pick the wrong block.
- **Better Alternatives**: Should use LangChain's `with_structured_output` or a proper recursive JSON parser.

**User Review:**

---

## 🟡 Minor Issues & Smells

### 5. Hardcoded Eval Verification Logic

**Problem:** `run_evals.py` contains hardcoded checks for specific task IDs (like `bp-011`).

**Evidence:**

- [run_evals.py:L129-152](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/evals/run_evals.py#L129-152)

**The Smell:**

- **Lack of Scalability**: Adding new benchmarks requires modifying the runner script.
- **De-localization**: The definition of "success" for a benchmark should live with the benchmark dataset or within a dedicated verification module.

**User Review:**

---

### 6. Empty Simulation Schemas

**Problem:** `shared/simulation/schemas.py` is an empty file.

**Evidence:**

- `shared/simulation/schemas.py`

**The Smell:**

- **Broken Promise**: The file structure suggests structured schemas for simulation results, but developers are left with nothing or forced to use `dict[str, Any]` in `backends.py`.

**User Review:**

---

### 7. Repeated sys.path hacks

**Problem:** `run_evals.py` uses `sys.path.append` to find the project root.

**Evidence:**

- [run_evals.py:L11](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/evals/run_evals.py#L11)

**The Smell:**

- **Environment Fragility**: This assumes a specific directory structure and can fail if run from different CWDs. Use proper package installation (`pip install -e .`) instead.

**User Review:**

---

## Summary

| Issue | Severity | Effort |
|-------|----------|--------|
| 1. Ubiquitous `dict` usage | 🔴 Critical | High |
| 2. Manual Tool Loops | 🔴 Critical | Medium |
| 3. SRP Violation in graph runner | 🟠 Moderate | Medium |
| 4. Brittle JSON Parsing | 🟠 Moderate | Low |
| 5. Hardcoded Eval Verification | 🟡 Minor | Low |
| 6. Empty Simulation Schemas | 🟡 Minor | Trivial |
| 7. sys.path hacks | 🟡 Minor | Trivial |

---

*Review conducted on 2026-02-15 by Gemini (Antigravity)*
