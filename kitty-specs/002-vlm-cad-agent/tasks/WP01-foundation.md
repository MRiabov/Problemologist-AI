---
work_package_id: "WP01"
title: "Foundation & Infrastructure"
lane: "done"
dependencies: []
subtasks:
  - "T001"
  - "T002"
  - "T003"
  - "T004"
  - "T005"
agent: "Antigravity"
reviewed_by: "MRiabov"
review_status: "approved"
shell_pid: "90978"
---

## Objective

Establish the foundational infrastructure for the VLM CAD Agent using the **DeepAgents** framework (based on `langgraph`). This includes setting up the Python project structure for a graph-based agent, installing LangChain ecosystem dependencies, and defining the core `AgentState` and Tool interfaces.

## Context

We are migrating from a custom ReAct loop to a robust Graph Architecture. WP01 focuses on the "skeleton" of this graph: the environment setup, the state definition (TypedDict), and the integration of the Environment's tools into the LangChain ecosystem.

## Subtasks

### T001: Scaffold Project Structure

**Purpose**: Create the directory hierarchy for the Graph-based agent.
**Steps**:

1. Create the following directory structure inside `src/agent/`:

   ```text
   src/agent/
   ├── graph/
   │   ├── nodes/
   │   ├── __init__.py
   │   ├── state.py       # AgentState definition
   │   └── graph.py       # Main GraphBuilder
   ├── tools/
   │   ├── __init__.py
   │   └── env.py         # Wrappers for Spec 001 tools
   ├── utils/
   │   ├── __init__.py
   │   └── config.py      # LLM Configuration
   └── __init__.py
   ```

2. Ensure `__init__.py` files are present.

### T002: Install Dependencies

**Purpose**: Install LangChain and DeepAgents ecosystem libraries.
**Steps**:

1. Add the following to `pyproject.toml` (or `requirements.txt`):
   - `langgraph`: Orchestration.
   - `deepagents`: (Presumed) Framework utilities.
   - `langchain-core`: Base abstractions.
   - `langchain-openai`: ChatModels for OpenAI.
   - `langchain-google-genai`: ChatModels for Gemini.
   - `langchain-anthropic`: ChatModels for Claude.
   - `pydantic`: Data validation (v2).
   - `rich`: Terminal UI.
2. Install dependencies.

### T003: Configure LangChain ChatModels

**Purpose**: Setup the ChatModel factory to easily switch between GPT-4o, Gemini 1.5 Pro, and Claude 3.5 Sonnet.
**Steps**:

1. Create `src/agent/utils/llm.py`.
2. Implement a factory function `get_model(model_name: str, temperature: float) -> BaseChatModel`.
3. Support environment variables for API keys (`OPENAI_API_KEY`, `GOOGLE_API_KEY`, etc.).

### T004: Define AgentState TypedDict

**Purpose**: Define the shared state that flows through the Graph.
**Steps**:

1. Create `src/agent/graph/state.py`.
2. Define `AgentState(TypedDict)` containing:
   - `messages`: Annotated[list[AnyMessage], add_messages] (Standard LangGraph history).
   - `plan`: str (The current implementation plan).
   - `step_count`: int (For safety limits).
   - `scratchpad`: dict (For intermediate node data).

### T005: Wrap Environmental Tools

**Purpose**: Convert Spec 001 JSON-schema tools into LangChain `BaseTool` or `@tool` functions.
**Steps**:

1. Create `src/agent/tools/env.py`.
2. Implement wrapper functions using the `@tool` decorator for:
   - `write_script(content: str, path: str)`
   - `edit_script(find: str, replace: str, path: str)`
   - `preview_design()`
   - `submit_design(control_path: str)`
   - `search_docs(query: str)`
3. Ensure each tool has a precise docstring (for LLM tool calling).

## Files to Create

- `src/agent/graph/state.py`
- `src/agent/utils/llm.py`
- `src/agent/tools/env.py`

## Acceptance Criteria

- [ ] `AgentState` matches the architectural needs.
- [ ] `get_model("gpt-4o")` returns a working ChatOpenAI instance.
- [ ] Environment tools are bindable to the model (`model.bind_tools([...])`).

## Activity Log

- 2026-02-01T08:29:17Z – Antigravity – shell_pid=105353 – lane=doing – Started implementation via workflow command
- 2026-02-01T08:33:16Z – Antigravity – shell_pid=105353 – lane=for_review – Ready for review: Foundation established with LangGraph state, tools and model factory
- 2026-02-01T08:34:11Z – Antigravity – shell_pid=90978 – lane=doing – Started review via workflow command
- 2026-02-01T08:35:35Z – Antigravity – shell_pid=90978 – lane=done – Review passed: Foundation established with LangGraph state, tools and model factory. All verification checks passed.
