---
work_package_id: WP01
title: Foundation & State
lane: "doing"
dependencies: []
subtasks: [T001, T002, T003, T004]
agent: "Gemini-CLI"
shell_pid: "479850"
---

## Objective

establish the foundational structure for the Engineer Agent (Spec 002). This includes the project scaffolding, state definitions using `TypedDict`, and the initial graph skeleton.

## Context

The Engineer Agent is a LangGraph-based system running within `deepagents`. It requires a shared state object to pass data between nodes (Architect, Engineer, Critic).

## Subtasks

### T001: Implement `src/agent/state.py`

Create the `AgentState` definition.

- **File**: `src/agent/state.py`
- **Requirements**:
  - Inherit from `TypedDict`.
  - Keys:
    - `messages`: List[BaseMessage] (standard LangGraph).
    - `task`: str (User input).
    - `plan`: str (The content of plan.md).
    - `todo`: str (The content of todo.md).
    - `current_step`: str (Focus for the Engineer).
    - `journal`: str (Episodic memory).
    - `iteration`: int (Loop counter).

### T002: Implement `src/agent/graph.py` skeleton

Set up the `StateGraph` object.

- **File**: `src/agent/graph.py`
- **Requirements**:
  - Import `AgentState`.
  - Initialize `StateGraph(AgentState)`.
  - Add placeholder comment for nodes.
  - Compile the graph.

### T003: Implement `src/agent/prompt_manager.py`

Create a manager for Jinja2 templates.

- **File**: `src/agent/prompt_manager.py`
- **Requirements**:
  - Class `PromptManager`.
  - Method `render(template_name: str, **kwargs) -> str`.
  - Add initial template for Architect and Engineer (can be placeholders or basic strings).

### T004: Create `tests/agent/test_state.py`

Verify the state initialization.

- **File**: `tests/agent/test_state.py`
- **Requirements**:
  - Test that `AgentState` can be instantiated with default values (if applicable) or mocked data.
  - Ensure compatibility with LangGraph's expected structure.

## Definition of Done

- All files created.
- `uv run pytest tests/agent` passes.

## Activity Log

- 2026-02-06T08:05:25Z – Gemini-CLI – shell_pid=479850 – lane=doing – Started implementation via workflow command
