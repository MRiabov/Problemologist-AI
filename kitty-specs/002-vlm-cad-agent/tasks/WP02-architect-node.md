---
work_package_id: WP02
title: Architect Node
lane: planned
dependencies: []
subtasks: [T005, T006, T007, T008, T009]
---

## Objective

Implement the Architect (Planner) node which is responsible for analyzing the user request and creating the initial execution plan and TODO list.

## Context

The Architect is the entry point of the agent. It runs once at the start (or when Critic rejects the entire approach). It reads available skills and produces `plan.md` and `todo.md`.

## Subtasks

### T005: Implement `src/agent/nodes/architect.py`

Create the Architect node function.

- **File**: `src/agent/nodes/architect.py`
- **Function**: `architect_node(state: AgentState) -> dict`
- **Logic**:
  - Retrieve `state["messages"]` (User request).
  - Call `prompt_manager` to get the system prompt.
  - Invoke LLM (Gemini/Claude).
  - Parse output.

### T006: Add logic to read `skills/` directory

The Architect needs to know what tools are available.

- **Implementation**:
  - In `architect_node`, use a helper (or `ls` tool logic) to list files in `skills/`.
  - specific skill content can be read if relevant context is needed (optional for V1).
  - Inject skill titles into the system prompt.

### T007: Implement 'create plan' logic

The Architect must write `plan.md`.

- **Implementation**:
  - The LLM should generate the plan content.
  - Use `write_file` tool (or helper) to write to `plan.md`.
  - Update `state["plan"]`.

### T008: Implement 'create todo' logic

The Architect must write `todo.md`.

- **Implementation**:
  - The LLM should generate the TODO list.
  - Use `write_file` tool to write to `todo.md`.
  - Update `state["todo"]`.

### T009: Test Architect node

Verify functionality.

- **File**: `tests/agent/test_architect.py`
- **Test**:
  - Mock the LLM response.
  - Verify `plan.md` and `todo.md` creation (mock filesystem or temporary dir).
  - Verify state update.

## Definition of Done

- Architect node correctly processes input and generates plan artifacts.
- Tests pass.
