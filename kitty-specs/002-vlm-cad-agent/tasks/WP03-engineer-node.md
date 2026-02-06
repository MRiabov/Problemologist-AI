---
work_package_id: WP03
title: Engineer Node
lane: planned
dependencies: []
subtasks: [T010, T011, T012, T013, T014]
---

## Objective

Implement the Engineer (Execution) node. This node picks items from the TODO list, writes code, and executes it on the Worker.

## Context

The Engineer is the workhorse. It writes `script.py` and executes it. It must handle syntax errors (by re-writing) and execution errors (by logging and potentially retrying or moving to Critic). Note that code execution happens on the Worker node via the environment API (Spec 001).

## Subtasks

### T010: Implement `src/agent/nodes/engineer.py`

Create the Engineer node function.

- **File**: `src/agent/nodes/engineer.py`
- **Function**: `engineer_node(state: AgentState) -> dict`
- **Logic**:
  - Parse `state["todo"]` to find the next active item.
  - Generate Python code using LLM.
  - Interact with tools.

### T011: Implement `view_file`, `write_file` tools

Wrappers for file operations.

- **Implementation**:
  - `view_file(path)`: Reads local file.
  - `write_file(path, content)`: Writes local file (which is synced to Worker/Sandbox).

### T012: Implement `run_command` tool wrapper

Execute code on the Worker.

- **Implementation**:
  - This MUST integrate with `001-agentic-cad-environment` API (or `deepagents` sandbox backend if configured).
  - For this task, assume we use `run_command("python script.py")`.
  - Ensure the command execution captures stdout/stderr.

### T013: Implement execution loop

The "Write -> Run -> Fix" loop.

- **Logic**:
  - 1. Write `script.py`.
  - 1. Run `script.py`.
  - 1. If exit code != 0:
    - Read stderr.
    - Feed back to LLM ("Fix this error").
    - Retry (up to N times).
  - 1. If success:
    - Update state (mark TODO done?) -> Pass to Critic.

### T014: Test Engineer node

Verify execution flow.

- **File**: `tests/agent/test_engineer.py`
- **Test**:
  - Mock `run_command` to fail once then succeed.
  - Verify retry logic.
  - Verify final success state.

## Definition of Done

- Engineer node can write and execute code.
- Retry logic handles basic errors.
