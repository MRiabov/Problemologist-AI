---
work_package_id: WP03
title: Engineer Node
lane: "for_review"
dependencies: [WP01, Spec 001 WP04]
base_branch: 002-vlm-cad-agent-WP02
base_commit: c1e31ad9b1f20f12d0472b097b13834971910509
created_at: '2026-02-06T14:24:54.820068+00:00'
subtasks: [T010, T011, T012, T013, T014]
shell_pid: "690687"
agent: "gemini-cli"
review_status: "has_feedback"
reviewed_by: "MRiabov"
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

### T011: Configure `deepagents` tools

Use the `RemoteFilesystem` middleware and tools defined in Spec 001 WP04.

- **Implementation**:
  - `ls_tool`, `read_tool`, `write_tool`: Proxy to Worker via `RemoteFilesystem`.

### T012: Configure `exec_tool`

Execute code on the Worker via Spec 001 `runtime/execute` API.

- **Implementation**:
  - `exec_tool`: Wraps the Worker client's `execute` method via a **Temporal Client**.
  - Ensure the execution captures stdout/stderr.
  - For long-running scripts (>30s), dispatch a **Temporal Workflow** and poll for completion (as defined in Spec 001 WP06). This ensures durable execution across preemption.

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

## Activity Log

- 2026-02-06T14:24:55Z – Gemini – shell_pid=641430 – lane=doing – Assigned agent via workflow command
- 2026-02-06T14:36:57Z – Gemini – shell_pid=641430 – lane=for_review – Ready for review: Engineer node implemented with execution loop and worker integration.
- 2026-02-06T14:57:55Z – gemini-cli – shell_pid=673307 – lane=doing – Started review via workflow command
- 2026-02-06T15:05:29Z – gemini-cli – shell_pid=673307 – lane=for_review – Ready for review: Engineer node implemented.
- 2026-02-06T15:15:36Z – gemini-cli – shell_pid=690687 – lane=doing – Started review via workflow command
- 2026-02-06T15:17:11Z – gemini-cli – shell_pid=690687 – lane=planned – Moved to planned
- 2026-02-06T15:31:46Z – gemini-cli – shell_pid=690687 – lane=planned – Moved to planned
- 2026-02-06T16:17:59Z – gemini-cli – shell_pid=690687 – lane=for_review – Ready for review: Implemented Engineer Node with robust tool binding, proper WorkerClient integration, and retry execution loop. Verified with tests.
