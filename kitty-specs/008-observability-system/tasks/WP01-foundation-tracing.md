---
work_package_id: "WP01"
title: "Foundation - Tracing & Persistence"
lane: "done"
dependencies: []
subtasks: ["T001", "T002", "T003", "T003c"]
agent: "Gemini"
shell_pid: "138061"
review_status: "has_feedback"
reviewed_by: "MRiabov"
---

# Work Package: Foundation - Tracing & Persistence

## Objective

Establish the core observability infrastructure by settings up LangFuse for distributed tracing of agent interactions and configuring a custom Postgres persistence layer for LangGraph to ensure durable agent state.

## Context

The observability system is critical for debugging, monitoring, and future training of the agents. We use **LangFuse** to capture traces of LLM calls, tool usage, and agent reasoning. **LangGraph** requires a persistence layer to save agent state (checkpoints), which we will back with **Postgres** (the same instance used by the controller, likely in a separate schema or table set).

## Implementation Guide

### Subtask T001: Implement `LangFuseCallbackHandler` and initialization logic

**Purpose**: Enable tracing of LangChain/LangGraph executions to LangFuse.

**Steps**:

1. Create `src/observability/tracing.py`.
2. Implement a `get_trace_callback()` function that returns a configured `CallbackHandler` from `langfuse`.
3. Ensure it reads configuration (Public/Secret keys, Host) from environment variables.
4. Add a decorator or context manager example for tracing non-LangChain functions if needed (using `@observe()` from langfuse SDK).

**Files**:

- `src/observability/tracing.py` (New)

**Validation**:

- Verify that `get_trace_callback()` returns a valid handler when env vars are set.
- Verify it handles missing env vars gracefully (e.g., returns None or a dummy handler, or raises specific config error).

---

### Subtask T002: Implement `PostgresSaver` customization for LangGraph

**Purpose**: Persist agent state to Postgres instead of in-memory.

**Steps**:

1. Create `src/observability/persistence.py`.
2. Implement a class (e.g., `ControllerCheckpointSaver`) that extends or utilizes `langgraph.checkpoint.postgres.PostgresSaver`.
3. Ensure it connects to the main application Postgres database using `sqlalchemy`.
4. Implement `setup_persistence()` async function that initializes the tables if they don't exist.

**Files**:

- `src/observability/persistence.py` (New)

**Validation**:

- Can instantiate the saver with a valid DB connection string.
- `setup_persistence()` runs without error on an empty DB.

---

### Subtask T003c: Configure `structlog`

**Purpose**: Ensure all logs are emitted as structured JSON for easy parsing by observability tools.

**Steps**:

1. Create `src/observability/logging.py`.
2. Configure `structlog` processors:
    - `TimeStamper(fmt="iso")`
    - `JSONRenderer()`
    - Add correlation IDs (trace_id) if available in context.
3. Replace standard `logging` handlers with structlog configuration.

**Files**:

- `src/observability/logging.py` (New)

**Validation**:

- Run a script that logs a message and verify output is valid JSON.

---

### Subtask T003: Write unit tests for tracing and persistence configuration

**Purpose**: Verify the foundation code works in isolation.

**Steps**:

1. Create `tests/observability/test_tracing.py`.
    - Test config loading.
    - Mock LangFuse SDK to verify initialization args.
2. Create `tests/observability/test_persistence.py`.
    - Use a temporary Postgres container (or `testing.postgresql` / `pytest-postgresql`) or mock the DB driver.
    - Test `setup_persistence()` creates expected tables.
    - Test saving and loading a simple checkpoint (mock serialized data).

**Files**:

- `tests/observability/test_tracing.py` (New)
- `tests/observability/test_persistence.py` (New)

**Validation**:

- Run `pytest tests/observability/` and ensure all pass.

## Definition of Done

- [ ] `src/observability/tracing.py` exists and correctly initializes LangFuse.
- [ ] `src/observability/logging.py` configures Structlog.
- [ ] `src/observability/persistence.py` exists and provides a Postgres checkpointer.
- [ ] Unit tests pass for both components.
- [ ] No hardcoded credentials (all from env).
- [ ] Code is linted and typed.

## Reviewer Guidance

- Check that `LangFuse` is not imported if the package is optional (though it should be a dep).
- Ensure Postgres connection pooling is considered (passing an engine vs connection).

## Activity Log

- 2026-02-06T08:07:50Z – gemini-cli – shell_pid=483300 – lane=doing – Started implementation via workflow command
- 2026-02-06T14:34:49Z – gemini-cli – shell_pid=483300 – lane=planned – Moved to planned
- 2026-02-06T15:51:53Z – gemini – shell_pid=424666 – lane=doing – Started implementation via workflow command
- 2026-02-06T15:59:18Z – gemini – shell_pid=424666 – lane=for_review – Ready for review: LangFuse tracing, PostgresSaver persistence, structlog - 17 tests passing
- 2026-02-06T15:59:21Z – gemini – shell_pid=734593 – lane=doing – Started review via workflow command
- 2026-02-06T16:26:46Z – gemini – shell_pid=734593 – lane=done – Review passed: Implemented tracing with LangFuse, Postgres persistence for LangGraph, and structured logging with structlog. 17 unit tests passed.
- 2026-02-07T06:21:18Z – Gemini – shell_pid=138061 – lane=doing – Started review via workflow command
- 2026-02-07T06:36:09Z – Gemini – shell_pid=138061 – lane=done – Review passed: LangFuse tracing, PostgresSaver persistence, and structured logging verified with 17 tests. T003b was not in the WP scope.
