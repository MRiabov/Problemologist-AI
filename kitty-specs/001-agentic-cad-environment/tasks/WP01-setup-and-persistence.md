---
work_package_id: "WP01"
subtasks:
  - "T001"
  - "T002"
  - "T003"
  - "T004"
  - "T005"
title: "Setup, Persistence & Foundation"
phase: "Phase 1 - Foundation"
lane: "doing"
dependencies: []
agent: "Antigravity"
shell_pid: "8493"
history:
  - timestamp: "{{TIMESTAMP}}"
    lane: "planned"
    agent: "system"
    action: "Prompt generated via /spec-kitty.tasks"
---

# Work Package Prompt: WP01 – Setup, Persistence & Foundation

## Objectives & Success Criteria

- **Goal**: Initialize the project structure, install detailed dependencies, and implement the SQLite logging infrastructure which is critical for the "Data Engine" aspect of the feature.
- **Success Criteria**:
  - Project structure exists with `src/` folders.
  - `uv` or `poetry` environment is active with `build123d` and `mujoco` installed.
  - `persistence.py` creates a database with Episodes/Steps/Artifacts tables.
  - Tests verify that data can be written to and read from the database.

## Context & Constraints

- **Spec**: `kitty-specs/001-agentic-cad-environment/spec.md`
- **Plan**: `kitty-specs/001-agentic-cad-environment/plan.md`
- **Architecture**: Single `src/` directory.
- **Persistence**: Using standard `sqlite3` for simplicity and portability.

## Subtasks & Detailed Guidance

### Subtask T001 – Create Project Directory Structure

- **Purpose**: Establish the standard folder layout defined in the plan.
- **Steps**:
  - Create directories:
    - `src/environment`
    - `src/workbenches`
    - `src/compiler`
    - `src/rag`
    - `tests/`
- **Files**: Directories only.
- **Parallel?**: No.
- **Notes**: Ensure identifying `__init__.py` files are created in each python package directory.

### Subtask T002 – Initialize Dependencies

- **Purpose**: Setup the python environment.
- **Steps**:
  - Initialize project (e.g., `uv init` or `poetry init`).
  - Add core dependencies:
    - `build123d`
    - `mujoco`
    - `gymnasium`
    - `numpy`
  - Add dev dependencies:
    - `pytest`
    - `ruff` (for linting)
- **Files**: `pyproject.toml`, `uv.lock` (or similar).
- **Parallel?**: No.

### Subtask T003 – Configure Tooling

- **Purpose**: Enforce code quality from the start.
- **Steps**:
  - Configure `ruff` or `black`/`isort` in `pyproject.toml`.
  - Create a simple test in `tests/test_hello.py` to verify imports work.
- **Files**: `pyproject.toml`, `tests/test_hello.py`.
- **Parallel?**: Yes.

### Subtask T004 – Implement Database Schema

- **Purpose**: Create the tables for logging agent interactions.
- **Steps**:
  - Create `src/environment/persistence.py`.
  - Define `init_db(db_path)` function.
  - Create tables:
    - `Episodes`: `id` (pk), `prompt` (text), `start_time` (iso string/timestamp), `result` (json/text).
    - `Steps`: `id` (pk), `episode_id` (fk), `tool_name` (text), `tool_input` (text), `tool_output` (text), `duration` (float).
    - `Artifacts`: `id` (pk), `step_id` (fk), `type` (text: code, render), `path` (text), `content` (blob/text).
- **Files**: `src/environment/persistence.py`.
- **Parallel?**: No.
- **Notes**: Use Foreign Keys.

### Subtask T005 – Implement Data Access Methods

- **Purpose**: Provide easy API for the environment to log events.
- **Steps**:
  - In `src/environment/persistence.py`, implement class `Logger` or helper functions:
    - `create_episode(prompt) -> episode_id`
    - `log_step(episode_id, tool_name, input, output, duration) -> step_id`
    - `save_artifact(step_id, type, content/path)`
    - `get_episode(episode_id)` (for testing)
- **Files**: `src/environment/persistence.py`.
- **Parallel?**: No.

## Test Strategy

- **Test File**: `tests/test_persistence.py`
- **Cases**:
  - Initialize DB -> Check file exists.
  - Create Episode -> Check row count.
  - Log Step with FK -> Check integration.
  - Retrieve data -> Check consistency.

## Risks & Mitigations

- **Concurrency**: SQLite is fine for single-threaded agent, but if parallel evals happen later, might need locking or WAL mode. Enable WAL mode by default (`PRAGMA journal_mode=WAL;`).

## Activity Log

- 2026-01-31T20:16:03Z – Antigravity – shell_pid=8493 – lane=doing – Started implementation via workflow command
