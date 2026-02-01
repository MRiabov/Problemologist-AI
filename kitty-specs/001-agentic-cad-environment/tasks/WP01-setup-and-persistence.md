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
lane: "done"
dependencies: []
assignee: "Antigravity"
agent: "Antigravity"
shell_pid: "72514"
review_status: "has_feedback"
reviewed_by: "MRiabov"
history:
  - timestamp: "{{TIMESTAMP}}"
    lane: "planned"
    assignee: "Antigravity"
agent: "system"
    action: "Prompt generated via /spec-kitty.tasks"
---

# Work Package Prompt: WP01 – Setup, Persistence & Foundation

## Objectives & Success Criteria

- **Goal**: Initialize the project structure, install detailed dependencies, and implement the SQLite logging infrastructure which is critical for the "Data Engine" aspect of the feature.
- **Success Criteria**:
  - Project structure exists with `src/` folders.
  - `uv` or `poetry` environment is active with `build123d`, `mujoco`, `sqlalchemy`, and `alembic` installed.
  - `persistence.py` defines SQLAlchemy models for Episodes/Steps/Artifacts.
  - Alembic is initialized and an initial migration is generated.
  - Tests verify that data can be persisted and retrieved via SQLAlchemy sessions.

## Context & Constraints

- **Spec**: `kitty-specs/001-agentic-cad-environment/spec.md`
- **Plan**: `kitty-specs/001-agentic-cad-environment/plan.md`
- **Architecture**: Single `src/` directory.
- **Persistence**: Using SQLAlchemy ORM for robust data mapping and Alembic for migrations.

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
    - `sqlalchemy`
    - `alembic`
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

### Subtask T004 – Implement SQLAlchemy Models

- **Purpose**: Define the ORM mapping for logging agent interactions.
- **Steps**:
  - Create `src/environment/persistence.py`.
  - Define `Base` using `DeclarativeBase`.
  - Define models:
    - `Episode`: `id` (UUID pk), `problem_id` (str), `start_time` (DateTime), `status` (str), `result_metrics` (JSON).
    - `Step`: `id` (UUID pk), `episode_id` (fk), `sequence_index` (int), `tool_name` (str), `tool_input` (text), `tool_output` (text), `duration_ms` (int).
    - `Artifact`: `id` (UUID pk), `step_id` (fk), `artifact_type` (str), `file_path` (str), `content_hash` (str).
- **Files**: `src/environment/persistence.py`.
- **Parallel?**: No.
- **Notes**: Use Relationship for easy navigation between Episodes/Steps/Artifacts.

### Subtask T005 – Initialize Alembic

- **Purpose**: Manage database schema versions.
- **Steps**:
  - Run `alembic init src/migrations`.
  - Configure `alembic.ini` and `env.py` to point to the SQLAlchemy models in `persistence.py`.
  - Create the initial migration: `alembic revision --autogenerate -m "Initial schema"`.
- **Files**: `alembic.ini`, `src/migrations/*`.
- **Parallel?**: Yes.

### Subtask T006 – Implement Data Access Methods

- **Purpose**: Provide a clean session-based API for logging.
- **Steps**:
  - In `src/environment/persistence.py`, implement a `DatabaseManager` or similar:
    - `session_factory` configured for SQLite (`history.db`).
    - Methods to add/update episodes, steps, and artifacts using SQLAlchemy sessions.
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
- 2026-01-31T21:18:41Z – Antigravity – shell_pid=8493 – lane=planned – Moved to planned
- 2026-02-01T07:06:53Z – Antigravity – shell_pid=63746 – lane=doing – Started implementation via workflow command
- 2026-02-01T07:16:08Z – Antigravity – shell_pid=63746 – lane=for_review – Complete implementation with SQLAlchemy and Alembic. All tests passing.
- 2026-02-01T07:28:41Z – Antigravity – shell_pid=72514 – lane=doing – Started review via workflow command
- 2026-02-01T07:33:10Z – Antigravity – shell_pid=72514 – lane=done – Review passed: Implementation follows spec and plan. Persistence layer verified with tests. Alembic initialized correctly.
