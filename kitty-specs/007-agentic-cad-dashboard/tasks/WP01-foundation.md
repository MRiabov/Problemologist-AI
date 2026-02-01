---
work_package_id: "WP01"
title: "Foundation & Data Layer"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004"]
agent: "Antigravity"
shell_pid: "313636"
review_status: "has_feedback"
reviewed_by: "MRiabov"
---

# WP01: Foundation & Data Layer

**Goal**: Initialize the dashboard project structure, install dependencies, and implement the data access layer for `history.db`.

**Implementation Command**: `spec-kitty implement WP01`

## Subtasks

### T001: Install Dependencies

**Purpose**: Ensure all required libraries are available.
**Steps**:

1. Add the following dependencies to `pyproject.toml` (if using uv/poetry) or install via pip:
   - `streamlit`
   - `stpyvista`
   - `pyvista`
   - `trimesh`
   - `sqlalchemy`
2. Run installation command (`uv sync` or `pip install ...`).
3. Verify installation by importing them in a test script.

### T002: Create Directory Structure

**Purpose**: Set up the module organization.
**Steps**:

1. Create `src/dashboard/` directory.
2. Create `src/dashboard/__init__.py`.
3. Create subdirectories:
   - `src/dashboard/components/`
   - `src/dashboard/assets/` (if needed)
4. Create empty component files:
   - `src/dashboard/components/__init__.py`
   - `src/dashboard/components/chat.py`
   - `src/dashboard/components/code.py`
   - `src/dashboard/components/viewer_3d.py`
   - `src/dashboard/components/sidebar.py`
5. Create `src/dashboard/main.py` (entry point).
6. Create `src/dashboard/data.py` (data layer).
7. Create `src/dashboard/utils.py` (utilities).

### T003: Implement Utils (`src/dashboard/utils.py`)

**Purpose**: Helper functions for path resolution.
**Steps**:

1. Implement `get_project_root()` returning the absolute path to the repo root.
2. Implement `resolve_artifact_path(relative_path: str) -> Path`:
   - Should join project root with the relative path stored in DB.
   - Verify file existence.

### T004: Implement Data Layer (`src/dashboard/data.py`)

**Purpose**: Read-only interface to `history.db` using SQLAlchemy.
**Steps**:

1. Import models (`Episode`, `Step`, `Artifact`) from `src.environment.persistence` (if available) OR define read-only mappings matching the schema.
2. Setup SQLAlchemy engine:
   - Connection string: `sqlite:///history.db` (ensure absolute path).
   - **Crucial**: Use WAL mode or ensure read-only pragmas to avoid locking the writer (Agent).
3. Implement query functions:
   - `get_all_episodes() -> List[Episode]`: Ordered by start time desc.
   - `get_episode_by_id(episode_id: str) -> Episode`: With steps loaded.
   - `get_step_artifacts(step_id: str) -> List[Artifact]`: Fetch associated files.
4. **Validation**: Create a small script `tests/test_dashboard_data.py` to fetch data from an existing DB (or mock/create a temp DB if none exists) and print one episode.

## Validation Strategy

- Run `uv run python src/dashboard/data.py` (or test script) and ensure it connects to the DB without errors.
- Verify `src/dashboard` structure exists.

## Activity Log

- 2026-02-01T13:22:02Z – Antigravity – shell_pid=322329 – lane=doing – Started implementation via workflow command
- 2026-02-01T13:24:26Z – Antigravity – shell_pid=322329 – lane=for_review – Foundation and data layer implemented with tests.
- 2026-02-01T13:33:57Z – Antigravity – shell_pid=313636 – lane=doing – Started review via workflow command
- 2026-02-01T13:37:34Z – Antigravity – shell_pid=313636 – lane=planned – Moved to planned
