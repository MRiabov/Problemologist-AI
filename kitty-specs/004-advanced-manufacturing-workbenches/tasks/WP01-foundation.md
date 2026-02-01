---
work_package_id: "WP01"
title: "Foundation & Shared Utilities"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004", "T005"]
---

### Objective
Set up the project environment with necessary dependencies, configuration files, and the base utility module for geometric analysis. This lays the groundwork for the advanced manufacturing workbenches.

### Context
We are building CNC and Injection Molding support. This requires `trimesh` for analysis and a flexible configuration system for costs. We need a central place (`analysis_utils.py`) to handle geometry conversion (`build123d` -> `trimesh`) and config management.

### Subtask T001: Add Dependencies
**Purpose**: Install required libraries.
**Steps**:
1.  Edit `pyproject.toml`.
2.  Add `trimesh[easy]>=4.0.0` to dependencies (ensure `scipy` and `networkx` come with it for graph ops).
3.  Add `pyyaml>=6.0` to dependencies.
4.  Run `uv sync` (or equivalent) to update lockfile (agent should use `run_shell_command`).

### Subtask T002: Create Default Config
**Purpose**: Define the cost and constraint parameters in a hot-swappable file.
**Steps**:
1.  Create `src/workbenches/manufacturing_config.yaml`.
2.  Populate it with the schema defined in `research.md` (Section 2).
    *   Include sections for `defaults`, `cnc`, and `injection_molding`.
    *   Set default materials (Al6061, ABS) and costs.

### Subtask T003: Create Analysis Utils & Mesh Conversion
**Purpose**: Create the shared utility module and implement the critical `part_to_trimesh` function.
**Steps**:
1.  Create `src/workbenches/analysis_utils.py`.
2.  Implement `part_to_trimesh(part: Part) -> trimesh.Trimesh`:
    *   Use `export_stl` from `build123d` to an in-memory buffer (`io.BytesIO`).
    *   Load with `trimesh.load(buffer, file_type='stl')`.
    *   Ensure the function returns a valid `trimesh.Trimesh` object.

### Subtask T004: Implement Config Loading
**Purpose**: Load the YAML config as a Python dictionary/object.
**Steps**:
1.  In `src/workbenches/analysis_utils.py`, add `load_config() -> dict`.
2.  It should read `src/workbenches/manufacturing_config.yaml`.
3.  Use `functools.lru_cache` so we don't hit the disk on every call.

### Subtask T005: Foundation Tests
**Purpose**: Verify the setup works.
**Steps**:
1.  Create `tests/test_analysis_utils.py`.
2.  Test `part_to_trimesh`: Create a simple `Box`, convert it, assert `mesh.is_watertight` and vertex count.
3.  Test `load_config`: Assert it returns a dict and contains expected keys (e.g., `cnc`, `materials`).

### Definition of Done
*   `pyproject.toml` has new deps.
*   `manufacturing_config.yaml` exists with valid data.
*   `analysis_utils.py` exists with conversion and config loading.
*   Tests pass.
