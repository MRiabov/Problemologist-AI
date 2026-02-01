---
work_package_id: "WP01"
title: "Foundation & Validator"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004", "T005"]
---

# Work Package 1: Foundation & Validator

## Objective
Establish the directory structure for the Benchmark Scenario Generator and implement the critical `Validator` component. The Validator is the "Judge" that ensures any generated simulation is geometrically sound and physically stable.

## Context
This is a standalone utility package within the repo. It uses `build123d` for geometry and `mujoco` for physics checks. We need a solid foundation for file management (staging vs. production benchmarks) and a robust validation loop before we bring in the AI agent.

## Subtasks

### T001: Scaffold Package Structure
**Purpose**: Create the module layout.
**Steps**:
1. Create `src/generators/benchmark/` directory.
2. Create `src/generators/benchmark/__init__.py`.
3. Create `src/generators/benchmark/utils/__init__.py`.
4. Create `datasets/benchmarks/` and `datasets/staging/` directories in the project root (ensure `.gitkeep` is present).

### T002: Implement AssetManager
**Purpose**: Handle file paths and saving logic.
**File**: `src/generators/benchmark/utils/assets.py`
**Requirements**:
- Class `AssetManager(base_dir: Path)`.
- Method `get_staging_path(run_id: str) -> Path`: Returns a unique folder in staging.
- Method `save_mesh(mesh: build123d.Part, name: str, folder: Path)`: Exports STL.
- Method `save_mjcf(xml_content: str, name: str, folder: Path)`: Saves XML.
- Ensure all paths are absolute and resolved.

### T003: Implement Validator Logic
**Purpose**: The core physics sanity check.
**File**: `src/generators/benchmark/validator.py`
**Requirements**:
- Class `Validator`.
- Method `validate_mesh(stl_path: Path) -> bool`:
  - Check if file exists and size > 0.
  - Optional: Use `trimesh` (if available) or `build123d` to check if watertight. For now, simple existence/size is MVP.
- Method `validate_physics(mjcf_path: Path, sim_duration: float = 1.0) -> ValidationReport`:
  - Load model: `mujoco.MjModel.from_xml_path()`.
  - Create data: `mujoco.MjData(model)`.
  - Step simulation loop for `sim_duration` seconds (default step size).
  - **Stability Check**: At each step, check `data.qvel` (velocity). If max(abs(qvel)) > 100.0 (arbitrary high limit), return `failed=True, reason="Explosion detected"`.
  - Return `success=True` if loop completes.
- Handle `mujoco.FatalError` (e.g., bad XML syntax) by catching exceptions and returning fail.

### T004: Create Test Fixtures
**Purpose**: Verify the validator works on known good/bad data.
**Steps**:
1. Create `tests/fixtures/scenarios/valid_box.xml` (A simple box on a plane).
2. Create `tests/fixtures/scenarios/exploding_box.xml` (Two boxes overlapping heavily, which should fly apart violently).
3. Create `tests/fixtures/scenarios/broken.xml` (Invalid XML tags).

### T005: Unit Tests
**Purpose**: Test the code.
**File**: `tests/generators/benchmark/test_validator.py`
**Tests**:
- `test_validator_valid_scene`: Load `valid_box.xml`, assert success.
- `test_validator_exploding_scene`: Load `exploding_box.xml`, assert failure (velocity check).
- `test_validator_syntax_error`: Load `broken.xml`, assert failure (exception caught).
- `test_asset_manager_paths`: specific tests for path resolution.

## Definition of Done
- `src/generators/benchmark` exists.
- `Validator` correctly identifies stable vs. unstable simulations.
- `pytest tests/generators/benchmark/` passes.
