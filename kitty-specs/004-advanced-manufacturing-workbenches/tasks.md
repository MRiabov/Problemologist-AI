# Work Packages: Advanced Manufacturing Workbenches

## Work Package 1: Foundation & Shared Utilities
**Goal**: Set up dependencies, configuration system, and shared geometric analysis infrastructure.
**Priority**: High (Blocker)
**Tests**: `tests/test_analysis_utils.py`
**Estimated Prompt Size**: ~300 lines

- [x] **T001**: Add dependencies (`trimesh[easy]`, `pyyaml`) to `pyproject.toml`.
- [x] **T002**: Create default `src/workbenches/manufacturing_config.yaml`.
- [x] **T003**: Create `src/workbenches/analysis_utils.py` with `part_to_trimesh` conversion.
- [x] **T004**: Implement configuration loading utility in `analysis_utils.py`.
- [x] **T005**: Write unit tests for mesh conversion and config loading.

## Work Package 2: Geometric Analysis Algorithms
**Goal**: Implement core DFM analysis algorithms (Draft, Undercut, Wall Thickness) using `trimesh`.
**Priority**: High (Core Logic)
**Tests**: `tests/test_analysis_algorithms.py`
**Estimated Prompt Size**: ~400 lines

- [x] **T006**: Implement `check_draft_angle(mesh, pull_vector, min_angle)` in `analysis_utils.py`.
- [x] **T007**: Implement `check_undercuts(mesh, pull_vector)` using raycasting in `analysis_utils.py`.
- [x] **T008**: Implement `check_wall_thickness(mesh, min, max)` (sampling approach) in `analysis_utils.py`.
- [x] **T009**: Write comprehensive tests for each analysis algorithm with simple geometry (Cube, Cylinder).

## Work Package 3: CNC Workbench Implementation
**Goal**: Implement the CNC Workbench class with specific validation and cost modeling.
**Priority**: Medium
**Tests**: `tests/test_workbench_cnc.py`
**Estimated Prompt Size**: ~350 lines

- [x] **T010**: Create `src/workbenches/cnc.py` inheriting from base Workbench.
- [x] **T011**: Implement `CNCWorkbench.validate_geometry` using utils (Undercut, Tool Access).
- [x] **T012**: Implement `CNCWorkbench.calculate_cost` (Material + Run + Setup) using config.
- [x] **T013**: Write tests for CNC validation (pass/fail cases) and cost calculation.

## Work Package 4: Injection Molding Workbench Implementation
**Goal**: Implement the Injection Molding Workbench class with specific validation and cost modeling.
**Priority**: Medium
**Tests**: `tests/test_workbench_im.py`
**Estimated Prompt Size**: ~350 lines

- [x] **T014**: Create `src/workbenches/injection_molding.py` inheriting from base Workbench.
- [x] **T015**: Implement `InjectionMoldingWorkbench.validate_geometry` using utils (Draft, Undercut, Thickness).
- [x] **T016**: Implement `InjectionMoldingWorkbench.calculate_cost` (Tooling + Unit) using config.
- [x] **T017**: Write tests for IM validation (pass/fail cases) and cost calculation.

## Work Package 5: Tool Integration & Caching
**Goal**: Expose the manufacturability check as a high-level agent tool with caching for performance.
**Priority**: Medium
**Tests**: `tests/test_tool_integration.py`
**Estimated Prompt Size**: ~300 lines

- [x] **T018**: Update `src/environment/tools.py` to import new workbenches.
- [x] **T019**: Implement `check_manufacturability` function with `lru_cache` on mesh hash.
- [x] **T020**: Ensure tool output matches the JSON schema contract.
- [x] **T021**: Verify end-to-end flow with a new integration test case.
