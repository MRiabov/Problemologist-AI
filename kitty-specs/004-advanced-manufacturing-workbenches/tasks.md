# Tasks: Advanced Manufacturing Workbenches

## Work Package 1: Foundation & Data Models

- **Goal**: Set up the core Pydantic data structures, configuration system, and functional interfaces for the workbench system.
- **Priority**: High (Blocking)
- **Independent Test**: `pytest tests/workbenches/test_config.py` passes and models can be instantiated.

### Included Subtasks

- [x] T001: Implement Core Pydantic Models (WorkbenchResult, DFMReport, ManufacturingMethod, MaterialDefinition).
- [x] T002: Create manufacturing_config.yaml with initial cost/material data.
- [x] T003: Implement Configuration Loader with Pydantic validation and structlog.
- [x] T004: Define standard functional interfaces for workbenches in base.py.

### Implementation Sketch

1. Create `src/workbenches/models.py`.
2. Create `src/workbenches/manufacturing_config.yaml`.
3. Create `src/workbenches/config.py` using `structlog` for loading events.
4. Define protocol/types in `src/workbenches/base.py`.

### Parallel Opportunities

- None (This is the foundation).

### Dependencies

- None.

---

## Work Package 2: CNC Workbench Implementation

- **Goal**: Implement the CNC-specific functional validation logic (raycasting, corner radii) and cost estimation.
- **Priority**: High
- **Independent Test**: `pytest tests/workbenches/test_cnc.py` verifies geometry constraints and pricing.

### Included Subtasks

- [x] T005: Implement analyze_cnc function skeleton in cnc.py.
- [x] T006: Implement geometry validation logic (undercut check) with structlog tracing.
- [x] T007: Implement internal corner radius check helper function.
- [x] T008: Implement CNC cost estimation function.
- [x] T009: Write comprehensive tests for CNC functional validation.

### Implementation Sketch

1. Create `src/workbenches/cnc.py`.
2. Implement `analyze_cnc()` using `trimesh` and `structlog`.
3. Integrate cost model from Pydantic config.
4. Write tests in `tests/workbenches/test_cnc.py`.

### Parallel Opportunities

- Can be done in parallel with WP03 (IM Workbench).

### Dependencies

- WP01

---

## Work Package 3: Injection Molding Workbench Implementation

- **Goal**: Implement the Injection Molding functional validation logic (draft angles, wall thickness) and cost estimation.
- **Priority**: High
- **Independent Test**: `pytest tests/workbenches/test_im.py` verifies draft checks and pricing.

### Included Subtasks

- [ ] T010: Implement analyze_im function skeleton in injection_molding.py.
- [ ] T011: Implement draft angle validation logic with structlog tracing.
- [ ] T012: Implement wall thickness verification function.
- [ ] T013: Implement IM cost estimation function.
- [ ] T014: Write comprehensive tests for IM functional validation.

### Implementation Sketch

1. Create `src/workbenches/injection_molding.py`.
2. Implement `analyze_im()` with `structlog`.
3. Implement wall thickness check.
4. Write tests in `tests/workbenches/test_im.py`.

### Parallel Opportunities

- Can be done in parallel with WP02.

### Dependencies

- WP01

---

## Work Package 4: Agent Integration & Facade

- **Goal**: Expose the workbench functionality to the Agent via a unified functional facade.
- **Priority**: Medium
- **Independent Test**: `validate_and_price` calls correct analysis function and returns formatted Pydantic DFM report.

### Included Subtasks

- [ ] T015: Implement validate_and_price functional facade with structlog.
- [ ] T016: Create integration verification script/test ensuring Pydantic compliance.

### Implementation Sketch

1. Create `src/worker/utils/dfm.py`.
2. Implement dispatch logic calling functional workbenches.
3. Log results with `structlog`.
4. Verify end-to-end flow.

### Parallel Opportunities

- Depends on WP02 and WP03.

### Dependencies

- WP02, WP03
