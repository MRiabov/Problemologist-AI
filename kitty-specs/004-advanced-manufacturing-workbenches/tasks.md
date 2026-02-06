# Tasks: Advanced Manufacturing Workbenches

## Work Package 1: Foundation & Data Models

- **Goal**: Set up the core data structures, configuration system, and abstract base classes for the workbench system.
- **Priority**: High (Blocking)
- **Independent Test**: `pytest tests/workbenches/test_config.py` passes and models can be instantiated.

### Included Subtasks

- [ ] **T001**: Implement Core Data Models (`WorkbenchResult`, `ManufacturingMethod`, `MaterialDefinition`).
- [ ] **T002**: Create `manufacturing_config.yaml` with initial cost/material data.
- [ ] **T003**: Implement Configuration Loader with Pydantic validation.
- [ ] **T004**: Create abstract `Workbench` base class defining the interface.

### Implementation Sketch

1. Create `src/workbenches/models.py`.
2. Create `src/workbenches/manufacturing_config.yaml`.
3. Create `src/workbenches/config.py` to load YAML.
4. Create `src/workbenches/base.py`.

### Parallel Opportunities

- None (This is the foundation).

### Dependencies

- None.

---

## Work Package 2: CNC Workbench Implementation

- **Goal**: Implement the CNC-specific validation logic (raycasting, corner radii) and cost estimation.
- **Priority**: High
- **Independent Test**: `pytest tests/workbenches/test_cnc.py` verifies geometry constraints and pricing.

### Included Subtasks

- [ ] **T005**: Implement `CNCWorkbench` class skeleton.
- [ ] **T006**: Implement geometry validation (raycasting for visibility/undercuts).
- [ ] **T007**: Implement internal corner radius check.
- [ ] **T008**: Implement CNC cost estimation algorithm.
- [ ] **T009**: Write comprehensive tests for CNC workbench.

### Implementation Sketch

1. Create `src/workbenches/cnc.py`.
2. Implement `analyze()` method using `trimesh` for geometry checks.
3. Integrate cost model from config.
4. Write tests in `tests/workbenches/test_cnc.py`.

### Parallel Opportunities

- Can be done in parallel with WP03 (IM Workbench).

### Dependencies

- WP01

---

## Work Package 3: Injection Molding Workbench Implementation

- **Goal**: Implement the Injection Molding validation logic (draft angles, wall thickness) and cost estimation.
- **Priority**: High
- **Independent Test**: `pytest tests/workbenches/test_im.py` verifies draft checks and pricing.

### Included Subtasks

- [ ] **T010**: Implement `InjectionMoldingWorkbench` class skeleton.
- [ ] **T011**: Implement draft angle validation.
- [ ] **T012**: Implement wall thickness verification (raycasting/sampling).
- [ ] **T013**: Implement IM cost estimation algorithm (mold base + cycle).
- [ ] **T014**: Write comprehensive tests for IM workbench.

### Implementation Sketch

1. Create `src/workbenches/injection_molding.py`.
2. Implement `analyze()` method using `trimesh` normals for draft checks.
3. Implement wall thickness check (random sampling raycast).
4. Write tests in `tests/workbenches/test_im.py`.

### Parallel Opportunities

- Can be done in parallel with WP02.

### Dependencies

- WP01

---

## Work Package 4: Agent Integration & Facade

- **Goal**: Expose the workbench functionality to the Agent via a unified, simple API.
- **Priority**: Medium
- **Independent Test**: `validate_and_price` calls correct workbench and returns formatted DFM report.

### Included Subtasks

- [ ] **T015**: Implement `validate_and_price` facade function.
- [ ] **T016**: Create integration verification script/test.

### Implementation Sketch

1. Create `src/worker/utils/dfm.py`.
2. Implement dispatch logic based on metadata.
3. Verify end-to-end flow.

### Parallel Opportunities

- Depends on WP02 and WP03.

### Dependencies

- WP02, WP03
