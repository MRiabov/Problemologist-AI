---
work_package_id: WP02
title: Geometry & Workbenches
lane: "done"
dependencies: []
subtasks:
- T006
- T007
- T008
- T009
- T010
phase: Phase 2 - Domain Logic
agent: "Antigravity"
shell_pid: "96023"
reviewed_by: "MRiabov"
review_status: "approved"
history:
- timestamp: '{{TIMESTAMP}}'
  lane: planned
  agent: system
  action: Prompt generated via /spec-kitty.tasks
---

# Work Package Prompt: WP02 – Geometry & Workbenches

## Objectives & Success Criteria

- **Goal**: Implement the core geometry processing capability (converting CAD objects to Meshes) and the "Workbench" validation system.
- **Success Criteria**:
  - `geometry.py` can take a `build123d.Part` and return an STL path or Mesh object.
  - A dummy convex decomposition works or is stubbed adequately.
  - `Print3DWorkbench` correctly identifies non-manifold or multi-body parts.
  - Cost calculations return valid floats based on volume.

## Context & Constraints

- **Spec**: See 4.3 Workbench Architecture & 4.4 Simulation Bridge (geometry compiler part).
- **Libraries**: `build123d` has `export_stl`. Trimesh might be useful for checks if build123d lacks them (add to deps if needed, but build123d wraps OCP which has `IsClosed` etc).

## Subtasks & Detailed Guidance

### Subtask T006 – Implement Geometry Compiler (Export)

- **Purpose**: Convert parametric CAS objects into triangular meshes for rendering and physics.
- **Steps**:
  - Create `src/compiler/geometry.py`.
  - Implement `export_mesh(part: Part, filepath: str, tolerance: float) -> str`.
  - Use `build123d.exporters.export_stl` or OCP equivalent.
- **Files**: `src/compiler/geometry.py`.
- **Parallel?**: No.
- **Notes**: Ensure high resolution for visuals, maybe lower for physics hull if needed.

### Subtask T007 – Implement Convex Decomposition (Stub/Basic)

- **Purpose**: Physics engines hate concave meshes. We need convex hulls.
- **Steps**:
  - In `src/compiler/geometry.py`, add `generate_colliders(part: Part) -> List[Mesh]`.
  - **MVP strategy**: Use `part.bounding_box()` or just return the convex hull of the mesh (`trimesh.convex.convex_hull` or OCP BRepBndLib/BRepMesh).
  - Ideally, just wrap the whole object in one convex hull for now if decomposition is too hard without `coacd`.
- **Files**: `src/compiler/geometry.py`.
- **Parallel?**: No.

### Subtask T008 – Implement Workbench ABC

- **Purpose**: Define the interface for all future constraints.
- **Steps**:
  - Create `src/workbenches/base.py`.
  - Define class `Workbench(ABC)`.
  - Abstract methods:
    - `validate(self, part: Part) -> List[Exception|String]`
    - `calculate_cost(self, part: Part) -> float`
- **Files**: `src/workbenches/base.py`.
- **Parallel?**: No.

### Subtask T009 – Implement 3D Print Workbench Constraints

- **Purpose**: The specific checks for the MVP.
- **Steps**:
  - Create `src/workbenches/print_3d.py` inheriting `Workbench`.
  - Implement `validate`:
    - **Manifold**: Check `if part.is_valid()` and `part.is_closed` (OC.TopoDS_Shape properties).
    - **Single Body**: Check `len(part.solids()) == 1`.
  - Return list of violations.
- **Files**: `src/workbenches/print_3d.py`.
- **Parallel?**: Yes.

### Subtask T010 – Implement Cost Model

- **Purpose**: Simple feedback metric.
- **Steps**:
  - In `src/workbenches/print_3d.py`, implement `calculate_cost`.
  - Formula: `part.volume * MATERIAL_COST` (e.g. 0.05).
- **Files**: `src/workbenches/print_3d.py`.
- **Parallel?**: Yes.

## Test Strategy

- **Test File**: `tests/test_geometry.py`, `tests/test_workbench_print3d.py`.
- **Cases**:
  - Export a cube -> file exists, size > 0.
  - Validate Cube -> Pass.
  - Validate 2 Cubes (Compound) -> Fail (Single Body).
  - Open Shell -> Fail (Manifold).
  - Cost Cube(10x10x10) -> Check calculation.

## Risks & Mitigations

- **OCP Complexity**: OpenCASCADE API is deep. Use `build123d` wrappers where possible. If `part.is_closed` isn't exposed directly, check `part.encapsulating_solid` or similar properties.

## Activity Log

- 2026-02-01T07:36:18Z – Antigravity – shell_pid=60800 – lane=doing – Started implementation via workflow command
- 2026-02-01T07:43:52Z – Antigravity – shell_pid=60800 – lane=for_review – Implemented geometry compiler (STL export/colliders) and 3D printing workbench (validation/cost). Verified with unit tests.
- 2026-02-01T08:12:01Z – Antigravity – shell_pid=96023 – lane=doing – Started review via workflow command
- 2026-02-01T08:14:58Z – Antigravity – shell_pid=96023 – lane=done – Review passed: Geometry compiler and 3D printing workbench implemented correctly with passing tests. Rebased on main.
