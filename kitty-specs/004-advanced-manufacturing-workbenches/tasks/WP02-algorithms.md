---
work_package_id: WP02
title: Geometric Analysis Algorithms
lane: "for_review"
dependencies: [WP01]
base_branch: 004-advanced-manufacturing-workbenches-WP01
base_commit: fb2a701efb41c088043d89a828fbd4257cb38eb8
created_at: '2026-02-01T11:14:19.528161+00:00'
subtasks: [T006, T007, T008, T009]
shell_pid: "217205"
---

### Objective
Implement the core geometric algorithms for Design for Manufacturing (DFM) analysis: Draft Angle check, Undercut detection (Raycasting), and Wall Thickness analysis.

### Context
These functions will live in `src/workbenches/analysis_utils.py` and be used by both CNC and Injection Molding workbenches. They perform the heavy lifting of checking if a geometry is manufacturable.

### Subtask T006: Draft Angle Analysis
**Purpose**: Identify faces that are too steep relative to the pull direction.
**Steps**:
1.  In `analysis_utils.py`, implement `check_draft_angle(mesh: trimesh.Trimesh, pull_vector: tuple, min_angle_deg: float) -> list[int]`.
2.  Logic:
    *   Iterate over `mesh.face_normals`.
    *   Calculate angle between normal and pull vector.
    *   Identify "vertical" walls (angle approx 90 degrees).
    *   Return list of face indices where `abs(angle - 90) < min_angle_deg`.

### Subtask T007: Undercut Detection
**Purpose**: Detect geometry that is occluded (trapped) from the tool/mold direction.
**Steps**:
1.  In `analysis_utils.py`, implement `check_undercuts(mesh: trimesh.Trimesh, pull_vector: tuple) -> list[int]`.
2.  Logic (Raycasting):
    *   origins = `mesh.triangles_center` + (`pull_vector` * epsilon).
    *   direction = `pull_vector`.
    *   Use `mesh.ray.intersects_any(origins, direction)`.
    *   If a ray hits *another* part of the mesh, the origin face is an undercut.
    *   Return list of face indices that are occluded.

### Subtask T008: Wall Thickness Analysis (Basic)
**Purpose**: Ensure the part isn't too thin or too thick.
**Steps**:
1.  In `analysis_utils.py`, implement `check_wall_thickness(mesh: trimesh.Trimesh, min_mm: float, max_mm: float) -> list[int]`.
2.  Logic (Raycasting approximation):
    *   For each face, cast a ray *inwards* (negative normal).
    *   Find the first intersection distance (thickness).
    *   If distance < `min_mm` or > `max_mm`, flag face.
    *   *Note*: This is an approximation. Perfect thickness is hard. This "Ray vs Backface" check is sufficient for MVP.

### Subtask T009: Algorithm Tests
**Purpose**: Verify the math is correct.
**Steps**:
1.  Create `tests/test_analysis_algorithms.py`.
2.  **Draft Test**: Create a pyramid (sloped) and a box (vertical). Assert box fails draft check, pyramid passes.
3.  **Undercut Test**: Create a "T" shape or mushroom. Assert the faces under the overhang are detected.
4.  **Thickness Test**: Create a hollow box with known wall thickness. Verify checks.

### Definition of Done
*   All 3 analysis functions implemented in `analysis_utils.py`.
*   Tests cover positive and negative cases for each algorithm.
*   Functions utilize `trimesh` efficiently (vectorized operations where possible).

## Activity Log

- 2026-02-01T11:26:01Z – unknown – shell_pid=217205 – lane=for_review – Core DFM algorithms (Draft, Undercut, Thickness) implemented and tested.
