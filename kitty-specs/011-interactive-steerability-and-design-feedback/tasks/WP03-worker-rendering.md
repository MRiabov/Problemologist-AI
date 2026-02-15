---
work_package_id: WP03
title: Worker Topology & Rendering
lane: "for_review"
dependencies: []
base_branch: main
base_commit: 1aefa54b0226d5e5187abdc79cf88b3d6caa06de
created_at: '2026-02-15T10:16:25.819782+00:00'
subtasks: [T005, T006, T007]
shell_pid: "141786"
agent: "gemini"
review_status: "has_feedback"
reviewed_by: "MRiabov"
---

# WP03 - Worker Topology & Rendering

## Objective
Enable the worker to provide high-fidelity geometric feedback by implementing topological inspection and automated isometric snapshot rendering.

## Context
- Research: `kitty-specs/011-interactive-steerability-and-design-feedback/research.md` (Rendering section)
- Design: `specs/wp4_research_report.md` (Topological naming section)

## Subtasks

### T005: Implement inspect_topology Tool
**Purpose**: Provide geometric properties of selected features to the agent.
**Steps**:
1. Create `worker/tools/topology.py`.
2. Implement `inspect_topology(target_id: str)`:
    - Load the current `build123d` assembly in memory.
    - Resolve `target_id` (e.g. `face_12`).
    - Extract: `center`, `normal_at()`, `area`, `bbox`.
    - Return structured JSON.
**Validation**:
- [ ] Tool returns correct normals and centers for known features.

### T006: Implement Best-Angle View Selection
**Purpose**: Deterministically find the best isometric angle to view a selected feature.
**Steps**:
1. Create `shared/simulation/view_utils.py`.
2. Implement `get_best_isometric_view(normal: Vector3) -> Matrix4x4`:
    - Define the 8 standard isometric unit vectors.
    - Perform a dot product between the target `normal` and each isometric vector.
    - Return the camera matrix for the vector with the highest dot product.
**Validation**:
- [ ] Selecting a "Top" face normal returns the [1,1,1] or similar top-view angle.

### T007: Implement Headless Highlight Snapshots
**Purpose**: Generate PNG snapshots with highlighted features.
**Steps**:
1. Update `worker/activities/rendering.py`.
2. Implement `render_selection_snapshot(ids, view_matrix)`:
    - Use `pyvista` or `VTK` to render the GLB.
    - Apply a bright emissive color or highlight shader to parts/faces in `ids`.
    - Use the calculated `view_matrix`.
    - Save to a temp PNG and upload to S3.
**Validation**:
- [ ] Generated image shows the selected face highlighted from the optimal angle.

## Definition of Done
- `inspect_topology` tool is available to agents.
- Worker can generate and upload "best-angle" snapshots of selected features.

## Activity Log

- 2026-02-15T10:16:25Z – Gemini – shell_pid=120025 – lane=doing – Assigned agent via workflow command
- 2026-02-15T10:31:14Z – Gemini – shell_pid=120025 – lane=for_review – Implemented worker-side topology inspection tool and best-angle isometric snapshot rendering with VTK. Added unit tests for topology inspection and view selection.
- 2026-02-15T10:31:54Z – Gemini – shell_pid=120025 – lane=planned – Changes requested: Fixed broken simulation loop and NameError in backend.
- 2026-02-15T10:34:25Z – gemini – shell_pid=141786 – lane=doing – Started implementation via workflow command
- 2026-02-15T11:01:25Z – gemini – shell_pid=141786 – lane=for_review – Ready for review: fixed NameError in rendering activity and broken simulation loop, implemented topology inspection and best-angle view selection. All tests pass.
