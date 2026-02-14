---
work_package_id: "WP02"
title: "Advanced Asset Pipeline"
lane: "planned"
dependencies: ["WP01"]
subtasks: ["T006", "T007", "T008"]
---

# WP02: Advanced Asset Pipeline

## Objective
Enhance the asset pipeline to support tetrahedralization (required for FEM) and automated mesh repair to ensure high-quality volumetric meshes for simulation.

## Context
FEM simulation requires volumetric (tetrahedral) meshes, whereas our current pipeline mostly handles surface (triangular) meshes for rigid bodies. We also need to be robust against non-manifold geometry produced by agents.

## Detailed Guidance

### T006: Mesh repair logic
- Update `worker/utils/mesh_utils.py`.
- Integrate `trimesh` to perform automated repairs on STLs before tetrahedralization.
- Implement `repair_mesh(stl_path: Path) -> Path` which handles:
  - Removing duplicate faces.
  - Filling holes (where possible).
  - Fixing self-intersections (using `trimesh.repair`).

### T007: Tetrahedralization (Gmsh/TetGen)
- Implement `tetrahedralize_mesh(stl_path: Path) -> Path` in `worker/utils/mesh_utils.py`.
- Use `gmsh` or `tetgen` (provide a wrapper around the CLI tool if necessary).
- The output must be a `.msh` file compatible with Genesis.
- Handle TetGen non-zero exit codes and provide diagnostic feedback (e.g., "Non-manifold geometry near coordinates...").

### T008: Worker integration
- Update the worker's asset generation flow (likely in `worker/agent/tools.py` or `worker/simulation/loop.py`).
- When `fem_enabled` is true, route manufactured parts through the new `repair -> tetrahedralize` pipeline.
- Store the resulting `.msh` files alongside the existing `.obj` assets.

## Test Strategy
- Create a unit test that takes a known manifold STL and verifies a `.msh` file is produced.
- Create a test with a slightly "broken" STL and verify the repair pass allows it to be tetrahedralized.

## Definition of Done
- [ ] `trimesh` repair logic is active and tested.
- [ ] `.msh` files are successfully generated from valid geometry.
- [ ] Worker correctly triggers tetrahedralization for FEM parts.
- [ ] Assets are correctly stored and accessible to the simulation engine.

## Risks
- TetGen crashes or infinite loops on extremely complex geometry.
- Large `.msh` files increasing latency.
- Asset versioning issues (mixing rigid .obj and FEM .msh).
