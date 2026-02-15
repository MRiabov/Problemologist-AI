---
work_package_id: WP02
title: Asset Pipeline & FEM Material Config
lane: planned
dependencies: []
subtasks: [T006, T007, T008, T009]
---

# WP02: Asset Pipeline & FEM Material Config

## Objective
Extend the asset generation pipeline to produce tetrahedral meshes (`.msh`) required for Genesis FEM simulations.

## Context
MuJoCo uses STL/OBJ surface meshes. Genesis FEM requires volumetric tetrahedral meshes. We need to integrate Gmsh/TetGen for meshing and trimesh for repair.

## Guidance

### T006: Extend `manufacturing_config.yaml`
- Add FEM fields to the material schema:
  - `youngs_modulus_pa`: float
  - `poissons_ratio`: float
  - `ultimate_stress_pa`: float
  - `density_kg_m3`: float (ensure present)
  - `material_class`: enum (`rigid`, `soft`, `elastomer`)

### T007: `tetrahedralize_mesh` Utility
- Location: `worker/utils/mesh_utils.py`.
- Use `gmsh` or `tetgen` python bindings.
- Input: STL file path.
- Output: `.msh` file path (Genesis format).

### T008: Mesh Repair Logic
- Use `trimesh` to check for manifoldness and water-tightness.
- Attempt automatic repair (`mesh.fill_holes()`, `mesh.fix_normals()`).
- If repair fails after N attempts, raise `MeshProcessingError`.

### T009: Genesis-Specific Asset Generation
- Update the worker's asset generation task.
- If `fem_enabled` is true for a part, trigger the tetrahedralization flow.
- Ensure resulting `.msh` files are cached/stored correctly.

## Definition of Done
- [ ] Materials in `manufacturing_config.yaml` have the new FEM fields.
- [ ] Worker can convert an STL to a tetrahedral `.msh` file.
- [ ] Automatic repair handles common mesh errors (small holes, inverted normals).
- [ ] Integration tests verify `.msh` file existence for FEM-enabled simulations.

## Risks
- Gmsh/TetGen installation issues in the Docker container.
- Complex geometries failing to mesh even after repair.
