# Architecture Scaffold: WP2 Part 2 — Meshing Pipeline (FEM)

## Overview

This document scaffolds the architecture for the second part of Work Package 2 (Fluids & FEM), focusing on the **Meshing Pipeline**. The goal is to enable the conversion of CAD geometry (`build123d`) into tetrahedral meshes (`.msh`) suitable for Finite Element Method (FEM) simulation in Genesis.

## 1. Components

### 1.1 `worker/utils/mesh_utils.py` (Enhancements)
- **`repair_mesh(mesh: trimesh.Trimesh) -> trimesh.Trimesh`**:
    - Robustly handle non-manifold and non-watertight geometry.
    - Use `trimesh`'s repair functions: `fill_holes`, `fix_normals`, `remove_infinite_values`.
- **`convert_stl_to_msh(stl_path: Path, msh_path: Path, method: Literal["tetgen", "gmsh"] = "gmsh") -> Path`**:
    - Main entry point for tetrahedralization.
    - Support both TetGen and Gmsh (preferring Gmsh for better `.msh` support).
    - Handle quality parameters: radius-edge ratio, maximum volume.
- **`validate_mesh_quality(msh_path: Path) -> bool`**:
    - Check for degenerate tetrahedra or extreme aspect ratios.

### 1.2 `worker/simulation/builder.py` (Genesis Integration)
- **`GenesisSimulationBuilder.build_from_assembly(...)`**:
    - Detect if `fem_enabled` is true for a part.
    - If true, invoke the tetrahedralization pipeline instead of simple OBJ export.
    - Use `gs.morphs.SoftMesh(file='...')` in Genesis to load the `.msh` file.
    - Map material properties (Young's modulus, Poisson's ratio) to the `gs.materials.FEM` or `gs.materials.NeoHookean` objects.

### 1.3 `worker/simulation/loop.py` (Stress Monitoring)
- **`SimulationLoop.monitor_stresses()`**:
    - Query `backend.get_stress_field(body_id)` at regular intervals.
    - Compute `StressSummary` for each FEM body.
    - Check against `yield_stress` and `ultimate_stress` from `manufacturing_config.yaml`.
    - Trigger `PART_BREAKAGE` failure if limits are exceeded.

## 2. Updated Task List (Part 2)

| Task ID | Description | File(s) |
|---|---|---|
| **T006.1** | Integrate `gmsh-sdk` or CLI into `mesh_utils.py`. | `worker/utils/mesh_utils.py` |
| **T007.1** | Implement `STL -> .msh` with quality constraints. | `worker/utils/mesh_utils.py` |
| **T008.1** | Update `GenesisSimulationBuilder` to cache/reference `.msh` assets. | `worker/simulation/builder.py` |
| **T009.1** | Implement robust `repair_mesh` as a pre-meshing gate. | `worker/utils/mesh_utils.py` |
| **T009.2** | Add CAD validation gate for FEM (verify material properties exist). | `worker/utils/validation.py` |

## 3. Implementation Plan

1.  **Environment Check**: Verify `gmsh` availability (pip install `gmsh` if needed).
2.  **Core Utilities**: Implement `repair_mesh` and `convert_stl_to_msh` in `mesh_utils.py`.
3.  **Integration**: Hook these utilities into `GenesisSimulationBuilder`.
4.  **Verification**: Add a test case that takes a `build123d` part, meshing it, and loads it into a Genesis scene.
5.  **Stress Logic**: Implement the per-step stress monitoring in `SimulationLoop`.

## 4. Open Questions / Constraints

- **Genesis .msh format**: Genesis uses its own internal loader for `.msh`. We must ensure Gmsh/TetGen output is compatible (typically Gmsh v2 or v4 ASCII).
- **Compute Overhead**: Tetrahedralization can be slow. We should implement simple hashing/caching of `.msh` files based on the STL content.
