# Research: WP2 Fluids & Deformable Materials

## Decisions

### 1. Tetrahedralization Strategy

- **Decision**: Use **Gmsh** as the primary tetrahedralization engine on the worker, falling back to **TetGen** for simpler geometries or if Gmsh is unavailable.
- **Rationale**: Gmsh provides better support for BREP-derived meshes and more robust MSH v4 export, which Genesis consumes more reliably.
- **Alternatives**: Using `gs.morphs.Mesh` with `material=gs.materials.FEM` directly (Genesis internal tetrahedralization). Rejected because external control via Gmsh allows for better mesh density management (`refine_level`).

### 2. Mesh Repair Location

- **Decision**: Implement `trimesh.repair` both in the CAD generation worker (after `build123d` export) AND as a safeguard in the simulation worker (before tetrahedralization).
- **Rationale**: Prevention is better at the source (CAD worker), but simulation-time repair ensures robustness against external assets.
- **Why Needed**: TetGen/Gmsh crash or produce invalid volumes if the surface mesh is non-manifold (self-intersections, holes).

### 3. Backend Abstraction

- **Decision**: Keep the `PhysicsBackend` protocol in `shared/simulation/backends.py` and enforce a single backend per episode execution.
- **Rationale**: Switching backends mid-simulation (e.g., rigid to FEM) would require complex state migration (MuJoCo to Genesis), which is out of scope for WP2.

### 4. GPU/CPU Scaling

- **Decision**: Use `gs.init(backend=gs.gpu if cuda_available else gs.cpu)` with an override in `simulation_config.yaml`.
- **Rationale**: Allows local development on CPU and production scaling on GPU.

## Research Gaps Resolved

- **Q**: How to extract stress arrays for visualization?
- **A**: `gs.Entity.get_state().von_mises` for FEM. We will serialize this as a flat float array in `SimulationResult`.
