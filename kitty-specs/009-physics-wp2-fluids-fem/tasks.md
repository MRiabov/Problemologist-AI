# Tasks: 009-physics-wp2-fluids-fem

**Feature Directory**: `kitty-specs/009-physics-wp2-fluids-fem`

## Status

- [ ] WP01: Genesis Backend & Core Physics <!-- id: WP01 -->
- [ ] WP02: Meshing Pipeline (FEM) <!-- id: WP02 -->
- [ ] WP03: Validation & Rendering <!-- id: WP03 -->
- [ ] WP04: Agent Reasoning & Benchmarks <!-- id: WP04 -->

## Work Packages

### WP01: Genesis Backend & Core Physics

**Establish the actual Genesis physics integration and fix the current skeleton implementation.**

- [ ] T001: Implement `GenesisBackend.build(scene_data)` to handle scene construction from JSON. <!-- id: T001 -->
- [ ] T002: Integrate real `genesis` Python API for stepping and state retrieval. <!-- id: T002 -->
- [ ] T003: Replace dummy stress data in `get_stress_field` with actual FEM results from Genesis. <!-- id: T003 -->
- [ ] T004: Replace dummy particle data in `get_particle_positions` with real MPM particle states. <!-- id: T004 -->
- [ ] T005: Fix existing integration tests to rely on real physics instead of triggers. <!-- id: T005 -->

**Implementation Notes**:

- Requires `genesis-world` or `genesis-physics` package in the worker.
- Must handle GPU/CPU falls-backs as per `objectives.yaml`.
- Ensure `get_stress_field` correctly maps to mesh nodes.

**Dependencies**: None
**Prompt Size**: ~450 lines

---

### WP02: Meshing Pipeline (FEM)

**Implement a functional tetrahedralization utility for generating FEM assets.**

- [ ] T006: Properly integrate `TetGen` or `Gmsh` into `worker/utils/mesh_utils.py`. <!-- id: T006 -->
- [ ] T007: Implement `STL` -> `.msh` conversion with appropriate quality constraints for FEM. <!-- id: T007 -->
- [ ] T008: Ensure `GenesisSimulationBuilder` correctly caches and references these assets. <!-- id: T008 -->
- [ ] T009: Implement part repair/watertightness check as a prerequisite for meshing. <!-- id: T009 -->

**Implementation Notes**:

- Large meshes should be handled efficiently.
- Prefer `Gmsh` if `TetGen` integration remains brittle.

**Dependencies**: WP01
**Prompt Size**: ~400 lines

---

### WP03: Validation & Rendering

**Develop tools for visualizing stress and validating fluid containment.**

- [ ] T010: Implement `preview_stress` render logic using PyVista or similar to generate heatmaps. <!-- id: T010 -->
- [ ] T011: Implement fluid particle visualization in `VideoRenderer`. <!-- id: T011 -->
- [ ] T012: Extend `SimulationResult` to include stress heatmap export (base64 or S3 paths). <!-- id: T012 -->
- [ ] T013: Implement real flow rate calculation logic in `SimulationLoop`. <!-- id: T013 -->

**Implementation Notes**:

- Heatmaps should be deliverable to the frontend GLB viewer.
- Use `colormap` for stress visualization (blue to red).

**Dependencies**: WP01, WP02
**Prompt Size**: ~350 lines

---

### WP04: Agent Reasoning & Benchmarks

**Update agent prompts and create fluid/FEM-specific benchmarks.**

- [ ] T014: Update `Architect` system prompt to include material stress and fluid reasoning. <!-- id: T014 -->
- [ ] T015: Update `Engineer` prompt with `preview_stress` and stress-aware optimization instructions. <!-- id: T015 -->
- [ ] T016: Create a baseline fluid containment benchmark variant. <!-- id: T016 -->
- [ ] T017: Create a structural stress benchmark (e.g., bridge or bracket under load). <!-- id: T017 -->

**Implementation Notes**:

- Prompts should use `config/manufacturing_config.yaml` as the source of truth for stress limits.
- Benchmarks should be highly randomized to test agent robustness.

**Dependencies**: WP01, WP03
**Prompt Size**: ~300 lines
