# Tasks: Fluids & Deformable Materials (WP2)

This document tracks the implementation of the WP2 Fluids and FEM system.

## Status Summary

- **Foundation**: [ ]
- **Asset Pipeline**: [ ]
- **FEM Implementation**: [ ]
- **Fluid Implementation**: [ ]
- **Agent Integration**: [ ]
- **Visualization**: [ ]
- **Verification**: [ ]

---

## Work Packages

### WP01: Simulation Foundation & Backend Abstraction
**Goal**: Introduce the `PhysicsBackend` interface and support togglable backends (MuJoCo/Genesis) in the controller and worker.
**Priority**: P1
**Estimated Prompt Size**: ~350 lines
**Subtasks**:
- [x] T001: Implement `PhysicsBackend` abstract base class in `shared/simulation/backends.py`.
- [x] T002: Refactor existing MuJoCo code into `MuJoCoBackend` in `worker/simulation/mujoco_backend.py`.
- [x] T003: Implement `GenesisBackend` skeleton in `worker/simulation/genesis_backend.py`.
- [x] T004: Update `SimulationRequest` and `SimulationResult` schemas (inc. `ELECTRONICS_FLUID_DAMAGE`).
- [x] T005: Update `controller/simulation/loop.py` to handle backend selection and validation.

**Dependencies**: None
**Implementation Sketch**: Define the interface first, then move MuJoCo code, then update the controller to use the interface.

---

### WP02: Asset Pipeline & FEM Material Config
**Goal**: Prepare the system for FEM by extending material definitions and implementing volumetric meshing.
**Priority**: P1
**Estimated Prompt Size**: ~300 lines
**Subtasks**:
- [x] T006: Extend `manufacturing_config.yaml` with FEM fields (Young's modulus, Poisson ratio, etc.).
- [x] T007: Implement `tetrahedralize_mesh` utility using Gmsh/TetGen in `worker/utils/mesh_utils.py`.
- [x] T008: Implement mesh repair logic in `worker/utils/mesh_utils.py` using `trimesh`.
- [x] T009: Update asset generation to produce `.msh` files for Genesis when `fem_enabled` is true.

**Dependencies**: WP01
**Implementation Sketch**: Add fields to config, then build the mesh processing utilities, then hook them into the asset generator.

---

### WP03: Genesis FEM Implementation
**Goal**: Implement deformable body simulation, stress tracking, and breakage detection using the Genesis backend.
**Priority**: P1
**Estimated Prompt Size**: ~400 lines
**Subtasks**:
- [ ] **T010**: Implement scene loading for rigid and deformable bodies in `GenesisBackend`.
- [ ] **T011**: Implement FEM stress calculation and `StressSummary` generation.
- [ ] **T012**: Implement `PART_BREAKAGE` detection and simulation termination logic.
- [ ] **T013**: Add support for linear and hyperelastic (Neo-Hookean) material models.

**Dependencies**: WP02
**Implementation Sketch**: Implement scene setup, then the stepping logic with stress extraction, then the failure checks.

---

### WP04: Fluid Simulation (MPM)
**Goal**: Implement MPM-based fluid simulation and objective evaluation in the Genesis backend.
**Priority**: P2
**Estimated Prompt Size**: ~400 lines
**Subtasks**:
- [ ] **T014**: Implement fluid particle definition and spawning in `GenesisBackend`.
- [ ] **T015**: Implement fluid containment objective evaluation.
- [ ] **T016**: Implement fluid flow-rate objective evaluation.
- [ ] **T017**: Implement GPU OOM detection and particle count retry logic.

**Dependencies**: WP03
**Implementation Sketch**: Add fluid support to Genesis loading, then implement the particle-based objective metrics.

---

### WP05: Agent Tools & Prompt Updates
**Goal**: Equip the agents with tools to reason about stress and fluids, and update their prompts for stress-aware design.
**Priority**: P2
**Estimated Prompt Size**: ~350 lines
**Subtasks**:
- [ ] **T018**: Implement `get_stress_report` tool in `controller/tools/simulation.py`.
- [ ] **T019**: Implement `define_fluid` tool for agents.
- [ ] **T020**: Implement `preview_stress` tool to generate heatmap visualizations.
- [ ] **T021**: Update Engineer and Planner agent prompts for stress/fluid reasoning.

**Dependencies**: WP04
**Implementation Sketch**: Build the tools first, then update the prompts to use them in the engineering loop.

---

### WP06: Visualization & Frontend Integration
**Goal**: Render simulation results (particles and stress) and display them in the frontend UI.
**Priority**: P3
**Estimated Prompt Size**: ~300 lines
**Subtasks**:
- [ ] **T022**: Update simulation video rendering to include MPM particles.
- [ ] **T023**: Implement stress heatmap image generation (static renders).
- [ ] **T024**: Update frontend `DesignViewer` to display heatmaps and fluid metrics.

**Dependencies**: WP05
**Implementation Sketch**: Update the worker's rendering code, then update the frontend to handle the new result fields.

---

### WP07: Benchmark Planning & Integration Tests
**Goal**: Create benchmarks for the new physics and verify the system end-to-end with automated tests.
**Priority**: P2
**Estimated Prompt Size**: ~300 lines
**Subtasks**:
- [ ] **T025**: Update benchmark planner to generate stress/fluid-based `objectives.yaml`.
- [ ] **T026**: Implement integration tests for physics parity (MuJoCo vs Genesis).
- [ ] **T027**: Implement integration tests for FEM breakage and fluid containment.

**Dependencies**: WP06
**Implementation Sketch**: Update the planner logic, then write and run the integration test suite.
