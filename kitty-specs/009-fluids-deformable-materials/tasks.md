# Tasks: Fluids & Deformable Materials

This document breaks down the implementation of WP2 (Fluids & Deformable Materials) into actionable work packages.

## Subtask List

| ID | [P] | Description |
|---|---|---|
| T001 | | Implement `PhysicsBackend` protocol in `shared/simulation/backends.py` |
| T002 | | Refactor MuJoCo implementation to `worker/simulation/mujoco_backend.py` |
| T003 | | Implement `GenesisBackend` skeleton in `worker/simulation/genesis_backend.py` |
| T004 | [P] | Update `manufacturing_config.yaml` and schema with FEM material fields |
| T005 | [P] | Update `SimulationResult` and shared schemas for stress/fluids |
| T006 | | Enhance `worker/utils/mesh_utils.py` with `trimesh` repair logic |
| T007 | | Implement tetrahedralization (Gmsh/TetGen) in `worker/utils/mesh_utils.py` |
| T008 | | Integrate tetrahedralization into worker's asset pipeline |
| T009 | | Implement rigid-body support in `GenesisBackend` |
| T010 | | Implement FEM support in `GenesisBackend` (SoftMesh, .msh loading) |
| T011 | | Implement stress computation and `StressSummary` reporting in Genesis |
| T012 | | Implement breakage detection logic in simulation loop |
| T013 | | Implement MPM fluid support in `GenesisBackend` |
| T014 | | Implement `fluid_containment` metric evaluation |
| T015 | | Implement `flow_rate` metric evaluation |
| T016 | [P] | Implement `define_fluid` agent tool |
| T017 | [P] | Implement `get_stress_report` agent tool |
| T018 | [P] | Implement `preview_stress` agent tool (heatmap rendering) |
| T019 | | Update `simulate` tool for Genesis/FEM parameters |
| T020 | | Update Agent prompts (Planner, CAD, Reviewer) for stress/fluid awareness |
| T021 | [P] | Add observability events for breakage, metrics, and backend selection |
| T022 | | Implement stress heatmap image generation in asset pipeline |
| T023 | | Update benchmark generator for fluid and stress objectives |
| T024 | | Integration tests: Backend parity (MuJoCo vs Genesis) |
| T025 | | Integration tests: Part breakage detection |
| T026 | | Integration tests: Fluid containment metrics |

## Work Packages

### WP01: Simulation Foundation & Abstraction
**Goal**: Establish the `PhysicsBackend` protocol and update data models.
**Priority**: P0
**Test**: Unit tests for schema validation; verify MuJoCo still runs via protocol.
**Included Subtasks**: T001, T002, T003, T004, T005
**Implementation Sketch**: Define Protocol -> Move MuJoCo code -> Create Genesis skeleton -> Extend YAML schemas.

**Depends on**: None

### WP02: Advanced Asset Pipeline
**Goal**: Support tetrahedralization and mesh repair for FEM.
**Priority**: P1
**Test**: Verify `.msh` files are generated from valid/invalid STLs.
**Included Subtasks**: T006, T007, T008
**Implementation Sketch**: Add trimesh repair -> Integrate Gmsh/TetGen -> Hook into worker asset pipeline.

**Depends on**: WP01

### WP03: Genesis FEM & Rigid Simulation
**Goal**: Implement core physics for rigid and deformable bodies in Genesis.
**Priority**: P1
**Test**: Run simulation with load; verify stress summaries and breakage.
**Included Subtasks**: T009, T010, T011, T012
**Implementation Sketch**: Map Genesis entities -> Load .msh -> Hook stress computation -> Implement abort-on-break.

**Depends on**: WP02

### WP04: Fluid Simulation (MPM)
**Goal**: Add MPM fluid support and evaluation metrics.
**Priority**: P2
**Test**: Verify particle containment ratio on a simple box benchmark.
**Included Subtasks**: T013, T014, T015
**Implementation Sketch**: Implement MPM particles -> Particle-in-box counting -> Flow crossing counting.

**Depends on**: WP03

### WP05: Agent Tools & Reasoning
**Goal**: Equip agents with tools to interact with the new physics.
**Priority**: P1
**Test**: Agent can call `get_stress_report` and correctly identifies high stress.
**Included Subtasks**: T016, T017, T018, T019, T020
**Implementation Sketch**: Implement tools -> Update prompts -> Add stress-aware logic to CAD engineer.

**Depends on**: WP03

### WP06: Observability, Benchmarks & UI
**Goal**: Finalize delivery with events, heatmaps, and benchmarking.
**Priority**: P2
**Test**: Verify stress heatmap images appear in simulation results.
**Included Subtasks**: T021, T022, T023
**Implementation Sketch**: Add events -> Implement heatmap renderer -> Extend benchmark planner logic.

**Depends on**: WP05

### WP07: Verification & Testing
**Goal**: Ensure system reliability via comprehensive integration tests.
**Priority**: P1
**Test**: All integration tests pass.
**Included Subtasks**: T024, T025, T026
**Implementation Sketch**: Write parity tests -> Write breakage tests -> Write fluid tests.

**Depends on**: WP06
