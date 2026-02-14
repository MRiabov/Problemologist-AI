# Tasks: Fluids & Deformable Materials

**Feature Directory**: `kitty-specs/009-fluids-deformable-materials`

## Status

- [ ] WP01: Simulation Foundation & Abstraction <!-- id: WP01 -->
- [ ] WP02: Advanced Asset Pipeline <!-- id: WP02 -->
- [ ] WP03: Genesis FEM & Rigid Simulation <!-- id: WP03 -->
- [ ] WP04: Fluid Simulation (MPM) <!-- id: WP04 -->
- [ ] WP05: Agent Tools & Reasoning <!-- id: WP05 -->
- [ ] WP06: Observability, Benchmarks & UI <!-- id: WP06 -->
- [ ] WP07: Verification & Testing <!-- id: WP07 -->

## Work Packages

### WP01: Simulation Foundation & Abstraction

**Goal**: Establish the `PhysicsBackend` protocol and update data models.

- [ ] T001: Implement `PhysicsBackend` protocol in `shared/simulation/backends.py`.
- [ ] T002: Refactor MuJoCo implementation to `worker/simulation/mujoco_backend.py`.
- [ ] T003: Implement `GenesisBackend` skeleton in `worker/simulation/genesis_backend.py`.
- [ ] T004: Update `manufacturing_config.yaml` and schema with FEM material fields.
- [ ] T005: Update `SimulationResult` and shared schemas for stress/fluids.

**Implementation**:
1. Define Protocol.
2. Move MuJoCo.
3. Genesis Skeleton.

**Dependencies**: None
**Prompt Size**: ~350 lines

---

### WP02: Advanced Asset Pipeline

**Goal**: Support tetrahedralization and mesh repair for FEM.

- [ ] T006: Enhance `worker/utils/mesh_utils.py` with `trimesh` repair logic.
- [ ] T007: Implement tetrahedralization (Gmsh/TetGen) in `worker/utils/mesh_utils.py`.
- [ ] T008: Integrate tetrahedralization into worker's asset pipeline.

**Implementation**:
1. Mesh repair.
2. Gmsh/TetGen.
3. Pipeline integration.

**Dependencies**: WP01
**Prompt Size**: ~250 lines

---

### WP03: Genesis FEM & Rigid Simulation

**Goal**: Implement core physics for rigid and deformable bodies in Genesis.

- [ ] T009: Implement rigid-body support in `GenesisBackend`.
- [ ] T010: Implement FEM support in `GenesisBackend` (SoftMesh, .msh loading).
- [ ] T011: Implement stress computation and `StressSummary` reporting in Genesis.
- [ ] T012: Implement breakage detection logic in simulation loop.

**Implementation**:
1. Rigid support.
2. FEM support.
3. Stress reporting.
4. Breakage detection.

**Dependencies**: WP02
**Prompt Size**: ~400 lines

---

### WP04: Fluid Simulation (MPM)

**Goal**: Add MPM fluid support and evaluation metrics.

- [ ] T013: Implement MPM fluid support in `GenesisBackend`.
- [ ] T014: Implement `fluid_containment` metric evaluation.
- [ ] T015: Implement `flow_rate` metric evaluation.

**Implementation**:
1. MPM particles.
2. Containment metric.
3. Flow rate metric.

**Dependencies**: WP03
**Prompt Size**: ~300 lines

---

### WP05: Agent Tools & Reasoning

**Goal**: Equip agents with tools to interact with the new physics.

- [ ] T016: Implement `define_fluid` agent tool.
- [ ] T017: Implement `get_stress_report` agent tool.
- [ ] T018: Implement `preview_stress` agent tool.
- [ ] T019: Update `simulate` tool for Genesis/FEM parameters.
- [ ] T020: Update Agent prompts (Planner, CAD, Reviewer) for stress/fluid awareness.

**Implementation**:
1. Agent tools.
2. Prompt updates.

**Dependencies**: WP03
**Prompt Size**: ~500 lines

---

### WP06: Observability, Benchmarks & UI

**Goal**: Finalize delivery with events, heatmaps, and benchmarking.

- [ ] T021: Add observability events for breakage, metrics, and backend selection.
- [ ] T022: Implement stress heatmap image generation in asset pipeline.
- [ ] T023: Update benchmark generator for fluid and stress objectives.

**Implementation**:
1. Observability.
2. Heatmaps.
3. Benchmarks.

**Dependencies**: WP05
**Prompt Size**: ~300 lines

---

### WP07: Verification & Testing

**Goal**: Ensure system reliability via comprehensive integration tests.

- [ ] T024: Integration tests: Backend parity (MuJoCo vs Genesis).
- [ ] T025: Integration tests: Part breakage detection.
- [ ] T026: Integration tests: Fluid containment metrics.

**Implementation**:
1. Test suite.

**Dependencies**: WP06
**Prompt Size**: ~250 lines
