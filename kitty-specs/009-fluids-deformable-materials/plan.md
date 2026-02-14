# Implementation Plan: Fluids & Deformable Materials

**Branch**: `009-fluids-deformable-materials` | **Date**: 2026-02-14 | **Spec**: [spec.md/spec.md](spec.md)
**Input**: Feature specification for WP2 Fluids/FEM.

## Summary

Extend the physics environment to support fluids and deformable materials using Genesis. We'll introduce a `PhysicsBackend` abstraction to switch between MuJoCo (rigid) and Genesis (FEM/MPM). The asset pipeline will be extended with tetrahedralization (Gmsh/TetGen) and mesh repair (trimesh) to support volumetric FEM meshes. Agents will be equipped with tools to analyze stress and optimize designs accordingly.

## Technical Context

**Language/Version**: Python 3.12
**Primary Dependencies**: `genesis`, `trimesh`, `gmsh`, `tetgen`, `pytorch` (for Genesis)
**Storage**: S3 for transient simulation data (MP4, stress arrays), PostgreSQL for metadata
**Testing**: `pytest` for backend parity and objective validation
**Target Platform**: Linux (Ubuntu 22.04+) with optional CUDA 12.x
**Project Type**: Multi-worker (Controller + Simulation Workers)
**Performance Goals**: < 1 minute for 10s fluid simulation on GPU; < 5 minutes on CPU.
**Constraints**: Meshes MUST be manifold for FEM; Genesis backend is fixed per execution.
**Scale/Scope**: Entire WP2 vision - complete rewrite of unreliable previous implementation.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- [x] Use Pydantic models for data exchange (Strict enforcement in `SimulationResult`)
- [x] Functional paradigm (Functions over classes where appropriate in `mesh_utils`)
- [x] Proper naming conventions (`spec-123 WP##` etc.)

## Project Structure

### Documentation (this feature)

```
kitty-specs/009-fluids-deformable-materials/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output (generated later)
```

### Source Code (repository root)

```
shared/
└── simulation/
    ├── backends.py      # Abstract interface
    └── schemas.py       # Pydantic models (StressSummary, etc.)

worker/
├── simulation/
│   ├── mujoco_backend.py
│   └── genesis_backend.py
└── utils/
    └── mesh_utils.py    # Tetrahedralization and Repair

controller/
└── simulation/
    └── loop.py          # Orchestration and breakage detection
```

**Structure Decision**: Integrated into the existing Controller-Worker architecture. `GenesisBackend` is the primary focus.

## Verification Plan

### Automated Tests

- `pytest tests/integration/test_physics_parity.py`: Verifies MuJoCo and Genesis produce similar results for rigid-body benchmarks.
- `pytest tests/integration/test_fem_breakage.py`: Tests `PART_BREAKAGE` failure mode.
- `pytest tests/integration/test_fluid_containment.py`: Tests MPM particle tracking and metrics.

### Manual Verification

1. Run a stress-limited benchmark and verify the agent's CAD output has added material in high-stress zones.
2. Verify fluid visualization in the frontend designs.
