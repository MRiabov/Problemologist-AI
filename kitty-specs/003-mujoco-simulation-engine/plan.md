# Implementation Plan: MuJoCo Simulation Engine

*Path: kitty-specs/003-mujoco-simulation-engine/plan.md*

**Branch**: `003-mujoco-simulation-engine` | **Date**: 2026-01-31 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/003-mujoco-simulation-engine/spec.md`

## Summary

The **MuJoCo Simulation Engine** is a specialized backend service (FastAPI) responsible for the physics verification of agent-designed CAD models. It bridges the gap between static geometry (`build123d`) and dynamic function (`mujoco`).

It operates as an **ephemeral**, **stateless** service that:

1. Accepts a **Simulation Bundle** (Environment + Agent Design + Control Script).
2. Compiles this into a valid MuJoCo `scene.xml` (MJCF) with convex collision meshes.
3. Simulates the interaction in a **process-isolated sandbox** to execute untrusted agent control code safely.
4. Returns a **Simulation Report** (Success/Fail metrics) and optionally a replay trace.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:

- `fastapi`, `uvicorn`: API Server
- `mujoco`: Physics Engine
- `build123d`: Geometry processing and export
- `numpy`: Numerical data
- `trimesh`: Mesh convex decomposition (vhacd)
**Storage**: **Ephemeral**. No database. Artifacts (STL/MJCF/Videos) are temp-only or returned to caller.
**Testing**: `pytest` with simple physics scenarios (block dropping, joint rotation).
**Target Platform**: Docker (Linux). Needs `libgl1-mesa-glx` or similar for headless rendering if video needed.
**Security**: **Process Isolation** for Agent Scripts. Code is executed in a separate process (using `multiprocessing` or `subprocess`) to prevent main server crashes and allow resource limits.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[No conflicts identified. This feature focuses on robust execution and validation.]

## Project Structure

### Documentation (this feature)

```
kitty-specs/003-mujoco-simulation-engine/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output (API Schema)
├── contracts/           # Phase 1 output (OpenAPI spec)
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```
src/
└── simulation_engine/  # New Module
    ├── main.py         # FastAPI App & Entrypoint
    ├── api.py          # Routes (POST /simulate)
    ├── builder.py      # Scene Compiler (CAD -> MJCF)
    ├── simulation.py   # Physics Loop (Mujoco)
    └── runner.py       # Process Isolation & Sandbox Logic
```

**Structure Decision**: A dedicated `simulation_engine` package inside `src/`. It can be run as a library (by importing `runner`) or a server (via `main`).

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Process Isolation | Agent code is untrusted and can crash/hang | `eval()` in main loop would crash the entire server one infinite loop or segfault |
| FastAPI | Decoupling | Allows running simulation on a beefy GPU node separately from the main agent logic later |

## Planning Questions & Discovery

*Resolved during Phase 0*

1. **Isolation Strategy**: How strictly to sandbox? *Decision*: Standard `multiprocessing` with time-limit is sufficient for V1 (Trusted Internal Agent).
2. **Mesh Generation**: `build123d` export vs `trimesh`? *Decision*: `build123d` for geometry, passed to `trimesh` for convex hull decomposition.
