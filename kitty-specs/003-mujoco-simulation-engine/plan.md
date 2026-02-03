# Implementation Plan: MuJoCo Simulation Engine & Safe Execution Environment

*Path: kitty-specs/003-mujoco-simulation-engine/plan.md*

**Branch**: `003-mujoco-simulation-engine` | **Date**: 2026-02-02 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/003-mujoco-simulation-engine/spec.md`

## Summary

The **MuJoCo Simulation Engine** has been upgraded to a **Safe Execution Environment**. It provides a secure, containerized backend for potentially untrusted agent code execution and physics validation.

It operates as a **Sandbox Manager** that:

1. Accepts a **Simulation Bundle** (Environment + Agent Design + Control Script).
2. Compiles this into a valid MuJoCo `scene.xml` (MJCF) with convex collision meshes.
3. Executes the simulation (or arbitrary python code) inside a **Podman Container** with strict resource and network limits.
4. Returns a **Simulation Report** (Success/Fail metrics) and execution logs.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:

- `mujoco`: Physics Engine (runs inside container)
- `build123d`: Geometry processing and export
- `trimesh`: Mesh convex decomposition (vhacd)
- `podman`: External CLI tool for container management
- **Docker/Podman Image**: Custom `problemologist-sandbox` image containing pre-installed dependencies.

**Storage**: **Ephemeral**. Artifacts (STL/MJCF/Videos) are written to a temporary workspace volume mounted to the container.
**Testing**: `pytest` with simple physics scenarios (block dropping, joint rotation).
**Target Platform**: Linux (requires Podman installed and configured for rootless mode).
**Security**: **Container Isolation**. Code is executed in a transient Podman container with `network=none` to prevent side-effects and ensuring a clean environment for every run.

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
└── environment/        # Refactored from simulation_engine
    ├── sandbox.py      # Podman Wrapper (PodmanSandbox)
    └── runner.py       # High-level execution logic
└── simulation/         # Simulation Logic (mounts into container)
    ├── builder.py      # Scene Compiler (CAD -> MJCF)
    └── loop.py         # Physics Loop (runs INSIDE container)
```

**Structure Decision**: Moved core execution logic to `environment/sandbox.py` to support general python execution. Simulation-specific logic resides in `simulation/` and is injected/mounted into the sandbox during runtime.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Container Isolation (Podman) | Agent code is untrusted, needs specific env, and can hang/crash | `multiprocessing` shares system libs and doesn't prevent file system damage or guarantee clean environment state. |
| Convex Decomposition | Physics engines require convex meshes for stability | Using raw concave STL meshes results in unstable collisions and "explosions" in MuJoCo. |

## Planning Questions & Discovery

*Resolved during Phase 0 & 1*

1. **Isolation Strategy**: How strictly to sandbox? *Decision*: **Podman Containers**. It provides near-VM isolation, allowing us to control CPU/Mem/Net and dependencies precisely.
2. **Mesh Generation**: `build123d` export vs `trimesh`? *Decision*: `build123d` for geometry, passed to `trimesh` for convex hull decomposition.
3. **Runner Architecture**: How to verify simulation? *Decision*: The "Loop" code runs *inside* the container, returning metrics via JSON to stdout. The Host system just parses the output.

## Refactoring & Alignment (Feb 3)

Based on the [Architecture Review](../../docs/code-smells-feb-3.md), the following changes are required:

1.  **Stateless Simulation**: Ensure `builder.py` and `loop.py` operate as pure functions or context-manager-scoped instances without reliance on global state.
2.  **Performance Caching**: Optimization (e.g., `lru_cache`) must be restricted to pure computational functions (mesh generation) and must NOT cache side-effects (like execution results).
