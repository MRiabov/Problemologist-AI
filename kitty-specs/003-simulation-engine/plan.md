# Implementation Plan: MuJoCo Simulation Engine

Path: `kitty-specs/003-simulation-engine/plan.md`

**Branch**: `003-simulation-engine` | **Date**: 2026-02-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/003-simulation-engine/spec.md`

## Summary

Implement the **MuJoCo Simulation Engine** as a core `utils` library within the **Worker Node**. This features a complete pipeline to convert `build123d` CAD models into MJCF, execute physics simulations with collision/zone logic, render videos, and persist results to S3. Long-running simulations are managed via **Temporal**.

## Technical Context

**Language/Version**: Python 3.10+
**Dependencies**:

- `mujoco`: Physics.
- `ffmpeg-python`: Video encoding.
- `temporalio`: Orchestration.
- `minio`: S3 client.
- `build123d`: Geometry analysis.
**Infrastructure**:
- Worker Container (GPU preferred for rendering, but CPU fallback is required).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[No conflicts. Aligned with Reproducibility and Observability goals.]

## Project Structure

### Documentation

```text
kitty-specs/003-simulation-engine/
├── plan.md              # This file
├── research.md          # Research
├── data-model.md        # DB Schema
├── contracts/           # API
└── tasks.md             # Tasks
```

### Source Code

```text
src/worker/
├── simulation/
│   ├── builder.py       # CAD -> MJCF Compiler
│   ├── loop.py          # Physics Engine Loop
│   ├── renderer.py      # Video Generation
│   └── temporal/        # Temporal Activities
└── utils/
    └── simulation.py    # Public API (simulate())
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
| :--- | :--- | :--- |
| Temporal | Simulation reliability. | Running huge sims in a simple HTTP handler leads to timeouts and zombie processes. |
| FFmpeg | Video compression. | Raw frames fill up disk space instantly; GIFs are too large/low quality. |
| Automatic Zone Detection | UX for Agents. | Agents struggle to configure physics engine params manually; Geometry-based config is intuitive. |
