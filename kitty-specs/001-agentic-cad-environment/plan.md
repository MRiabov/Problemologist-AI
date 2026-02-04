# Implementation Plan: Agentic CAD Environment

*Path: templates/plan-template.md*

**Branch**: `001-agentic-cad-environment` | **Date**: 2026-01-31 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/001-agentic-cad-environment/spec.md`

## Summary

Implement the infrastructure for the **Agentic CAD Environment**, a `ToolRuntime`-based interface that allows an autonomous agent to generate `build123d` CAD models, validate them against physics tasks in `mujoco`, and persistently log all interactions.

Crucially, this plan covers the **Environment** (World) only, not the Agent (LLM).

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:

- `build123d`: Code-based CAD kernel
- `mujoco`: Physics engine
- `numpy`: Numerical operations
- `fastapi` / `uvicorn`: Host-Container communication (OpenAPI)
**Storage**: SQLAlchemy/SQLModel (`history.db`) for robust logging of episodes, steps, and artifacts via ORM.
**Testing**: `pytest` for unit tests; `schemathesis` for OpenAPI contract validation.
**Target Platform**: Linux (Development), Podman (Deployment/Sandbox).
**Security**: 
- **Strict Sandbox**: All agent-generated code executes inside a Podman container.
- **OpenAPI**: Communication between Host and Container is strictly typed via FastAPI.
- **Baked-in Env Code**: Supporting libraries and environment code are baked into the container image or mounted read-only.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[No conflicts identified. This is a core feature implementation.]

## Project Structure

### Documentation (this feature)

```
kitty-specs/001-agentic-cad-environment/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output (SQLite Schema)
├── contracts/           # Phase 1 output (Env Interface, Tool Definitions)
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```
src/
├── environment/
│   ├── runtime.py        # ToolRuntime (Core Orchestrator)
│   ├── tools.py          # Tool definitions (RAG, Edit, Preview)
│   └── persistence.py    # SQLite logger
├── workbenches/
│   ├── base.py           # Workbench ABC
│   └── print_3d.py       # MVP Workbench (Watertight + Cost)
├── compiler/
│   └── sim_client.py     # HTTP Client for Spec 003 Simulation Engine
└── rag/
    └── search.py         # Simple Grep-based documentation search
```

**Structure Decision**: Single Python project structure in `src/`.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Custom Geometry Compiler | Need to automate CAD -> Physics collision mesh generation | standard export doesn't handle convex decomposition for stable physics |
