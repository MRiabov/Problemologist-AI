# Implementation Plan: Agentic CAD Dashboard & 3D Debugger

*Path: kitty-specs/007-agentic-cad-dashboard/plan.md*

**Branch**: `main` | **Date**: 2026-02-01 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/007-agentic-cad-dashboard/spec.md`

## Summary

The **Agentic CAD Dashboard** is a Streamlit-based visual interface designed for real-time monitoring and historical review of the AI agent's 3D design process. It integrates interactive 3D rendering using `stpyvista` to allow developers to inspect generated models (STLs) alongside the agent's reasoning and code.

## Technical Context

**Language/Version**: Python 3.12  
**Primary Dependencies**: `streamlit`, `stpyvista`, `pyvista`, `sqlalchemy`, `trimesh`  
**Storage**: SQLite (`history.db`) with WAL mode for concurrent read-only access.  
**Testing**: `pytest` for data layer and utility functions.  
**Target Platform**: Linux Desktop / Local Development  
**Project Type**: Single project (Dashboard module within existing source tree)  
**Performance Goals**:

- Render STL files < 2 seconds.
- Real-time updates < 1 second after DB commit.
**Constraints**:
- Read-only database connection to avoid interference with the agent.
- Access to local `artifacts/` directory for mesh retrieval.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[SKIPPED: Constitution file not present in repository.]

## Project Structure

### Documentation (this feature)

```
kitty-specs/007-agentic-cad-dashboard/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technical research and discoveries
├── data-model.md        # UI state and data fetching contracts
├── quickstart.md        # Setup and execution guide
├── contracts/           # API/Data schema definitions
└── tasks.md             # Implementation tasks (generated later)
```

### Source Code (repository root)

```
src/
└── dashboard/           # Dashboard module
    ├── main.py          # Entry point (Streamlit app)
    ├── data.py          # Data layer (SQLAlchemy integration)
    ├── components/      # UI components
    │   ├── chat.py      # Reasoning and logs viewer
    │   ├── code.py      # Syntax-highlighted code viewer
    │   ├── viewer_3d.py # interactive PyVista viewport
    │   └── benchmark_gen.py # Interactive generator pipeline
    └── utils.py         # Path resolution and formatting
```

**Structure Decision**: A dedicated `dashboard` directory inside `src/`. This follows the existing project organization while keeping the UI code isolated from the agent logic.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| stpyvista | Specialized 3D visualization | `st.code` or static images are insufficient for debugging complex geometry. |
| SQLite WAL | Concurrent access | Standard mode would cause database locks when the agent and dashboard access `history.db` simultaneously. |
