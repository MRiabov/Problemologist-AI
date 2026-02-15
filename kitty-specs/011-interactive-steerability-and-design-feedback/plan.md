# Implementation Plan: Interactive Steerability and Design Feedback
*Path: kitty-specs/011-interactive-steerability-and-design-feedback/plan.md*

**Branch**: `011-interactive-steerability-and-design-feedback` | **Date**: 2026-02-15 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/011-interactive-steerability-and-design-feedback/spec.md`

## Mandatory Reading

- **Roadmap (Package 4)**: [roadmap.md](../../roadmap.md)
- **WP4 Research Report**: [specs/wp4_research_report.md](../../specs/wp4_research_report.md)
- **Desired Architecture (Steerability)**: [specs/desired_architecture.md](../../specs/desired_architecture.md) (Lines 1016-1100)

## Summary

Implement a multi-modal "Steerability" framework to transition the AI from a generator to a co-pilot. Key features include topological selection (faces, parts, subassemblies) via `three-cad-viewer` with optimal isometric snapshots rendered by workers, targeted code steering using `@filename:line-range`, BOM @-mentions with autocomplete, and an in-memory async interaction queue for graceful mid-turn feedback.

## Identified Planning Gaps & Challenges

1.  **View Selection Logic**: We need a deterministic mapping between a selection's normal vector and the "best" of the 8 isometric views.
    - *Gap*: No utility currently exists in `worker/simulation/builder.py` to calculate the camera matrix from a face normal.
2.  **Agent Discovery Tool**: For the agent to translate `face_12` into a robust semantic selector, it needs a "Topological Inspector" tool that returns the face's properties (normal, center, area) without running a full simulation.
    - *Gap*: The `SimulationBuilder` needs a lightweight "inspection mode" that doesn't trigger MJCF generation.
3.  **Queue Injection into LangGraph**: The `asyncio.Queue` in the Controller must be checked at the end of every node execution in the LangGraph.
    - *Gap*: Current LangGraph implementation assumes a single user input at the start. We need a custom `Checkpointer` or a "Queue-Peeking" node that updates the state between tool calls.
4.  **ID Synchronization**: The names exported in the GLB (e.g., `face_12`) must be traceable back to the specific `build123d` object instance in the worker's memory.
    - *Gap*: Naming is currently transient during export; we need a way to "pin" a model version to a specific selection state to avoid index-shift bugs.

## Technical Context

**Language/Version**: Python 3.12 (Backend), TypeScript 5+ (Frontend)
**Primary Dependencies**: FastAPI (Controller), LangGraph/LangFuse (Agent logic), three-cad-viewer (Frontend), build123d (CAD Engine), Temporal (Orchestration)
**Storage**: In-memory `asyncio.Queue` (TurnQueue), S3-compatible storage (Snapshots), Postgres (User Preferences/Memory)
**Testing**: pytest (unit/integration), Playwright/Vitest (Frontend)
**Target Platform**: Linux (Railway/Workers)
**Project Type**: Full-stack (React Frontend + FastAPI Backend)
**Performance Goals**: <500ms for queue delivery; <1s for reference resolution/autocomplete.
**Constraints**: Snapshots must use worker-side headless rendering for consistency.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Rule 1 (Microservice-first)**: Isolate TurnQueue in the Controller; worker-side rendering logic stays in Worker activities.
- **Rule 2 (No Reinvention)**: Use `asyncio.Queue` for transient interaction queuing; use standard isometric projection math for snapshots.
- **Rule 3 (Early Stopping)**: Fail fast if geometric indices are stale or code references are out of bounds.

## Project Structure

### Documentation (this feature)

```
kitty-specs/011-interactive-steerability-and-design-feedback/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks/               # (Placeholder for /spec-kitty.tasks)
```

### Source Code

```
controller/
├── api/
│   ├── routes/
│   │   └── steerability.py  # New: Steering & Queue endpoints
├── services/
│   └── steerability/        # Logic for TurnQueue and Reference Resolution
├── persistence/
│   └── steering_memory.py   # User specific preferences

worker/
├── activities/
│   └── rendering.py         # New/Updated: Headless isometric snapshots
├── simulation/
│   └── builder.py           # Updated: Feature highlighting logic

frontend/
├── src/
│   ├── components/
│   │   ├── CADViewer/       # Selection modes & Mode Toggle
│   │   └── Chat/            # @-mention autocomplete & Queue status
│   └── services/
│       └── steerability.ts  # API clients
```

**Structure Decision**: Hybrid full-stack implementation across Controller (routing/state), Worker (heavy rendering), and Frontend (UX).

## Complexity Tracking

*No violations identified.*
