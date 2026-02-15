# Implementation Plan: Interactive Steerability and Design Feedback
*Path: kitty-specs/011-interactive-steerability-and-design-feedback/plan.md*

**Branch**: `011-interactive-steerability-and-design-feedback` | **Date**: 2026-02-15 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/011-interactive-steerability-and-design-feedback/spec.md`

## Summary

Implement a multi-modal "Steerability" framework to transition the AI from a generator to a co-pilot. Key features include topological selection (faces, parts, subassemblies) via `three-cad-viewer` with optimal isometric snapshots rendered by workers, targeted code steering using `@filename:line-range`, BOM @-mentions with autocomplete, and an in-memory async interaction queue for graceful mid-turn feedback.

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
