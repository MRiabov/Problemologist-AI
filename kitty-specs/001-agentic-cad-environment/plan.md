# Implementation Plan: Agentic CAD Environment

*Path: templates/plan-template.md*

**Branch**: `001-agentic-cad-environment` | **Date**: 2026-02-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/001-agentic-cad-environment/spec.md`

## Summary

Implement the **Agentic CAD Environment** using the **`deepagents`** framework. This involves setting up the **Controller Node** (LangGraph orchestration) and the **Worker Node** (Podman execution sandbox), connected via a strict OpenAPI contract. The environment abstracts the Worker's filesystem (backed by MinIO/S3) and provides the "Utils" library for agents to perform CAD/Sim actions via Python calls.

## Technical Context

**Language/Version**: Python 3.10+
**Frameworks**:

- `deepagents`: Middleware for Filesystem, Process management.
- `fastapi`: API for Worker node.
- `temporal`: Workflow orchestration for robust long-running tasks.
- `s3fs` / `minio`: Filesystem backend.
**Dependencies**:
- `build123d`: CAD kernel (Worker).
- `mujoco`: Physics (Worker).
- `langfuse`: Observability (Controller).
**Infrastructure**:
- **Controller**: Railway Service (Postgres DB).
- **Worker**: Podman Container (MinIO S3).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[No conflicts. Aligned with Distributed Architecture.]

## Project Structure

### Documentation (this feature)

```
kitty-specs/001-agentic-cad-environment/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Database Schemas
├── contracts/           # OpenAPI Spec (Controller <-> Worker)
└── tasks.md             # Tasks
```

### Source Code

```
src/
├── controller/           # Main Application (Railway)
│   ├── api/              # Frontend API
│   ├── graph/            # LangGraph Agents
│   └── persistence.py    # Postgres/S3 Logic
├── worker/               # Worker Container (Podman)
│   ├── app.py            # FastAPI Entrypoint
│   ├── filesystem/       # S3 FUSE/Virtual FS
│   ├── runtime/          # Python Execution Wrapper
│   └── utils/            # The "Utils" library agents import (simulate, validate)
└── deepagents/           # Shared Framework Integrations
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Distributed Architecture | Safety & Scalability. Agents execute arbitrary code. | Running code on the Controller is a massive security risk and blocks the event loop. |
| Temporal | Durability. Simulations take time and containers die. | Simple async/await fails if the container restarts or times out. |
| S3 Filesystem | Persistence across worker restarts & Observability. | Local ephemeral storage loses reasoning traces and code history on crash. |
