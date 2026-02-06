---
work_package_id: "WP07"
title: "Observability & Persistence"
lane: "planned"
dependencies: ["WP04"]
subtasks: ["T032", "T033", "T034", "T035"]
---

# WP07: Observability & Persistence

**Goal**: set up structured logging, Postgres persistence, and LangFuse.

## Context

We need high-fidelity observability to debug agents and train future models.

## Subtasks

### T032: Configure Structlog

File: `src/shared/logging.py` (Create shared module)

- Configure `structlog` to output JSON in production, colored text in dev.
- Add `trace_id` and `span_id` processors.
- Apply to both Controller and Worker.

### T033: Setup Postgres

File: `src/controller/persistence/db.py`

- Setup SQLAlchemy Async Engine.
- Configure Alembic for migrations.
- Env: `POSTGRES_URL`.

### T034: Define Schema

File: `src/controller/persistence/models.py`

- `Episode`: Tracks a full agent run.
- `Trace`: Stores raw LangChain traces (or link to LangFuse).
- `Asset`: S3 paths to videos/files generated.

### T035: Integrate LangFuse

File: `src/controller/observability/langfuse.py`

- Initialize `LangfuseHandler`.
- Attach to LangGraph agent (WP04).
- Verify traces are sent.

## Verification

1. Run Controller.
2. Check logs: should be JSON strings.
3. Check `alembic upgrade head` works.
4. Run Agent. Check LangFuse dashboard.
