---
work_package_id: WP07
title: Observability & Persistence
lane: "doing"
dependencies: [WP04]
base_branch: 001-agentic-cad-environment-WP04
base_commit: 371c916b02b74710a197bd76731c7afc0b293061
created_at: '2026-02-06T14:59:01.526671+00:00'
subtasks: [T032, T033, T034, T035]
shell_pid: "674358"
agent: "gemini-cli"
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
- **Reproducibility**: Include fields for **skill git hashes** and template versions in the `Episode` or metadata schemas.

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

## Activity Log

- 2026-02-06T14:59:01Z – gemini-cli – shell_pid=674358 – lane=doing – Assigned agent via workflow command
