# Implementation Plan: Observability System (Spec 008)

*Path: kitty-specs/008-observability-system/plan.md*

**Branch**: `008-observability-system` | **Date**: 2026-02-04 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/008-observability-system/spec.md`

## Summary

This feature implements a robust **Observability System** to capture, stream, and persist all agent activities (thoughts, tool calls, errors). It introduces a centralized **Observability SDK** used by the Controller and Worker nodes to log events to a **SQLite database** (WAL enabled). It also provides an endpoint to trigger **compressed S3 backups**.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:

- `sqlmodel` / `sqlalchemy`: ORM and Database interaction
- `alembic`: Database migrations
- `boto3`: S3 interactions for backup
- `fastapi` / `pydantic`: API and Schema validation
- `asyncio`: Internal event queue management

**Storage**: SQLite (`observability.db`) with WAL mode enabled for concurrency.
**Testing**: `pytest` for unit/integration tests; Manual verification for S3 backups.
**Constraints**:

- Single "Controller" node manages the DB writes and event stream.
- strictly-typed events (no `dict` dumping).
- minimal latency contribution to agent execution.

## Constitution Check

*GATE: Passed.*

- **Strict Typing**: Enforced via Pydantic/SQLModel.
- **Fail Fast**: Validation errors raise exceptions immediately.
- **Observability**: This *is* the observability feature itself.

## Project Structure

### Documentation

```
kitty-specs/008-observability-system/
├── plan.md              # This file
├── data-model.md        # Database and Event Schemas
├── contracts/           # OpenAPI specs (backup endpoint)
└── tasks.md             # Implementation tasks
```

### Source Code

```
src/
└── observability/
    ├── __init__.py      # Exports
    ├── models.py        # SQLModel entities (Run, Step, Artifact)
    ├── provider.py      # Main SDK (ObservabilityProvider)
    ├── backend/
    │   ├── sqlite.py    # DB Persistence logic
    │   └── s3.py        # Backup logic
    ├── api/
    │   ├── routes.py    # FastAPI routes (backup, stream)
    │   └── deps.py      # Dependencies
    └── events.py        # Internal Pub/Sub (asyncio)

tests/
    └── observability/   # New test suite
```

**Structure Decision**: Created a top-level `observability` package to serve as a platform-level dependency for `agent`, `environment`, and `api` layers.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| `SQLModel` + `Alembic` | Need robust schema migrations and strict typing for long-term project health. | Raw SQLite or JSON files lack strict schemas and migration safety, leading to data corruption risk over time. |
| In-process Queue | Decouple logging from execution loop to avoid blocking operations. | Blocking writes would slow down the agent interactions noticeably. |

## Proposed Changes

### [NEW] Observability Core

#### [NEW] `src/observability/models.py`

- Define `ObsRun`, `ObsStep`, `ObsArtifact` using SQLModel.
- Define Pydantic models for Event payloads.

#### [NEW] `src/observability/backend/sqlite.py`

- Implement `SQLiteBackend` with `create_run`, `log_step`.
- Ensure WAL mode PRAGMA is execution.

#### [NEW] `src/observability/backend/s3.py`

- Implement `S3Backup` class with `create_backup(upload: bool = True)`.
- Use `shutil.make_archive` for compression.

#### [NEW] `src/observability/provider.py`

- Implement `ObservabilityProvider` singleton/context manager.
- Method `log(step_type, data)` that pushes to queue.
- Background worker consumes queue -> DB.

### [NEW] API & Integration

#### [NEW] `src/observability/api/routes.py`

- `POST /observability/backup`: Triggers backup.

#### [MODIFY] `src/agent/graph/graph.py`

- Inject `ObservabilityProvider` into the graph state or ensuring it's accessible.
- Update nodes to call `provider.log(...)`.

## Verification Plan

### Automated Tests

- **Unit Tests**:
  - `pytest tests/observability/test_models.py`: Verify Pydantic validation.
  - `pytest tests/observability/test_provider.py`: Verify queue consumption and event emission.
- **Integration Tests**:
  - `pytest tests/observability/test_persistence.py`: Run a mock agent flow, ensure data is in SQLite.
- **Backup Verification**:
  - Mock S3 in tests to verify `boto3.upload_file` is called with a `.tar.gz`.

### Manual Verification

1. **End-to-End Agent Run**:
   - Run a benchmark generation job.
   - Inspect `observability.db` using a SQLite viewer to confirm rows exist.
2. **Dashboard Stream (Simulated)**:
   - Connect a dummy websocket/listener (if applicable) and assert events arrive.
3. **Backup Trigger**:
   - Call `curl -X POST http://localhost:8000/observability/backup`.
   - Verify log message "Backup successful: s3://...".
