# Feature Specification: Observability System

**Feature**: 008-observability-system
**Status**: Draft
**Mission**: software-dev

## 1. Overview

The **Observability System** ensures that every action taken by the **Problemologist** agents—from high-level reasoning to low-level physics simulation—is captured, structured, and persisted.

It distinguishes between **Controller Observability** (LLM Traces, Agent State) and **Worker Observability** (Execution Logs, Simulation Artifacts).

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Unified Tracing**: Use **LangFuse** to track LLM execution trees on the Controller.
2. **Asset Persistence**: Store heavy artifacts (Videos, Git Archives, Images) in **S3** (Railway/MinIO).
3. **Strict Contracts**: Enforce OpenAPI schemas for all observability events using `schemathesis`.
4. **Disaster Recovery**: Automated daily backups of Postgres and S3 buckets.

### 2.2. Success Criteria

- **Trace Coverage**: 100% of Agent Steps (Thoughts/Tools) are visible in LangFuse.
- **Asset Linkage**: Every "Step" in the DB links to relevant S3 artifacts (e.g., "Result: [Video](s3://...)").
- **Schema Validation**: Zero API contract violations during CI/CD fuzzing.

## 3. Functional Requirements

### 3.1. Controller Observability (Postgres + LangFuse)

- **Agent State**: Persisted via LangGraph Checkpointer (Postgres).
- **LLM Traces**: Streamed to LangFuse (Self-hosted or Cloud).
- **Structured Logs**: `structlog` for application events (Info/Error).

### 3.2. Worker Observability (S3 + Ephemeral)

- **Execution Logs**: Captured stdout/stderr from `run_command`, returned to Controller, and stored in Postgres.
- **Simulation Artifacts**:
  - **Videos**: Rendered `.mp4` uploaded to S3.
  - **Git Bundles**: `.tar.gz` of the workspace uploaded to S3 after significant steps.

### 3.3. Schemas & Contracts

- **OpenAPI**: The Controller <-> Worker API is strictly typed.
- **Validation**: `schemathesis` runs in CI to fuzz inputs/outputs against the schema.

### 3.4. Backups

- **Postgres**: Standard Railway backups / pg_dump to S3.
- **S3**: Lifecycle policies to archive old videos (Cost optimization).

## 4. Technical Design

### 4.1. Tech Stack

- **Databases**: Postgres (Controller), SQLite (Local Dev / Worker Ephemeral).
- **Tracing**: `langfuse` Python SDK.
- **Storage**: AWS S3 / MinIO.
- **Logs**: `structlog`.
- **Testing**: `schemathesis`.

### 4.2. Data Flow

1. **Agent Action**: LangGraph Node executes.
2. **Trace**: LangFuse Handler catches LLM I/O.
3. **State**: Checkpointer writes to Postgres `checkpoints`.
4. **Tool Call**:
    - Agent calls `write_file`.
    - Worker writes to MinIO.
    - Worker logs access.
5. **Simulation**:
    - Worker generates `sim.mp4`.
    - Worker uploads to Global S3.
    - Worker returns `{"video_url": "..."}`.
    - Controller saves URL in Step Output.
