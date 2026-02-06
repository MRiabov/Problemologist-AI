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

### 1.1 Run (Episode)

- **Agent State**: Persisted via LangGraph Checkpointer in **Postgres**.
- **LLM Traces**: Streamed to **Langfuse** (Postres-backed).
- **Execution Logs**: Captured and stored in **Postgres** on the Controller.

### 1.2 Storage

- **Artifacts**: All light and heavy artifacts (Meshes, Videos, Code) are stored in **S3**.
- **Persistence**: Any worker-side persistence is ephemeral; the source of truth is the central Controller database.

## 4. Technical Design

### 4.1. Tech Stack

- **Datastores**: Postgres (Controller), S3 (Global Asset Store).
- **Tracing**: Langfuse.
- **Durable Execution**: Temporal.
- **Models**: Pydantic/SQLAlchemy for strict typing.

### 4.2. Data Flow

1. **Agent Action**: LangGraph Node executes.
2. **Trace**: LangFuse Handler catches LLM I/O.
3. **State**: Checkpointer writes to Postgres `checkpoints`.
4. **Tool Call**:
    - Agent calls `write_file`.
    - Worker writes to local sandbox.
    - Worker optional: uploads specifically designated assets to S3.
5. **Simulation**:
    - Worker generates `sim.mp4` in `/renders/`.
    - `CompositeBackend` transparently routes write to S3.
    - Worker returns `{"video_url": "..."}`.
    - Controller saves URL in Step Output.
