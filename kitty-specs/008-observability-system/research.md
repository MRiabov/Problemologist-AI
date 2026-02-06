# Research: Observability System

## Observability Architecture

The system must track all agent movements, tool calls, and simulation results to enable reproducible debugging and model training.

### 1. Unified Logging: Structlog

**Decision**: Use `structlog` for all Python nodes (Controller/Worker).
**Rationale**:

- Provides JSON-formatted structured logs natively.
- Easy to grep and inject into the observability database.
- Supports context propagation across distributed boundaries.

### 2. LLM Tracing: LangFuse

**Decision**: Integrate **LangFuse** for high-fidelity LLM observability.
**Rationale**:

- Native support for LangGraph/LangChain.
- Tracks token usage, latency, and cost per episode.
- Provides a UI for reviewing agent "think" steps and tool call structures.

### 3. Schema Integrity: Schemathesis & Pydantic

**Decision**: Use `schemathesis` for API fuzzing and strict Pydantic v2 validation.
**Rationale**:

- Ensures that the Controller-Worker and Dashbord-Controller APIs stay in sync.
- Prevents silent data corruption in the distributed system.

### 4. Distributed Trace Reconstruction

**Decision**: Link assets (S3) to traces (Postgres).
**Rationale**:

- Traces are verbose; instead of duplicating them, we store the structure in Postgres and link to the relevant S3 assets (Git snapshots, videos).
- Allows "time-traveling" reconstruction of why an agent failed at any point in the history.
