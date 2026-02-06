# Data Model: Agentic CAD Environment

## Core Entities (Pydantic Models)

We use `pydantic.BaseModel` for all data exchange between the Controller and Worker nodes.

### 1. EpisodeState

Tracks the high-level state of an agentic session.

```python
class EpisodeState(BaseModel):
    episode_id: UUID
    status: Literal["planning", "executing", "verifying", "success", "failed"]
    problem_id: str
    start_time: datetime
    last_updated: datetime
    metrics: Dict[str, float] = {}
```

### 2. SandboxFile

Represents a file within the ephemeral worker sandbox.

```python
class SandboxFile(BaseModel):
    path: str  # Relative to sandbox root
    content: str
    is_readonly: bool = False
    content_hash: str
```

### 3. StepTrace

A single reasoning or tool-calling step, persisted for observability.

```python
class StepTrace(BaseModel):
    step_id: UUID
    episode_id: UUID
    sequence_index: int
    tool_name: str
    tool_input: Dict[str, Any]
    tool_output: str
    duration_ms: int
    journal_entry: Optional[str]
```

## Persistence Strategy

1. **Relational Data**: Stored in a central **Postgres** database (via SQLAlchemy) on the Controller node.
2. **Artifacts (Renders, Meshes, Videos)**: Uploaded to **S3 (MinIO/Railway Buckets)**. The DB stores the S3 URL.
3. **Orchestration State**: Managed by **Temporal** in a dedicated Postgres partition.
4. **Episodic Memory**: Stored in the `journal.md` within the worker's filesystem, synced back to the observability DB on completion.
