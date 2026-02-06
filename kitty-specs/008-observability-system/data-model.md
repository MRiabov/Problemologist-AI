# Data Model: Observability System

**Status**: Draft
**Spec**: `008-observability-system/spec.md`

## 1. Core Entities

We use SQLModel (Pydantic + SQLAlchemy) to define both the DB schema and validation models.

### Enums

```python
class StepType(str, Enum):
    THOUGHT = "thought"
    TOOL_CALL = "tool_call"
    TOOL_RESULT = "tool_result"
    ERROR = "error"
    PLAN_UPDATE = "plan_update"

class RunStatus(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"
```

### 1.1 Run (Episode)

Represents a single execution session of an agent.

| Field | Type | Description |
|-------|------|-------------|
| `id` | `UUID` | Primary Key |
| `agent_id` | `str` | Identifier of the agent (e.g., "benchmark-generator") |
| `task_input` | `JSON` | The initial prompt or configuration |
| `status` | `RunStatus` | Current state of execution |
| `created_at` | `DateTime` | UTC Timestamp |
| `updated_at` | `DateTime` | UTC Timestamp |
| `metadata` | `JSON` | Arbitrary context (user ID, environment) |

### 1.2 Step

An atomic action within a run. Strictly ordered.

| Field | Type | Description |
|-------|------|-------------|
| `id` | `UUID` | Primary Key |
| `run_id` | `UUID` | Foreign Key to Run |
| `sequence` | `int` | Ordered index (0, 1, 2...) |
| `type` | `StepType` | Discriminator for payload |
| `content` | `JSON` | The actual data (thought text, tool args, etc.) |
| `timestamp` | `DateTime` | When this step occurred |
| `latency_ms` | `float` | Execution time (if applicable) |

### 1.3 Artifact

A file or heavy object generated during a step.

| Field | Type | Description |
|-------|------|-------------|
| `id` | `UUID` | Primary Key |
| `step_id` | `UUID` | Foreign Key to Step |
| `role` | `str` | Purpose (e.g., "output_mesh", "debug_log") |
| `path` | `str` | Relative path to storage |
| `mime_type` | `str` | Media type |
| `size_bytes` | `int` | File size |

## 2. Pydantic Schemas (API / Event Stream)

These models are used for the `Event` stream payload.

```python
class EventPayload(BaseModel):
    run_id: UUID
    step_id: UUID
    sequence: int
    type: StepType
    timestamp: datetime
    data: Union[ThoughtData, ToolCallData, ToolResultData, ErrorData]

class ThoughtData(BaseModel):
    text: str
    reasoning: Optional[str]

class ToolCallData(BaseModel):
    tool_name: str
    arguments: Dict[str, Any]

class ToolResultData(BaseModel):
    tool_name: str
    output: Any
    is_error: bool

class ErrorData(BaseModel):
    message: str
    traceback: Optional[str]
    code: Optional[str]
```

## 3. Database Schema (SQLAlchemy/SQLModel)

* **Engine**: Postgres (Shared instance with different partitions for Temporal/App).
* **Migrations**: Alembic.
* **Naming Convention**: `obs_runs`, `obs_steps`, `obs_artifacts`.

```python
# SQLModel definition for central persistence
class ObsRun(SQLModel, table=True):
    __tablename__ = "obs_runs"
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    agent_id: str
    status: RunStatus
    created_at: datetime = Field(default_factory=datetime.utcnow)
```
