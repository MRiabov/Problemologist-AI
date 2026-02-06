# Data Model: Agentic CAD Environment

## Core Entities (Pydantic Models)

We use `pydantic.BaseModel` (v2) for all data exchange between Controller, Worker, and Frontend.

### 1. EpisodeMetadata

High-level tracking for an agentic session.

```python
class AgentType(StrEnum):
    ENGINEER = "engineer"
    BENCHMARK_GENERATOR = "benchmark_generator"
    PLANNER = "planner"
    REVIEWER = "reviewer"

class EpisodeStatus(StrEnum):
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    PREEMPTED = "preempted"

class EpisodeMetadata(BaseModel):
    episode_id: UUID
    agent_type: AgentType
    status: EpisodeStatus
    start_time: datetime
    end_time: Optional[datetime] = None
    config_snapshot: Dict[str, Any]  # YAML config used for this run
```

### 2. SandboxCommand

Payload for executing a command on the worker.

```python
class SandboxCommand(BaseModel):
    command: str
    timeout_seconds: int = 300
    env_vars: Dict[str, str] = {}
    workdir: str = "."
```

### 3. SimulationResult (Feedback Model)

Data returned from the simulation engine to the agent.

```python
class SimulationResult(BaseModel):
    success: bool
    summary: str  # Markdown summary for the agent
    video_url: Optional[HttpUrl] = None
    telemetry: Dict[str, List[float]]  # Coordinates, energy, etc.
    error_log: Optional[str] = None
```

### 4. TraceStep

A single tool call or reasoning block for observability.

```python
class TraceStep(BaseModel):
    step_id: UUID
    episode_id: UUID
    timestamp: datetime
    input: Dict[str, Any]  # Tool inputs or Prompt
    output: str  # Tool output or LLM Response
    internal_thought: Optional[str] = None
    journal_link: Optional[str] = None
```

## Persistence Strategy

1. **Postgres (Relational)**:
    - `episodes`, `traces`, `tasks`: Main business logic and state.
    - `temporal`: Orchestration state (in a separate DB partition).
2. **S3 (Artifacts)**:
    - `/assets/videos/`: Simulation recordings.
    - `/assets/renders/`: Multi-view images.
    - `/assets/snapshots/`: Git bundle of worker filesystem at decision points.
3. **Local Worker Cache (SQLite)**:
    - Ephemeral caching of `validate_and_price` results to speed up retries.
4. **Episodic Memory (Filesystem)**:
    - `journal.md`, `todo.md`: Core working files synced to S3 on completion.
