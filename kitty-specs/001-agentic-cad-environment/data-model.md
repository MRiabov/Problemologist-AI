# Data Model: Agentic CAD Environment

## Core Entities (Pydantic Models)

We use `pydantic.BaseModel` (v2) for all data exchange between Controller, Worker, and Frontend.

### 1. EpisodeMetadata

High-level tracking for an agentic session.

```python
from datetime import datetime
from typing import Any, Dict, List, Optional
from uuid import UUID
from enum import StrEnum
from pydantic import BaseModel, StrictStr, StrictInt, StrictBool, HttpUrl

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
    config_snapshot: Dict[StrictStr, Any]  # YAML config used for this run

### 2. SandboxCommand

Payload for executing a command on the worker.

```python
class SandboxCommand(BaseModel):
    command: StrictStr
    timeout_seconds: StrictInt = 300
    env_vars: Dict[StrictStr, StrictStr] = {}
    workdir: StrictStr = "."
```

### 3. SimulationResult (Feedback Model)

Data returned from the simulation engine to the agent.

```python
class SimulationResult(BaseModel):
    success: StrictBool
    summary: StrictStr  # Markdown summary for the agent
    video_url: Optional[HttpUrl] = None
    telemetry: Dict[StrictStr, List[float]]  # Coordinates, energy, etc.
    error_log: Optional[StrictStr] = None
```

### 4. TraceStep

A single tool call or reasoning block for observability.

```python
class TraceStep(BaseModel):
    step_id: UUID
    episode_id: UUID
    timestamp: datetime
    input: Dict[StrictStr, Any]  # Tool inputs or Prompt
    output: StrictStr  # Tool output or LLM Response
    internal_thought: Optional[StrictStr] = None
    journal_link: Optional[StrictStr] = None
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
