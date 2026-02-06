# Data Model: Observability System

## Core Observability Models (Pydantic)

### 1. StepTrace

A single step in an episode, encompassing reasoning and results.

```python
class StepTrace(BaseModel):
    step_id: UUID
    episode_id: UUID
    sequence_index: int
    thought: str
    tool_calls: List[ToolCall]
    tool_results: List[ToolResult]
    artifacts: List[str]  # S3 URLs
    metadata: Dict[str, Any]
```

### 2. EpisodeMetadata

High-level session tracking.

```python
class EpisodeMetadata(BaseModel):
    episode_id: UUID
    agent_id: str
    status: Literal["running", "success", "failed", "interrupted"]
    start_time: datetime
    end_time: Optional[datetime]
    token_usage: Dict[str, int]
    cost_usd: float
```

## Persistence strategy

1. **Relational (Postgres)**: Traces and Metadata are stored in the Observability DB for sub-second retrieval.
2. **Deep Traces (LangFuse)**: Detailed LLM internals are streamed directly to LangFuse.
3. **Assets (S3)**: Heavy artifacts (meshes, videos, git diffs) are stored in S3 and linked via `StepTrace.artifacts`.
