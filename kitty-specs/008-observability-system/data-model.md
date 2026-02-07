# Data Model: Observability System

## Core Observability Models (Pydantic)

### 1. TraceEvent

Captures discrete reasoning or execution events.

```python
class TraceEvent(BaseModel):
    trace_id: StrictStr
    agent_id: StrictStr
    input_tokens: Optional[StrictInt]
    output_tokens: Optional[StrictInt]
    latency_ms: Optional[float]
    content: Any
    timestamp: datetime
```

### 2. AssetRecord

Links heavy artifacts in S3 to specific traces.

```python
class AssetRecord(BaseModel):
    asset_id: StrictStr
    trace_id: StrictStr
    s3_key: StrictStr
    asset_type: AssetType # Enum: VIDEO, IMAGE, LOG
    created_at: datetime
```

### 3. StepTrace (Legacy/Internal)

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

### 4. EpisodeMetadata (Legacy/Internal)

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
3. **Assets (S3)**: Heavy artifacts (meshes, videos, git diffs) are stored in S3 and linked via `AssetRecord`.