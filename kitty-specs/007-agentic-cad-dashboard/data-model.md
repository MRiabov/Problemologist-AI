# Data Model: Dashboard Views

## UI State Entities (Pydantic)

### 1. DashboardSession

```python
class DashboardSession(BaseModel):
    mode: Literal["live", "history"]
    selected_episode_id: Optional[UUID]
    current_step_index: int = 0
```

### 2. EpisodeSummary

```python
class EpisodeSummary(BaseModel):
    id: UUID
    problem_id: str
    start_time: datetime
    status: str
    total_steps: int
    summary_text: Optional[str]
```

### 3. StepDetail

```python
class StepDetail(BaseModel):
    sequence_index: int
    tool_name: str
    thought: str
    code_snippet: Optional[str]
    output_log: str
    asset_urls: List[str]  # S3 URLs for renders/meshes
```

## Data Fetching Contracts

Internal API endpoints provided by the FastAPI backend on the Controller:

1. **`GET /api/episodes`**: List all recorded episodes.
2. **`GET /api/episodes/{id}`**: Get full detail for a specific episode.
3. **`GET /api/episodes/{id}/live`**: SSE or WebSocket for real-time trace updates.
