# Data Model: Dashboard Views

## UI Entities (Pydantic Models)

### 1. DashboardUpdate (Live Stream)

The payload for real-time UI updates via WebSockets.

```python
class DashboardUpdate(BaseModel):
    update_type: Literal["log", "trace", "asset", "status_change"]
    content: Any  # Polymorphic based on type
    timestamp: datetime
```

### 2. InterruptionRequest

Payload for human intervention.

```python
class InterruptionRequest(BaseModel):
    episode_id: UUID
    command: Literal["stop", "pause", "resume", "rewrite_plan"]
    message: Optional[str] = None
```

## API Contracts

1. **`GET /api/episodes`**: Paginated list of all engineering/benchmark sessions.
2. **`GET /api/episodes/{id}/assets`**: URLs for videos, meshes, and rendered images.
3. **`POST /api/episodes/{id}/interrupt`**: Endpoint for HITL control.
4. **`GET /api/episodes/{id}/ws`**: WebSocket endpoint for live monitoring.
