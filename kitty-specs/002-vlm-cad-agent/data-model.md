# Data Model: Engineer Agent

## Core Entities (Pydantic Models)

We use Pydantic models for structured state management within the `deepagents` graph.

### 1. AgentState (Pydantic)

```python
class AgentState(BaseModel):
    plan: List[PlanItem]
    current_task_id: Optional[str]
    journal: List[JournalEntry]
    inventory: List[PartInfo]
    total_cost: float = 0.0
    iteration_count: int = 0
```

### 2. PlanItem

```python
class PlanItem(BaseModel):
    id: str
    description: str
    status: Literal["pending", "in_progress", "done", "failed"]
    dependencies: List[str]
    assigned_to: Literal["architect", "engineer", "critic"]
```

### 3. JournalEntry

```python
class JournalEntry(BaseModel):
    topic: str
    lesson: str
    context: str
    timestamp: datetime = Field(default_factory=datetime.now)
```

## Persistence

1. **Step Traces**: Persisted to the global `StepTrace` model in the observability DB.
2. **Episodic Data**: The `journal.md` and `todo.md` are persisted in the worker sandbox and archived to S3.
3. **Checkpoints**: LangGraph state is checkpointed in **Postgres** for Human-in-the-Loop (HITL) support.
