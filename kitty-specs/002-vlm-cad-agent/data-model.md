# Data Model: Engineer Agent

## Graph State (LangGraph)

The internal state of the agent graph, managed by LangGraph and checkpointed in Postgres.

### 1. EngineerState

```python
class EngineerState(BaseModel):
    """The state shared across Architect, Engineer, and Critic."""
    episode_id: UUID
    goal: str
    todo_list: List[TodoItem]
    current_script_hash: Optional[str]
    last_simulation_result: Optional[SimulationResult]
    iteration_count: int
    is_finished: bool
```

### 2. TodoItem (Pydantic)

Synchronized with the `todo.md` on the worker.

```python
class TodoItem(BaseModel):
    id: str
    task: str
    status: Literal["todo", "doing", "done", "refused"]
    reflection: Optional[str] = None
```

## Worker Filesystem State

These files are the "ground truth" for the agent's work.

- **`journal.md`**: Persisted narrative.
- **`todo.md`**: Current execution plan.
- **`plan.md`**: High-level architectural plan.
- **`script.py`**: The solution code.

## Persistence Strategy

1. **State Checkpoints**: LangGraph checkpoints are stored in the **Postgres** DB on the Controller for HITL resumption.
2. **Episodic Narrative**: `journal.md` is synced to the **Observability DB** as a `LongTermMemory` record.
3. **Trace Alignment**: All tool calls are tagged with `agent_persona` (architect/engineer/critic) in LangFuse.
