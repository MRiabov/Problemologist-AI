# Data Model: Engineer Agent

## 1. Traceability & Logs (Database)

All interactions, reasoning, and tool calls are persisted to the **Postgres/SQLAlchemy** database (or SQLite in local development) as defined in the system architecture.

### 1.1. Run Persistence

| Entity | Fields |
|--------|--------|
| **Episode** | `id`, `prompt`, `start_time`, `result_status`, `total_cost` |
| **Step** | `id`, `episode_id`, `phase` (Architect/Engineer/Critic), `thought`, `tool_call`, `observation`, `timestamp` |
| **Artifact** | `id`, `step_id`, `path_to_mesh`, `path_to_render`, `code_snapshot` |

## 2. Long-Term Memory

### 2.1. Journal (`journal.md`)

A flat Markdown file for storing learned lessons.

```markdown
# Journal

## [Topic: Syntax] build123d Loft
* **Date**: 2023-10-27
* **Lesson**: The `loft` operation requires profiles to be aligned. Use `align=True`.
* **Context**: Failed to loft between circle and square.

## [Topic: Geometry] Press Fits
* **Date**: 2023-10-28
* **Lesson**: For PLA printing, add 0.2mm tolerance to holes.
```

## 3. State Management

### 3.1. Session Context

In-memory object tracking the current run.

| Field | Type | Description |
|-------|------|-------------|
| `goal` | str | The user's original prompt |
| `max_unit_cost` | float | Target budget for the assembly |
| `target_quantity` | int | Intended production volume |
| `plan` | List[PlanItem] | Steps to solve the problem |
| `history` | List[Message] | Chat history (sliding window) |
| `scratchpad` | Dictionary | Current variables/observations |
| `attempts` | int | Number of `submit_design` calls |
| `force_submit_flag`| bool | Whether Actor has signaled cost limit reached |

### 3.2. Plan Item

```python
class PlanItem(BaseModel):
    id: str
    description: str
    status: Literal["pending", "in_progress", "done", "failed"]
    dependencies: List[str]
```
