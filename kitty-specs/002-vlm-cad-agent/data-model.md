# Data Model: VLM CAD Agent

## 1. Traceability & Logs

### 1.1. Agent Trace (`agent_trace.jsonl`)

Every step in the ReAct loop is logged as a JSON object.

```json
{
  "trace_id": "uuid-v4",
  "session_id": "session-timestamp",
  "timestamp": "ISO-8601",
  "step_index": 0,
  "phase": "Plan|Code|Review",
  "input": {
    "role": "user|system|assistant",
    "content": "..."
  },
  "thought": "I need to check the dimensions...",
  "tool_calls": [
    {
      "tool": "preview_design",
      "args": {}
    }
  ],
  "observation": {
    "output": "Image generated...",
    "image_path": "traces/img_01.png"
  },
  "cost": {
    "tokens_in": 100,
    "tokens_out": 50,
    "model": "gpt-4o"
  }
}
```

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
| `plan` | List[PlanItem] | Steps to solve the problem |
| `history` | List[Message] | Chat history (sliding window) |
| `scratchpad` | Dictionary | Current variables/observations |
| `attempts` | int | Number of `submit_design` calls |

### 3.2. Plan Item

```python
class PlanItem(BaseModel):
    id: str
    description: str
    status: Literal["pending", "in_progress", "done", "failed"]
    dependencies: List[str]
```
