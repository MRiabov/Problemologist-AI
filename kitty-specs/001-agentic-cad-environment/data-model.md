# Data Model: Agentic CAD Environment

## Database Schema (SQLite)

The persistence layer uses a local SQLite database `history.db`.

### 1. Episodes

Represents a single attempt by an agent to solve a problem.

| Column | Type | Description |
|---|---|---|
| `id` | TEXT (UUID) | Primary Key. |
| `start_time` | DATETIME | ISO8601 Timestamp. |
| `problem_id` | TEXT | Identifier of the problem scenario (e.g., "move_block_v1"). |
| `status` | TEXT | 'running', 'completed', 'failed', 'error'. |
| `result_metrics` | JSON | Final scores (success, energy, damage, cost). |

### 2. Steps

Represents a single action taken by the agent within an episode.

| Column | Type | Description |
|---|---|---|
| `id` | TEXT (UUID) | Primary Key. |
| `episode_id` | TEXT | Foreign Key to Episodes. |
| `sequence_index` | INTEGER | Order of execution (0, 1, 2...). |
| `tool_name` | TEXT | 'write_script', 'preview', 'submit', etc. |
| `tool_input` | TEXT | The full arguments passed to the tool. |
| `tool_output` | TEXT | The text response (stdout, error, or return value). |
| `duration_ms` | INTEGER | Execution time in milliseconds. |

### 3. Artifacts

Binary or large text files generated during a step.

| Column | Type | Description |
|---|---|---|
| `id` | TEXT (UUID) | Primary Key. |
| `step_id` | TEXT | Foreign Key to Steps. |
| `artifact_type` | TEXT | 'script', 'render', 'mesh', 'log'. |
| `file_path` | TEXT | Relative path to the file on disk (storage is hybrid DB/FILESYSTEM). |
| `content_hash` | TEXT | SHA256 of content. |

> **Note**: While the spec mentioned storing code snapshots in DB, for large renders/meshes, we stick to storing paths in DB and files on disk to keep DB lean, or strict text for scripts.

## Object Model (Python)

```python
@dataclass
class ProblemScenario:
    id: str
    description: str
    template_path: str  # Path to MJCF xml template
    goals: Dict[str, Any]

@dataclass
class ValidationResult:
    success: bool
    metrics: Dict[str, float]
    violation: Optional[str]
```
