# Dashboard Contracts: Database Schema

The dashboard expects the following schema in `history.db`. This matches the definitions in `src/environment/persistence.py`.

## Episodes Table

| Column | Type | Description |
|--------|------|-------------|
| id | UUID | Primary Key |
| problem_id | String | Reference to the task |
| start_time | DateTime | Start of the episode |
| status | String | started, success, failure |
| result_metrics| JSON | Optional KPIs |

## Steps Table

| Column | Type | Description |
|--------|------|-------------|
| id | UUID | Primary Key |
| episode_id | UUID | FK -> Episodes |
| sequence_index| Integer | Order of steps |
| tool_name | String | Name of the tool called |
| tool_input | Text | Raw input (JSON/String) |
| tool_output | Text | Raw output/logs |
| duration_ms | Integer | Execution time |

## Artifacts Table

| Column | Type | Description |
|--------|------|-------------|
| id | UUID | Primary Key |
| step_id | UUID | FK -> Steps |
| artifact_type | String | mesh, image, log, etc. |
| file_path | String | Path relative to root or absolute |
| content_hash | String | For cache validation |
