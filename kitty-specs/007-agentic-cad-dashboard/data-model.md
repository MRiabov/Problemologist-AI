# Data Model: Dashboard Views

The dashboard interacts with the persistence layer but defines its own view-models for the UI transition.

## UI State Entities

### DashboardSession

- `mode`: Literal["Live", "History"]
- `selected_episode_id`: UUID | None
- `current_step_index`: int

### EpisodeView

- `id`: UUID
- `problem_id`: str
- `timestamp`: datetime
- `status`: str
- `total_steps`: int

### StepDetailView

- `index`: int
- `tool_name`: str
- `reasoning`: str (Extracted from tool_input or output)
- `code`: str (Extracted from tool_input or output)
- `output_log`: str
- `mesh_path`: str | None

## Data Fetching Contracts

### `get_episodes()`

Returns a list of `EpisodeView` summaries for the sidebar.

### `get_episode_detail(episode_id: UUID)`

Returns the full sequence of steps and associated artifacts.

### `get_latest_update(episode_id: UUID, last_step_index: int)`

Checks for new steps for the "Live" monitoring mode.
