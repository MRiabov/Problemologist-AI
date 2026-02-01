---
work_package_id: WP02
title: Dashboard Shell & Text Components
lane: "doing"
dependencies: []
subtasks: [T005, T006, T007, T008]
agent: "gemini-cli"
shell_pid: "350435"
---

# WP02: Dashboard Shell & Text Components

**Goal**: Build the main Streamlit application layout and the text-based visualization components (Chat/logs and Code).

**Implementation Command**: `spec-kitty implement WP02 --base WP01`

## Subtasks

### T005: Create Layout Skeleton (`src/dashboard/main.py`)

**Purpose**: Main entry point and page structure.
**Steps**:

1. Set page config: data-testid="stMain", wide layout, title "Problemologist Dashboard".
2. Initialize `SessionState` for:
   - `selected_episode_id`
   - `selected_step_index`
   - `live_mode` (boolean)
3. Define the high-level layout:
   - Sidebar area.
   - Main area split: Two columns (Left: Chat/Code, Right: 3D/Artifacts). *Note: 3D part will be empty/placeholder for now.*

### T006: Implement Sidebar (`src/dashboard/components/sidebar.py`)

**Purpose**: Navigation and filtering.
**Steps**:

1. Create `render_sidebar()` function.
2. Add "Mode" toggle (Live / History).
3. **History Mode**:
   - Use `data.get_all_episodes()` to fetch list.
   - Show selectbox/radio to choose an Episode (format: `[Timestamp] - ID...`).
   - Store selection in `SessionState`.
4. **Step Navigation** (if an episode is selected):
   - Slider or Number Input to select `step_index`.
   - Update `SessionState.selected_step_index`.

### T007: Implement Chat Viewer (`src/dashboard/components/chat.py`)

**Purpose**: visualizes the Agent's reasoning (CoT) and interactions.
**Steps**:

1. Create `render_chat(steps: List[Step], current_index: int)` function.
2. Filter steps up to `current_index`.
3. Iterate and display:
   - **User Prompt**: `st.chat_message("user")`.
   - **Agent Thought**: `st.chat_message("assistant")`. Use distinct styling or markdown for reasoning traces.
   - **Tool Calls**: Display tool name and inputs (use `st.expander`).
   - **Tool Outputs**: Display logs/text output.

### T008: Implement Code Viewer (`src/dashboard/components/code.py`)

**Purpose**: Show generated code.
**Steps**:

1. Create `render_code(step: Step)` function.
2. Extract code from the step explanation or tool inputs (depending on how code is stored - e.g. `write_file` tool input).
3. If code is found, render using `st.code(body, language='python')`.
4. Handle cases where no code is generated in the current step (show placeholder or "No code changes").

## Integration

- Update `src/dashboard/main.py` to import and call `render_sidebar`.
- Based on selection, fetch full episode data using `data.get_episode_by_id`.
- Call `render_chat` in the left column.
- Call `render_code` in the left column (or a tab).

## Validation

- Run `uv run streamlit run src/dashboard/main.py`.
- Verify you can switch between episodes.
- Verify sliding the step slider updates the displayed chat history.
- Verify code snippets appear when relevant steps are selected.

## Activity Log

- 2026-02-01T13:53:25Z – gemini-cli – shell_pid=350435 – lane=doing – Started implementation via workflow command
