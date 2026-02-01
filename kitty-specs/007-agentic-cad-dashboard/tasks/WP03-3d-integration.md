---
work_package_id: WP03
title: 3D Visualization & Integration
lane: planned
dependencies: []
subtasks: [T009, T010, T011, T012]
---

# WP03: 3D Visualization & Integration

**Goal**: Implement the interactive 3D viewer, integrate it into the dashboard, and enable real-time updates.

**Implementation Command**: `spec-kitty implement WP03 --base WP02`

## Subtasks

### T009: Implement 3D Viewer Component (`src/dashboard/components/viewer_3d.py`)

**Purpose**: Render STL/mesh files using `stpyvista`.
**Steps**:

1. Import `pyvista` and `stpyvista`.
2. Create `render_3d_artifact(file_path: Path)` function.
3. **Logic**:
   - Check if file exists.
   - Load mesh: `mesh = pyvista.read(str(file_path))`.
   - Setup Plotter: `plotter = pyvista.Plotter(window_size=[400, 400])`.
   - Add mesh: `plotter.add_mesh(mesh, color='white', lighting=True)`.
   - Add axes: `plotter.add_axes()`.
   - Render: `stpyvista(plotter, key="3d_viewer")`.
4. Handle errors (invalid file, corrupt mesh) gracefully with `st.error`.

### T010: Integrate 3D Viewer

**Purpose**: Connect the viewer to the selected step's data.
**Steps**:

1. In `main.py` (Right Column), fetch artifacts for the current step using `data.get_step_artifacts`.
2. Filter for 3D file types (e.g., `.stl`, `.obj`).
3. If an artifact exists, resolve its path using `utils.resolve_artifact_path`.
4. Call `render_3d_artifact(path)`.
5. If multiple artifacts exist, allow selection via a dropdown (optional, or just show latest).

### T011: Implement Live Mode

**Purpose**: Auto-refresh the dashboard when "Live Mode" is execution.
**Steps**:

1. In `main.py`, inside the `if live_mode:` block:
   - Use `st.empty()` placeholder or `st.experimental_rerun()` loop (or `st_autorefresh` if allowed, otherwise simple sleep loop).
   - Polling: Check `data.get_latest_episode()` every X seconds.
   - If a new step/episode appears, update SessionState and rerun.
   - Ensure the UI doesn't "jump" annoyingly (preserve scroll if possible, or always scroll to bottom).

### T012: Final Polish & Error Handling

**Purpose**: Ensure robust UX.
**Steps**:

1. Add empty states: "No episode selected", "No artifacts for this step".
2. Styling: Check `index.css` (if using custom CSS) or standard Streamlit theming to match intended aesthetics (Dark mode preferred).
3. **Performance Check**: Ensure loading a heavy STL doesn't crash the tab (Streamlit limits).
4. Verify "Constitution" compliance (if applicable) or general best practices (docstrings, type hints).

## Validation

- **3D Test**: Run dashboard, select an episode with an STL artifact. Verify the 3D model appears and is rotatable.
- **Live Test**: Run a dummy script that writes to `history.db` in the background. Verify dashboard updates without manual refresh.
- **Navigation**: Switching steps updates the 3D model to the version corresponding to that step.
