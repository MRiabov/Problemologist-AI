# Research: Agentic CAD Dashboard Technical Details

## Persistence Schema Discovery

The dashboard needs to read from the "Thought-Process" database. Research into `src/environment/persistence.py` revealed the following SQLAlchemy models:

- **Episode**: Tracks a single run (problem_id, start_time, status, result_metrics).
- **Step**: Sequential steps within an episode (sequence_index, tool_name, tool_input, tool_output, duration_ms).
- **Artifact**: Files generated during a step (artifact_type, file_path, content_hash).

### Key Findings for Dashboard

- SQLite WAL mode is enabled in `DatabaseManager`, which is good for concurrent reads (dashboard) and writes (agent).
- Artifacts are linked to Steps, which allows correlating visual output with specific reasoning/tool-calls.

## 3D Rendering with stpyvista

`stpyvista` is confirmed as the rendering engine.

### Integration Pattern

1. Load mesh (STL) using `pyvista.read()` or `trimesh.load()`.
2. Wrap in a `pyvista` plotter.
3. Use `stpyvista.st_pyvista(plotter)` to render in Streamlit.

### Performance Considerations

- PyVista rendering is client-side (via vtk.js behind stpyvista).
- Large meshes should be simplified if latency becomes an issue, though MVP meshes are expected to be light.

## Directory Structure Design

Proposed structure for the dashboard module:

```
src/dashboard/
├── main.py          # Streamlit entry point
├── data.py          # Data access layer (SQLAlchemy queries)
├── components/      # UI components (Chat, Code, 3D Viewport)
│   ├── chat.py
│   ├── code_viewer.py
│   └── viewer_3d.py
└── utils.py         # Formatting and path resolution
```
