# Dashboard Quickstart

## Installation

Ensure you have the development dependencies installed:

```bash
uv sync --group dev
```

## Running the Dashboard

Launch the Streamlit app from the repository root:

```bash
uv run streamlit run src/dashboard/main.py
```

The dashboard will be available at `http://localhost:8501`.

## Features

- **Episode Browser**: Select past runs from the sidebar.
- **3D Debugger**: Interactive PyVista viewer for STL artifacts.
- **Step-by-Step Replay**: Use the slider to step through the agent's actions.
- **Live Mode**: Watch active agent runs update in real-time.
