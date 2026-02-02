# Feature Specification: Agentic CAD Environment

**Feature**: 001-agentic-cad-environment
**Status**: Foundation
**Mission**: software-dev

## 1. Overview

The **Agentic CAD Environment** is a specialized software development environment designed to enable a Large Language Model (LLM) to function as an autonomous mechanical engineer.

Unlike traditional Reinforcement Learning (RL) environments that rely on discrete, low-level actions (e.g., "move cursor", "place box"), this environment implements a **Code-as-Policy** paradigm. The agent interacts with the world by writing, editing, and executing high-level Python scripts using the `build123d` parametric CAD library.

### 1.1 Superseded/Extended By
As the project evolved, the following specifications expanded on the foundations laid here:
* **[Spec 003](../003-mujoco-simulation-engine/spec.md)**: Detailed implementation of the MuJoCo simulation engine and physics bridge.
* **[Spec 004](../004-advanced-manufacturing-workbenches/spec.md)**: Advanced CNC and Injection Molding workbenches with DFM (Design for Manufacturing) checks.
* **[Spec 006](../006-cots-assembly-system/spec.md)**: COTS (Commercial Off-The-Shelf) part indexing and assembly tools.
* **[Spec 007](../007-agentic-cad-dashboard/spec.md)**: Streamlit-based dashboard for real-time monitoring and 3D debugging.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Enable Code-Based Design**: Create a robust loop where an LLM can iteratively write and fix `build123d` scripts to solve geometric prompts.
2. **Solve Dynamic Problems**: Support scenarios where the agent's design must perform work (move objects, apply forces) in a physics simulation, not just satisfy static geometry.
3. **Data Capture**: Persist 100% of agent interactions (successful and failed) to build a "Thought-Process" dataset.
4. **Extensible Architecture**: Establish a "Workbench" system to easily add domain-specific constraints (Injection Molding, Sheet Metal) in future updates. Note: Workbenches include specific cost calculations in addition to geometric constraints.

### 2.2. Success Criteria

* **Pipeline Latency**: "Render/Preview" tool returns a visual snapshot within 2 seconds for typical parts.
* **Data Integrity**: All agent actions, including code diffs and execution logs, are successfully stored in SQLite.
* **MVP Workbench**: A "3D Printing" workbench is implemented that successfully detects and rejects multi-body (non-contiguous) parts.
* **Safety**: Agent code execution is sandboxed (or at least isolated) to prevent system damage.

## 3. User Stories (The "Agent" as User)

* **As an Agent**, I want to search the `build123d` documentation (RAG) so that I use the correct syntax for complex operations (e.g., Lofts, Sweeps).
* **As an Agent**, I want to see a visual render of my current code (Preview) so that I can correct geometric errors before submission.
* **As an Agent**, I want to receive specific error messages (Python tracebacks or geometric violations) so that I can self-correct my script.
* **As an Agent**, I want to know the "Problem Scenario" (e.g., obstacles, target locations) so I can design a mechanism that fits the environment.
* **As an Agent**, I want to submit my final design so that it can be evaluated against the physical/functional requirements (Success/Fail, Energy Used).

## 4. Functional Requirements

### 4.1. The Environment Loop

The system shall implement a `gymnasium`-like (or compatible) interface, but adapted for tool-use:

1. **Observation**: Current file content, console output (stdout/stderr), last render (image), and **Task Description** (Natural Language + Geometric Constraints).
2. **Action**: Tool calls (Edit, RAG, Preview, Submit).
3. **Reward/Feedback**:
    * **Preview**: No external reward; intrinsic visual feedback only.
    * **Submit**: Detailed metrics including:
        * **Success**: Boolean (Task Completed?).
        * **Efficiency**: Energy consumed (Joules).
        * **Safety**: Environment damage (Collision impulse magnitude).
        * **Constraints**: Pass/Fail on static checks.

### 4.2. Tool Suite

The Environment shall expose the following tools to the Agent:

| Tool Name | Input | Output | Description |
| :--- | :--- | :--- | :--- |
| `search_docs` | `query` (str) | `snippets` (str) | RAG retrieval from `build123d` and `problemologist` docs. |
| `write_script` | `content` (str), `path` (str) | `status` (str) | Writes content to a specific file (e.g., `design.py`, `controller.py`). |
| `edit_script` | `find` (str), `replace` (str), `path` (str) | `status` (str) | Performs string replacement on the specified file. |
| `preview_design` | None | `image_path` (str) | Runs the script, exports an STL/SVG, renders it, and returns the view. No penalties. |
| `submit_design` | `control_path` (str) | `report` (json) | Runs the script, performs full Workbench validation, and returns final grades. Uses the script at `control_path` for motor logic. Calls Spec 003. |

### 4.3. Workbench Architecture

The system shall support pluggable "Workbenches" that define specific constraints and cost models.

* **Interface**:
  * `validate(geometry: Compound) -> List[Violation]`
  * `calculate_cost(geometry: Compound) -> float`
* **MVP Workbench (3D Printing)**:
  * **Constraint 1**: `ManifoldCheck` (Is the mesh watertight?).
  * **Constraint 2**: `SingleBodyCheck` (Does the design consist of exactly one solid? No floating islands).
  * **Cost Model**: Volume-based material cost (e.g., $0.05 per cm³).

### 4.4. Simulation Bridge (MuJoCo)

* The `submit_design` tool shall trigger the **Simulation Engine** (Spec 003).
* **Input**: `build123d` Compound objects.
* **Context**: The current **Problem Scenario** (defined as a partially filled MJCF XML template with pre-existing environment obstacles and goals).
* **Process**:
    1. Generate High-Res Mesh (Visual).
    2. Generate Convex Hull Decomposition (Collision).
    3. Generate `standard.xml` (MJCF) file structure.
    4. **Inject** the agent's design into the Problem Scenario XML.
    5. Run Simulation for $T$ seconds.
* **Output**:
  * `success`: Did the target object reach the goal zone?
  * `energy`: $\int \tau \cdot \omega dt$ (Total actuator work).
  * `damage`: Sum of impact forces on "Environment/Forbidden" geoms.

### 4.5. Persistence (The Black Box)

* **Database**: SQLite (`history.db`) managed via **SQLAlchemy**.
* **Schema**:
  * `Episodes`: (id, prompt, start_time, result).
  * `Steps`: (id, episode_id, tool_name, tool_input, tool_output, duration).
  * `Artifacts`: (id, step_id, code_snapshot, render_path).

## 5. Technical Design

### 5.1. Tech Stack

* **Language**: Python 3.10+
* **CAD Kernel**: `build123d` (wrapping OpenCASCADE).
* **Physics**: `mujoco` (Python bindings).
* **Rendering**: `vtk` or `pyglet` (via `build123d`'s export capability).
* **Database**: `sqlite3`.
* **LLM Interface**: Open-ended; the environment exposes an API compatible with standard Tool-Use loops (e.g., PydanticAI, LangChain, or raw OpenAI API).

### 5.2. Directory Structure

```
src/
├── environment/
│   ├── core.py           # Main Env class
│   ├── tools.py          # Tool definitions
│   └── persistence.py    # SQLite logger
├── workbenches/
│   ├── base.py           # Abstract Base Class
│   └── print_3d.py       # MVP Workbench
├── compiler/
│   ├── geometry.py       # Mesh generation/cleaning
│   └── mujoco_bridge.py  # MJCF XML generation
└── rag/
    └── vector_store.py   # ChromaDB/FAISS (or simple grep for MVP)
```

## 6. Assumptions & Constraints

* **Single Threaded**: The environment assumes a single synchronous agent per instance.
* **Local Execution**: Code is executed locally; security relies on the container/environment being ephemeral.
* **RAG Scope**: Initial RAG will be limited to a static dump of `build123d` documentation.
