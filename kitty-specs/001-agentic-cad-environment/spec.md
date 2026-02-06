# Feature Specification: Agentic CAD Environment

**Feature**: 001-agentic-cad-environment
**Status**: Foundation
**Mission**: software-dev

## 1. Overview

The **Agentic CAD Environment** is the execution runtime and infrastructure that enables the **Engineer Agent** (Spec 002) and **Benchmark Generator** (Spec 005) to solve mechanical engineering problems.

This environment is built on top of the **`deepagents`** framework, utilizing a distributed architecture where the Agent (Controller) logic is separated from the Code Execution (Worker).

### 1.1 Core Architecture

- **DeepAgents Framework**: Leverages `deepagents` middleware for Filesystem, TodoList, and Subagent management.
- **Distributed Execution**:
  - **Controller Node**: Runs the LLM, parses tool calls, and orchestrates the workflow. Deployed on Railway.
  - **Worker Node**: A sandboxed container (Podman) where code is executed, simulation runs. The environment is reset from Git on every session.
- **Filesystem**: Agents operate on a `FilesystemMiddleware` using a **Hybrid Architecture**:
  - **Sandbox Filesystem Backend**: Primary workspace for code edits, logs, and execution state. Deployed as a safe, disposable environment in workers.
- **Selective S3 Routing**: Specific paths (like `/renders/` for large media) are transparently routed to S3-compatible storage (MinIO/Railway) via a `CompositeBackend`.
- **Tools Paradigm**: Agents use a minimal set of "OS-level" tools (`ls`, `view_file`, `write_file`) and high-level "Utils" (Python functions imported in the script) for domain-specific actions.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Distributed & Safe**: Execute untrusted LLM-generated code in isolated Worker containers, physically separated from the Controller.
2. **Code-as-Policy**: Enable agents to solve problems by writing Python scripts using `build123d` and `mujoco`.
3. **Hybrid Persistence**: Use local sandbox for speed during reasoning, and S3 for persistence of final assets and large simulation media.
4. **High-Fidelity Observability**: Capture reasoning traces, code evolution (git), and simulation results (video/summary) for future training.
5. **Manufacturability & Cost**: Integrate "Workbenches" that provide feedback on cost and manufacturability via Python APIs.

### 2.2. Success Criteria

- **Latency**: Local filesystem operations overhead < 100ms. Worker execution overhead < 500ms.
- **Resilient Media**: Simulation videos are rendered on-demand and uploaded to S3 immediately to save volume space.
- **Disposable Workers**: Workers can be preempted; state is manageable because reasoning and small files are in the sandbox, while persistent artifacts are on S3.

## 3. User Stories (The "Agent" as User)

- **As an Agent**, I want a familiar filesystem structure (Skills, Utils, Templates) so I can orient myself quickly.
- **As an Agent**, I want my "tools" to be just Python imports (e.g., `from utils import simulate`) so I can write standard Python code without confusing tool-calling syntax.
- **As an Agent**, I want to receive feedback in Markdown (linting errors, simulation results) because I understand it better than JSON.
- **As an Agent**, I want to know that my environment is randomized (benchmarks) but my tools are deterministic.

## 4. Functional Requirements

### 4.1. Filesystem & Directory Structure

The Worker container initializes with the following structure:

```text
.
├── skills/                     # [Read-Only] Learned skills (markdown/code)
├── utils/                      # [Read-Only] Fixed Python utilities (simulation, validation)
├── renders/                    # [S3-Backed/Write] Large media (videos, images)
├── reviews/                    # [Read-Only] Reviews from the Reviewer agent
├── journal.md                  # [Local/Write] Episodic memory
├── todo.md                     # [Local/Write] Planner's TODO list
├── plan.md                     # [Local/Read] High-level plan
└── script.py                   # [Local/Write] Main execution script (from template)
```

**Filesystem Middleware**:

- Provides a unified POSIX-like interface to the Agent.
- Uses `SandboxFilesystemBackend` for high-frequency operations and safe isolation.
- Uses `CompositeBackend` to route `/renders/` to S3 buckets.
- Enforces Read-Only paths (`skills/`, `utils/`).

### 4.2. Tool Suite (OS Level)

The Controller exposes these tools to the LLM. The Controller forwards relevant commands to the Worker.

| Tool Name | Input | Description |
| :--- | :--- | :--- |
| `ls` | `path` (str) | List directory contents. |
| `view_file` | `path` (str) | Read file content. |
| `write_file` | `path`, `content` | Write full file content (triggers linting). |
| `edit_file` | `path`, `find`, `replace` | Edit file content (triggers linting). |
| `wait` | `seconds` | Pause execution (rarely used, mostly implicit). |

### 4.3. Utils (Python Level)

Domain-specific actions are performed by writing Python code that imports and calls these functions. The `deepagents` runtime ensures these are available in `PYTHONPATH`.

| Function Signature | Description |
| :--- | :--- |
| `validate_and_price(component) -> dict` | Validates geometry for manufacturability and calculates cost. |
| `simulate(component) -> SimulationResult` | Commits code to git, runs MuJoCo simulation, returns pass/fail and summary. Automatically handles `/renders/` upload. |
| `submit_for_review(component)` | Submits the final design to the Reviewer/User. |

### 4.4. Simulation & Validation Loop

1. **Agent writes code**: Code calls `simulate(compound)`.
2. **Worker executes**:
    - **Git Commit**: SNAPSHOT the current state.
    - **Compile**: Convert `build123d` to MJCF.
    - **Simulate**: Run MuJoCo.
    - **Upload**: Render results are written to `/renders/`, transparently persisted to S3.
3. **Feedback**: Return Markdown summary to the Agent (pass/fail, energy used, damage).

### 4.5. Persistence & Observability

- **Databases**:
  - **Postgres (Controller)**: LangChain traces, Agent State, Task Status.
  - **Postgres (Temporal)**: Orchestration state.
  - **S3 (Global)**: Final Assets (Videos, Images, Git Archives).
- **Filesystem Persistence**:
  - Local Sandbox is ephemeral; results intended for persistence must be moved to S3 or committed to Git.
- **Logging**: `structlog` for structured logs.
- **LangFuse**: Integrated for LLM trace observability.

## 5. Technical Design

### 5.1. Tech Stack

- **Framework**: `deepagents` (extending LangChain/LangGraph).
- **Orchestration**: `Temporal` (for long-running worker tasks).
- **Runtime**: Python 3.10+, Podman (for Worker sandboxing).
- **Schema**: OpenAPI strictly defined between Controller and Worker.
- **CAD/Sim**: `build123d`, `mujoco`.

### 5.2. Component Interaction

1. **Controller**: Receives "Task". Spawns "Engineer Agent".
2. **Agent**: Decides to "Write Script". Calls `write_file` tool.
3. **Controller**: Sends `write_file` request to **Worker** API.
4. **Worker**: Writes to S3 backing. Runs `ruff` linter. Returns result.
5. **Agent**: Decides to "Run Script". Calls `run_command("python script.py")`.
6. **Controller**: Sends command to Worker.
7. **Worker**: Executes script. Script calls `validate_and_price`. Worker runs logic. Returns stdout/stderr.

## 6. Assumptions & Constraints

- **Latency**: Network latency between Controller and Worker is acceptable vs LLM think time.
- **Stateless Workers**: Workers can be preempted; Temporal handles retries/resourcing.
- **Security**: Agents cannot escape the Podman container or modifying Read-Only utils.
