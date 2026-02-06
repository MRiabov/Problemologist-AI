# Research: Agentic CAD Environment

## Architectural Foundation

The core goal is to enable LLMs to solve mechanical engineering problems by writing code (`build123d`). The environment must be safe, distributed, and highly observable.

### 1. Framework: DeepAgents

**Decision**: Use the **`deepagents`** framework.
**Rationale**:

- DeepAgents (introduced late 2025) provides the necessary abstractions for long-running coding agents.
- **`FilesystemMiddleware`**: Allows the agent to use standard tools (`ls`, `read`, `write`) over a virtual filesystem.
- **`TodoListMiddleware`**: Provides native support for `todo.md` management.
- **`Subagent Management`**: Simplifies calling specialist agents (e.g., for documentation or COTS search).

### 2. Execution: Distributed Worker/Controller

**Decision**: Split logic into a **Controller** (LLM, LangGraph) and a **Worker** (Isolated container).
**Rationale**:

- **Safety**: LLM-generated code runs in a Podman sandbox on the Worker.
- **Performance**: High-compute tasks (simulation, rendering, linting) happen on the Worker.
- **Latency**: Network latency is negligible compared to LLM reasoning time.

### 3. Geometry & Simulation Pipeline

**Decision**: `build123d` -> STL -> MJCF -> `mujoco`.
**Rationale**:

- `build123d` is the primary CAD tool used by the Engineer and Generator.
- `MuJoCo` is chosen for its speed and stability in contact dynamics, despite lack of FEA/Fluid support in MVP.
- **XML Verification**: MJCF files are verified by XML schema and a few frames of simulation before full execution.

### 4. Orchestration & Persistence

**Decision**: Use **Temporal** for worker orchestration and **Railway S3/Postgres** for persistence.
**Rationale**:

- **Temporal**: Manages long-running (>30s) simulations and renders, handling container preemption.
- **Postgres**: Stores traces, episode metadata, and relational state.
- **S3 (Railway Buckets)**: Stores large assets (VIDEOS, images, git snapshots).

### 5. Learning & Skills

**Decision**: Persistent **`SKILL.md`** files and a sidecar **Learner Agent**.
**Rationale**:

- Skills are versioned via Git and read-only for the primary agent.
- The **Learner Agent** runs asynchronously to update skills from successful reasoning traces in `journal.md`.
- Mitigation for LLM struggles with `build123d` syntax (e.g., `Compound` children).
