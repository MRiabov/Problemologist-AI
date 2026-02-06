# Phase 0 Research: VLM CAD Agent

## 1. Resolved Unknowns

### 1.1. Architecture Coupling

* **Question**: Should the Agent be tightly coupled (module import) or loosely coupled (process/container) to the Environment?
* **Decision**: **Distributed Orchestration** via **`deepagents`**.
* **Rationale**: The Controller (Agent) communicates with Worker nodes over the network via HTTP/S3. This provides maximum security and scalability.

### 1.2. VLM Integration

* **Decision**: Use `litellm` within the `deepagents` framework.
* **Rationale**: Standardized access to GPT-4o, Gemini, and DeepSeek, critical for benchmarking.

## 2. Technical Decisions

* **Loop**: **`deepagents` Graph (LangGraph)**.
* **Middleware**:
  * `FilesystemMiddleware`: For distributed file access.
  * `TodoListMiddleware`: For structured task management.
* **Learner Sidecar**: A dedicated sub-agent processes `journal.md` to update `SKILL.md` files.

> [!NOTE]
> For more details on the framework implementation, see [deepagents-research.md](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/kitty-specs/002-vlm-cad-agent/deepagents-research.md).

* **Memory**: Flat file `journal.md`.
  * *Reason*: Simplicity and portability. Vector DB adds unnecessary complexity for <1000 journal entries.
