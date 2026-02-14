# Research: DeepAgents Framework (LangChain)

**Date**: 2026-02-05
**Source**: Web Research & Architecture Alignment
**Subject**: Migration from Native Tools to `deepagents` Framework

## 1. Overview

`deepagents` is a framework by LangChain developers designed to build "deep" agents—agents capable of long-horizon planning, iteration, and complex task execution. It moves beyond simple "shallow" tool-calling loops by providing robust middleware for state, memory, and filesystem management.

Key philosophy:

* **Planning First**: Explicit planning steps before execution.
* **Deep Memory**: Uses file-system based memory (workspace) to persist context and state across long runs.
* **Sub-Agent Delegation**: Capable of spawning sub-agents for specialized tasks.
* **Graph-Based**: Leverages LangGraph for state management.
* **Middleware-Driven**: Encapsulates common agent requirements (Filesystem, TODOs) into reusable middleware.

## 2. Key Capabilities & Usage in VLM CAD Agent

We are using `deepagents` to its **fullest extent**, leveraging its middleware and protocol adapters to orchestrate a distributed, reliable coding agent.

### 2.1. Middleware & Architecture

* **FilesystemMiddleware**: We use this to manage the agent's interaction with files.
  * **S3 Backend**: The filesystem is backed by an S3-compatible layer (exposing the worker's environment), allowing persistent and distributed access to files.
  * **Docker Integration**: We implement the `SandboxBackendProtocol` to interface with logic running inside **Docker** sandboxes. This gives the agent direct control over a sandboxed environment for safe code execution.
* **TodoListMiddleware**: Provides a structured `todo_list` capability, essential for the "Architect -> Engineer" workflow where tasks are strictly tracked.

### 2.2. Distributed Execution (Controller/Worker)

The architecture splits the "Brain" from the "Body":

* **Controller (Brain)**: Runs the `deepagents` graph, managing state, planning, and tool dispatch.
* **Worker (Body)**: Runs a **Docker** sandbox where the actual code execution, linting, and simulation happen.
* **Temporal Integration**: Long-running tools (like `simulate`) are orchestrated via Temporal. The `deepagents` tool implementation delegates to a Temporal workflow, ensuring durable execution and retries without blocking the agent's prompt loop.

### 2.3. Planning & Reflection Loop

* **Planner Node**: Generates high-level plans using the `TodoListMiddleware`.
* **Executor Node**: Executes steps in the Docker sandbox using `FilesystemMiddleware`.
* **Learner/Reflector**: A sidecar process (or sub-agent) that monitors execution, updates `journal.md`, and persists learned patterns to `SKILL.md` files.

### 2.4. Sub-Agents

We leverage the hierarchy capabilities to spawn specialized sub-agents. DeepAgents supports **Context Quarantine**, where sub-agents execute tasks without polluting the main agent's context.

* **Configuration**: We will use the dictionary-based configuration for most sub-agents (lighter weight).
  * **Required**: `name`, `description` (for routing), `system_prompt`, `tools`.
  * **Optional**: `interrupt_on` (can override main agent's safety settings).
* **Use Case**:
  * **Documentation Agent**: A sub-agent invoked via `get_docs_for`.
  * **Skill Learner**: A sidecar sub-agent that processes logs potentially in parallel.

### 2.5. Human-in-the-loop (HITL) & Safety

To satisfy the debugging requirements ("Interrupt them before they finish"), we utilize `deepagents` native interruption capabilities.

* **Configuration**: `interrupt_on` parameter mapping tools to permission levels.
  * `True`: Enables `approve`, `edit`, `reject` workflow.
  * `{"allowed_decisions": ["approve"]}`: For critical paths where rejection isn't an option but pause is needed.
* **Checkpointer**: Required for HITL. We use `PostgresSaver` (via LangGraph) to persist state, allowing the user to resume execution days later if needed.
* **Batched Interrupts**: If multiple tools are called (e.g. `write_file` and `simulate`), the user reviews them in a batch.

### 2.6. Skills & Progressive Disclosure

Skills are not just static text but a directory structure (`SKILL.md` + assets).

* **Structure**:

    ```text
    skills/
    ├── build123d-basics/
    │   ├── SKILL.md (Frontmatter + Instructions)
    │   └── examples.py
    ```

* **Progressive Disclosure**: The agent first reads the frontmatter. Only if the skill is relevant does it read the full content/assets. This saves context tokens.

### 2.7. Async Execution

Since the Controller communicates with the Worker (Docker) over the network (HTTP/S3), we **must** use the asynchronous API provided by `deepagents`.

* **Methods**: Use `aread`, `awrite`, `aedit`, `aexecute` instead of their synchronous counterparts.
* **Rationale**: This prevents blocking the Controller's event loop during file I/O and command execution, which is critical for a responsive, high-throughput agent system.
* **LangGraph Compatibility**: LangGraph nodes should be defined as `async def` functions to natively await these calls.

## 3. Implementation Changes

### 3.1. Tech Stack

* **Framework**: `deepagents`, `langgraph`, `langchain-core`.
* **Runtime**: `Docker` (programmatic sandbox control).
* **Orchestration**: `Temporal` (for durability of long tools).
* **Observability**: `LangFuse` (via `deepagents` callbacks/integration).

### 3.2. Architecture

* **State**: Managed by LangGraph, persisted in Postgres.
* **Files**: Persisted in Docker Container (ephemeral) + S3 (Assets/Snapshots).
* **Communication**: Controller talks to Worker via Docker SDK / HTTP (S3 protocol).

## 4. Resources

* **Official Docs**: `deepagents` documentation.
* **Docker SDK**: Usage of programmatic sandbox control.
* **LangGraph**: Core state machine.
